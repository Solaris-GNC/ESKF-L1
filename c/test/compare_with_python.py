"""Compara el ESKF en C (ejecutable replay) con la referencia eskf.py paso a paso.

Uso:
    python3 compare_with_python.py RUTA/replay [--ref-dir DIR] [--seeds 8] [--steps 3000]
                                   [--tol 1e-10] [--psd-tol 1e-9]

Referencia Python (por orden de prioridad):
    1. --ref-dir DIR
    2. variable de entorno ESKF_PY_REF
    3. ../.. respecto a este archivo (estructura del repositorio solaris-eskf: c/test/ -> raíz)
El directorio debe contener eskf.py, quaternion.py, linalg_utils.py y sensors.py. Se
informa de la ruta, el commit de git (si lo hay) y el SHA-256 de los archivos usados.

Códigos de salida:
    0  equivalencia comprobada
    1  diferencia, formato inválido, valores no finitos o covarianza no semidefinida
    2  el ejecutable C falló
    3  referencia Python no disponible (no se ha comparado nada)

Antes de calcular ningún error se comprueba que la salida del C tiene exactamente una línea
por comando, el número exacto de campos, números válidos y todos finitos. Un NaN siempre
hace fallar la comparación (antes, NaN > peor era siempre falso y daba OK con error 0).
"""
import argparse
import hashlib
import importlib
import os
import subprocess
import sys

import numpy as np

N_STATE = 16
N_P = 15 * 15
N_FIELDS = 2 + N_STATE + N_P  # "OUT", ret, estado, P

# Códigos de eskf.h que intervienen en la comparación
ESKF_OK = 0
ESKF_E_MOTION = -8

REF_FILES = ("eskf.py", "quaternion.py", "linalg_utils.py", "sensors.py")


class CompareError(Exception):
    """Salida inválida o no comparable."""


# --------------------------------------------------------------------------- referencia

def locate_reference(ref_dir=None):
    """Devuelve (ruta, origen) o (None, motivo)."""
    candidates = []
    if ref_dir:
        candidates.append((ref_dir, "--ref-dir"))
    elif os.environ.get("ESKF_PY_REF"):
        candidates.append((os.environ["ESKF_PY_REF"], "ESKF_PY_REF"))
    else:
        candidates.append((os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."),
                           "ruta del repositorio (c/test/../..)"))
    for path, origin in candidates:
        path = os.path.abspath(path)
        missing = [f for f in REF_FILES if not os.path.isfile(os.path.join(path, f))]
        if not missing:
            return path, origin
        return None, f"{origin} = {path}: faltan {', '.join(missing)}"
    return None, "sin candidatos"


def describe_reference(path):
    lines = [f"referencia Python: {path}"]
    try:
        head = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain", "--", *REF_FILES],
                               capture_output=True, text=True, check=True).stdout.strip()
        lines.append(f"  git {head}{' (con cambios locales en la referencia)' if dirty else ''}")
    except (OSError, subprocess.CalledProcessError):
        lines.append("  git: no disponible")
    for f in REF_FILES[:3]:
        h = hashlib.sha256(open(os.path.join(path, f), "rb").read()).hexdigest()[:16]
        lines.append(f"  sha256({f}) = {h}…")
    return "\n".join(lines)


def load_reference(path):
    sys.path.insert(0, path)
    try:
        mod = importlib.import_module("eskf")
        quat = importlib.import_module("quaternion")
    finally:
        sys.path.pop(0)
    if os.path.realpath(os.path.dirname(mod.__file__)) != os.path.realpath(path):
        raise CompareError(f"se importó eskf desde {mod.__file__}, no desde {path}")
    return mod.ESKF, quat


# --------------------------------------------------------------------------- escenarios

def scenario(seed, steps, quat):
    """Escenario aleatorio coherente: rotación, aceleración, todos los sensores."""
    rng = np.random.default_rng(seed)
    mag_ref = np.array([27.0, -0.5, 34.0]) * rng.uniform(0.8, 1.2)
    params = dict(sigma_a=rng.uniform(0.01, 0.2), sigma_w=rng.uniform(1e-3, 1e-2),
                  sigma_aw=rng.uniform(1e-5, 1e-3), sigma_ww=rng.uniform(1e-6, 1e-4),
                  mag_ref=mag_ref, accel_threshold=0.5)
    pdiag = np.concatenate([np.full(3, 4.0), np.full(3, 0.25), np.full(3, 0.05 ** 2),
                            np.full(3, 0.1 ** 2), np.full(3, 0.01 ** 2)])
    q0 = quat.quat_normalize(rng.normal(size=4))
    x0 = np.concatenate([rng.normal(size=3), rng.normal(size=3) * 0.1, q0,
                         rng.normal(size=3) * 0.02, rng.normal(size=3) * 0.002])
    dt = 0.01
    cmds = [("PARAMS", [params["sigma_a"], params["sigma_w"], params["sigma_aw"], params["sigma_ww"],
                        params["accel_threshold"], 0.0, 1, *mag_ref, 1.5 * dt, 0.0]),
            ("PDIAG", list(pdiag)), ("STATE", list(x0)),
            # Python no tiene el concepto de reposo confirmado: se habilita para poder
            # comparar la actualización de gravedad, que en C lo exige.
            ("REST", [1])]

    q_t = q0.copy(); p_t = x0[0:3].copy(); v_t = x0[3:6].copy()
    g = np.array([0, 0, -9.81])
    for k in range(steps):
        w = 0.5 * np.array([np.sin(0.01 * k), np.cos(0.013 * k), 0.3 * np.sin(0.007 * k)])
        acc_nav = 0.0 if (k // 500) % 2 == 0 else 2.0 * np.array([np.sin(0.02 * k), 0.3, -0.5])
        Rt = quat.rotation_matrix_hamilton(q_t)
        f_body = Rt.T @ (acc_nav + g)
        a_m = f_body + rng.normal(size=3) * params["sigma_a"]
        w_m = w + rng.normal(size=3) * params["sigma_w"]
        cmds.append(("PRED", [*a_m, *w_m, dt]))
        p_t = p_t + v_t * dt; v_t = v_t + acc_nav * dt
        q_t = quat.quat_normalize(quat.quat_product(q_t, quat.quat_from_rotvec(w * dt)))
        Rt = quat.rotation_matrix_hamilton(q_t)
        if k % 10 == 0:
            cmds.append(("GPS", [*(p_t + rng.normal(size=3) * 2), *(v_t + rng.normal(size=3) * 0.2),
                                 2.0, 2.0, 4.0, 0.2, 0.2, 0.4]))
        if k % 4 == 1:
            m = Rt.T @ mag_ref + rng.normal(size=3) * 0.5
            cmds.append(("MAG", [*m, 0.5, 0.5, 0.5]))
        if k % 5 == 2:
            cmds.append(("BAR", [p_t[2] + rng.normal() * 0.5, 0.5]))
        if k % 2 == 0:
            cmds.append(("ACC", [*a_m, 0.3, 0.3, 0.3]))
        if k % 50 == 7:
            cmds.append(("VEL", [*(v_t + rng.normal(size=3) * 0.05), 0.05, 0.05, 0.05]))
    return params, cmds


def state_vec(f):
    return np.concatenate([f.p, f.v, f.q, f.b_a, f.b_g, f.P.ravel()])


def run_python(ESKF, params, cmds):
    """Ejecuta los comandos en la referencia. Devuelve [(ret_esperado_en_C, estado)]."""
    f = ESKF(**params)
    out = []
    for name, a in cmds:
        a = np.asarray(a, dtype=float)
        ret = ESKF_OK
        if name in ("PARAMS", "REST"):
            pass
        elif name == "PDIAG":
            f.P = np.diag(a)
        elif name == "STATE":
            f.p, f.v, f.q = a[0:3].copy(), a[3:6].copy(), a[6:10].copy()
            f.b_a, f.b_g = a[10:13].copy(), a[13:16].copy()
        elif name == "PRED":
            f._predict(a[0:3], a[3:6], a[6])
        elif name == "GPS":
            f.update_gps(a[0:3], a[3:6], a[6:9], a[9:12]); ret = 6
        elif name == "VEL":
            H_x = np.zeros((3, 16)); H_x[:, 3:6] = np.eye(3)
            f._update(a[0:3], f.v.copy(), f._observation_jacobian(H_x), np.diag(a[3:6] ** 2)); ret = 3
        elif name == "MAG":
            f.update_mag(a[0:3], a[3:6]); ret = 3
        elif name == "ACC":
            ret = 3 if f.update_accel(a[0:3], a[3:6]) else ESKF_E_MOTION
        elif name == "BAR":
            f.update_baro(a[0], a[1]); ret = 1
        else:
            raise CompareError(f"comando sin equivalente en Python: {name}")
        out.append((ret, state_vec(f)))
    return out


# --------------------------------------------------------------------------- salida del C

def commands_to_text(cmds):
    return "".join(f"{n} " + " ".join(repr(float(x)) for x in a) + "\n" for n, a in cmds)


def parse_c_output(text, n_cmds):
    """Valida y convierte la salida del replay. Lanza CompareError ante cualquier anomalía."""
    lines = text.splitlines()
    if len(lines) != n_cmds:
        raise CompareError(f"el C produjo {len(lines)} líneas para {n_cmds} comandos")
    out = []
    for i, line in enumerate(lines):
        parts = line.split()
        if len(parts) != N_FIELDS:
            raise CompareError(f"línea {i}: {len(parts)} campos, se esperaban {N_FIELDS}")
        if parts[0] != "OUT":
            raise CompareError(f"línea {i}: no empieza por OUT")
        try:
            ret = int(parts[1])
            vals = np.array([float(x) for x in parts[2:]], dtype=float)
        except ValueError as e:
            raise CompareError(f"línea {i}: valor no numérico ({e})") from None
        bad = ~np.isfinite(vals)
        if bad.any():
            raise CompareError(f"línea {i}: {int(bad.sum())} valores no finitos (primer índice {int(np.argmax(bad))})")
        out.append((ret, vals))
    return out


def run_c(exe, cmds):
    res = subprocess.run([exe], input=commands_to_text(cmds), capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"{exe} terminó con código {res.returncode}: {res.stderr.strip()}")
    return parse_c_output(res.stdout, len(cmds))


# --------------------------------------------------------------------------- comparación

def check_covariance(P, psd_tol):
    """Simetría y autovalor mínimo >= -psd_tol·max(1, max diag). Devuelve (ok, λmin)."""
    if not np.array_equal(P, P.T):
        return False, float("nan")
    lam = float(np.linalg.eigvalsh(P).min())
    return lam >= -psd_tol * max(1.0, float(np.abs(np.diag(P)).max())), lam


def compare(py, c, cmds, tol, psd_tol):
    """Devuelve (peor_error, dónde, lista_de_fallos)."""
    failures = []
    if len(py) != len(c):
        return np.inf, None, [f"Python {len(py)} salidas, C {len(c)}"]
    worst, where = 0.0, None
    for i, ((rp, sp), (rc, sc)) in enumerate(zip(py, c)):
        name = cmds[i][0]
        if not np.all(np.isfinite(sp)):
            failures.append(f"comando {i} {name}: la REFERENCIA Python produjo valores no finitos")
            continue
        if rp != rc:
            failures.append(f"comando {i} {name}: ret Python={rp} C={rc}")
        Pc = sc[N_STATE:].reshape(15, 15)
        ok, lam = check_covariance(Pc, psd_tol)
        if not ok:
            failures.append(f"comando {i} {name}: P del C no simétrica o autovalor mínimo {lam:.3e}")
        qn = np.linalg.norm(sc[6:10])
        if abs(qn - 1.0) > 1e-5:
            failures.append(f"comando {i} {name}: ‖q‖ = {qn}")
        if name == "PARAMS":
            continue  # P inicial distinta hasta PDIAG
        if np.dot(sp[6:10], sc[6:10]) < 0:  # q y -q son la misma rotación
            sc = sc.copy(); sc[6:10] *= -1
        Pp = sp[N_STATE:].reshape(15, 15)
        sig = np.sqrt(np.maximum(np.diag(Pp), 1e-300))
        err = max(float(np.max(np.abs(sp[:N_STATE] - sc[:N_STATE]))),
                  float(np.max(np.abs(Pp - Pc) / np.outer(sig, sig))))
        if not np.isfinite(err):
            failures.append(f"comando {i} {name}: error no finito")
            continue
        if err > worst:
            worst, where = err, (i, name)
        if len(failures) > 20:
            failures.append("… (se omiten más fallos)")
            break
    if worst > tol:
        failures.append(f"error máximo {worst:.3e} > tolerancia {tol:.0e} (en {where})")
    return worst, where, failures


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exe")
    ap.add_argument("--ref-dir", default=None)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--steps", type=int, default=3000)
    ap.add_argument("--tol", type=float, default=1e-10, help="estado absoluto y P escalada por σiσj")
    ap.add_argument("--psd-tol", type=float, default=1e-9, help="autovalor mínimo relativo admitido")
    args = ap.parse_args(argv)
    if args.seeds < 1 or args.steps < 1:
        print(f"PARÁMETROS INVÁLIDOS: --seeds {args.seeds} --steps {args.steps}: no se compararía nada.")
        return 1

    ref, origin = locate_reference(args.ref_dir)
    if ref is None:
        print(f"REFERENCIA PYTHON NO DISPONIBLE ({origin}).\n"
              "Indique --ref-dir o ESKF_PY_REF con la raíz de solaris-eskf. No se ha comparado nada.")
        return 3
    print(describe_reference(ref) + f"\n  (origen: {origin})")
    ESKF, quat = load_reference(ref)

    all_ok, worst_all, compared = True, 0.0, 0
    for seed in range(args.seeds):
        params, cmds = scenario(seed, args.steps, quat)
        py = run_python(ESKF, params, cmds)
        try:
            c = run_c(args.exe, cmds)
        except RuntimeError as e:
            print(f"seed {seed}: {e}")
            return 2
        except CompareError as e:
            print(f"seed {seed}: SALIDA DEL C INVÁLIDA: {e}")
            all_ok = False
            continue
        worst, where, failures = compare(py, c, cmds, args.tol, args.psd_tol)
        compared += len(cmds)
        n_acc = sum(1 for n, _ in cmds if n == "ACC")
        n_acc_used = sum(1 for (n, _), (r, _) in zip(cmds, py) if n == "ACC" and r > 0)
        print(f"seed {seed}: {len(cmds)} comandos, acc usados {n_acc_used}/{n_acc}, "
              f"error máx {worst:.2e} (en {where})")
        for msg in failures:
            print("   FALLO:", msg)
        all_ok &= not failures
        worst_all = max(worst_all, worst)
    if compared == 0:
        print("RESULTADO: FALLO (no se ha comparado ningún comando)")
        return 1
    print("RESULTADO:", "OK" if all_ok else "FALLO", f"(peor {worst_all:.2e}, tol {args.tol:.0e}, {compared} comandos)")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
