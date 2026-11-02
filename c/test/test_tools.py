"""Pruebas de las herramientas de verificación (no del filtro):

  - comparador: no puede dar OK con salidas no finitas, truncadas, con líneas de menos o
    con una diferencia real (fallo 3 de la revisión);
  - replay: rechaza entradas malformadas (fallo 11).

Uso: python3 test_tools.py RUTA/replay [--ref-dir DIR]
Las pruebas de extremo a extremo del comparador necesitan la referencia Python; si no está,
se indica OMITIDO y se ejecutan solo las que no la necesitan.
"""
import argparse
import os
import stat
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compare_with_python as cwp  # noqa: E402

results = []


def check(name, cond, detail=""):
    results.append(cond)
    print(f"  [{'OK' if cond else 'FALLO'}] {name}{'' if cond else ' — ' + detail}")


def valid_line(ret=0, value=0.5):
    return "OUT %d " % ret + " ".join([repr(value)] * (cwp.N_FIELDS - 2))


def parser_unit_tests():
    print("comparador: validación de la salida del C (sin referencia)")
    good = "\n".join([valid_line(), valid_line(1)])
    try:
        out = cwp.parse_c_output(good, 2)
        check("salida válida aceptada", len(out) == 2)
    except cwp.CompareError as e:
        check("salida válida aceptada", False, str(e))

    bad_cases = {
        "NaN en el estado": valid_line().replace("0.5", "nan", 1),
        "inf en P": valid_line()[::-1].replace("5.0", "fni", 1)[::-1],
        "campo de menos": valid_line().rsplit(" ", 1)[0],
        "campo de más": valid_line() + " 0.5",
        "texto no numérico": valid_line().replace("0.5", "abc", 1),
        "ret no entero": valid_line().replace("OUT 0", "OUT x", 1),
        "prefijo distinto": valid_line().replace("OUT", "XXX", 1),
    }
    for name, line in bad_cases.items():
        try:
            cwp.parse_c_output(line + "\n" + valid_line(), 2)
            check(f"rechaza: {name}", False, "aceptada")
        except cwp.CompareError:
            check(f"rechaza: {name}", True)
    try:
        cwp.parse_c_output(valid_line(), 2)
        check("rechaza: línea de menos", False, "aceptada")
    except cwp.CompareError:
        check("rechaza: línea de menos", True)

    import numpy as np
    P = np.eye(15)
    P[0:3, 0:3] = -0.9
    np.fill_diagonal(P, 1.0)
    ok, lam = cwp.check_covariance(P, 1e-9)
    check("comparador rechaza P con autovalor -0.8", (not ok) and abs(lam + 0.8) < 1e-12, f"λmin={lam}")


WRAPPER = """#!{py}
import subprocess, sys
out = subprocess.run([{exe!r}], stdin=sys.stdin, capture_output=True, text=True).stdout.splitlines()
mode = {mode!r}
for i, line in enumerate(out):
    p = line.split()
    if mode == "nan":            # reproducción exacta del fallo informado
        p = p[:2] + ["nan"] * (len(p) - 2)
    elif mode == "truncate" and i == 10:
        p = p[:-1]
    elif mode == "drop" and i == 10:
        continue
    elif mode == "perturb" and i == len(out) // 2:
        p[2 + 16 + 2 * 15 + 2] = repr(float(p[2 + 16 + 2 * 15 + 2]) * (1 + 1e-6))
    print(" ".join(p))
"""


def end_to_end_tests(exe, ref_dir):
    ref, origin = cwp.locate_reference(ref_dir)
    if ref is None:
        print(f"comparador de extremo a extremo: OMITIDO, referencia no disponible ({origin})")
        return
    print("comparador de extremo a extremo (envoltorios que alteran la salida del C)")
    with tempfile.TemporaryDirectory() as tmp:
        def wrapper(mode):
            path = os.path.join(tmp, f"wrap_{mode}.py")
            with open(path, "w") as fh:
                fh.write(WRAPPER.format(py=sys.executable, exe=os.path.abspath(exe), mode=mode))
            os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
            return path

        base = ["--seeds", "1", "--steps", "300"] + (["--ref-dir", ref_dir] if ref_dir else [])
        for args_extra, name in [(["--seeds", "0"], "--seeds 0"), (["--steps", "0"], "--steps 0")]:
            cmd = [sys.executable, os.path.join(HERE, "compare_with_python.py"), os.path.abspath(exe),
                   *(["--ref-dir", ref_dir] if ref_dir else []), *args_extra]
            res = subprocess.run(cmd, capture_output=True, text=True)
            check(f"{name}: no puede dar éxito sin comparar (v3: OK)", res.returncode == 1,
                  f"código {res.returncode}: {res.stdout.strip()[-80:]}")
        for mode, expect in [("identity", 0), ("nan", 1), ("truncate", 1), ("drop", 1), ("perturb", 1)]:
            cmd = [sys.executable, os.path.join(HERE, "compare_with_python.py"), wrapper(mode), *base]
            res = subprocess.run(cmd, capture_output=True, text=True)
            last = (res.stdout.strip().splitlines() or [""])[-1]
            check(f"envoltorio '{mode}': código {expect}", res.returncode == expect,
                  f"código {res.returncode}, última línea: {last}")


def replay_tests(exe):
    print("replay: entradas malformadas")
    cases = [
        ("PRED 0", 2, "faltan argumentos"),
        ("BAR abc 0.5", 2, "número inválido"),
        ("BAR 3", 2, "faltan argumentos"),
        ("BAR 3 0.5 7", 2, "sobran argumentos"),
        ("BAR 3x 0.5", 2, "número inválido"),
        ("BAR 1e999 0.5", 2, "fuera de rango"),
        ("FOO 1", 2, "comando desconocido"),
        ("REST 2", 2, "REST debe ser"),
        ("PARAMS 0.05 0.005 1e-4 1e-5 0.5 0 2 0 0 0 0.015 0", 2, "has_mag"),
        ("PARAMS 0.05 0.005 1e-4 1e-5 0.5 0 0 0 0 0 0 0", 3, "configuración inválida"),
        ("BAR " + "1" * 5000 + " 0.5", 2, "demasiado larga"),
    ]
    for line, code, msg in cases:
        res = subprocess.run([exe], input=line + "\n", capture_output=True, text=True)
        check(f"'{line[:40]}' -> código {code}", res.returncode == code and msg in res.stderr,
              f"código {res.returncode}, stderr: {res.stderr.strip()[:80]}")
    ok_lines = "# comentario\n\nBAR nan 0.5\nBAR 0.0 0.5\n"
    res = subprocess.run([exe], input=ok_lines, capture_output=True, text=True)
    lines = res.stdout.splitlines()
    check("'nan' explícito llega al filtro y éste lo rechaza (ret -4)",
          res.returncode == 0 and len(lines) == 2 and lines[0].split()[1] == "-4",
          f"código {res.returncode}, salida {[l[:12] for l in lines]}")


NAV_CFG = """CFG eskf.sigma_a 0.05
CFG eskf.sigma_w 0.005
CFG eskf.sigma_aw 1e-4
CFG eskf.sigma_ww 1e-5
CFG eskf.dt_max 0.0075
CFG eskf.ba0_std 0.2
CFG eskf.bg0_std 0.01
CFG rest_window_s 1.0
CFG rest_acc_dev_max 0.5
CFG rest_acc_std_max 0.2
CFG rest_gyro_max 0.2
CFG liftoff_acc 19.62
CFG acc_sat_limit 0
CFG gyro_sat_limit 0
CFG baro_sigma 0.5
CFG baro_latency_s 0
CFG eskf.accel_threshold 0.5
CFG eskf.align_acc_tol 0.5
CFG eskf.align_gyro_max 0.2
CFG rest_tilt_change_max 0.005
"""


def nav_replay_tests(exe):
    print("nav_replay: configuración, formato y ejecución")
    def run(text):
        return subprocess.run([exe], input=text, capture_output=True, text=True)
    cases = [
        ("INIT\n", 3, "eskf.sigma_a"),
        (NAV_CFG.replace("CFG baro_latency_s 0\n", "") + "INIT\n", 3, "baro_latency_s"),
        (NAV_CFG + "INIT\nTICK nan\n", 2, "TICK"),
        ("CFG nada 1\n", 2, "campo de configuración desconocido"),
        ("CFG eskf.dt_max abc\n", 2, "número inválido"),
        ("IMU 0 0 0 -9.81 0 0 0 0\n", 2, "IMU antes de INIT"),
        (NAV_CFG + "INIT\nCFG baro_sigma 1\n", 2, "CFG después de INIT"),
        (NAV_CFG + "INIT\nIMU 0 0 0\n", 2, "IMU necesita"),
        (NAV_CFG + "INIT\nIMU 0 0 0 -9.81 0 0 0 64\n", 2, "sat debe ser"),
        (NAV_CFG + "INIT\nBARO 0\n", 2, "BARO necesita"),
        (NAV_CFG + "INIT\nFOO\n", 2, "orden desconocida"),
    ]
    for text, code, msg in cases:
        r = run(text)
        last = text.strip().splitlines()[-1]
        check(f"'{last[:30]}' -> código {code}", r.returncode == code and msg in r.stderr,
              f"código {r.returncode}, stderr: {r.stderr.strip()[:90]}")

    # ejecución: 1 s de reposo (alineación en t = 0), vuelo a v_z constante imposible de
    # imponer desde fuera, así que se comprueba el estado publicado y el hueco
    lines = [NAV_CFG, "INIT"]
    for k in range(-200, 1):
        t = k * 0.005
        lines.append(f"IMU {t!r} 0 0 -9.81 0 0 0 0")
        if k % 2 == 0 and k < 0:
            lines.append(f"BARO {t + 0.001!r} 100.0")
    lines += ["IMU 0.005 0 0 -9.81 0 0 0 0", "BARO 0.006 100.2", "IMU 0.003 0 0 -9.81 0 0 0 0",
              "IMU nan 0 0 -9.81 0 0 0 0", "IMU 0.5 0 0 -9.81 0 0 0 0", "BARO 0.501 100.0"]
    r = run("\n".join(lines) + "\n")
    out = [l.split() for l in r.stdout.splitlines()]
    check("ejecución sin errores y una salida por muestra", r.returncode == 0 and len(out) == len(lines) - 2,
          f"código {r.returncode}, {len(out)} salidas, stderr {r.stderr[:80]}")
    if len(out) >= 6:
        v_al = out[-7]   # muestra de alineación (t = 0)
        check("alineado en t = 0: VÁLIDO", v_al[2] == "0" and v_al[3] == "2", " ".join(v_al[:5]))
        check("baro tras alinear aplicado (ret 1)", out[-5][1] == "1", " ".join(out[-5][:5]))
        check("0.003 atrasada (−105)", out[-4][1] == "-105", " ".join(out[-4][:5]))
        check("t = nan (−103)", out[-3][1] == "-103", " ".join(out[-3][:5]))
        check("hueco de 0.495 s (−108), NO VÁLIDO, t_state 0.005",
              out[-2][1] == "-108" and out[-2][3] == "0" and float(out[-2][2]) == 0.005, " ".join(out[-2][:5]))
        check("baro posterior no aplicado (−107)", out[-1][1] == "-107", " ".join(out[-1][:5]))
        check("contadores en stderr", "huecos 1" in r.stderr and "atrasadas 1" in r.stderr, r.stderr.strip()[:120])
        check("salida con alt_ref_seq y f_degraded (33 campos)", all(len(o) == 33 for o in out), str(len(out[0])))

    # órdenes de reproducción exacta (códigos crudos y µs)
    imuc = ["IMUC imu.acc_lsb_per_g 2048", "IMUC imu.gyro_lsb_per_dps 16.4", "IMUC imu.g_si 9.80665",
            "IMUC imu.axis_map_x 0", "IMUC imu.axis_map_y 1", "IMUC imu.axis_map_z 2",
            "IMUC imu.axis_sign_x 1", "IMUC imu.axis_sign_y 1", "IMUC imu.axis_sign_z 1", "IMUC imu.sat_margin_lsb 0"]
    lines = [NAV_CFG] + imuc + ["INIT"]
    for k in range(0, 1001):
        lines.append(f"IMURAW {k * 5000} 0 0 -2048 0 0 0")
        if k % 4 == 0:
            lines.append(f"BAROUS {k * 5000 + 100} 100.0")
    lines += ["TICKUS 5020000", "IMUNAN 5010000", "IMURAW 5015000 32767 0 -2048 0 0 0"]
    r = run("\n".join(lines) + "\n")
    out = [l.split() for l in r.stdout.splitlines()]
    check("IMURAW/BAROUS/TICKUS/IMUNAN: una salida por orden", r.returncode == 0 and
          len(out) == len(lines) - 2 - len(imuc), f"código {r.returncode} {r.stderr[:100]}")
    if r.returncode == 0 and len(out) >= 3:
        check("IMUNAN: valor no válido (−106)", out[-2][1] == "-106", " ".join(out[-2][:5]))
        check("IMURAW saturado (+32767): no válido", out[-1][3] == "0", " ".join(out[-1][:5]))
        check("t = t_us·1e-6", float(out[-4][2]) == 5000000 * 1e-6, out[-4][2])
    for bad, why in [("IMUC imu.axis_map_x 3", "mapa fuera de rango"), ("IMUC imu.axis_sign_x 0", "signo 0"),
                     ("IMUC imu.foo 1", "campo desconocido"), ("IMURAW 1 2 3", "faltan campos"),
                     ("IMURAW 1.5 0 0 0 0 0 0", "t_us no entero"), ("IMURAW 1 40000 0 0 0 0 0", "código > int16")]:
        pre = [NAV_CFG] + imuc + ["INIT"] if not bad.startswith("IMUC") else [NAV_CFG]
        r = run("\n".join(pre + [bad]) + "\n")
        check(f"rechaza: {why}", r.returncode == 2, f"código {r.returncode}")
    r = run("\n".join([NAV_CFG, "INIT", "IMURAW 1 0 0 0 0 0 0"]) + "\n")
    check("IMURAW sin IMUC: código 3", r.returncode == 3, f"código {r.returncode}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("exe")
    ap.add_argument("--ref-dir", default=None)
    ap.add_argument("--nav-replay", default=None)
    args = ap.parse_args()
    parser_unit_tests()
    replay_tests(args.exe)
    if args.nav_replay:
        nav_replay_tests(args.nav_replay)
    end_to_end_tests(args.exe, args.ref_dir)
    n_fail = results.count(False)
    print(f"herramientas: {len(results) - n_fail}/{len(results)} OK")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
