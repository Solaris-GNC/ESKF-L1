#!/usr/bin/env python3
"""compare_runs.py — comparación NUMÉRICA placa–PC con tolerancias (no igualdad binaria).

COMPLETA (por defecto; la única que puede dar equivalencia):

  compare_runs.py --placa salida_placa.bin --pc-mensajes salida_pc.bin --out-id 0x10 [--json informe.json]

  --placa        mensajes del KPID de salida registrados en la placa (bin/hex/xxd).
  --pc-mensajes  mensajes de `svc_replay --reproducir traza_placa.bin --mensajes salida_pc.bin
                 entradas.bin`: la MISMA secuencia de llamadas que hizo la placa (traza del pegamento).

  Identidad determinista: (sesión, id) de cada snapshot, diagnóstico y compacta. Falla si:
    * alguna captura está incompleta: CRC malo, snapshot incompleto o mezclado, mensaje inválido,
      salto o repetición en la secuencia SPP de la salida, snapshot sin cerrar al final;
    * queda alguna salida de la placa SIN pareja en el PC, o del PC sin pareja en la placa;
    * difiere algún campo exacto (marcas t_state/t_eval/t_proc, banderas, máscara, motivos,
      validez, fase, capacidades, fuente, readiness, config_id, alt_ref_seq; diagnóstico entero);
    * algún valor numérico supera su tolerancia.

PARCIAL (sólo si se pide con --parcial; nunca es una equivalencia):

  compare_runs.py --parcial --placa salida_placa.bin --pc salidas_pc.txt --out-id 0x10

  Sin traza de la placa no hay identidad por (sesión, id). Cada snapshot se empareja con las
  líneas OUT de `svc_replay --salidas` con el MISMO t_state, alt_ref_seq y motivos del núcleo (sin
  STALE). Si con ese t_state hay varios estados distintos (p. ej. un baro aplicado sin IMU nueva),
  la identidad es AMBIGUA: ese snapshot NO se compara (no se elige el de menor error) y se cuenta.
  Código 3 si todo lo comparado está dentro de tolerancia: «parcial», no «equivalente».

Valores no finitos (se comprueban COMPONENTE A COMPONENTE antes de reducir; nunca se convierten en 0):
  * finito frente a NaN/inf, o NaN frente a inf: fallo, con el campo y el índice;
  * infinito: nunca es un valor del esquema; fallo aunque coincida en ambas capturas;
  * dos NaN: sólo en un campo legítimamente NO disponible: fragmento ausente de la máscara, estado
    inexistente (t_state = INT64_MIN: antes de alinear o sin navegación) o alt_ref sin la capacidad
    BARO_REF (la que describe la disponibilidad ACTUAL de la referencia: p. ej. alineado y válido
    inercialmente con NO_ALT_REF, o sin BMP); compacta: `eskf_age_ms`/`bb_age_ms` = 0xFFFF, o altura
    del respaldo sin referencia (ni válido ni congelado). Ni la validez inercial del núcleo ni el
    contador de revisiones alt_ref_seq dicen que haya referencia ahora;
  * cada captura por separado: lo que una salida promete debe ser finito (con estado publicado: p,
    v, q, σ y sesgos; BARO_REF: alt_ref; DEGRADADO/VÁLIDO o ESKF_NAV: estado; compacta con estado
    ESKF o respaldo válido/congelado: sus alturas y velocidades); σ >= 0; cuaternión finito y de
    norma 1 ± 1e-3 (diagnóstico explícito si no).

Tolerancias (justificación en docs/VALIDACION.md), por eje:
  posición  |Δ| <= 0.01 m + 5 % de σ_p publicada   (la horizontal no es observable: σ grande)
  velocidad |Δ| <= 0.01 m/s + 5 % de σ_v
  actitud   ángulo entre cuaterniones (q ≡ −q) <= 1e-3 rad + 5 % de la mayor σ_θ
  σ         relativa <= 5 %
  sesgos    acel. 0.01 m/s² (~8 LSB), giro 1e-4 rad/s (< 1 LSB de 1/32.8 °/s)
  alt_ref   0.01 m; compacta: alturas 0.01 m, velocidades 0.01 m/s
Códigos: 0 equivalencia completa; 1 fallo (captura incompleta, salidas sin pareja o fuera de
tolerancia); 2 nada comparable o argumentos incoherentes; 3 comparación PARCIAL superada.
"""
import argparse
import json
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_out_decode as nod  # noqa: E402
import spp_wire  # noqa: E402

STALE = 0x0010
SNAP_EXACT = ("t_state_us", "t_eval_us", "t_proc_us", "nav_reasons", "svc_reasons", "readiness", "why_code", "config_id",
              "nav_validity", "svc_validity", "phase", "at_rest", "heading_known", "profile", "time_source", "caps", "source",
              "alt_ref_seq", "mask", "banderas")
OPS_EXACT = ("t_ms", "eskf_age_ms", "bb_age_ms", "nav_reasons", "svc_reasons", "readiness", "config_id", "boot_id",
             "nav_validity", "svc_validity", "caps", "source", "phase", "profile", "bb_valid", "bb_ref_frozen", "banderas")
OPS_NUM = (("eskf_h_m", 0.01), ("eskf_vz_mps", 0.01), ("bb_h_m", 0.01), ("bb_vz_mps", 0.01))
# fragmento de la máscara que lleva cada campo del snapshot (esquema v4)
FRAG = {"p": 2, "v": 2, "alt_ref": 2, "q": 4, "sigma_th": 4, "sigma_p": 8, "sigma_v": 8, "b_a": 16, "b_g": 16}
STATE = ("p", "v", "q", "sigma_th", "sigma_p", "sigma_v", "b_a", "b_g")
SIGMAS = ("sigma_p", "sigma_v", "sigma_th")
CAP_BARO_REF, CAP_ESKF_NAV = 0x02, 0x04
Q_NORM_TOL = 1e-3  # comprobación de cordura (float32 normalizado: ~1e-7), no una tolerancia de comparación
AGE_NONE = 0xFFFF


def fin(x):
    return isinstance(x, float) and math.isfinite(x) or isinstance(x, int)


def vals(v):
    return list(v) if isinstance(v, (tuple, list)) else [v]


def snap_available(s, k):
    """¿Promete el snapshot el campo k? (fragmento presente y estado/referencia disponibles AHORA)"""
    if not (s.get("mask", 0) & FRAG[k]):
        return False
    if k == "alt_ref":  # la capacidad BARO_REF describe la referencia vigente
        return bool(s.get("caps", 0) & CAP_BARO_REF)
    return s.get("t_state_us") is not None


def snap_required(s, k):
    """Disponible, o exigido por lo que declara la salida (validez, capacidades, fuente)."""
    if not (s.get("mask", 0) & FRAG[k]):
        return False
    if snap_available(s, k):
        return True
    if k == "alt_ref":
        return False  # sin BARO_REF no se promete (la validez inercial del núcleo no basta)
    return ((s.get("svc_validity") or 0) >= 1 or (s.get("nav_validity") or 0) >= 1 or
            bool(s.get("caps", 0) & CAP_ESKF_NAV) or s.get("source") == 1)


def quat_problem(q):
    if not all(fin(x) for x in q):
        return f"cuaternión no finito {tuple(q)}"
    n = math.sqrt(sum(x * x for x in q))
    if not abs(n - 1.0) <= Q_NORM_TOL:
        return f"cuaternión de norma {n:.6g} (se espera 1 ± {Q_NORM_TOL:g})"
    return None


def check_snapshot(s):
    """Coherencia de UNA salida: lo que promete es finito; nada infinito; σ >= 0; q unitario."""
    prob = []
    for k in FRAG:
        if not (s.get("mask", 0) & FRAG[k]):
            continue
        xs = vals(s[k])
        for i, x in enumerate(xs):
            if isinstance(x, float) and math.isinf(x):
                prob.append(f"{k}[{i}] = {x} (infinito: nunca es un valor del esquema)")
            elif snap_required(s, k) and not fin(x):
                why = "estado publicado" if snap_available(s, k) else "declarado válido/utilizable"
                prob.append(f"{k}[{i}] = {x} con {why}")
            elif k in SIGMAS and fin(x) and x < 0:
                prob.append(f"{k}[{i}] = {x} < 0")
        if k == "q" and snap_required(s, k):
            qp = quat_problem(xs)
            if qp:
                prob.append(qp)
    return prob


def ops_required(o, f):
    if f.startswith("eskf_"):
        return o["eskf_age_ms"] != AGE_NONE
    if f == "bb_vz_mps":
        return o["bb_age_ms"] != AGE_NONE or bool(o["bb_valid"])
    return bool(o["bb_valid"]) or bool(o["bb_ref_frozen"])


def check_ops(o):
    prob = []
    for f, _ in OPS_NUM:
        x = o[f]
        if math.isinf(x):
            prob.append(f"{f} = {x} (infinito)")
        elif ops_required(o, f) and not math.isfinite(x):
            prob.append(f"{f} = {x} con el dato declarado disponible")
    return prob


def load_outputs(path, out_id):
    """Decodifica y comprueba la integridad de una captura de la salida. Devuelve (salidas, problemas)."""
    data, _, _ = spp_wire.load(path)
    pkts, issues, _ = spp_wire.split_packets(data)
    r = nod.Reasm()
    out = {"snap": {}, "diag": {}, "ops": {}}
    prob = []
    last = None
    for p in pkts:
        if p["id"] != out_id:
            continue
        if not p["crc_ok"]:
            prob.append(f"paquete con CRC incorrecto (secuencia {p['seq']})")
            continue
        if last is not None and ((p["seq"] - last) & 0xFFFF) != 1:
            prob.append(f"secuencia SPP de salida {last} → {p['seq']}: paquetes perdidos o repetidos")
        last = p["seq"]
        k = r.push(p["payload"])
        if k:
            key = (k[1]["session"], k[1].get("snap_id", k[1].get("diag_id", k[1].get("ops_id"))))
            if key in out[k[0]]:
                prob.append(f"{k[0]} repetido {key}")
            out[k[0]][key] = k[1]
            bad = check_snapshot(k[1]) if k[0] == "snap" else check_ops(k[1]) if k[0] == "ops" else []
            for b in bad:
                prob.append(f"{k[0]} {key}: valor inválido: {b}")
    if r.cur:
        r.incomplete += 1
        prob.append("snapshot sin cerrar al final de la captura")
    for name, n in (("incompletos", r.incomplete), ("CRC del snapshot", r.crc_bad), ("mezclados", r.mixed),
                    ("inválidos", r.invalid)):
        if n:
            prob.append(f"{n} mensajes {name}")
    for i in issues:
        if i.get("tipo") not in ("relleno",):
            prob.append(f"lectura: {i.get('tipo')}")
    return out, prob


def qangle(a, b):
    """Ángulo entre cuaterniones (q ≡ −q). Sólo con cuaterniones ya comprobados (finitos, norma ~1)."""
    d = abs(sum(x * y for x, y in zip(a, b)))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    return 2.0 * math.acos(min(1.0, d / (na * nb)))


def same(x, y):
    if isinstance(x, float) and isinstance(y, float) and math.isnan(x) and math.isnan(y):
        return True
    return x == y


def pair_status(x, y, avail):
    """'ok' (ambos finitos), 'na' (dos NaN en un campo no disponible) o el texto del fallo."""
    fx, fy = fin(x), fin(y)
    if fx and fy:
        return "ok"
    if isinstance(x, float) and isinstance(y, float) and math.isnan(x) and math.isnan(y) and not avail:
        return "na"
    if fx != fy:
        return "finito frente a no finito"
    if isinstance(x, float) and isinstance(y, float) and math.isnan(x) and math.isnan(y):
        return "NaN en ambas con el campo disponible"
    return "no finito en ambas (infinito)"


def snap_diff(s, c, a):
    """Componente a componente: devuelve (exceso sobre la tolerancia, diferencias brutas, valores inválidos).

    s es la salida de la placa y c la del PC; la disponibilidad se toma de lo que declaran (si una dice
    disponible y la otra no, ya difieren sus campos exactos). Nada no finito se convierte en 0."""
    ks = a.tol_sigma
    over, raw, bad = {}, {}, []

    def avail(k):
        return snap_available(s, k) or ("mask" in c and snap_available(c, k))

    def note(group, excess, rawd):
        raw[group] = max(raw.get(group, 0.0), rawd)
        if excess > 0:
            over[group] = max(over.get(group, 0.0), excess)

    def sig_of(k, i):
        sg = vals(c[k])[i]
        return sg if fin(sg) and sg >= 0 else 0.0  # sin σ utilizable no se amplía la tolerancia

    for k, group, tol in (("p", "p", a.tol_p), ("v", "v", a.tol_v), ("b_a", "b_a", a.tol_ba), ("b_g", "b_g", a.tol_bg),
                          ("alt_ref", "alt_ref", a.tol_p)):
        for i, (x, y) in enumerate(zip(vals(s[k]), vals(c[k]))):
            st = pair_status(x, y, avail(k))
            if st == "na":
                continue
            if st != "ok":
                bad.append(f"{k}[{i}]: placa {x!r}, PC {y!r} ({st})")
                continue
            extra = ks * sig_of("sigma_" + k, i) if k in ("p", "v") else 0.0
            note(group, abs(x - y) - extra - tol, abs(x - y))
    for k in SIGMAS:
        for i, (x, y) in enumerate(zip(vals(s[k]), vals(c[k]))):
            st = pair_status(x, y, avail(k))
            if st == "na":
                continue
            if st != "ok":
                bad.append(f"{k}[{i}]: placa {x!r}, PC {y!r} ({st})")
                continue
            if x < 0 or y < 0:
                bad.append(f"{k}[{i}]: σ negativa (placa {x!r}, PC {y!r})")
                continue
            r = abs(x - y) / (abs(y) + 1e-4)
            note("sig", r - a.tol_sig_rel, r)
    qs, qc = vals(s["q"]), vals(c["q"])
    if avail("q") or not all(isinstance(x, float) and math.isnan(x) for x in qs + qc):
        ps, pc_ = quat_problem(qs), quat_problem(qc)
        if ps or pc_:
            bad.append("actitud: " + "; ".join(x for x in (ps and f"placa {ps}", pc_ and f"PC {pc_}") if x))
        else:
            sth = [x for x in vals(c["sigma_th"]) if fin(x) and x >= 0]
            ang = qangle(qs, qc)
            note("att", ang - ks * (max(sth) if sth else 0.0) - a.tol_att, ang)
    return over, raw, bad


def complete(a):
    B, pb = load_outputs(a.placa, a.out_id)
    P, pp = load_outputs(a.pc_mensajes, a.out_id)
    rep = {"modo": "completa", "problemas_placa": pb, "problemas_pc": pp, "sin_pareja_placa": {}, "sin_pareja_pc": {},
           "fuera_de_tolerancia": [], "campos_exactos_distintos": [], "valores_invalidos": [], "max_dif_bruta": {}}
    n_cmp = 0
    worst = rep["max_dif_bruta"]
    for kind in ("snap", "diag", "ops"):
        kb, kp = set(B[kind]), set(P[kind])
        rep["sin_pareja_placa"][kind] = sorted(kb - kp)[:10] + ([f"... {len(kb - kp)} en total"] if len(kb - kp) > 10 else [])
        rep["sin_pareja_pc"][kind] = sorted(kp - kb)[:10] + ([f"... {len(kp - kb)} en total"] if len(kp - kb) > 10 else [])
        for key in sorted(kb & kp):
            s, c = B[kind][key], P[kind][key]
            n_cmp += 1
            if kind == "diag":
                bad = [k for k in s if k not in ("banderas",) and not same(s[k], c[k])] + \
                    (["banderas"] if s["banderas"] != c["banderas"] else [])
                if bad:
                    rep["campos_exactos_distintos"].append({"tipo": kind, "id": key, "campos": bad})
                continue
            if kind == "ops":
                bad = [k for k in OPS_EXACT if not same(s[k], c[k])]
                if bad:
                    rep["campos_exactos_distintos"].append({"tipo": kind, "id": key, "campos": bad})
                for f, tol in OPS_NUM:
                    x, y = s[f], c[f]
                    st = pair_status(x, y, ops_required(s, f) or ops_required(c, f))
                    if st == "na":
                        continue
                    if st != "ok":
                        rep["valores_invalidos"].append({"tipo": kind, "id": key,
                                                         "campos": [f"{f}: placa {x!r}, PC {y!r} ({st})"]})
                        continue
                    dd = abs(x - y)
                    worst["ops_" + f] = max(worst.get("ops_" + f, 0.0), dd)
                    if not dd <= tol:
                        rep["fuera_de_tolerancia"].append({"tipo": kind, "id": key, "campo": f, "dif": dd})
                continue
            bad = [k for k in SNAP_EXACT if not same(s.get(k), c.get(k))]
            if bad:
                rep["campos_exactos_distintos"].append({"tipo": kind, "id": key, "campos": bad})
            over, raw, inval = snap_diff(s, c, a)
            for k, v in raw.items():
                worst[k] = max(worst.get(k, 0.0), v)
            if over:
                rep["fuera_de_tolerancia"].append({"tipo": kind, "id": key, "t_state_us": s["t_state_us"], "exceso": over})
            if inval:
                rep["valores_invalidos"].append({"tipo": kind, "id": key, "campos": inval})
    unpaired = sum(len(v) for v in rep["sin_pareja_placa"].values()) + sum(len(v) for v in rep["sin_pareja_pc"].values())
    ok = (not pb and not pp and unpaired == 0 and not rep["fuera_de_tolerancia"] and not rep["campos_exactos_distintos"]
          and not rep["valores_invalidos"])
    rep["comparadas"] = n_cmp
    rep["salidas_placa"] = {k: len(v) for k, v in B.items()}
    rep["salidas_pc"] = {k: len(v) for k, v in P.items()}
    rep["resultado"] = "EQUIVALENTE (dentro de tolerancia)" if ok and n_cmp else "NO EQUIVALENTE"
    w = lambda k: worst.get(k, float("nan"))  # noqa: E731
    print(f"compare_runs [COMPLETA]: {n_cmp} salidas comparadas por (sesión, id) — placa {rep['salidas_placa']}, "
          f"PC {rep['salidas_pc']}")
    print(f"  diferencias máximas brutas: posición {w('p'):.3g} m, velocidad {w('v'):.3g} m/s, actitud {w('att'):.3g} rad, "
          f"σ relativa {w('sig'):.3g}, sesgos {w('b_a'):.3g} m/s² / {w('b_g'):.3g} rad/s")
    for name, lst in (("captura de la placa", pb), ("captura del PC", pp)):
        for x in lst[:5]:
            print(f"  PROBLEMA en la {name}: {x}")
    for kind in ("snap", "diag", "ops"):
        if rep["sin_pareja_placa"][kind]:
            print(f"  {kind}: salidas de la placa SIN pareja en el PC: {rep['sin_pareja_placa'][kind]}")
        if rep["sin_pareja_pc"][kind]:
            print(f"  {kind}: salidas del PC SIN pareja en la placa: {rep['sin_pareja_pc'][kind]}")
    for x in rep["campos_exactos_distintos"][:5]:
        print(f"  campos exactos distintos: {x}")
    if rep["valores_invalidos"]:
        print(f"  VALORES NO FINITOS O INVÁLIDOS en {len(rep['valores_invalidos'])} salidas emparejadas:")
    for x in rep["valores_invalidos"][:5]:
        print(f"    {x['tipo']} {x['id']}: {'; '.join(x['campos'])}")
    for x in rep["fuera_de_tolerancia"][:5]:
        print(f"  fuera de tolerancia: {x}")
    print(f"  RESULTADO: {rep['resultado']}")
    return rep, (0 if ok and n_cmp else (2 if n_cmp == 0 and not pb and not pp else 1))


def read_pc_lines(path):
    by_t = defaultdict(list)
    for line in open(path):
        f = line.split()
        if not f or f[0] != "OUT":
            continue
        v = [float(int(x, 16)) if x.startswith("0x") else float(x) for x in f[1:]]
        if not math.isfinite(v[1]):
            continue
        rec = {"t_state_us": int(round(v[1] * 1e6)), "nav_validity": int(v[2]), "nav_reasons": int(f[4], 16),
               "p": v[4:7], "v": v[7:10], "q": v[10:14], "sigma_p": v[14:17], "sigma_v": v[17:20], "sigma_th": v[20:23],
               "alt_ref": v[23], "b_a": v[24:27], "b_g": v[27:30], "alt_ref_seq": int(v[30])}
        lst = by_t[rec["t_state_us"]]
        if not lst or any(not same(rec[k], lst[-1][k]) for k in ("p", "v", "q", "sigma_p", "alt_ref_seq", "nav_reasons")):
            lst.append(rec)  # los nav_tick repiten el estado: no son candidatos distintos
    return by_t


def partial(a):
    B, pb = load_outputs(a.placa, a.out_id)
    pc = read_pc_lines(a.pc)
    rep = {"modo": "PARCIAL", "problemas_placa": pb, "comparados": 0, "ambiguos": 0, "sin_pareja": 0, "fuera_de_tolerancia": [],
           "valores_invalidos": [], "max_dif_bruta": {}}
    for key, s in sorted(B["snap"].items()):
        if s.get("t_state_us") is None or (s["mask"] & 0x1F) != 0x1F:
            rep["sin_pareja"] += 1
            continue
        cands = [c for c in pc.get(s["t_state_us"], []) if c["alt_ref_seq"] == s["alt_ref_seq"] and
                 (c["nav_reasons"] | STALE) == (s["nav_reasons"] | STALE)]
        if not cands:
            rep["sin_pareja"] += 1
            continue
        if len(cands) > 1:
            rep["ambiguos"] += 1  # identidad no determinable sin la traza: no se compara
            continue
        rep["comparados"] += 1
        over, raw, inval = snap_diff(s, cands[0], a)
        for k, v in raw.items():
            rep["max_dif_bruta"][k] = max(rep["max_dif_bruta"].get(k, 0.0), v)
        if over:
            rep["fuera_de_tolerancia"].append({"id": key, "t_state_us": s["t_state_us"], "exceso": over})
        if inval:
            rep["valores_invalidos"].append({"id": key, "campos": inval})
    tot = len(B["snap"])
    print(f"compare_runs [PARCIAL — NO es una equivalencia]: {rep['comparados']} de {tot} snapshots comparados; "
          f"{rep['ambiguos']} con identidad ambigua (no comparados), {rep['sin_pareja']} sin pareja; diagnósticos y "
          f"compactas NO comparados; fuera de tolerancia {len(rep['fuera_de_tolerancia'])}")
    for x in pb[:5]:
        print(f"  PROBLEMA en la captura de la placa: {x}")
    for x in rep["valores_invalidos"][:5]:
        print(f"  VALOR NO FINITO O INVÁLIDO: snap {x['id']}: {'; '.join(x['campos'])}")
    if rep["fuera_de_tolerancia"] or pb or rep["valores_invalidos"]:
        return rep, 1
    return rep, (3 if rep["comparados"] else 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--placa", required=True)
    ap.add_argument("--out-id", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--pc-mensajes")
    ap.add_argument("--pc")
    ap.add_argument("--parcial", action="store_true")
    ap.add_argument("--json")
    ap.add_argument("--tol-p", type=float, default=0.01)
    ap.add_argument("--tol-v", type=float, default=0.01)
    ap.add_argument("--tol-att", type=float, default=1e-3)
    ap.add_argument("--tol-sig-rel", type=float, default=0.05)
    ap.add_argument("--tol-ba", type=float, default=0.01)
    ap.add_argument("--tol-bg", type=float, default=1e-4)
    ap.add_argument("--tol-sigma", type=float, default=0.05, help="fracción de la σ publicada que se admite además")
    a = ap.parse_args()
    if a.parcial:
        if not a.pc or a.pc_mensajes:
            print("compare_runs: --parcial usa --pc (líneas OUT de svc_replay --salidas)")
            return 2
        rep, code = partial(a)
    else:
        if not a.pc_mensajes or a.pc:
            print("compare_runs: la comparación completa necesita --pc-mensajes (svc_replay --reproducir la traza de la "
                  "placa); con líneas OUT sólo cabe --parcial")
            return 2
        rep, code = complete(a)
    if a.json:
        json.dump(rep, open(a.json, "w"), indent=1, default=str, ensure_ascii=False)
    return code


if __name__ == "__main__":
    sys.exit(main())
