#!/usr/bin/env python3
"""compare_board_pc.py — compara la navegación de la placa con la misma entrada pasada por
navigation.c en el PC (nav_replay).

  compare_board_pc.py registro.bin --nav-replay c/build/nav_replay_float [--exact] [--json res.json]

1. log_to_replay (modo exacto: mismas entradas, mismo orden, mismo reloj de nav_tick).
2. nav_replay → una salida por orden.
3. Por cada NAVEV de la placa: ret, validez y motivos deben ser IGUALES.
   Por cada NAVOUT: t_state, validez, motivos, alt_ref_seq y f_degraded IGUALES; estado con
   tolerancia |a − b| <= atol + rtol·|b| (actitud: ángulo entre cuaterniones).

Tolerancias por defecto (TOL): ver firmware/COMPARACION.md. Salen del experimento
`make -C firmware fma`: el mismo escenario con y sin contracción FMA (a·b + c con un solo
redondeo) en el PC. Es una perturbación de redondeo en casi todas las operaciones, del orden
de las diferencias esperables placa/PC (madd.s del ESP32-S3, sqrtf/sinf/cosf de newlib frente
a glibc). Son 10× el máximo observado, relativas a la σ que publica el filtro cuando procede
(las diferencias de redondeo crecen con la incertidumbre en vuelo). HIPÓTESIS hasta comparar
con un registro real de la placa.
--exact exige igualdad bit a bit (sirve para comprobar la propia cadena con gen_vnlog).
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import log_to_replay  # noqa: E402
import vnlog  # noqa: E402

# grupo: (atol, rtol, ksig): |placa − PC| <= atol + rtol·|placa| + ksig·σ (σ de la placa para ese
# grupo). Experimento FMA (make -C firmware fma: 9 escenarios, COMPARACION.md). Máximos observados:
# |Δp|/σp 0.015, |Δv|/σv 0.016, ángulo/‖σθ‖ 0.0027, |Δσ|/σ 0.010, |Δb_a| 0.0052 m/s²,
# |Δb_g| 1.4e-5 rad/s, alt_ref 0, eventos distintos 0. Tolerancia = 10× lo observado (redondeado
# hacia arriba). Son una comprobación de cordura: el criterio fuerte es que ret/validez/motivos de
# CADA evento sean idénticos.
TOL = {
    "p": (1e-6, 0.0, 0.2), "v": (1e-6, 0.0, 0.2), "att_rad": (1e-7, 0.0, 0.03),
    "sigma_p": (1e-7, 0.1, 0.0), "sigma_v": (1e-7, 0.1, 0.0), "sigma_th": (1e-9, 0.1, 0.0),
    "b_a": (0.06, 0.0, 0.0), "b_g": (2e-4, 0.0, 0.0), "alt_ref": (1e-6, 1e-9, 0.0),
}
SIGMA_OF = {"p": "sigma_p", "v": "sigma_v"}
# columnas de OUT (tras la palabra OUT)
C_RET, C_TS, C_VAL, C_REAS = 0, 1, 2, 3
C_P, C_V, C_Q, C_SP, C_SV, C_STH, C_ALT, C_BA, C_BG, C_SEQ, C_DEG = 4, 7, 10, 14, 17, 20, 23, 24, 27, 30, 31


def parse_out(text):
    rows = [l.split()[1:] for l in text.splitlines() if l.startswith("OUT ")]
    if not rows:
        return np.zeros((0, 32))
    a = np.empty((len(rows), 32))
    for i, r in enumerate(rows):
        r[C_REAS] = str(int(r[C_REAS], 16))
        a[i] = [float(x) for x in r]
    return a


def board_navout(nv):
    q = np.stack([nv["qw"], nv["qx"], nv["qy"], nv["qz"]], 1).astype(np.float64)
    g = lambda *k: np.stack([nv[x] for x in k], 1).astype(np.float64)
    return {"t_state": nv["t_state"], "validity": nv["validity"], "reasons": nv["reasons"],
            "alt_ref_seq": nv["alt_ref_seq"], "f_degraded": nv["f_degraded"],
            "p": g("px", "py", "pz"), "v": g("vx", "vy", "vz"), "q": q, "sigma_p": g("spx", "spy", "spz"),
            "sigma_v": g("svx", "svy", "svz"), "sigma_th": g("sthx", "sthy", "sthz"),
            "b_a": g("bax", "bay", "baz"), "b_g": g("bgx", "bgy", "bgz"), "alt_ref": nv["alt_ref"]}


def pc_rows(out, rows):
    """Campos float de la placa: el texto de nav_replay (%.9g) vuelve al MISMO float al
    redondear a float32; t_state y alt_ref son double (%.17g)."""
    o = out[rows]
    f = lambda a: a.astype(np.float32).astype(np.float64)
    o = o.copy()
    for c0, n in ((C_P, 3), (C_V, 3), (C_Q, 4), (C_SP, 3), (C_SV, 3), (C_STH, 3), (C_BA, 3), (C_BG, 3)):
        o[:, c0:c0 + n] = f(o[:, c0:c0 + n])
    return {"t_state": o[:, C_TS], "validity": o[:, C_VAL], "reasons": o[:, C_REAS],
            "alt_ref_seq": o[:, C_SEQ], "f_degraded": o[:, C_DEG], "p": o[:, C_P:C_P + 3],
            "v": o[:, C_V:C_V + 3], "q": o[:, C_Q:C_Q + 4], "sigma_p": o[:, C_SP:C_SP + 3],
            "sigma_v": o[:, C_SV:C_SV + 3], "sigma_th": o[:, C_STH:C_STH + 3], "b_a": o[:, C_BA:C_BA + 3],
            "b_g": o[:, C_BG:C_BG + 3], "alt_ref": o[:, C_ALT]}


def nan_eq(a, b):
    return (a == b) | (np.isnan(a) & np.isnan(b))


def compare(log, nav_replay, exact=False, tol=None, workdir=None):
    tol = tol or TOL
    lines, mapa, rep = log_to_replay.convert(log)
    res = {"mode": rep["mode"], "missing": rep["missing"][:5], "n_cmds": int(len(mapa["kind"])), "ok": True,
           "errors": []}
    if rep["mode"] != "exacto":
        res["ok"] = False
        res["errors"].append("el registro no tiene NAVEV (perfil de adquisición): nada que comparar")
        return res
    with tempfile.TemporaryDirectory(dir=workdir) as td:
        inp = os.path.join(td, "replay.txt")
        with open(inp, "w") as f:
            f.write("\n".join(lines) + "\n")
        with open(inp) as fi:
            r = subprocess.run([nav_replay], stdin=fi, capture_output=True, text=True)
    if r.returncode != 0:
        res["ok"] = False
        res["errors"].append(f"nav_replay terminó con {r.returncode}: {r.stderr.strip()[:200]}")
        return res
    out = parse_out(r.stdout)
    if len(out) != len(mapa["kind"]):
        res["ok"] = False
        res["errors"].append(f"nav_replay dio {len(out)} salidas para {len(mapa['kind'])} órdenes")
        return res

    # 1) eventos: ret, validez, motivos
    ev_rows = np.nonzero(mapa["kind"] > 0)[0]
    nv = log["NAVEV"][mapa["idx"][ev_rows]]
    d_ret = out[ev_rows, C_RET] != nv["ret"]
    d_val = out[ev_rows, C_VAL] != nv["validity"]
    d_rea = out[ev_rows, C_REAS] != nv["reasons"]
    bad = d_ret | d_val | d_rea
    res["events"] = int(len(ev_rows))
    res["event_mismatch"] = int(bad.sum())
    if bad.any():
        i = int(np.argmax(bad))
        res["ok"] = False
        res["errors"].append(f"evento {i}: placa ret {nv['ret'][i]} val {nv['validity'][i]} mot 0x{nv['reasons'][i]:04x}"
                             f" / PC ret {int(out[ev_rows[i], C_RET])} val {int(out[ev_rows[i], C_VAL])} "
                             f"mot 0x{int(out[ev_rows[i], C_REAS]):04x}")

    # 2) salidas
    b = board_navout(log["NAVOUT"][mapa["navout_idx"]])
    rows = mapa["navout_cmd"]
    valid = rows >= 0
    b = {k: v[valid] for k, v in b.items()}
    pc = pc_rows(out, rows[valid])
    res["navout"] = int(valid.sum())
    for k in ("t_state", "validity", "reasons", "alt_ref_seq", "f_degraded"):
        eq = nan_eq(np.asarray(b[k], np.float64), pc[k])
        if not eq.all():
            i = int(np.argmin(eq))
            res["ok"] = False
            res["errors"].append(f"NAVOUT {i}: {k} placa {b[k][i]} PC {pc[k][i]}")
    res["max_err"] = {}
    for k in ("p", "v", "sigma_p", "sigma_v", "sigma_th", "b_a", "b_g", "alt_ref"):
        bb, pp = np.asarray(b[k], np.float64), pc[k]
        both_nan = np.isnan(bb) & np.isnan(pp)
        err = np.where(both_nan, 0.0, np.abs(bb - pp))
        atol, rtol, ksig = (0.0, 0.0, 0.0) if exact else tol[k]
        lim = atol + rtol * np.abs(np.where(np.isnan(bb), 0.0, bb))
        if ksig and k in SIGMA_OF:
            sg = np.asarray(b[SIGMA_OF[k]], np.float64)
            lim = lim + ksig * np.where(np.isfinite(sg), sg, 0.0)
        over = ~(err <= lim)
        res["max_err"][k] = float(np.nanmax(err)) if err.size else 0.0
        if over.any():
            res["ok"] = False
            i = np.unravel_index(int(np.argmax(np.where(over, err, -1))), err.shape)
            res["errors"].append(f"{k}: error {err[i]:.3g} > {lim[i]:.3g} en NAVOUT {i[0]} (placa {bb[i]}, PC {pp[i]})")
    qb, qp = b["q"], pc["q"]
    dot = np.clip(np.abs(np.sum(qb * qp, 1)) / (np.linalg.norm(qb, axis=1) * np.linalg.norm(qp, axis=1)), 0, 1)
    ang = np.where(np.isnan(dot), 0.0, 2 * np.arccos(dot))
    if exact:
        same = nan_eq(qb, qp).all(1)
        ang = np.where(same, 0.0, np.maximum(ang, 1e-30))
    res["max_err"]["att_rad"] = float(ang.max()) if ang.size else 0.0
    if exact:
        lim = np.zeros_like(ang)
    else:
        sth = np.linalg.norm(np.asarray(b["sigma_th"], np.float64), axis=1)
        lim = tol["att_rad"][0] + tol["att_rad"][2] * np.where(np.isfinite(sth), sth, 0.0)
    if (ang > lim).any():
        res["ok"] = False
        i = int(np.argmax(ang - lim))
        res["errors"].append(f"actitud: {ang[i]:.3g} rad > {lim[i]:.3g} en NAVOUT {i}")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--nav-replay", required=True)
    ap.add_argument("--exact", action="store_true")
    ap.add_argument("--json")
    a = ap.parse_args()
    log = vnlog.Log(a.log)
    if log.is_sim():
        print("AVISO: registro simulado en el PC (gen_vnlog), no de la placa")
    res = compare(log, a.nav_replay, exact=a.exact)
    print(f"comparación placa/PC ({'exacta' if a.exact else 'con tolerancias'}): {res.get('events', 0)} eventos, "
          f"{res.get('navout', 0)} salidas")
    for k, v in res.get("max_err", {}).items():
        print(f"  máx. |error| {k:9s} {v:.3g}")
    for e in res["errors"]:
        print("  FALLO: " + e)
    print("  RESULTADO: " + ("COINCIDE" if res["ok"] else "NO COINCIDE"))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1, default=str)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
