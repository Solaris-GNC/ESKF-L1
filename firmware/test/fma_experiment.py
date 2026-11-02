#!/usr/bin/env python3
"""fma_experiment.py — de dónde salen las tolerancias de compare_board_pc.py.

El mismo escenario sintético se pasa por navigation.c con contracción FMA (gen_vnlog_fma:
a·b + c con UN redondeo, como madd.s en el FPU del ESP32-S3) y sin ella (nav_replay_float en
x86-64 sin -mfma). Las entradas son idénticas bit a bit (códigos crudos, marcas, reloj, orden);
sólo cambia el redondeo de las operaciones. Se mide la diferencia máxima de cada grupo, en
absoluto y relativa a la σ que publica el filtro, y se comprueba que las tolerancias TOL de
compare_board_pc.py son >= 10× lo observado.

  fma_experiment.py --gen-fma build/gen_vnlog_fma --nav-replay-float ../c/build/nav_replay_float
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
import compare_board_pc as cb  # noqa: E402
import log_to_replay  # noqa: E402
import vnlog  # noqa: E402

SCEN = [
    ("vuelo 12 s", ["--seconds", 12, "--flight", "--seed", 1]),
    ("vuelo + subir/bajar", ["--seconds", 12, "--flight", "--seed", 2, "--updown", "1.5,4,1,6.5"]),
    ("vuelo, otra semilla", ["--seconds", 12, "--flight", "--seed", 3]),
    ("30 s + giro + vuelo", ["--seconds", 30, "--flight", "--seed", 4, "--updown", "1.5,4,2,10", "--spin", "0.5,22,3"]),
    ("30 s, semilla 5", ["--seconds", 30, "--flight", "--seed", 5, "--updown", "1.5,4,2,10", "--spin", "0.5,22,3"]),
    ("30 s, semilla 6", ["--seconds", 30, "--flight", "--seed", 6, "--updown", "1.5,4,2,10", "--spin", "0.5,22,3"]),
    ("sesgos grandes", ["--seconds", 20, "--flight", "--seed", 7, "--acc-bias", "0.2,-0.1,0.15",
                        "--gyro-bias", "0.01,-0.008,0.005"]),
    ("eje −x arriba", ["--seconds", 20, "--flight", "--seed", 8, "--up", "-x"]),
    ("modelo acel (una irq)", ["--seconds", 20, "--flight", "--seed", 9, "--imu-model", "acc"]),
]


def measure(path, nav_replay):
    log = vnlog.Log(path)
    lines, mapa, rep = log_to_replay.convert(log)
    r = subprocess.run([nav_replay], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    out = cb.parse_out(r.stdout)
    b = cb.board_navout(log["NAVOUT"][mapa["navout_idx"]])
    pc = cb.pc_rows(out, mapa["navout_cmd"])
    m = {}
    for k, s in (("p", "sigma_p"), ("v", "sigma_v")):
        d = np.abs(np.asarray(b[k], float) - pc[k])
        sg = np.asarray(b[s], float)
        ok = np.isfinite(d) & np.isfinite(sg) & (sg > 0)
        m[k] = float(d[ok].max()) if ok.any() else 0.0
        m[k + "/σ"] = float((d[ok] / sg[ok]).max()) if ok.any() else 0.0
    qb, qp = b["q"], pc["q"]
    dot = np.clip(np.abs(np.sum(qb * qp, 1)) / (np.linalg.norm(qb, axis=1) * np.linalg.norm(qp, axis=1)), 0, 1)
    ang = np.where(np.isnan(dot), 0.0, 2 * np.arccos(dot))
    sth = np.linalg.norm(np.asarray(b["sigma_th"], float), axis=1)
    ok = np.isfinite(sth) & (sth > 0)
    m["att"] = float(ang.max())
    m["att/σ"] = float((ang[ok] / sth[ok]).max()) if ok.any() else 0.0
    for k in ("sigma_p", "sigma_v", "sigma_th"):
        bb = np.asarray(b[k], float)
        d = np.abs(bb - pc[k])
        ok = np.isfinite(d) & (np.abs(bb) > 0)
        m[k + " rel"] = float((d[ok] / np.abs(bb[ok])).max()) if ok.any() else 0.0
    for k in ("b_a", "b_g", "alt_ref"):
        d = np.abs(np.asarray(b[k], float) - pc[k])
        m[k] = float(np.nanmax(d)) if np.isfinite(d).any() else 0.0
    ev = np.nonzero(mapa["kind"] > 0)[0]
    nv = log["NAVEV"][mapa["idx"][ev]]
    m["eventos distintos"] = int(((out[ev, 0] != nv["ret"]) | (out[ev, 2] != nv["validity"]) |
                                  (out[ev, 3] != nv["reasons"])).sum())
    m["eventos"] = int(len(ev))
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gen-fma", required=True)
    ap.add_argument("--nav-replay-float", required=True)
    a = ap.parse_args()
    rows = []
    with tempfile.TemporaryDirectory() as td:
        for name, args in SCEN:
            p = os.path.join(td, "x.bin")
            subprocess.run([a.gen_fma, "--profile", "bench", *map(str, args), p], check=True, capture_output=True)
            rows.append((name, measure(p, a.nav_replay_float)))
    keys = ["p", "p/σ", "v", "v/σ", "att", "att/σ", "sigma_p rel", "sigma_v rel", "sigma_th rel", "b_a", "b_g",
            "alt_ref", "eventos distintos"]
    print("| escenario | " + " | ".join(keys) + " |")
    print("|---|" + "---|" * len(keys))
    for name, m in rows:
        print(f"| {name} | " + " | ".join(f"{m[k]:.2g}" if isinstance(m[k], float) else str(m[k]) for k in keys) + " |")
    mx = {k: max(m[k] for _, m in rows) for k in keys}
    print("| **máximo** | " + " | ".join(f"{mx[k]:.2g}" if isinstance(mx[k], float) else str(mx[k]) for k in keys)
          + " |")
    # comprobación de TOL (10× lo observado)
    T = cb.TOL
    need = {"p/σ": T["p"][2], "v/σ": T["v"][2], "att/σ": T["att_rad"][2], "sigma_p rel": T["sigma_p"][1],
            "sigma_v rel": T["sigma_v"][1], "sigma_th rel": T["sigma_th"][1], "b_a": T["b_a"][0], "b_g": T["b_g"][0]}
    ok = True
    for k, tol in need.items():
        good = tol >= 10 * mx[k]
        ok &= good
        print(f"  TOL {k:13s} {tol:.2g} {'>=' if good else '<'} 10 × {mx[k]:.2g}")
    print(f"  eventos (ret/validez/motivos) distintos en total: {mx['eventos distintos']}")
    print("RESULTADO: " + ("tolerancias coherentes con el experimento" if ok else "REVISAR TOL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
