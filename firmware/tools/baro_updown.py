#!/usr/bin/env python3
"""baro_updown.py — signo y LATENCIA del barómetro respecto a las marcas de la IMU.

  baro_updown.py registro.bin [--reposo 8] [--max-lag 0.1] [--json res.json]

Procedimiento (PROCEDIMIENTOS.md): placa quieta >= --reposo s, después subir y bajar en vertical
1–2 m varias veces (periodo 3–6 s, sin girarla; mejor con guía o polea), y reposo al final.

Método: altura de la IMU = doble integral de (a·ĝ − |g|), con ĝ y |g| del reposo inicial. Para
cada retardo τ se ajusta  baro(t) = s·h_imu(t − τ) + c0 + c1·t + c2·t²  (los términos en t
absorben la deriva de la doble integral y la meteorología) y se elige el τ de menor residuo.
  * s ≈ +1: signo y escala correctos (s < 0: el baro sube cuando la placa baja → error grave).
  * τ: latencia efectiva del baro respecto a la marca de la IMU (incluye conversión, sondeo,
    lectura y el retardo de los filtros de la IMU). Es lo que significa baro_latency_s en
    navigation.c (t_medida = t_lectura − latencia), con ESTE firmware y ESTA configuración.
  * σ_τ: incertidumbre (linealización); repetir la prueba y usar la media.
Los tiempos de conversión de la hoja de datos NO sustituyen a esta medida.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402


def analyze(log, rest_s=8.0, max_lag=0.1, step=0.0005):
    pars = log.params()
    lsb_a = float(pars["imu.acc_lsb_per_g"]["value"]) if "imu.acc_lsb_per_g" in pars else 2048.0
    g_si = float(pars["imu.g_si"]["value"]) if "imu.g_si" in pars else 9.80665
    imu = log["IMU"]
    ok = ((imu["flags"] & vnlog.IMU_F["I2C_ERR"]) == 0) & ((imu["flags"] & vnlog.IMU_F["ACC_NEW"]) != 0)
    t = imu["t_drdy_us"][ok].astype(np.int64)
    A = np.stack([imu["ax"], imu["ay"], imu["az"]], 1)[ok].astype(float) / lsb_a * g_si
    ts = (t - t[0]) * 1e-6
    q = (ts > 1.0) & (ts < rest_s)
    if q.sum() < 100:
        raise SystemExit("reposo inicial insuficiente")
    gm = A[q].mean(0)
    gn = np.linalg.norm(gm)
    u = gm / gn  # la fuerza específica en reposo apunta hacia ARRIBA
    a_up = A @ u - gn
    # doble integral (trapecios) desde el final del reposo
    dt = np.diff(ts)
    v = np.concatenate([[0.0], np.cumsum(0.5 * (a_up[1:] + a_up[:-1]) * dt)])
    h = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * dt)])
    b = log["BARO"]
    bok = (b["flags"] & vnlog.BARO_F["I2C_ERR"]) == 0
    tb = (b["t_read_us"][bok].astype(np.int64) - t[0]) * 1e-6
    alt = b["alt_m"][bok].astype(float)
    sel = (tb > rest_s * 0.5) & (tb < ts[-1] - max_lag)
    tb, alt = tb[sel], alt[sel]
    T0 = tb.mean()
    taus = np.arange(0.0, max_lag + step / 2, step)
    res = []
    for tau in taus:
        hi = np.interp(tb - tau, ts, h)
        X = np.stack([hi, np.ones_like(tb), tb - T0, (tb - T0) ** 2], 1)
        coef, rss, *_ = np.linalg.lstsq(X, alt, rcond=None)
        r = alt - X @ coef
        res.append((float(r @ r), coef))
    rss = np.array([x[0] for x in res])
    k = int(np.argmin(rss))
    # mínimo subdivisional por parábola
    tau = taus[k]
    if 0 < k < len(taus) - 1:
        y0, y1, y2 = rss[k - 1], rss[k], rss[k + 1]
        den = y0 - 2 * y1 + y2
        if den > 0:
            tau = taus[k] + 0.5 * step * (y0 - y2) / den
    coef = res[k][1]
    s = float(coef[0])
    n = len(alt)
    sig_res = math.sqrt(rss[k] / max(1, n - 4))
    dh = np.gradient(np.interp(tb - tau, ts, h), tb)
    sig_tau = sig_res / (abs(s) * math.sqrt(float(dh @ dh))) if abs(s) > 0 and dh @ dh > 0 else float("inf")
    # amplitud del movimiento con la deriva cuadrática del ajuste quitada (la de la doble integral)
    hb = np.interp(tb - tau, ts, h)
    Xd = np.stack([np.ones_like(tb), tb - T0, (tb - T0) ** 2], 1)
    hb = hb - Xd @ np.linalg.lstsq(Xd, hb, rcond=None)[0]
    amp = float(np.percentile(hb, 95) - np.percentile(hb, 5))
    return {"latencia_s": float(tau), "sigma_latencia_s": float(sig_tau), "escala": s, "residuo_m": sig_res,
            "amplitud_imu_m": amp, "n_baro": int(n), "en_borde": bool(k == 0 or k == len(taus) - 1),
            "simulado": log.is_sim()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--reposo", type=float, default=8.0)
    ap.add_argument("--max-lag", type=float, default=0.1)
    ap.add_argument("--json")
    a = ap.parse_args()
    R = analyze(vnlog.Log(a.log), a.reposo, a.max_lag)
    print("baro arriba/abajo" + (" (AVISO: registro SIMULADO)" if R["simulado"] else ""))
    print(f"  amplitud del movimiento (IMU): {R['amplitud_imu_m']:.2f} m, {R['n_baro']} lecturas de baro")
    print(f"  escala baro/IMU s = {R['escala']:+.3f} " +
          ("(signo correcto)" if R["escala"] > 0.5 else "(SIGNO O ESCALA INCORRECTOS)"))
    print(f"  latencia efectiva respecto a la IMU: {R['latencia_s'] * 1e3:.1f} ms ± {R['sigma_latencia_s'] * 1e3:.1f} ms"
          f" (1σ); residuo {R['residuo_m']:.3f} m")
    if R["en_borde"]:
        print("  AVISO: el mínimo está en el borde del intervalo de búsqueda: ampliar --max-lag o revisar la prueba")
    if R["amplitud_imu_m"] < 0.5:
        print("  AVISO: movimiento pequeño: la latencia es poco fiable")
    print("  → baro_latency_s: media de varias repeticiones; anotar como medido (con fecha y boot_id)")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(R, f, indent=1)
    ok = R["escala"] > 0.5 and not R["en_borde"]
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
