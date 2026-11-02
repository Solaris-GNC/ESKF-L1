#!/usr/bin/env python3
"""analyze_static.py — análisis de un registro ESTÁTICO del Viradinha MKII (placa quieta).

  analyze_static.py registro.bin [--skip 5] [--json res.json] [--plot adev.png]

Informa de (todo medido en ESTE registro; nada sale de hojas de datos):
  * tiempos: frecuencia real de acelerómetro y giróscopo (por cambio de valor), intervalos entre
    interrupciones (media, jitter, percentiles), huecos (> dt_max), interrupciones perdidas,
    errores I2C, lecturas en carrera, latencia de lectura, tramas perdidas del registro;
  * baro: frecuencia, intervalos, retardo de detección por sondeo, ruido de altitud;
  * ruido: desviación de Allan por eje; densidad de ruido blanco N (m/s²/√Hz, rad/s/√Hz);
    inestabilidad del sesgo y paseo aleatorio K sólo si el registro es lo bastante largo (lo dice
    explícitamente si no);
  * UNIDADES DEL ESKF (eskf.c, predict_step): σ_a y σ_w son desviaciones POR EVENTO
    (Q_v = σ_a²·dt²), σ_aw y σ_ww son densidades de paseo aleatorio (Q_ba = σ_aw²·dt). Con el DLPF
    las muestras están correladas y, además, con interrupción por cualquier sensor la mitad de los
    eventos repiten un valor retenido: la desviación por muestra NO sirve tal cual. Lo que
    conserva el paseo aleatorio de velocidad real (N²·t) es σ_eq = N·√(E[dt]/E[dt²]) con los
    intervalos reales entre eventos entregados a navigation.c (se calcula aquí);
  * sesgos: media del giróscopo; norma media del acelerómetro frente a g (el sesgo por eje
    necesita las seis posiciones: six_position.py);
  * ventanas de reposo con la duración de rest_window_s: |‖a‖ − g|, σ(‖a‖), ‖ω‖ y cambio de
    inclinación entre mitades, para elegir umbrales con margen.
Los «candidatos» que imprime son PROPUESTAS (medida × margen indicado): hay que revisarlos y
anotarlos en vn_params.c con su estado y su origen (este registro: fecha y boot_id).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402

G_STD = 9.80665
TEMP_SENS, TEMP_OFFSET_C = 333.87, 21.0  # ICM-20948: T = código/333.87 + 21 °C (hoja de datos)


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def stats(x):
    x = np.asarray(x, float)
    if len(x) == 0:
        return {"n": 0}
    return {"n": int(len(x)), "media": float(x.mean()), "std": float(x.std()), "min": float(x.min()),
            "p1": pct(x, 1), "p50": pct(x, 50), "p99": pct(x, 99), "max": float(x.max())}


def adev(y, tau0, n_taus=40):
    """Desviación de Allan solapada de una serie de tasa y muestreada cada tau0.
    Devuelve (tau, adev, n_clusters)."""
    y = np.asarray(y, float)
    n = len(y)
    if n < 10:
        return np.array([]), np.array([]), np.array([])
    th = np.concatenate([[0.0], np.cumsum(y)]) * tau0
    ms = np.unique(np.logspace(0, math.log10(max(1, (n - 1) // 2)), n_taus).astype(int))
    taus, devs, ncl = [], [], []
    for m in ms:
        if n - 2 * m < 1:
            break
        d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
        avar = np.sum(d * d) / (2.0 * (m * tau0) ** 2 * (n - 2 * m + 1))
        taus.append(m * tau0)
        devs.append(math.sqrt(avar))
        ncl.append(n // m)
    return np.array(taus), np.array(devs), np.array(ncl)


def allan_summary(taus, devs, ncl, T):
    """Ajuste AVAR(τ) = N²/τ + c + K²·τ/3 (blanco + suelo de inestabilidad + paseo aleatorio) por
    mínimos cuadrados no negativos con peso relativo, sólo en τ con >= 9 grupos (error relativo
    de la ADEV < ~25 %). N se da siempre que haya τ entre 0.1 y 1 s; B y K sólo si su término
    domina en algún τ fiable; si no, se da una COTA SUPERIOR y lo que haría falta para medirlo."""
    r = {}
    ok = ncl >= 9
    r["tau_max_fiable"] = float(taus[ok].max()) if ok.any() else 0.0
    sel = ok & (taus >= 0.1) & (taus <= 1.0)
    if sel.sum() >= 3:
        r["N_directo"] = float(np.median(devs[sel] * np.sqrt(taus[sel])))
        sl = np.polyfit(np.log(taus[sel]), np.log(devs[sel]), 1)[0]
        r["pendiente_0p1_1s"] = float(sl)
        r["N_blanco_ok"] = bool(abs(sl + 0.5) < 0.15)
    t, d = taus[ok & (taus >= 0.05)], devs[ok & (taus >= 0.05)]
    r["N"] = r.get("N_directo")
    r["B"] = r["K"] = None
    if len(t) >= 4:
        M = np.stack([1.0 / t, np.ones_like(t), t / 3.0], 1) / (d * d)[:, None]
        y = np.ones_like(t)
        try:
            from scipy.optimize import nnls
            coef, _ = nnls(M, y)
        except ImportError:  # sin scipy: mínimos cuadrados y recorte a >= 0
            coef = np.clip(np.linalg.lstsq(M, y, rcond=None)[0], 0, None)
        n2, c, k2 = coef
        tm = t.max()
        r["N_ajuste"] = float(math.sqrt(n2)) if n2 > 0 else None
        if r["N"] is None and n2 > 0:
            r["N"] = float(math.sqrt(n2))
        # ¿domina cada término en algún τ fiable?
        if c > 0 and np.any(c > np.maximum(n2 / t, k2 * t / 3.0)):
            r["B"] = float(math.sqrt(c) / 0.664)
        if k2 > 0 and k2 * tm / 3.0 > n2 / tm + c:
            r["K"] = float(math.sqrt(k2))
            r["K_nota"] = (f"K estimado con τ fiable ≤ {tm:.0f} s: pocos grupos independientes, incertidumbre "
                           f"grande (factor ~2); confirmar con un registro estático largo")
        else:
            # cota: el K que habría igualado al resto en el τ fiable más largo
            k_up = math.sqrt(3.0 * (n2 / tm + c) / tm)
            r["K_cota_superior"] = float(k_up)
            r["K_nota"] = (f"K no observable en {T:.0f} s (τ fiable ≤ {tm:.0f} s): K < {k_up:.2g}; para medirlo "
                           f"hace falta que domine el paseo aleatorio, registro estático mucho más largo "
                           f"(orientativo ≥ {max(1.0, 30 * tm / 3600):.1f} h a temperatura estable)")
        i = int(np.argmin(d))
        r["adev_min"] = float(d[i])
        r["tau_min"] = float(t[i])
    return r


def windows(t, acc, gyr, win_s):
    """Ventanas consecutivas de win_s: |‖a‖−g| máx, σ(‖a‖), ‖ω‖ máx y cambio de inclinación entre mitades."""
    out = []
    if len(t) == 0:
        return out
    t0 = t[0]
    k = 0
    while True:
        a0, a1 = t0 + k * win_s, t0 + (k + 1) * win_s
        if a1 > t[-1]:
            break
        m = (t >= a0) & (t < a1)
        h = (t >= a0) & (t < a0 + win_s / 2)
        if m.sum() > 10 and h.sum() > 5 and (m & ~h).sum() > 5:
            an = np.linalg.norm(acc[m], axis=1)
            u1 = acc[h].mean(0)
            u2 = acc[m & ~h].mean(0)
            c = np.dot(u1, u2) / (np.linalg.norm(u1) * np.linalg.norm(u2))
            out.append({"acc_dev_max": float(np.max(np.abs(an - G_STD))), "acc_std": float(an.std()),
                        "gyro_max": float(np.max(np.linalg.norm(gyr[m], axis=1))),
                        "tilt_change": float(math.acos(min(1.0, max(-1.0, c)))),
                        "acc_mean_dev": float(abs(np.linalg.norm(acc[m].mean(0)) - G_STD)),
                        "gyro_mean": float(np.linalg.norm(gyr[m].mean(0)))})
        k += 1
    return out


def param(pars, name, default):
    p = pars.get(name)
    if p is None or p["value"] != p["value"]:
        return default
    return float(p["value"])


def analyze(log, skip=5.0):
    R = {"registro": log.path, "simulado": log.is_sim(), "arranques": len(log.boots())}
    pars = log.params()
    lsb_a = param(pars, "imu.acc_lsb_per_g", 2048.0)
    lsb_g = param(pars, "imu.gyro_lsb_per_dps", 16.4)
    g_si = param(pars, "imu.g_si", G_STD)
    dt_max = param(pars, "eskf.dt_max", 0.007)
    win_s = param(pars, "rest_window_s", 5.0)
    R["config_usada"] = {"acc_lsb_per_g": lsb_a, "gyro_lsb_per_dps": lsb_g, "dt_max": dt_max, "rest_window_s": win_s}

    imu = log["IMU"]
    if len(imu) < 100:
        raise SystemExit("registro sin datos de IMU suficientes")
    t_all = imu["t_drdy_us"].astype(np.int64)
    t0 = t_all[0] + int(skip * 1e6)
    sel = t_all >= t0
    imu = imu[sel]
    t = imu["t_drdy_us"].astype(np.int64)
    T = (t[-1] - t[0]) * 1e-6
    R["duracion_s"] = T
    fl = imu["flags"]
    F = vnlog.IMU_F

    # ---- tiempos de la IMU
    dt = np.diff(t) * 1e-6
    R["imu_eventos"] = int(len(t))
    R["imu_frecuencia_eventos_hz"] = float((len(t) - 1) / T)
    R["imu_intervalo_s"] = stats(dt)
    R["huecos_dt_max"] = {"umbral_s": dt_max, "n": int((dt > dt_max).sum()),
                          "primeros": [{"t_s": float((t[i] - t[0]) * 1e-6), "dt_s": float(dt[i])}
                                       for i in np.nonzero(dt > dt_max)[0][:10]]}
    seq = imu["seq"].astype(np.int64)
    R["imu_tramas_perdidas_registro"] = int(np.sum(np.diff(seq) - 1))
    for k in ("POLLED", "I2C_ERR", "IRQ_LOST", "RACE", "TIMEOUT", "MAG"):
        R[f"imu_flag_{k.lower()}"] = int(np.count_nonzero(fl & F[k]))
    an = (fl & F["ACC_NEW"]) != 0
    gn = (fl & F["GYR_NEW"]) != 0
    ok = (fl & F["I2C_ERR"]) == 0
    R["eventos_por_sensor"] = {"solo_acel": int((an & ~gn & ok).sum()), "solo_giro": int((~an & gn & ok).sum()),
                               "ambos": int((an & gn & ok).sum()), "ninguno": int((~an & ~gn & ok).sum())}
    ta, tg = t[an & ok], t[gn & ok]
    R["acel_frecuencia_hz"] = float((len(ta) - 1) / ((ta[-1] - ta[0]) * 1e-6)) if len(ta) > 2 else None
    R["giro_frecuencia_hz"] = float((len(tg) - 1) / ((tg[-1] - tg[0]) * 1e-6)) if len(tg) > 2 else None
    R["lectura_latencia_us"] = stats(imu["t_read_us"] - imu["t_drdy_us"])
    dtn = dt[(dt > 0) & (dt <= dt_max)]  # eventos entregados a navigation.c, sin huecos
    R["eventos_E_dt_s"] = float(dtn.mean())
    R["eventos_E_dt2_s2"] = float((dtn * dtn).mean())
    R["factor_sigma_eq"] = float(math.sqrt(R["eventos_E_dt_s"] / R["eventos_E_dt2_s2"]))  # σ_eq = N·factor
    st = log["STATS"]
    if len(st):
        s = st[-1]
        R["stats_final"] = {k: int(s[k]) for k in ("imu_irq_lost", "imu_q_drop", "baro_q_drop", "ring_drop_recs",
                                                   "i2c_err_imu", "i2c_err_baro", "sink_discard_bytes")}

    # ---- ruido y sesgos (sólo muestras nuevas de cada sensor)
    A = np.stack([imu["ax"], imu["ay"], imu["az"]], 1).astype(float) / lsb_a * g_si
    W = np.stack([imu["gx"], imu["gy"], imu["gz"]], 1).astype(float) / lsb_g * math.pi / 180.0
    acc, gyr = A[an & ok], W[gn & ok]
    tau_a = 1.0 / R["acel_frecuencia_hz"] if R["acel_frecuencia_hz"] else None
    tau_g = 1.0 / R["giro_frecuencia_hz"] if R["giro_frecuencia_hz"] else None
    R["acel_media_ms2"] = acc.mean(0).tolist()
    R["acel_std_ms2"] = acc.std(0).tolist()
    R["acel_norma_media_menos_g"] = float(np.linalg.norm(acc.mean(0)) - G_STD)
    R["giro_media_rads"] = gyr.mean(0).tolist()
    R["giro_std_rads"] = gyr.std(0).tolist()
    R["giro_media_dps"] = (gyr.mean(0) * 180 / math.pi).tolist()
    R["temp_c"] = stats(imu["temp"][ok] / TEMP_SENS + TEMP_OFFSET_C)
    R["allan"] = {}
    curves = {}
    for name, data, tau0 in (("acel", acc, tau_a), ("giro", gyr, tau_g)):
        for i, ax in enumerate("xyz"):
            taus, devs, ncl = adev(data[:, i], tau0)
            curves[f"{name}_{ax}"] = (taus, devs)
            R["allan"][f"{name}_{ax}"] = allan_summary(taus, devs, ncl, T)

    # ---- ventanas de reposo
    tw = t[an & gn & ok] * 1e-6
    wins = windows(tw, A[an & gn & ok], W[an & gn & ok], win_s)
    R["ventanas_reposo"] = {"n": len(wins)}
    if wins:
        for k in wins[0]:
            R["ventanas_reposo"][k + "_max"] = float(max(w[k] for w in wins))

    # ---- baro
    b = log["BARO"]
    if len(b):
        b = b[b["t_read_us"] >= t0]
        bok = (b["flags"] & vnlog.BARO_F["I2C_ERR"]) == 0
        tb = b["t_read_us"][bok].astype(np.int64)
        alt = b["alt_m"][bok].astype(float)
        R["baro_lecturas"] = int(len(b))
        R["baro_errores_i2c"] = int((~bok).sum())
        R["baro_tramas_perdidas_registro"] = int(np.sum(np.diff(b["seq"].astype(np.int64)) - 1)) if len(b) > 1 else 0
        if len(tb) > 10:
            Tb = (tb[-1] - tb[0]) * 1e-6
            R["baro_frecuencia_hz"] = float((len(tb) - 1) / Tb)
            R["baro_intervalo_s"] = stats(np.diff(tb) * 1e-6)
            det = b["t_detect_us"][bok] - b["t_prev_poll_us"][bok]
            R["baro_retardo_deteccion_us"] = stats(det[b["t_prev_poll_us"][bok] > 0])
            R["baro_lectura_us"] = stats(b["t_read_us"][bok] - b["t_detect_us"][bok])
            # ruido: residuo tras quitar la deriva lenta (mediana móvil de 10 s ≈ meteorología)
            k = max(3, int(10.0 * R["baro_frecuencia_hz"]) | 1)
            if len(alt) > 3 * k:
                pad = np.pad(alt, (k // 2, k // 2), mode="edge")
                from numpy.lib.stride_tricks import sliding_window_view
                trend = np.median(sliding_window_view(pad, k), axis=1)
                res = alt - trend
            else:
                res = alt - np.polyval(np.polyfit((tb - tb[0]) * 1e-6, alt, 1), (tb - tb[0]) * 1e-6)
            R["baro_ruido_alt_m"] = float(res.std())
            R["baro_deriva_alt_m"] = float(alt.max() - alt.min())
            taus, devs, ncl = adev(alt - alt.mean(), 1.0 / R["baro_frecuencia_hz"])
            curves["baro_alt"] = (taus, devs)
            R["baro_adev_tau0_m"] = float(devs[0]) if len(devs) else None
            R["baro_presion_pa"] = stats(b["pressure_pa"][bok])
            R["baro_temp_c"] = stats(b["temp_c"][bok])

    R["candidatos"] = candidates(R)
    return R, curves


def candidates(R):
    """Propuestas a partir de lo medido (× margen indicado). NO se aplican solas."""
    c = {}
    al = R["allan"]
    Na = [al[f"acel_{a}"].get("N") for a in "xyz"]
    Ng = [al[f"giro_{a}"].get("N") for a in "xyz"]
    f = R["factor_sigma_eq"]
    sa = R["acel_std_ms2"]
    sw = R["giro_std_rads"]
    if all(x is not None for x in Na):
        c["eskf.sigma_a"] = {"N_max": max(Na), "sigma_eq_por_evento": max(Na) * f, "std_por_muestra_max": max(sa),
                             "propuesta": None,
                             "nota": "σ_eq = N·√(E[dt]/E[dt²]) (m/s² por evento, unidades de eskf.c). En REPOSO: "
                                     "cota inferior; en vuelo hay vibración (pendiente de registro con motor). "
                                     "Si cambia el patrón de eventos (ODR, interrupción), recalcular"}
    if all(x is not None for x in Ng):
        c["eskf.sigma_w"] = {"N_max": max(Ng), "sigma_eq_por_evento": max(Ng) * f, "std_por_muestra_max": max(sw),
                             "propuesta": None, "nota": "ídem para el giróscopo (rad/s por evento)"}
    for key, sen in (("eskf.sigma_aw", "acel"), ("eskf.sigma_ww", "giro")):
        ks = {a: al[f"{sen}_{a}"].get("K") for a in "xyz"}
        found = {a: v for a, v in ks.items() if v is not None}
        notes = [al[f"{sen}_{a}"].get("K_nota") for a in "xyz" if ks[a] is None]
        e = {"medido_max": max(found.values()) if found else None, "ejes_con_K": "".join(found) or "ninguno"}
        if found:
            e["nota"] = al[f"{sen}_{next(iter(found))}"].get("K_nota")
        if notes:
            e["cota_superior_sin_K"] = max(al[f"{sen}_{a}"].get("K_cota_superior", 0.0) for a in "xyz" if ks[a] is None)
            e["nota"] = notes[0]
        c[key] = e
    if R.get("baro_adev_tau0_m") is not None:
        c["baro_sigma"] = {"medido": R["baro_ruido_alt_m"], "propuesta": None,
                           "nota": "ruido en reposo a 50 Hz; en vuelo añadir efectos dinámicos (pendiente)"}
    iv = R["imu_intervalo_s"]
    c["eskf.dt_max"] = {"intervalo_max_medido": iv["max"], "p99": iv["p99"],
                        "nota": "debe superar el máximo intervalo normal; un hueco real debe seguir superándolo"}
    w = R.get("ventanas_reposo", {})
    if w.get("n", 0) >= 3:
        for k, name, m in (("acc_dev_max_max", "rest_acc_dev_max", 2.0), ("acc_std_max", "rest_acc_std_max", 2.0),
                           ("gyro_max_max", "rest_gyro_max", 2.0), ("tilt_change_max", "rest_tilt_change_max", 3.0),
                           ("acc_mean_dev_max", "eskf.align_acc_tol", 3.0), ("gyro_mean_max", "eskf.align_gyro_max", 3.0)):
            if k in w:
                c[name] = {"medido_max": w[k], "margen": m, "propuesta": w[k] * m,
                           "nota": f"máximo en {w['n']} ventanas × {m}; comprobar con golpes/viento en rampa"}
    return c


def report(R, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    p(f"análisis estático: {R['registro']}  ({R['duracion_s']:.1f} s analizados)")
    if R["simulado"]:
        p("  AVISO: registro SIMULADO (gen_vnlog), no de la placa")
    if R["arranques"] > 1:
        p(f"  AVISO: {R['arranques']} arranques en el registro")
    iv = R["imu_intervalo_s"]
    p(f"IMU: {R['imu_eventos']} interrupciones, {R['imu_frecuencia_eventos_hz']:.2f} Hz; acel "
      f"{R['acel_frecuencia_hz']:.3f} Hz, giro {R['giro_frecuencia_hz']:.3f} Hz (por cambio de valor)")
    e = R["eventos_por_sensor"]
    p(f"  sensores nuevos por interrupción: sólo acel {e['solo_acel']}, sólo giro {e['solo_giro']}, ambos {e['ambos']}, "
      f"ninguno {e['ninguno']}")
    p(f"  intervalo: media {iv['media'] * 1e3:.3f} ms, σ {iv['std'] * 1e6:.1f} µs, p1 {iv['p1'] * 1e3:.3f}, "
      f"p99 {iv['p99'] * 1e3:.3f}, máx {iv['max'] * 1e3:.3f} ms")
    h = R["huecos_dt_max"]
    p(f"  huecos > dt_max ({h['umbral_s'] * 1e3:.1f} ms): {h['n']}" +
      ("" if not h["primeros"] else "  primeros: " + ", ".join(f"t={x['t_s']:.2f}s dt={x['dt_s'] * 1e3:.1f}ms"
                                                              for x in h["primeros"][:5])))
    p(f"  irq perdidas (marcas) {R['imu_flag_irq_lost']}, errores I2C {R['imu_flag_i2c_err']}, carreras "
      f"{R['imu_flag_race']}, sondeo de respaldo {R['imu_flag_timeout']}, por sondeo {R['imu_flag_polled']}, "
      f"tramas IMU perdidas del registro {R['imu_tramas_perdidas_registro']}")
    lat = R["lectura_latencia_us"]
    p(f"  dato listo → fin de lectura: media {lat['media']:.0f} µs, p99 {lat['p99']:.0f}, máx {lat['max']:.0f}")
    if "stats_final" in R:
        p("  STATS: " + ", ".join(f"{k} {v}" for k, v in R["stats_final"].items()))
    p("ruido y sesgos (ejes del SENSOR):")
    for nm, un, k in (("acel", "m/s²/√Hz", "acel"), ("giro", "rad/s/√Hz", "giro")):
        for ax in "xyz":
            a = R["allan"][f"{k}_{ax}"]
            N = a.get("N")
            extra = ""
            if a.get("B") is not None:
                extra += f", inestab. sesgo B {a['B']:.3g}"
            if a.get("K") is not None:
                extra += f", paseo K {a['K']:.3g}"
            elif a.get("K_cota_superior") is not None:
                extra += f", K no observable (< {a['K_cota_superior']:.2g})"
            p(f"  {nm} {ax}: N {N:.3g} {un}" + ("" if a.get("N_blanco_ok", True) else " (pendiente no −1/2)") +
              f", τ fiable ≤ {a['tau_max_fiable']:.0f} s{extra}")
    p(f"  eventos a navigation.c: E[dt] {R['eventos_E_dt_s'] * 1e3:.3f} ms, √E[dt²] "
      f"{math.sqrt(R['eventos_E_dt2_s2']) * 1e3:.3f} ms → σ_eq por evento = N × {R['factor_sigma_eq']:.2f} /√s")
    p(f"  acel media {np.round(R['acel_media_ms2'], 4).tolist()} m/s², ‖media‖ − g = "
      f"{R['acel_norma_media_menos_g'] * 1e3:.1f} mm/s² (sesgo por eje: six_position.py)")
    p(f"  giro media {np.round(R['giro_media_dps'], 4).tolist()} °/s  (la Tierra aporta hasta 0.0042 °/s)")
    p(f"  temperatura IMU: media {R['temp_c']['media']:.2f} °C, rango {R['temp_c']['min']:.2f} … "
      f"{R['temp_c']['max']:.2f} °C")
    if "baro_frecuencia_hz" in R:
        bi = R["baro_intervalo_s"]
        p(f"baro: {R['baro_lecturas']} lecturas, {R['baro_frecuencia_hz']:.3f} Hz, intervalo σ "
          f"{bi['std'] * 1e3:.2f} ms (máx {bi['max'] * 1e3:.1f} ms), errores I2C {R['baro_errores_i2c']}, "
          f"tramas perdidas {R['baro_tramas_perdidas_registro']}")
        d = R["baro_retardo_deteccion_us"]
        if d.get("n"):
            p(f"  sondeo: hueco entre sondeos con/sin dato media {d['media']:.0f} µs (máx {d['max']:.0f}); "
              f"lectura {R['baro_lectura_us']['media']:.0f} µs")
        p(f"  ruido de altitud {R['baro_ruido_alt_m']:.3f} m (ADEV τ0 {R['baro_adev_tau0_m']:.3f} m), deriva "
          f"total {R['baro_deriva_alt_m']:.2f} m")
    w = R.get("ventanas_reposo", {})
    if w.get("n"):
        p(f"ventanas de reposo de {R['config_usada']['rest_window_s']:.1f} s: {w['n']}; máximos: |‖a‖−g| "
          f"{w['acc_dev_max_max']:.3f} m/s², σ‖a‖ {w['acc_std_max']:.4f}, ‖ω‖ {w['gyro_max_max']:.4f} rad/s, "
          f"cambio de inclinación {w['tilt_change_max']:.5f} rad")
    p("candidatos (REVISAR; anotar en vn_params.c con estado y origen = este registro):")
    for k, v in R["candidatos"].items():
        s = ", ".join(f"{a} {b:.4g}" if isinstance(b, float) else f"{a} {b}" for a, b in v.items()
                      if a != "nota" and b is not None)
        p(f"  {k}: {s}" + (f"  — {v['nota']}" if v.get("nota") else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--skip", type=float, default=5.0, help="segundos iniciales descartados (arranque)")
    ap.add_argument("--json")
    ap.add_argument("--plot")
    a = ap.parse_args()
    R, curves = analyze(vnlog.Log(a.log), a.skip)
    report(R)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(R, f, indent=1)
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
        for k, (tt, dd) in curves.items():
            ax = axs[0] if k.startswith("acel") else axs[1] if k.startswith("giro") else axs[2]
            ax.loglog(tt, dd, label=k)
        for ax, ttl in zip(axs, ("acelerómetro (m/s²)", "giróscopo (rad/s)", "baro (m)")):
            ax.set_title("ADEV " + ttl)
            ax.set_xlabel("τ (s)")
            ax.grid(True, which="both", alpha=0.3)
            ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
    return 0


if __name__ == "__main__":
    sys.exit(main())
