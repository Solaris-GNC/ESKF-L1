#!/usr/bin/env python3
"""soak_report.py — informe de la prueba de resistencia (2 h) del firmware en la placa.

  soak_report.py registro.bin [--min-horas 2] [--json res.json]

Criterios (PROCEDIMIENTOS.md, «Prueba de 2 horas»); cada uno da OK / FALLO / AVISO:
  duración >= --min-horas; un solo arranque y reloj sin saltos; ningún intervalo de la IMU > dt_max
  (interrupciones «perdidas»: aviso); colas (IMU, baro, ordenación) sin desbordes; registro sin tramas perdidas; errores I2C = 0;
  tiempo de navigation.c: máximo < dt_max y uso medio de CPU de vn_nav < 50 %; pila libre mínima
  >= 25 % de la reserva en cada tarea; montón sin descenso tras los 10 primeros minutos;
  frecuencia de eventos estable (±1 %); con navegación: VÁLIDA todo el tiempo tras alinear
  (placa quieta), sin despegue falso y sin huecos.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402


def analyze(log, min_hours=2.0):
    checks = []

    def chk(name, ok, detail, warn=False):
        checks.append({"criterio": name, "estado": "OK" if ok else ("AVISO" if warn else "FALLO"), "detalle": detail})

    pars = log.params()
    pv = lambda n, d: (float(pars[n]["value"]) if n in pars and pars[n]["value"] == pars[n]["value"] else d)
    stack = pv("fw.stack_bytes", 16384.0)
    ring = pv("fw.ring_bytes", 49152.0)
    dt_max = pv("eskf.dt_max", 0.007)
    st = log["STATS"]
    if len(st) < 3:
        raise SystemExit("registro sin STATS suficientes")
    t = st["t_us"].astype(np.int64)
    dur_h = (t[-1] - t[0]) * 1e-6 / 3600.0
    chk("duración", dur_h >= min_hours, f"{dur_h:.2f} h (mínimo {min_hours} h)")
    boots = log.boots()
    mono = bool(np.all(np.diff(t) > 0))
    chk("un solo arranque, reloj monótono", len(boots) == 1 and mono, f"{len(boots)} arranques, monótono {mono}")
    last = st[-1]
    first = st[0]
    d_lost = int(last["imu_irq_lost"]) - int(first["imu_irq_lost"])
    chk("interrupciones IMU perdidas", d_lost == 0,
        f"{d_lost} durante la prueba (acumulado final {int(last['imu_irq_lost'])}). Con interrupción por cualquier "
        f"sensor, dos pulsos casi simultáneos de acel y giro cuentan aquí aunque no se pierda dato: el criterio "
        f"que manda es el de huecos", warn=True)
    imu = log["IMU"]
    if len(imu) > 2:
        ti = imu["t_drdy_us"].astype(np.int64)
        ti = ti[ti >= t[0]]
        gaps = int((np.diff(ti) > dt_max * 1e6).sum())
        chk("huecos de la IMU > dt_max", gaps == 0, f"{gaps} intervalos > {dt_max * 1e3:.1f} ms")
    for k, nm in (("imu_q_drop", "cola IMU→nav llena"),
                  ("baro_q_drop", "cola baro→nav llena"), ("merge_imu_overflow", "ordenación IMU desbordada"),
                  ("merge_baro_overflow", "ordenación baro desbordada"), ("ring_drop_recs", "tramas perdidas del registro"),
                  ("i2c_err_imu", "errores I2C IMU"), ("i2c_err_baro", "errores I2C baro")):
        d = int(last[k]) - int(first[k])
        chk(nm, d == 0, f"{d} durante la prueba (acumulado final {int(last[k])})")
    chk("sumidero sin descartes", int(last["sink_discard_bytes"]) == 0,
        f"{int(last['sink_discard_bytes'])} B descartados (¿USB desconectado?)", warn=True)
    ex = st["nav_exec_max_us"].astype(float)
    per = np.diff(t) * 1e-6
    util = st["nav_exec_sum_us"][1:].astype(float) * 1e-6 / per
    nav_on = log.header is not None and log.header["nav_enabled"]
    if nav_on:
        chk("tiempo máx. de navigation.c < dt_max", ex.max() < dt_max * 1e6,
            f"máx {ex.max():.0f} µs (p99 de los máximos por periodo {np.percentile(ex, 99):.0f} µs), dt_max "
            f"{dt_max * 1e6:.0f} µs")
        chk("uso medio de CPU de vn_nav < 50 %", util.max() < 0.5, f"máx por periodo {util.max() * 100:.1f} %")
    chk("lectura de la IMU", True, f"máx {int(st['acq_read_max_us'].max())} µs por lectura (informativo)")
    chk("escritura del registro", True, f"máx {int(st['log_write_max_us'].max())} µs por bloque (informativo)")
    for k, nm in (("hwm_acq", "vn_acq"), ("hwm_nav", "vn_nav"), ("hwm_log", "vn_log")):
        m = int(st[k].min())
        chk(f"pila libre mínima {nm} >= 25 %", m >= 0.25 * stack, f"{m} B de {int(stack)} B ({m / stack * 100:.0f} %)")
    chk("pila libre mínima loop", int(st["hwm_loop"].min()) >= 2048, f"{int(st['hwm_loop'].min())} B (tarea de Arduino)")
    t10 = t[0] + 600 * 10 ** 6
    h = st["heap_free_min"].astype(np.int64)
    after = h[t >= t10]
    drop = int(after[0] - after[-1]) if len(after) > 1 else 0
    chk("montón: mínimo estable tras 10 min", drop <= 1024, f"descenso {drop} B (mínimo final {int(h[-1])} B)")
    rf = int(st["ring_free_min"].min())
    chk("búfer de registro con holgura >= 25 %", rf >= 0.25 * ring, f"mínimo libre {rf} B de {int(ring)} B", warn=True)
    ie = np.diff(st["imu_events"].astype(np.int64)) / per
    be = np.diff(st["baro_events"].astype(np.int64)) / per
    sel = per > 0.5
    if sel.sum() > 2:
        mi = np.median(ie[sel])
        dev = float(np.max(np.abs(ie[sel] - mi)) / mi) if mi > 0 else 1.0
        chk("frecuencia de eventos IMU estable (±1 %)", dev <= 0.01, f"mediana {mi:.1f}/s, desviación máx "
                                                                    f"{dev * 100:.2f} %")
        mb = np.median(be[sel])
        chk("frecuencia de baro", abs(mb - 50.0) < 1.0, f"mediana {mb:.2f}/s")
    imu = log["IMU"]
    if len(imu):
        tc = imu["temp"].astype(float) / 333.87 + 21.0
        chk("temperatura de la IMU", True, f"{tc.min():.1f} … {tc.max():.1f} °C (informativo)")
    if nav_on and len(log["NAVOUT"]):
        nv = log["NAVOUT"]
        VALID = 2  # nav_validity_t: 0 NO VÁLIDO, 1 DEGRADADO, 2 VÁLIDO
        al = np.nonzero(nv["validity"] == VALID)[0]
        if len(al):
            after_al = nv[al[0]:]
            bad = after_al["validity"] != VALID
            reasons = int(np.bitwise_or.reduce(after_al["reasons"])) if len(after_al) else 0
            chk("navegación VÁLIDA tras alinear (placa quieta)", not bad.any(),
                f"{int(bad.sum())} salidas no válidas de {len(after_al)}; motivos 0x{reasons:04x}")
        else:
            chk("navegación alineada", False, "nunca VÁLIDA")
        chk("sin despegue falso (placa quieta: fase rampa)", not (nv["phase"] != 0).any(),
            f"{int((nv['phase'] != 0).sum())} salidas en fase de vuelo")
        c = log.navcnt_last()
        if c:
            chk("sin huecos en navigation.c", c["gaps"] == 0, f"gaps {c['gaps']}, align_fail {c['align_fail']}, "
                                                             f"inflate_fail {c['inflate_fail']}")
    return {"duracion_h": dur_h, "criterios": checks, "simulado": log.is_sim(),
            "resultado": "FALLO" if any(x["estado"] == "FALLO" for x in checks) else "OK"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--min-horas", type=float, default=2.0)
    ap.add_argument("--json")
    a = ap.parse_args()
    R = analyze(vnlog.Log(a.log), a.min_horas)
    print(f"prueba de resistencia: {a.log}" + ("  (AVISO: registro SIMULADO)" if R["simulado"] else ""))
    for c in R["criterios"]:
        print(f"  [{c['estado']:5s}] {c['criterio']}: {c['detalle']}")
    print(f"RESULTADO: {R['resultado']}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(R, f, indent=1)
    return 0 if R["resultado"] == "OK" else 1


if __name__ == "__main__":
    sys.exit(main())
