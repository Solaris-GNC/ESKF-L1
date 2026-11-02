#!/usr/bin/env python3
"""six_position.py — calibración del acelerómetro por seis posiciones (y sesgo del giróscopo).

  six_position.py pos1.bin pos2.bin ... pos6.bin [--g 9.80665] [--skip 3] [--json res.json]

Cada registro: placa QUIETA con un eje del sensor hacia arriba (+x, −x, +y, −y, +z, −z; el orden
da igual, se detecta). Con m(+i) y m(−i) las medias del eje i:
    sesgo b_i = (m(+i) + m(−i)) / 2,   escala k_i = (m(+i) − m(−i)) / (2 g) − 1
y la matriz de acoplamiento M[:, i] = (m(+i) − m(−i)) / (2 g) (ejes no ortogonales / montaje).
g: por defecto la normal (9.80665); la local de Huesca difiere en ~1e-3 relativo: pasar --g con el
valor local si se quiere separar escala de gravedad (la gravedad local es un dato externo).
Es una medida de NUESTRA unidad; sirve para ba0_std y para decidir si conviene corregir escala/sesgo
en imu_conv (hoy no se corrige: decisión pendiente del equipo).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402


def static_means(path, skip):
    log = vnlog.Log(path)
    pars = log.params()
    lsb_a = float(pars["imu.acc_lsb_per_g"]["value"]) if "imu.acc_lsb_per_g" in pars else 2048.0
    lsb_g = float(pars["imu.gyro_lsb_per_dps"]["value"]) if "imu.gyro_lsb_per_dps" in pars else 16.4
    g_si = float(pars["imu.g_si"]["value"]) if "imu.g_si" in pars else 9.80665
    imu = log["IMU"]
    t = imu["t_drdy_us"].astype(np.int64)
    ok = ((imu["flags"] & vnlog.IMU_F["I2C_ERR"]) == 0) & (t >= t[0] + int(skip * 1e6))
    an = ok & ((imu["flags"] & vnlog.IMU_F["ACC_NEW"]) != 0)
    gn = ok & ((imu["flags"] & vnlog.IMU_F["GYR_NEW"]) != 0)
    A = np.stack([imu["ax"], imu["ay"], imu["az"]], 1)[an].astype(float) / lsb_a * g_si
    W = np.stack([imu["gx"], imu["gy"], imu["gz"]], 1)[gn].astype(float) / lsb_g * math.pi / 180
    return {"path": path, "acc": A.mean(0), "acc_sem": A.std(0) / math.sqrt(len(A)), "gyro": W.mean(0),
            "n": len(A), "sim": log.is_sim(), "sat": int(np.count_nonzero(imu["sat"][ok]))}


def solve(ms, g):
    pos = {}
    for m in ms:
        i = int(np.argmax(np.abs(m["acc"])))
        s = "+" if m["acc"][i] > 0 else "-"
        key = s + "xyz"[i]
        if key in pos:
            raise SystemExit(f"dos registros con {key} hacia arriba: {pos[key]['path']} y {m['path']}")
        tilt = math.degrees(math.acos(min(1.0, abs(m["acc"][i]) / np.linalg.norm(m["acc"]))))
        m["tilt_deg"] = tilt
        pos[key] = m
    missing = [k for k in ("+x", "-x", "+y", "-y", "+z", "-z") if k not in pos]
    if missing:
        raise SystemExit("faltan posiciones: " + ", ".join(missing))
    b = np.zeros(3)
    M = np.zeros((3, 3))
    for i, a in enumerate("xyz"):
        mp, mn = pos["+" + a]["acc"], pos["-" + a]["acc"]
        b[i] = 0.5 * (mp[i] + mn[i])
        M[:, i] = (mp - mn) / (2 * g)
    k = np.diag(M) - 1.0
    off = M - np.diag(np.diag(M))
    gb = np.mean([m["gyro"] for m in pos.values()], axis=0)
    gspread = np.std([m["gyro"] for m in pos.values()], axis=0)
    # comprobación: con la corrección, ‖a‖ en cada posición frente a g
    Minv = np.linalg.inv(M)
    resid = {key: float(np.linalg.norm(Minv @ (m["acc"] - b)) - g) for key, m in pos.items()}
    return {"g": g, "sesgo_ms2": b.tolist(), "escala": k.tolist(), "acoplamiento": off.tolist(),
            "M": M.tolist(), "giro_sesgo_rads": gb.tolist(), "giro_dispersion_rads": gspread.tolist(),
            "residuo_norma_ms2": resid, "inclinacion_deg": {k2: m["tilt_deg"] for k2, m in pos.items()},
            "saturadas": sum(m["sat"] for m in pos.values()), "simulado": any(m["sim"] for m in pos.values())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs=6)
    ap.add_argument("--g", type=float, default=9.80665)
    ap.add_argument("--skip", type=float, default=3.0)
    ap.add_argument("--json")
    a = ap.parse_args()
    R = solve([static_means(p, a.skip) for p in a.logs], a.g)
    print("seis posiciones" + (" (AVISO: registros SIMULADOS)" if R["simulado"] else ""))
    print(f"  sesgo acel (m/s², ejes sensor): {np.round(R['sesgo_ms2'], 5).tolist()}")
    print(f"  error de escala: {np.round(np.array(R['escala']) * 1e3, 3).tolist()} ‰  (con g = {R['g']})")
    print(f"  acoplamiento entre ejes (máx): {np.max(np.abs(R['acoplamiento'])) * 1e3:.3f} ‰")
    print(f"  sesgo giro (rad/s): {np.round(R['giro_sesgo_rads'], 6).tolist()}, dispersión entre posiciones "
          f"{np.round(R['giro_dispersion_rads'], 6).tolist()}")
    print("  inclinación de cada posición respecto a la vertical (°): " +
          ", ".join(f"{k} {v:.2f}" for k, v in sorted(R["inclinacion_deg"].items())))
    print("  residuo de ‖a‖ corregida − g (m/s²): " +
          ", ".join(f"{k} {v:+.4f}" for k, v in sorted(R["residuo_norma_ms2"].items())))
    if R["saturadas"]:
        print(f"  AVISO: {R['saturadas']} muestras saturadas")
    print("  → ba0_std: del orden del mayor |sesgo| medido (con margen por temperatura y montaje);")
    print("    anotar en vn_params.c como medido con estos registros. La corrección de escala/sesgo NO se aplica.")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(R, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
