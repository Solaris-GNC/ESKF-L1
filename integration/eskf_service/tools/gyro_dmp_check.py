#!/usr/bin/env python3
"""gyro_dmp_check.py — coherencia de ESCALA entre el giro publicado y el cuaternión DMP (diagnóstico).

  gyro_dmp_check.py registro.bin [--imu-id 4] [--historico] [--ventana 45] [--min-rad 0.3]

En tramos SIN huecos (intervalos <= 7.5 ms) integra el giro publicado (°/s → rad/s, π/180) en
ventanas de N muestras y compara el ángulo girado con el que da el cuaternión DMP entre los
extremos de la ventana. Cociente ≈ 1: el giro publicado y el DMP usan la misma escala;
≈ 0.5 indicaría que /32.8 con FS = 2000 °/s divide por 2 la velocidad real.

NO es una calibración ni usa el DMP como verdad: es una comprobación cruzada que debe
confirmarse con giros de ángulo conocido (mesa giratoria o 90° contra topes).
"""
import argparse
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def ratios(t, w, q, win, min_rad):
    dt = np.diff(t)
    out = []
    i = 0
    while i < len(t) - win - 1:
        j = i + win
        if np.all(dt[i:j] <= 0.0075):
            r = np.array([1.0, 0, 0, 0])
            for k in range(i, j):
                th = w[k] * dt[k]
                a = np.linalg.norm(th)
                dq = np.array([1.0, 0, 0, 0]) if a < 1e-12 else np.concatenate([[np.cos(a / 2)], np.sin(a / 2) * th / a])
                r = qmul(r, dq)
            qd = qmul(q[i] * np.array([1, -1, -1, -1]), q[j])
            ag = 2 * np.arccos(min(1.0, abs(r[0])))
            ad = 2 * np.arccos(min(1.0, abs(qd[0])))
            if ad > min_rad:
                out.append((t[i], ag, ad, ag / ad))
        i += win
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("registro")
    ap.add_argument("--formato")
    ap.add_argument("--imu-id", type=lambda s: int(s, 0))
    ap.add_argument("--historico", action="store_true")
    ap.add_argument("--ventana", type=int, default=45)
    ap.add_argument("--min-rad", type=float, default=0.3)
    a = ap.parse_args()
    ident = a.imu_id if a.imu_id is not None else (0x8 if a.historico else 0x4)
    ln = 48 if a.historico else 44
    data, _, _ = spp_wire.load(a.registro, a.formato)
    pk, _, _ = spp_wire.split_packets(data)
    imu = [p for p in pk if p["crc_ok"] and p["id"] == ident and p["len"] == ln]
    if len(imu) < a.ventana + 2:
        print("gyro_dmp_check: muy pocas muestras")
        return 2
    t = np.array([p["ts"] for p in imu]) * 1e-3
    F = np.array([struct.unpack_from("<10f", p["payload"], 0) for p in imu], np.float64)
    r = ratios(t, np.radians(F[:, 3:6]), F[:, 6:10], a.ventana, a.min_rad)
    if not len(r):
        print("gyro_dmp_check: ninguna ventana con giro suficiente (hace falta movimiento)")
        return 3
    med = float(np.median(r[:, 3]))
    print(f"gyro_dmp_check: {len(r)} ventanas con giro DMP > {a.min_rad} rad; cociente ∫ω/DMP mediana {med:.3f} "
          f"(p10 {np.percentile(r[:, 3], 10):.3f}, p90 {np.percentile(r[:, 3], 90):.3f})")
    print("  diagnóstico: " + ("escala del giro coherente con el DMP" if 0.9 < med < 1.1 else
                               "posible factor 2 (FS 2000 °/s con /32.8)" if 0.45 < med < 0.55 else
                               "incoherente: revisar escala, ejes o marcas"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
