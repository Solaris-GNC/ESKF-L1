#!/usr/bin/env python3
"""explain_alignment.py — por qué una captura SPP alinea o no con un perfil, SIN cambiar parámetros.

  explain_alignment.py registro.bin --config cfg.txt [--imu-id 8]

Reproduce las condiciones de ventana de reposo de navigation.c con los umbrales de cfg.txt
(CFG de `svc_replay --dump-cfg banco`) y las marcas del paquete (ms):
  * tramos CONTINUOS de la IMU: intervalos <= dt_max (un intervalo mayor parte la ventana);
  * en cada tramo, el mayor sub-tramo que cumple los criterios por muestra (|‖a‖−g| y ‖ω‖);
  * si ese sub-tramo dura >= rest_window_s, se evalúan σ(‖a‖) y el cambio de inclinación
    entre mitades.
Unidades: hipótesis de banco (g y °/s). Es un diagnóstico: no propone valores.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

G = 9.80665


def cfg_values(path):
    v = {}
    for l in open(path):
        t = l.split()
        if len(t) == 3 and t[0] == "CFG":
            v[t[1]] = float(t[2])
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("registro")
    ap.add_argument("--config", required=True)
    ap.add_argument("--formato")
    ap.add_argument("--imu-id", type=lambda s: int(s, 0), default=8)
    a = ap.parse_args()
    c = cfg_values(a.config)
    dt_max, win = c["eskf.dt_max"], c["rest_window_s"]
    dev_max, gyr_max, std_max, tilt_max = (c["rest_acc_dev_max"], c["rest_gyro_max"], c["rest_acc_std_max"],
                                           c["rest_tilt_change_max"])
    data, f, _ = spp_wire.load(a.registro, a.formato)
    pkts, _, _ = spp_wire.split_packets(data)
    imu = [p for p in pkts if p["crc_ok"] and p["id"] == a.imu_id and p["len"] in (44, 48)]
    t = np.array([p["ts"] for p in imu], np.float64) * 1e-3
    F = np.array([struct.unpack("<6f", bytes(p["payload"][:24])) for p in imu], np.float64)
    acc, gyr = F[:, :3] * G, F[:, 3:6] * math.pi / 180
    an = np.linalg.norm(acc, axis=1)
    ok = (np.abs(an - G) <= dev_max) & (np.linalg.norm(gyr, axis=1) <= gyr_max)
    dt = np.diff(t)
    breaks = np.nonzero(dt > dt_max + 1e-9)[0]
    print(f"{a.registro}: {len(t)} IMU, dt_max {dt_max * 1e3:.1f} ms, ventana {win:.1f} s; "
          f"{len(breaks)} intervalos > dt_max (ms: {sorted(set(int(round(x * 1e3)) for x in dt[breaks]))})")
    print(f"  umbrales (perfil del archivo): |‖a‖−g| <= {dev_max}, ‖ω‖ <= {gyr_max} rad/s, σ‖a‖ <= {std_max}, "
          f"inclinación <= {tilt_max} rad")
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks + 1, [len(t)]])
    best = 0.0
    any_window = False
    for s0, e0 in zip(starts, ends):
        span = t[e0 - 1] - t[s0]
        # mayor sub-tramo de muestras que cumplen los criterios por muestra
        i, bl, bs = s0, 0.0, None
        while i < e0:
            if not ok[i]:
                i += 1
                continue
            j = i
            while j < e0 and ok[j]:
                j += 1
            L = t[j - 1] - t[i]
            if L > bl:
                bl, bs = L, (i, j)
            i = j
        best = max(best, bl)
        line = (f"  tramo continuo {t[s0] - t[0]:7.3f}–{t[e0 - 1] - t[0]:7.3f} s ({span:5.2f} s): mayor reposo "
                f"por muestra {bl:5.2f} s")
        if bs and bl >= win:
            i, j = bs
            m = slice(i, j)
            h = t[i] + bl / 2
            u1 = acc[i:j][t[i:j] < h].mean(0)
            u2 = acc[i:j][t[i:j] >= h].mean(0)
            tilt = math.acos(min(1.0, float(u1 @ u2 / np.linalg.norm(u1) / np.linalg.norm(u2))))
            line += f"; σ‖a‖ {an[m].std():.4f}, inclinación {tilt:.5f} rad"
            any_window |= an[m].std() <= std_max and tilt <= tilt_max
        print(line)
    print(f"  CONCLUSIÓN: el mayor reposo continuo es {best:.2f} s " +
          (f"< {win:.1f} s: ninguna ventana puede completarse; los intervalos > dt_max la parten"
           if best < win else ("y hay ventanas válidas" if any_window else "pero no cumple σ/inclinación")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
