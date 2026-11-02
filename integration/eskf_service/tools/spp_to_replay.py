#!/usr/bin/env python3
"""spp_to_replay.py — convierte un registro SPP (bin, hex o volcado) en órdenes para c/build/nav_replay.

  spp_to_replay.py registro.bin salida.txt --config cfg.txt [--historico] [--kpid I,B]
                   [--rotacion r00,r01,...,r22] [--acc-unit 9.80665] [--gyro-unit 0.0174532925]

cfg.txt: órdenes CFG de `svc_replay --dump-cfg banco` (o del perfil que se quiera). Salida: CFG,
INIT y una orden IMU/BARO por paquete, ORDENADAS por marca (IMU primero en empate), con t = ms·1e-3.

Conversión idéntica a eskf_service.c, en float32 y en el mismo orden: valor publicado × unidad (una
sola vez; los offsets del productor NO se restan otra vez) y después cuerpo = R·sensor como
(r0·s0 + r1·s1) + r2·s2. Saturación: candidatos en el dominio de códigos (v + offset)·divisor
>= 32768 − 1.5, en ejes del sensor y llevados a los ejes del cuerpo con R.

Esto es un CONVERSOR para analizar: no reproduce el calendario del anfitrión (espera de orden,
nav_tick, límites por llamada). Para reproducir exactamente las llamadas del servicio están
`svc_replay --entradas` (mismas llamadas que hizo el servicio) y la traza de validación.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

f32 = np.float32
OFF_A = [f32(0.004621), f32(-0.001157), f32(0.010014)]
OFF_G = [f32(0.681567), f32(0.428753), f32(-0.329620)]


def convert(data, cfg_lines, ki, kb, hist=False, rot=None, acc_unit=9.80665, gyro_unit=0.017453292519943295):
    r = [f32(x) for x in (rot or [1, 0, 0, 0, 1, 0, 0, 0, 1])]
    au, gu = f32(acc_unit), f32(gyro_unit)
    oa = [f32(0)] * 3 if hist else OFF_A
    og = [f32(0)] * 3 if hist else OFF_G
    imu_len = 48 if hist else 44
    pkts, issues, _ = spp_wire.split_packets(data)
    ev = []
    for p in pkts:
        if not p["crc_ok"]:
            continue
        t = p["ts"] * 1000 * 1e-6  # misma cuenta que el servicio: (double)t_us · 1e-6
        if p["id"] == ki and p["len"] == imu_len:
            v = [f32(x) for x in struct.unpack_from("<6f", p["payload"], 0)]
            fin = all(math.isfinite(float(x)) for x in v)
            ss = 0
            bad = not fin
            if fin:
                for j in range(3):
                    ca = (float(v[j]) + float(oa[j])) * 8192.0
                    cg = (float(v[3 + j]) + float(og[j])) * float(f32(32.8))
                    if abs(ca) > 32768.5 or abs(cg) > 32768.5:
                        bad = True
                    if abs(ca) >= 32768.0 - 1.5:
                        ss |= 1 << j
                    if abs(cg) >= 32768.0 - 1.5:
                        ss |= 1 << (3 + j)
            if bad:
                acc = gyr = [float("nan")] * 3
                sat = 0
            else:
                sa = [f32(v[j] * au) for j in range(3)]
                sg = [f32(v[3 + j] * gu) for j in range(3)]
                acc, gyr, sat = [], [], 0
                for i in range(3):
                    ri = r[3 * i:3 * i + 3]
                    acc.append(float(f32(f32(f32(ri[0] * sa[0]) + f32(ri[1] * sa[1])) + f32(ri[2] * sa[2]))))
                    gyr.append(float(f32(f32(f32(ri[0] * sg[0]) + f32(ri[1] * sg[1])) + f32(ri[2] * sg[2]))))
                    for j in range(3):
                        if abs(float(ri[j])) <= 1e-6:
                            continue
                        if ss >> j & 1:
                            sat |= 1 << i
                        if ss >> (3 + j) & 1:
                            sat |= 8 << i
            ev.append((p["ts"], 0, f"IMU {t:.17g} " + " ".join(f"{x:.9g}" for x in acc + gyr) + f" {sat}"))
        elif p["id"] == kb and p["len"] == 12:
            alt = struct.unpack_from("<f", p["payload"], 0)[0]
            ev.append((p["ts"], 1, f"BARO {t:.17g} {alt:.9g}"))
    ev.sort(key=lambda e: (e[0], e[1]))
    out = ["# spp_to_replay: conversor de análisis (sin calendario del anfitrión); marcas = ms de construcción"]
    out += cfg_lines + ["INIT"] + [e[2] for e in ev]
    return out, issues


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("registro")
    ap.add_argument("salida")
    ap.add_argument("--config", required=True)
    ap.add_argument("--formato")
    ap.add_argument("--historico", action="store_true")
    ap.add_argument("--kpid")
    ap.add_argument("--rotacion")
    ap.add_argument("--acc-unit", type=float, default=9.80665)
    ap.add_argument("--gyro-unit", type=float, default=0.017453292519943295)
    a = ap.parse_args()
    ki, kb = (0x8, 0x4) if a.historico else (0x4, 0x8)
    if a.kpid:
        ki, kb = (int(x, 0) for x in a.kpid.split(","))
    rot = [float(x) for x in a.rotacion.split(",")] if a.rotacion else None
    if rot and len(rot) != 9:
        raise SystemExit("--rotacion necesita 9 valores")
    cfg = [l.strip() for l in open(a.config) if l.startswith("CFG")]
    data, _, _ = spp_wire.load(a.registro, a.formato)
    lines, issues = convert(data, cfg, ki, kb, a.historico, rot, a.acc_unit, a.gyro_unit)
    with open(a.salida, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"spp_to_replay: {len(lines) - len(cfg) - 2} eventos; incidencias del registro {len(issues)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
