#!/usr/bin/env python3
"""gen_spp.py — registro SPP SINTÉTICO (sólo para pruebas; no sirve para calibrar).

  gen_spp.py salida.bin [--segundos 14] [--historico] [--hueco CADA_MS,DUR_MS] [--perder-imu N]
             [--movimiento T0,T1] [--semilla 1] [--relleno BYTES] [--kpid I,B]

Por defecto, el formato DEFINITIVO de SPP 9247d9e con la aritmética de los productores en float32:
  ICM 44 B: v = raw/8192.0f − offset y raw/32.8f − offset (offsets de icm20948.h), cuaternión
            (1,0,0,0), accuracy 3; IMU cada ~4.44 ms (225 Hz), marcas = ms de construcción.
  BMP 12 B: altura RELATIVA (T0/0.0065)(1 − (p/p0)^(1/5.255)); nada publicado durante las 500
            muestras de calibración (5 s a 100 Hz).
  KPID por orden de registro: ICM 0x0004, BMP 0x0008.
--historico: el formato de la captura del 24-oct (48 B sin offsets, altura ISA, IMU 0x0008, BMP 0x0004,
             sin retraso de calibración).
--hueco: parón del productor: las muestras de ese intervalo se descartan antes de publicar (la secuencia
         sigue consecutiva), como el driver que sólo publica el último paquete de la FIFO.
--perder-imu N: salta la secuencia de la IMU cada N paquetes (publicados y perdidos).
--movimiento: giro de ±100 °/s y ±0.5 g entre T0 y T1 (s).
"""
import argparse
import math
import os
import random
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

f32 = np.float32
OFF_A = [f32(0.004621), f32(-0.001157), f32(0.010014)]
OFF_G = [f32(0.681567), f32(0.428753), f32(-0.329620)]


def raw16(x):
    return max(-32768, min(32767, int(math.floor(x + 0.5))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("salida")
    ap.add_argument("--segundos", type=float, default=14.0)
    ap.add_argument("--historico", action="store_true")
    ap.add_argument("--hueco")
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--relleno", type=int, default=0)
    ap.add_argument("--perder-imu", type=int, default=0)
    ap.add_argument("--movimiento")
    ap.add_argument("--kpid")
    a = ap.parse_args()
    rnd = random.Random(a.semilla)
    ki, kb = (0x8, 0x4) if a.historico else (0x4, 0x8)
    if a.kpid:
        ki, kb = (int(x, 0) for x in a.kpid.split(","))
    g = [0.609, 0.020, 0.801]
    n = math.sqrt(sum(x * x for x in g))
    g = [x / n * 1.007 for x in g]
    gap_every, gap_len = (int(x) for x in a.hueco.split(",")) if a.hueco else (0, 0)
    mov = tuple(float(x) for x in a.movimiento.split(",")) if a.movimiento else None
    t0_us = 1_768_000
    t_end = t0_us + int(a.segundos * 1e6)
    baro_start = t0_us if a.historico else t0_us + 5_000_000
    ti, tb = t0_us, t0_us - 33_000
    si = sb = 0
    out = bytearray()
    next_gap = t0_us + gap_every * 1000 if gap_every else None
    k = 0
    p0, t0c = f32(94768.0), f32(21.4)
    while ti < t_end:
        if next_gap is not None and ti >= next_gap:
            ti += gap_len * 1000
            tb += gap_len * 1000
            next_gap += gap_every * 1000
        while tb <= ti:
            if tb >= baro_start:
                h = rnd.gauss(0, 0.2)
                if a.historico:
                    alt = 560.5 + h
                    p = 101325.0 * (1 - alt / 44330.0) ** (1 / 0.1903)
                    pl = struct.pack("<3f", alt, p, 21.4)
                else:
                    p = f32(float(p0) * (1 - 0.0065 * h / (float(t0c) + 273.15)) ** 5.255)
                    tk = f32(t0c + f32(273.15))
                    alt = (tk / f32(0.0065)) * (f32(1.0) - f32(math.pow(float(p / p0), float(f32(1.0) / f32(5.255)))))
                    pl = struct.pack("<3f", float(alt), float(p), float(t0c))
                out += spp_wire.build_packet(kb, sb, tb // 1000, pl)
                sb += 1
            tb += 9_870
        moving = mov and mov[0] <= (ti - t0_us) / 1e6 < mov[1]
        acc = [x + rnd.gauss(0, 0.006) for x in g]
        gyr = [rnd.gauss(0, 0.3) for _ in range(3)]
        if moving:
            ph = 2 * math.pi * (ti - t0_us) / 1e6
            acc = [acc[0] + 0.5 * math.sin(ph), acc[1], acc[2] + 0.3 * math.cos(ph)]
            gyr = [gyr[0] + 100 * math.sin(ph), gyr[1], gyr[2]]
        ra = [raw16(x * 8192) for x in acc]
        rg = [raw16(x * 32.8) for x in gyr]
        if a.historico:
            vals = [r / 8192 for r in ra] + [r / 32.8 for r in rg]
            pl = struct.pack("<12f", *[float(f32(v)) for v in vals], 1, 0, 0, 0, 304, 0)
        else:
            va = [f32(f32(r) / f32(8192.0)) - OFF_A[i] for i, r in enumerate(ra)]
            vg = [f32(f32(r) / f32(32.8)) - OFF_G[i] for i, r in enumerate(rg)]
            pl = struct.pack("<11f", *[float(v) for v in va + vg], 1, 0, 0, 0, 3)
        k += 1
        if not (a.perder_imu and k % a.perder_imu == 0):
            out += spp_wire.build_packet(ki, si, ti // 1000, pl)
        si = (si + 1) & 0xFFFF
        ti += 4_444
    out += bytes(a.relleno)
    with open(a.salida, "wb") as f:
        f.write(out)
    print(f"gen_spp: {len(out) // 64} paquetes, IMU {k}, baro {sb} ({'histórico 48 B' if a.historico else '44 B'}, SINTÉTICO)")


if __name__ == "__main__":
    main()
