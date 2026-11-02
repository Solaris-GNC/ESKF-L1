#!/usr/bin/env python3
"""gen_flight.py — registro SPP SINTÉTICO de un vuelo a partir de una serie OpenRocket extraída con
tools/ork_extract.py (test/data/openrocket_sim6_*.csv). Sólo para pruebas: no es un vuelo real,
no sirve para calibrar y no valida nada en placa.

  gen_flight.py serie.csv salida.bin [--verdad verdad.csv] [--rampa 20] [--t-fin S] [--semilla 1]
                [--sin-ruido] [--kpid 0x4,0x8]

Modelo (coherente físicamente con la serie; las hipótesis se escriben en la cabecera de verdad.csv):
  * Tiempo de placa: ICM desde 0.5 s a 225 Hz; BMP desde 0 s a 100 Hz, publica tras sus 500
    muestras de calibración (5 s); ignición en t = --rampa. Antes de la ignición, reposo en la guía.
  * Fuerza específica en ENU: f = (a_h · dirección horizontal, a_vertical + g), con a_vertical,
    |a_h| («Lateral acceleration») y g de la serie; la dirección horizontal sale de la derivada
    numérica de la velocidad horizontal (vuelo casi plano hacia el acimut).
  * Actitud: eje z de la IMU = eje del cohete hacia la punta (HIPÓTESIS de montaje del escenario;
    el servicio sigue con R = identidad, cuerpo = IMU); z = (cos e sen a, cos e cos a, sen e) con
    e = elevación y a = acimut desde el norte hacia el este (convenciones comprobadas por
    ork_extract.py); x horizontal perpendicular al acimut; alabeo 0 (la serie da 0). Tras el
    despliegue la serie deja de dar orientación: se congela (bajo paracaídas no hay datos).
  * Giro: ω = vee(Rᵀ dR/dt) por diferencias centradas de la actitud interpolada.
  * ICM: raw = redondeo((valor + offset del productor + sesgo residual)·divisor) recortado a
    int16 (±4 g y ±999 °/s en el dominio de códigos); publicado = raw/8192.0f − offset (float32,
    como icm20948.c). Ruido blanco 0.006 g y 0.25 °/s por eje (reposo observado); sesgos
    residuales fijos pequeños. Sin vibración de motor (no hay datos). Cuaternión DMP = actitud
    verdadera (sólo diagnóstico). Escala del giro: la del contrato (°/s publicados = °/s reales).
  * BMP: p = p0 (1 − 0.0065 h / T0)^5.25588 (atmósfera ISA de la serie), ruido 1.6 Pa (≈ 0.14 m),
    IIR de coeficiente 1 (registro 0x02 = 0x02 → campo 1), calibración de 500 muestras en
    float32 y altura = (T0/0.0065)(1 − (p/p0)^(1/5.255)) en float32, como bmp390.c.
verdad.csv: t de placa, t de simulación, altura y velocidad vertical verdaderas, fuerza
específica en ejes IMU (g) y si el acelerómetro recorta.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

f32 = np.float32
G0 = 9.80665
OFF_A = np.array([0.004621, -0.001157, 0.010014], dtype=f32)
OFF_G = np.array([0.681567, 0.428753, -0.329620], dtype=f32)
BIAS_A_G = np.array([0.003, -0.002, 0.004])     # sesgo residual tras los offsets (g)
BIAS_G_DPS = np.array([0.10, -0.08, 0.05])      # idem (°/s)


def read_series(path):
    head, rows, names = [], [], None
    for line in open(path):
        if line.startswith("#"):
            head.append(line.rstrip())
            continue
        if names is None:
            names = line.strip().split(",")
            continue
        rows.append([float(x) for x in line.strip().split(",")])
    D = np.array(rows)
    S = {n: D[:, i] for i, n in enumerate(names)}
    ev = {}
    for h in head:
        if h.startswith("# eventos:"):
            for kv in h.split(":", 1)[1].split(";"):
                k, v = kv.strip().split("=")
                ev.setdefault(k, float(v))
    return S, ev, head


def rot(el, az):
    """Columnas: ejes x, y, z de la IMU en ENU."""
    z = np.array([math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el)])
    x = np.array([math.cos(az), -math.sin(az), 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    w = math.sqrt(max(0.0, 1.0 + tr)) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return w, x, y, z


class Flight:
    def __init__(self, S, ev):
        self.S, self.ev = S, ev
        t = S["t_s"]
        self.t = t
        self.t_dep = ev.get("recoverydevicedeployment", t[-1])
        vE, vN = np.gradient(S["pos_e_m"], t), np.gradient(S["pos_n_m"], t)
        aE, aN = np.gradient(vE, t), np.gradient(vN, t)
        n = np.hypot(aE, aN)
        dE, dN = np.sin(S["azim_rad"]), np.cos(S["azim_rad"])  # por defecto, hacia el acimut
        ok = n > 1e-6
        dE = np.where(ok, aE / np.where(ok, n, 1), dE)
        dN = np.where(ok, aN / np.where(ok, n, 1), dN)
        self.fE = S["a_lat_mps2"] * dE
        self.fN = S["a_lat_mps2"] * dN
        self.fU = S["az_kin_mps2"] + S["g_mps2"]
        el, az = S["el_rad"].copy(), S["azim_rad"].copy()
        frz = t >= self.t_dep
        if np.any(frz):
            i0 = np.argmax(frz)
            el[frz], az[frz] = el[i0 - 1], az[i0 - 1]
        self.el, self.az = el, az

    def interp(self, ts, arr):
        return float(np.interp(min(max(ts, 0.0), self.t[-1]), self.t, arr))

    def att(self, ts):
        return rot(self.interp(ts, self.el), self.interp(ts, self.az))

    def sample(self, ts, h=1e-3):
        R = self.att(ts)
        fw = np.array([self.interp(ts, self.fE), self.interp(ts, self.fN), self.interp(ts, self.fU)]) if ts > 0 else \
            np.array([0.0, 0.0, self.S["g_mps2"][0]])
        fb = R.T @ fw
        if ts <= 0:
            w = np.zeros(3)
        else:
            Rd = (self.att(ts + h) - self.att(ts - h)) / (2 * h)
            W = R.T @ Rd
            w = np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]]) / 2.0
        return fb, w, R

    def alt(self, ts):
        return self.interp(ts, self.S["alt_m"]) if ts > 0 else 0.0

    def vz(self, ts):
        return self.interp(ts, self.S["vz_mps"]) if ts > 0 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("serie")
    ap.add_argument("salida")
    ap.add_argument("--verdad")
    ap.add_argument("--rampa", type=float, default=20.0)
    ap.add_argument("--t-fin", type=float, default=None, help="s de simulación (por defecto, hasta el impacto)")
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--sin-ruido", action="store_true")
    ap.add_argument("--kpid", default="0x4,0x8")
    a = ap.parse_args()
    ki, kb = (int(x, 0) for x in a.kpid.split(","))
    S, ev, head = read_series(a.serie)
    F = Flight(S, ev)
    rng = np.random.default_rng(a.semilla)
    na, ng, npa = (0.0, 0.0, 0.0) if a.sin_ruido else (0.006, 0.25, 1.6)
    ba, bg = (np.zeros(3), np.zeros(3)) if a.sin_ruido else (BIAS_A_G, BIAS_G_DPS)
    t_fin = float(S["t_s"][-1]) if a.t_fin is None else a.t_fin
    t_end = a.rampa + t_fin
    p0 = float(S["p_air_pa"][0])
    T0 = float(S["t_air_k"][0])
    events = []
    # ICM a 225 Hz desde 0.5 s
    k = 0
    t = 0.5
    seq_i = 0
    clip_n = 0
    truth = []
    while t < t_end:
        ts = t - a.rampa
        fb, w, R = F.sample(ts)
        a_g = fb / G0 + ba + rng.normal(0, na, 3)
        w_d = np.degrees(w) + bg + rng.normal(0, ng, 3)
        raw_a = np.clip(np.floor((a_g + OFF_A.astype(float)) * 8192.0 + 0.5), -32768, 32767)
        raw_g = np.clip(np.floor((w_d + OFF_G.astype(float)) * 32.8 + 0.5), -32768, 32767)
        clipped = bool(np.any(np.abs(raw_a) >= 32767))
        clip_n += clipped
        va = raw_a.astype(f32) / f32(8192.0) - OFF_A
        vg = raw_g.astype(f32) / f32(32.8) - OFF_G
        q = quat(R)
        pl = np.concatenate([va, vg, np.array(q, dtype=f32), np.array([3.0], dtype=f32)]).astype("<f4").tobytes()
        events.append((t, 0, pl))
        if k % 2 == 0:
            truth.append((t, ts, F.alt(ts), F.vz(ts), *(fb / G0), int(clipped)))
        k += 1
        t = 0.5 + k / 225.0
    # BMP a 100 Hz desde 0 s, publica tras 500 muestras
    pf = None
    psum = f32(0.0)
    tsum = f32(0.0)
    p0c = t0c = None
    n = 0
    t = 0.0
    while t < t_end:
        ts = t - a.rampa
        h = F.alt(ts)
        p = p0 * (1.0 - 0.0065 * h / T0) ** 5.25588 + rng.normal(0, npa)
        pf = p if pf is None else (pf + p) / 2.0  # IIR coeficiente 1
        tc = f32(F.interp(ts, S["t_air_k"]) - 273.15 + (0 if a.sin_ruido else rng.normal(0, 0.01)))
        pc = f32(pf)
        if n < 500:
            psum = f32(psum + pc)
            tsum = f32(tsum + tc)
            if n == 499:
                p0c = f32(psum / f32(500.0))
                t0c = f32(tsum / f32(500.0))
        else:
            tk = f32(t0c + f32(273.15))
            alt = f32(f32(tk / f32(0.0065)) * f32(f32(1.0) - np.power(f32(pc / p0c), f32(f32(1.0) / f32(5.255)))))
            events.append((t, 1, np.array([alt, pc, tc], dtype="<f4").tobytes()))
        n += 1
        t = n / 100.0
    events.sort(key=lambda e: (e[0], e[1]))
    seq = {0: 0, 1: 0}
    with open(a.salida, "wb") as fo:
        for t, kind, pl in events:
            fo.write(spp_wire.build_packet(ki if kind == 0 else kb, seq[kind] & 0xFFFF, int(math.floor(t * 1000.0)), pl))
            seq[kind] += 1
    if a.verdad:
        with open(a.verdad, "w") as fv:
            fv.write("# gen_flight.py: verdad del escenario sintético (no es un vuelo real)\n")
            for hl in head[:5]:
                fv.write(hl + "\n")
            fv.write(f"# ignición en t_placa = {a.rampa} s; ICM 225 Hz desde 0.5 s; BMP publica desde 5 s\n")
            fv.write("# hipótesis: eje z de la IMU = eje del cohete hacia la punta; R del servicio = identidad\n")
            fv.write(f"# ruido acel {na} g, giro {ng} dps, presión {npa} Pa; sesgos residuales {list(ba)} g, {list(bg)} dps\n")
            fv.write("t_placa_s,t_sim_s,alt_m,vz_mps,fx_g,fy_g,fz_g,recorte\n")
            for r in truth:
                fv.write(",".join(f"{x:.6f}" if isinstance(x, float) else str(x) for x in r) + "\n")
    print(f"{a.salida}: {seq[0]} IMU, {seq[1]} baro; recorte del acelerómetro en {clip_n} muestras "
          f"({clip_n / 225.0:.2f} s); p0 calibrada {float(p0c):.1f} Pa")
    return 0


if __name__ == "__main__":
    sys.exit(main())
