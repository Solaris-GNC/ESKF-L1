#!/usr/bin/env python3
"""flight_check.py — pasa un escenario de vuelo SINTÉTICO (tools/gen_flight.py) por el servicio
(svc_replay, perfil experimental) y resume, por fase, lo que publica frente a la verdad de la
serie OpenRocket. Es una prueba de PC con datos simulados: no valida el estimador en vuelo real.

  flight_check.py --svc build/svc_replay serie.csv [--json metricas.json] [--dir carpeta] [--rampa 20]

Vuelo ENTERO de la serie (hasta el impacto; 90,2 s nominal y 72,2 s adverso) tras 20 s en rampa.
Fases (tiempo de simulación OpenRocket): rampa (< 0), propulsión [0, apagado), planeo
[apagado, apogeo − 0.5), apogeo [apogeo ± 0.5], descenso (incluye el despliegue, hasta 2 s antes
del impacto) y aterrizaje (los 2 últimos segundos). Para cada una: validez del servicio publicada
(conjunto), motivos, capacidades, fuente, error de altura del ESKF y del respaldo barométrico
cuando se publican utilizables.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_flight  # noqa: E402
import nav_out_decode as nod  # noqa: E402

VAL = {0: "NO VÁLIDA", 1: "DEGRADADA", 2: "VÁLIDA"}
SVC_R = {0x0001: "NAV_DISABLED", 0x0002: "IMU_LOSS", 0x0004: "LATE_EVENT", 0x0008: "SAT_OUT_OF_MODEL",
         0x0010: "BARO_LOSS", 0x0020: "NO_BARO", 0x0040: "NO_ALT_REF", 0x0080: "SAT_CANDIDATE",
         0x0800: "SEQ_RESYNC", 0x4000: "BARO_REF_CHANGED", 0x8000: "RESTART_REFUSED"}
NAV_R = {0x0002: "NOT_ALIGNED", 0x0008: "TIME_GAP", 0x0010: "STALE", 0x0020: "ACC_SAT_LIMIT",
         0x1000: "ACC_SAT", 0x8000: "BARO_TIMEOUT"}


def decode(msg_path, out_id):
    snaps, _, ops, rep = nod.decode_capture(msg_path, out_id)
    if not rep["limpia"]:  # salida de svc_replay: cualquier incidencia es un error, no se oculta
        raise RuntimeError(f"salida simulada con incidencias: {rep['descartados'][:5]}")
    return snaps, ops, rep


def names(mask, table):
    return [n for b, n in table.items() if mask & b]


def run(svc, serie, work, rampa=20.0, t_fin=None, seed=1):
    S, ev, _ = gen_flight.read_series(serie)
    if t_fin is None:
        t_fin = float(S["t_s"][-1])  # hasta el impacto
    log = os.path.join(work, "vuelo.bin")
    tv = os.path.join(work, "verdad.csv")
    msg = os.path.join(work, "salida.bin")
    g = subprocess.run([sys.executable, os.path.join(HERE, "gen_flight.py"), serie, log, "--verdad", tv, "--rampa",
                        str(rampa), "--t-fin", str(t_fin), "--semilla", str(seed)], capture_output=True, text=True)
    if g.returncode:
        raise SystemExit(g.stderr)
    r = subprocess.run([svc, "--perfil", "exp", "--compacta-us", "100000", "--estado-us", "100000", "--mensajes", msg,
                        "--resumen", log], capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr)
    snaps, ops, ra = decode(msg, 0x10)
    t_sim = S["t_s"]
    alt = S["alt_m"]

    def truth_alt(tb):
        ts = tb - rampa
        return float(np.interp(ts, t_sim, alt)) if ts > 0 else 0.0

    def truth_vz(tb):
        ts = tb - rampa
        return float(np.interp(ts, t_sim, S["vz_mps"])) if ts > 0 else 0.0

    t_bo, t_ap, t_dep = ev["burnout"], ev["apogee"], ev.get("recoverydevicedeployment", 1e9)
    phases = [("rampa", -rampa, 0.0), ("propulsión", 0.0, t_bo), ("planeo", t_bo, t_ap - 0.5),
              ("apogeo", t_ap - 0.5, t_ap + 0.5), ("descenso", t_ap + 0.5, t_fin - 2.0), ("aterrizaje", t_fin - 2.0, t_fin)]
    M = {"duracion_simulada_s": t_fin,
         "eventos": {k: ev[k] for k in ("liftoff", "burnout", "apogee", "recoverydevicedeployment", "groundhit") if k in ev},
         "apogeo_verdad_m": float(np.max(alt[t_sim <= t_fin])), "fases": {}, "decodificacion": {
             "snapshots": len(snaps), "compactas": len(ops), "incompletos": ra["incompletos"],
             "crc_interno_mal": ra["crc_interno"], "crc_spp_mal": ra["crc_spp"]}}
    for name, a0, a1 in phases:
        sel = [s for s in snaps if s.get("t_state_us") is not None and a0 <= (s["t_proc_us"] * 1e-6 - rampa) < a1]
        selo = [o for o in ops if a0 <= (o["t_ms"] * 1e-3 - rampa) < a1]
        P = {"n": len(sel), "validez_servicio": sorted({VAL[s["svc_validity"]] for s in sel}),
             "validez_final": VAL[sel[-1]["svc_validity"]] if sel else None,
             "motivos_servicio": sorted({n for s in sel for n in names(s["svc_reasons"], SVC_R)}),
             "motivos_nucleo": sorted({n for s in sel for n in names(s["nav_reasons"], NAV_R)}),
             "fuente": sorted({nod.SOURCE[o["source"]] for o in selo}),
             "capacidades_finales": [n for b, n in nod.CAPS.items() if selo and selo[-1]["caps"] & b]}
        eh = [abs(-s["p"][2] - truth_alt(s["t_state_us"] * 1e-6)) for s in sel if s["svc_validity"] > 0]
        ev_ = [abs(-s["v"][2] - truth_vz(s["t_state_us"] * 1e-6)) for s in sel if s["svc_validity"] > 0]
        sv = [s["sigma_v"][2] for s in sel if s["svc_validity"] > 0]
        bh = [abs(o["bb_h_m"] - truth_alt(o["t_ms"] * 1e-3 - o["bb_age_ms"] * 1e-3)) for o in selo if o["bb_valid"]]
        bv = [abs(o["bb_vz_mps"] - truth_vz(o["t_ms"] * 1e-3 - o["bb_age_ms"] * 1e-3)) for o in selo if o["bb_valid"]]
        P["eskf_err_h_max_m"] = max(eh) if eh else None
        P["eskf_err_vz_max_mps"] = max(ev_) if ev_ else None
        P["eskf_sigma_vz_max_mps"] = max(sv) if sv else None
        P["respaldo_err_h_max_m"] = max(bh) if bh else None
        P["respaldo_err_vz_max_mps"] = max(bv) if bv else None
        P["respaldo_valido_frac"] = (sum(o["bb_valid"] for o in selo) / len(selo)) if selo else None
        M["fases"][name] = P
    fl = [s for s in snaps if s["phase"] == 1]
    M["despegue_detectado_s"] = (fl[0]["t_proc_us"] * 1e-6 - rampa) if fl else None  # resolución: snapshots a 10 Hz
    est = [(-s["p"][2], s["svc_validity"]) for s in snaps if s["svc_validity"] > 0 and s["phase"] == 1]
    M["apogeo_eskf_m"] = max(h for h, _ in est) if est else None
    M["apogeo_respaldo_m"] = max((o["bb_h_m"] for o in ops if o["bb_valid"]), default=None)
    inv = [s for s in snaps if s["phase"] == 1 and s["svc_validity"] == 0 and s["t_proc_us"] * 1e-6 - rampa < t_fin]
    sat = [i for i, s in enumerate(snaps) if s["nav_reasons"] & 0x1000]
    M["validas_tras_recorte"] = sum(1 for s in snaps[sat[0]:] if s["svc_validity"] == 2) if sat else None
    M["eskf_invalida_desde_s"] = (inv[0]["t_proc_us"] * 1e-6 - rampa) if inv else None
    M["resumen_servicio"] = dict(x.split("=", 1) for x in r.stdout.split() if "=" in x)
    return M


def report(name, M):
    print(f"== {name}")
    e = M["eventos"]
    print(f"  OpenRocket: despegue {e.get('liftoff')} s, apagado {e.get('burnout')} s, apogeo {e.get('apogee')} s "
          f"({M['apogeo_verdad_m']:.1f} m), despliegue {e.get('recoverydevicedeployment')} s, impacto {e.get('groundhit')} s; "
          f"simulado hasta {M['duracion_simulada_s']:.1f} s")
    print(f"  despegue visto en el primer snapshot (10 Hz) de t = {M['despegue_detectado_s']:.2f} s; "
          f"apogeo ESKF {M['apogeo_eskf_m'] and round(M['apogeo_eskf_m'], 1)} m, respaldo "
          f"{M['apogeo_respaldo_m'] and round(M['apogeo_respaldo_m'], 1)} m; ESKF NO VÁLIDA en vuelo: "
          + ("nunca" if M["eskf_invalida_desde_s"] is None else f"desde {M['eskf_invalida_desde_s']:.2f} s"))
    print("  (EXPERIMENTAL y NOT_VALIDATED están siempre en los motivos del servicio; no se listan)")
    for k, P in M["fases"].items():
        f = lambda x, d=2: "—" if x is None else f"{x:.{d}f}"  # noqa: E731
        print(f"  {k:10s} validez {P['validez_servicio']} fuente {P['fuente']} | ESKF |Δh| máx {f(P['eskf_err_h_max_m'])} m "
              f"|Δv_z| máx {f(P['eskf_err_vz_max_mps'])} m/s σv_z máx {f(P['eskf_sigma_vz_max_mps'])} | respaldo |Δh| máx "
              f"{f(P['respaldo_err_h_max_m'])} m |Δv_z| máx {f(P['respaldo_err_vz_max_mps'])} m/s válido "
              f"{f(P['respaldo_valido_frac'] and 100 * P['respaldo_valido_frac'], 0)} %")
        print(f"  {'':10s} motivos servicio {P['motivos_servicio']} núcleo {P['motivos_nucleo']} capacidades {P['capacidades_finales']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--svc", required=True)
    ap.add_argument("serie", nargs="+")
    ap.add_argument("--json")
    ap.add_argument("--dir")
    ap.add_argument("--rampa", type=float, default=20.0)
    a = ap.parse_args()
    out = {}
    for serie in a.serie:
        with tempfile.TemporaryDirectory() as tmp:
            work = a.dir or tmp
            if a.dir:
                work = os.path.join(a.dir, os.path.splitext(os.path.basename(serie))[0])
                os.makedirs(work, exist_ok=True)
            M = run(a.svc, serie, work, a.rampa)
        report(os.path.basename(serie), M)
        out[os.path.basename(serie)] = M
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
