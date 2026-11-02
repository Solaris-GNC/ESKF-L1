#!/usr/bin/env python3
"""ork_extract.py — extrae la serie temporal de una simulación OpenRocket (.ork) a un CSV con
procedencia, para los escenarios de vuelo de las pruebas (tools/gen_flight.py).

  ork_extract.py PUMEL_v3-7_Klima.ork "Simulation 6 - Nominal" salida.csv [--t-max S]   (por defecto: la serie entera)
  ork_extract.py PUMEL_v3-7_Klima.ork --listar

Sólo lee; no vuelve a simular nada. El .ork es un zip con un XML: <simulation><name>…
<flightdata><databranch types="…"><datapoint>…</datapoint><event time type/>. Las columnas se
buscan por su nombre en inglés (OpenRocket 24.12). Unidades SI; ángulos en radianes.

Comprobaciones de convenciones (se imprimen y se guardan en la cabecera del CSV; el programa
falla si no se cumplen):
  * «Vertical orientation (zenith)» es la ELEVACIÓN del eje (π/2 = vertical): en rampa vale
    90° − ángulo de la guía y en la propulsión difiere del ángulo de la trayectoria RESPECTO DEL
    AIRE, atan2(v_z, v_h + viento) (viento de cara: lanzamiento contra el viento), exactamente
    en el ángulo de ataque que publica OpenRocket;
  * «Lateral orientation (azimuth)» se mide desde el norte hacia el este: el desplazamiento
    horizontal durante la propulsión va en esa dirección;
  * «Vertical acceleration» es d(v_z)/dt y «Vertical velocity» es d(altitud)/dt (integración);
  * «Total acceleration»² = vertical² + lateral² («Lateral acceleration» es la horizontal).
"""
import argparse
import hashlib
import math
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

COLS = [  # (nombre en el CSV, columna OpenRocket)
    ("t_s", "Time"), ("alt_m", "Altitude"), ("vz_mps", "Vertical velocity"), ("az_kin_mps2", "Vertical acceleration"),
    ("a_lat_mps2", "Lateral acceleration"), ("a_tot_mps2", "Total acceleration"), ("pos_e_m", "Position East of launch"),
    ("pos_n_m", "Position North of launch"), ("v_lat_mps", "Lateral velocity"), ("el_rad", "Vertical orientation (zenith)"),
    ("azim_rad", "Lateral orientation (azimuth)"), ("roll_rate", "Roll rate"), ("pitch_rate", "Pitch rate"),
    ("yaw_rate", "Yaw rate"), ("g_mps2", "Gravitational acceleration"), ("p_air_pa", "Air pressure"),
    ("t_air_k", "Air temperature"), ("thrust_n", "Thrust"), ("mass_kg", "Mass"), ("wind_mps", "Wind velocity"),
    ("wind_dir_rad", "Wind direction"), ("aoa_rad", "Angle of attack"),
]


def load(path):
    raw = open(path, "rb").read()
    with zipfile.ZipFile(path) as z:
        name = [n for n in z.namelist() if n.endswith(".ork") or n.endswith(".xml")][0]
        root = ET.fromstring(z.read(name))
    return root, hashlib.sha256(raw).hexdigest()


def find_sim(root, prefix):
    for sim in root.iter("simulation"):
        n = sim.find("name").text or ""
        if n.startswith(prefix):
            return sim, n
    raise SystemExit(f"no hay simulación que empiece por «{prefix}»")


def series(sim):
    db = sim.find("flightdata").find("databranch")
    types = db.attrib["types"].split(",")
    idx = {n: i for i, n in enumerate(types)}
    rows = [[float(x) for x in p.text.split(",")] for p in db.findall("datapoint")]
    D = np.array(rows)
    out = {k: D[:, idx[c]] for k, c in COLS}
    ev = [(float(e.attrib["time"]), e.attrib["type"]) for e in db.findall("event")]
    return out, ev, sim.find("flightdata").attrib


ROD_DEG = [0.0]


def check_conventions(S, ev):
    """Devuelve una lista de (descripción, valor, ok)."""
    t = S["t_s"]
    res = []
    rod = next(tt for tt, k in ev if k == "launchrod")
    bo = next(tt for tt, k in ev if k == "burnout")
    dep = next((tt for tt, k in ev if k == "recoverydevicedeployment"), t[-1])
    # elevación: en rampa = 90° − guía; en propulsión |elevación − trayectoria respecto del aire| = ángulo de ataque
    d0 = abs(math.degrees(S["el_rad"][0]) - (90.0 - ROD_DEG[0]))
    res.append(("elevación inicial = 90° − ángulo de la guía (|dif|, °)", d0, d0 < 0.1))
    m = (t > rod) & (t < bo)
    gam = np.arctan2(S["vz_mps"][m], S["v_lat_mps"][m] + S["wind_mps"][m])
    d = np.degrees(np.max(np.abs(np.abs(S["el_rad"][m] - gam) - S["aoa_rad"][m])))
    res.append(("| |elevación − trayectoria respecto del aire| − ángulo de ataque | en propulsión (máx, °)", d, d < 1.0))
    # acimut: desplazamiento horizontal al apagado en la dirección del acimut (norte → este)
    i = np.argmin(np.abs(t - bo))
    ang = math.degrees(math.atan2(S["pos_e_m"][i], S["pos_n_m"][i]))
    az = math.degrees(S["azim_rad"][i])
    dd = abs((ang - az + 180) % 360 - 180)
    res.append(("acimut desde el norte hacia el este: dirección del desplazamiento − acimut (°)", dd, dd < 5.0))
    # integración: vz desde az, altitud desde vz
    vz_int = S["vz_mps"][0] + np.concatenate([[0], np.cumsum(0.5 * (S["az_kin_mps2"][1:] + S["az_kin_mps2"][:-1]) * np.diff(t))])
    m2 = t < dep - 0.01
    e1 = np.max(np.abs(vz_int[m2] - S["vz_mps"][m2]))
    res.append(("∫ aceleración vertical = velocidad vertical hasta el despliegue (máx |dif|, m/s)", e1, e1 < 1.5))
    h_int = np.concatenate([[0], np.cumsum(0.5 * (S["vz_mps"][1:] + S["vz_mps"][:-1]) * np.diff(t))])
    e2 = np.max(np.abs(h_int[m2] - S["alt_m"][m2]))
    res.append(("∫ velocidad vertical = altitud (máx |dif|, m)", e2, e2 < 2.0))
    tot = np.sqrt(S["az_kin_mps2"] ** 2 + S["a_lat_mps2"] ** 2)
    e3 = np.max(np.abs(tot - S["a_tot_mps2"]) / np.maximum(S["a_tot_mps2"], 1.0))
    res.append(("total² = vertical² + lateral² (máx error relativo)", e3, e3 < 0.01))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ork")
    ap.add_argument("simulacion", nargs="?")
    ap.add_argument("salida", nargs="?")
    ap.add_argument("--t-max", type=float, default=float("inf"))
    ap.add_argument("--listar", action="store_true")
    a = ap.parse_args()
    root, sha = load(a.ork)
    if a.listar or not a.salida:
        for sim in root.iter("simulation"):
            print(sim.attrib.get("status"), sim.find("name").text)
        return 0
    sim, name = find_sim(root, a.simulacion)
    S, ev, fd = series(sim)
    conds = {c.tag: (c.text or "").strip() for c in sim.find("conditions") if (c.text or "").strip()}
    ROD_DEG[0] = float(conds.get("launchrodangle", "0"))
    chk = check_conventions(S, ev)
    ok = all(x[2] for x in chk)
    for d, v, k in chk:
        print(f"  [{'OK' if k else 'FALLO'}] {d}: {v:.4g}")
    g0 = 9.80665
    f = np.sqrt((S["az_kin_mps2"] + S["g_mps2"]) ** 2 + S["a_lat_mps2"] ** 2) / g0
    i = int(np.nanargmax(f))
    over = S["t_s"][f > 4.0]
    m = S["t_s"] <= a.t_max
    with open(a.salida, "w") as fo:
        fo.write(f"# fuente: {a.ork.split('/')[-1]} sha256 {sha}\n")
        fo.write(f"# simulación: {name} (OpenRocket; resultados guardados en el .ork, sin volver a simular)\n")
        fo.write("# condiciones: " + "; ".join(f"{k}={v}" for k, v in conds.items() if k in (
            "launchrodlength", "launchrodangle", "launchroddirection", "windaverage", "windturbulence", "winddirection",
            "launchaltitude", "timestep")) + "\n")
        fo.write("# resumen: " + "; ".join(f"{k}={v}" for k, v in fd.items()) + "\n")
        fo.write("# eventos: " + "; ".join(f"{k}={tt:g}" for tt, k in ev) + "\n")
        fo.write(f"# fuerza específica máx {f[i]:.2f} g a {S['t_s'][i]:.3f} s; > 4 g de {over.min():.3f} a "
                 f"{over[over < 2].max():.3f} s en la propulsión" + (
                     f" y {len(over[over >= 2])} filas después" if np.any(over >= 2) else "") + "\n")
        for d, v, k in chk:
            fo.write(f"# convención {'OK' if k else 'FALLO'}: {d}: {v:.4g}\n")
        fo.write(("# serie completa hasta el impacto" if not math.isfinite(a.t_max) else f"# filas con t <= {a.t_max:g} s")
                 + f" ({S['t_s'][m][-1]:g} s); ángulos en rad; SI\n")
        fo.write(",".join(k for k, _ in COLS) + "\n")
        for r in range(len(S["t_s"])):
            if not m[r]:
                continue
            fo.write(",".join("nan" if math.isnan(S[k][r]) else repr(float(S[k][r])) for k, _ in COLS) + "\n")
    print(f"{name}: {int(m.sum())} filas → {a.salida}; f máx {f[i]:.2f} g a {S['t_s'][i]:.3f} s")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
