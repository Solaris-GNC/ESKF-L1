#!/usr/bin/env python3
"""log_to_replay.py — convierte un registro del Viradinha MKII en órdenes para nav_replay.

  log_to_replay.py registro.bin salida.txt [--config cfg.txt] [--map mapa.npz] [--continue]

Dos modos:
  * Registro CON navegación (perfil banco/vuelo, hay NAVEV): reproducción EXACTA de lo que hizo
    la placa. Configuración: los PARAM del propio registro (o --config). Orden: el de los NAVEV
    (el orden real en que vn_nav llamó a navigation.c) y los TICK (reloj de nav_tick). Cada
    evento usa los códigos crudos de su trama IMU (IMURAW: misma imu_conv que la placa) o la
    altitud float registrada (BAROUS). --map guarda la correspondencia para compare_board_pc.py.
    Si falta una trama necesaria (pérdidas del registro), se detiene ahí y lo dice
    (--continue sigue, pero la comparación deja de ser exacta desde ese punto).
  * Registro de ADQUISICIÓN (sin NAVEV): pasa los datos crudos por la navegación con la
    configuración de --config (p. ej. `dump_params --replay bench`). Orden por marca de tiempo
    (IMU por dato listo, baro por lectura; empate: IMU primero), como nav_events.c.

nav_replay debe compilarse con el mismo eskf_real que la placa (cabecera: sizeof_eskf_real 4 →
build/nav_replay_float).
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402

NAV_PREFIX_SKIP = ("hw.", "fw.", "doc.")
IMU_I2C_ERR = vnlog.IMU_F["I2C_ERR"]


def config_lines(log, cfg_path=None):
    """Órdenes CFG/IMUC: de --config o de los PARAM del primer bloque de cabecera del registro."""
    if cfg_path:
        with open(cfg_path) as f:
            return [l.strip() for l in f if l.strip() and not l.startswith("#")]
    params = {}
    for p in log["PARAM"]:
        params.setdefault(p["name"], p)  # primer bloque: lo vigente al arrancar
    out = []
    for name, p in params.items():
        if name.startswith(NAV_PREFIX_SKIP):
            continue
        v = p["value"]
        txt = "nan" if v != v else repr(float(v))
        if name.startswith("imu."):
            if v == v and float(v).is_integer():
                txt = str(int(v))
            out.append(f"IMUC {name} {txt}")
        else:
            if name == "eskf.has_mag_ref" and v == v:
                txt = str(int(v))
            out.append(f"CFG {name} {txt}")
    return out


def imu_cmd(r):
    if r["flags"] & IMU_I2C_ERR:
        return f"IMUNAN {int(r['t_drdy_us'])}"
    return (f"IMURAW {int(r['t_drdy_us'])} {int(r['ax'])} {int(r['ay'])} {int(r['az'])} "
            f"{int(r['gx'])} {int(r['gy'])} {int(r['gz'])}")


def baro_cmd(r):
    return f"BAROUS {int(r['t_read_us'])} {float(r['alt_m']):.9g}"


def convert(log, cfg_path=None, keep_going=False):
    """Devuelve (líneas, mapa, informe). mapa: arrays con, por cada orden tras INIT, su tipo
    (0 tick, 1 imu, 2 baro) y su índice en el registro, y por cada NAVOUT la orden previa."""
    lines = config_lines(log, cfg_path) + ["INIT"]
    rep = {"mode": None, "stopped_at_order": None, "missing": [], "navev": int(log.count("NAVEV"))}
    imu, baro = log["IMU"], log["BARO"]
    imu_by_seq = {int(s): i for i, s in enumerate(imu["seq"])} if len(imu) else {}
    baro_by_seq = {int(s): i for i, s in enumerate(baro["seq"])} if len(baro) else {}
    kinds, idxs, navout_cmd = [], [], []

    if log.count("NAVEV"):
        rep["mode"] = "exacto"
        ev, tk = log["NAVEV"], log["TICK"]
        t_navev, t_tick, t_navout = (vnlog_type(n) for n in ("NAVEV", "TICK", "NAVOUT"))
        n_ev = 0
        navout = log["NAVOUT"]
        stop = False
        for pos, (typ, k) in enumerate(zip(log.order_type, log.order_idx)):
            if typ == t_tick:
                lines.append(f"TICKUS {int(tk[k]['t_us'])}")
                kinds.append(0)
                idxs.append(k)
            elif typ == t_navev:
                e = ev[k]
                kind, seq = int(e["kind"]), int(e["src_seq"])
                table, mk = (imu_by_seq, imu_cmd) if kind == 1 else (baro_by_seq, baro_cmd)
                src = imu if kind == 1 else baro
                if seq not in table:
                    rep["missing"].append(("IMU" if kind == 1 else "BARO", seq, pos))
                    if not keep_going:
                        rep["stopped_at_order"] = pos
                        stop = True
                        break
                    continue
                lines.append(mk(src[table[seq]]))
                kinds.append(kind)
                idxs.append(k)
                n_ev += 1
            elif typ == t_navout:
                r = navout[k]
                if int(r["n_events"]) != n_ev and not keep_going:
                    rep["missing"].append(("NAVEV", int(r["n_events"]) - n_ev, pos))
                    rep["stopped_at_order"] = pos
                    stop = True
                    break
                navout_cmd.append((k, len(kinds) - 1))
            if stop:
                break
    else:
        rep["mode"] = "adquisicion"
        if not cfg_path:
            raise SystemExit("registro sin navegación: hace falta --config (p. ej. dump_params --replay bench)")
        ev = []
        for i in range(len(imu)):
            ev.append((int(imu[i]["t_drdy_us"]), 0, 1, i))
        for i in range(len(baro)):
            if not (baro[i]["flags"] & vnlog.BARO_F["I2C_ERR"]) and np.isfinite(baro[i]["alt_m"]):
                ev.append((int(baro[i]["t_read_us"]), 1, 2, i))
        ev.sort()
        for t, _, kind, i in ev:
            lines.append(imu_cmd(imu[i]) if kind == 1 else baro_cmd(baro[i]))
            kinds.append(kind)
            idxs.append(i)
    mapa = {"kind": np.array(kinds, np.int8), "idx": np.array(idxs, np.int64),
            "navout_idx": np.array([a for a, _ in navout_cmd], np.int64),
            "navout_cmd": np.array([b for _, b in navout_cmd], np.int64)}
    return lines, mapa, rep


def vnlog_type(name):
    return next(t for t, n in vnlog.REC.items() if n == name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--config")
    ap.add_argument("--map")
    ap.add_argument("--continue", dest="keep_going", action="store_true")
    a = ap.parse_args()
    log = vnlog.Log(a.log)
    lines, mapa, rep = convert(log, a.config, a.keep_going)
    with open(a.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    if a.map:
        np.savez(a.map, **mapa)
    h = log.header
    real = h["sizeof_eskf_real"] if h else "?"
    print(f"log_to_replay: modo {rep['mode']}, {len(mapa['kind'])} órdenes, eskf_real {real} B "
          f"→ usar {'nav_replay_float' if real == 4 else 'nav_replay'}")
    if rep["missing"]:
        print(f"  FALTAN {len(rep['missing'])} tramas necesarias (primera: {rep['missing'][0]})"
              + ("; detenido ahí" if rep["stopped_at_order"] is not None else "; se continúa (no exacto)"))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
