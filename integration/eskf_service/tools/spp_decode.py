#!/usr/bin/env python3
"""spp_decode.py — comprobación independiente de un registro SPP (bin, hex plano o volcado de texto).

  spp_decode.py spp_dump.bin [--formato bin|hex|xxd16|xxd64] [--imu-id 8] [--baro-id 4] [--fsm-id 2]
                [--json informe.json] [--csv carpeta/]

Informa, sin ocultar nada:
  * formato, bytes de paquetes, relleno final de ceros, truncamientos, ceros intermedios,
    resincronizaciones, CRC, versión y longitudes no válidas;
  * por identificador: número, secuencias (saltos, repetidos, retrocesos), dropCounter,
    marcas de tiempo (monotonía, intervalos, huecos largos y su periodicidad en paquetes);
  * ICM: formato definitivo de 44 B (11 float32 con los offsets del productor ya restados) o
    histórico de 48 B (11 + reservado, sin offsets; --imu-formato 48). Códigos reconstruidos
    (v + offset)·divisor: enteros si el formato es el esperado; candidatos a saturación en ese
    dominio; norma del cuaternión DMP; accuracy (índice 10); reservado (índice 11, sólo 48 B);
  * BMP (12 bytes = 3 float32): en 48 B (histórico) coherencia con la ISA (p0 = 101325 Pa); en
    44 B altura RELATIVA del productor: p0 implícita (T0 = primera temperatura) y su estabilidad.
    Es sólo un diagnóstico del registro: el servicio usa la altura publicada y no reconstruye p0;
  * segmentos CANDIDATOS a reposo (ventanas de 1 s): son candidatos, no reposo confirmado.
  * --csv carpeta/: imu.csv y baro.csv con los paquetes íntegros (CRC y longitud correctos), en
    las unidades publicadas (g, °/s, m, Pa, °C), sin convertir ni corregir nada.
Código de salida: 0 sin errores de integridad; 1 si hay CRC, longitudes, truncamiento,
resincronización o ceros intermedios; 2 si el archivo no se puede leer en el formato.
"""
import argparse
import json
import math
import os
import struct
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

EXPECTED_LEN = {"fsm": 4, "baro": 12, "imu": 44}
G0 = 9.80665
# offsets que el productor SPP 9247d9e ya resta (icm20948.h); el histórico no los tenía
OFF_A = np.array([0.004621, -0.001157, 0.010014], np.float32)
OFF_G = np.array([0.681567, 0.428753, -0.329620], np.float32)


def seq_analysis(seqs):
    """Saltos, repetidos y retrocesos de una secuencia de 16 bits (con vuelta en 65535→0)."""
    gaps = dup = back = missing = 0
    first_gap = None
    for i in range(1, len(seqs)):
        d = (seqs[i] - seqs[i - 1]) & 0xFFFF
        if d == 1:
            continue
        if d == 0:
            dup += 1
        elif d < 0x8000:
            gaps += 1
            missing += d - 1
            if first_gap is None:
                first_gap = (i, seqs[i - 1], seqs[i])
        else:
            back += 1
    return {"saltos": gaps, "perdidos": missing, "repetidos": dup, "retrocesos": back, "primer_salto": first_gap}


def interval_analysis(ts, idx, long_ms):
    ts = np.asarray(ts, np.int64)
    dt = np.diff(ts)
    r = {"n": int(len(ts))}
    if len(ts) < 2:
        return r
    r["t0_ms"] = int(ts[0])
    r["t1_ms"] = int(ts[-1])
    r["no_monotonas"] = int((dt < 0).sum())
    r["iguales"] = int((dt == 0).sum())
    span = (ts[-1] - ts[0]) / 1000.0
    r["frecuencia_media_hz"] = (len(ts) - 1) / span if span > 0 else None
    u, c = np.unique(dt, return_counts=True)
    r["histograma_ms"] = {int(a): int(b) for a, b in zip(u, c)}
    lg = np.nonzero(dt > long_ms)[0]
    r["largos"] = [{"tras_paquete": int(idx[k + 1]), "t_ms": int(ts[k]), "dt_ms": int(dt[k])} for k in lg]
    norm = dt[(dt > 0) & (dt <= long_ms)]
    if len(norm):
        r["frecuencia_sin_largos_hz"] = 1000.0 / norm.mean()
    if len(lg) >= 2:
        r["paquetes_entre_largos"] = sorted(set(int(x) for x in np.diff(np.asarray(idx)[lg + 1])))
        r["ms_entre_largos"] = sorted(set(int(x) for x in np.diff(ts[lg])))
        if len(norm):
            per = norm.mean()
            r["periodos_que_faltan_por_hueco"] = [round(float(dt[k] / per - 1), 2) for k in lg]
    return r


def quant(x, lsbs):
    """Fracción de valores que son múltiplos exactos de 1/lsb (pista de la escala; no confirmación)."""
    x = np.asarray(x, np.float64).ravel()
    out = {}
    for s in lsbs:
        y = x * s  # float32 publicado (y + offset): tolerancia de unos ulp, mínimo 0.01 LSB
        tol = np.maximum(4 * 2.0 ** -23 * np.maximum(np.abs(y), 1.0), 0.01)
        out[str(s)] = float(np.mean(np.abs(y - np.round(y)) <= tol))
    return out


def analyze(path, fmt=None, ids=None, out_id=None, imu_fmt=None):
    ids = ids or {"imu": 8, "baro": 4, "fsm": 2}
    data, f, load_rep = spp_wire.load(path, fmt)
    pkts, issues, rep = spp_wire.split_packets(data)
    R = {"archivo": path, "formato": f, **load_rep, **rep}
    by_name = {v: k for k, v in ids.items()}
    if out_id is not None:
        by_name[out_id] = "salida_eskf"
    cnt = Counter(p["id"] for p in pkts)
    R["paquetes"] = len(pkts)
    R["por_id"] = {f"0x{k:04x}" + (f" ({by_name[k]})" if k in by_name else ""): v for k, v in sorted(cnt.items())}
    R["crc_ok"] = sum(p["crc_ok"] for p in pkts)
    R["crc_mal"] = sum(not p["crc_ok"] for p in pkts)
    R["versiones"] = sorted({p["ver"] for p in pkts})
    R["drop_counter"] = {"no_nulos": sum(1 for p in pkts if p["drop"]), "max": max((p["drop"] for p in pkts), default=0)}
    R["incidencias"] = issues
    R["ids_desconocidos"] = sorted(f"0x{k:04x}" for k in cnt if k not in by_name)
    lens = defaultdict(Counter)
    for p in pkts:
        lens[p["id"]][p["len"]] += 1
    R["longitudes"] = {f"0x{k:04x}": dict(v) for k, v in lens.items()}
    if imu_fmt is None:  # el formato de la IMU se deduce de la longitud más frecuente (44 o 48)
        li = lens.get(ids["imu"], Counter())
        imu_fmt = 48 if li.get(48, 0) > li.get(44, 0) else 44
    R["imu_formato"] = imu_fmt
    expected = dict(EXPECTED_LEN, imu=imu_fmt)
    bad_len = []
    for name, ident in ids.items():
        for L, c in lens.get(ident, {}).items():
            if L != expected[name]:
                bad_len.append({"id": name, "len": L, "n": c})
    R["longitudes_inesperadas"] = bad_len
    fill = Counter()
    for p in pkts:
        fb = set(p["filler"])
        fill["0xff" if fb <= {0xFF} else "0x00" if fb <= {0} else "mezcla"] += 1 if p["filler"] else 0
    R["relleno_payload"] = dict(fill)

    good = [p for p in pkts if p["crc_ok"] and p["len_ok"]]
    R["ids"] = {}
    for name, ident in ids.items():
        ps = [p for p in good if p["id"] == ident]
        e = {"n": len(ps)}
        if ps:
            seqs = [p["seq"] for p in ps]
            e["seq_primera"], e["seq_ultima"] = seqs[0], seqs[-1]
            e["secuencia"] = seq_analysis(seqs)
            e["tiempos"] = interval_analysis([p["ts"] for p in ps], [p["idx"] for p in ps],
                                             long_ms=10 if name == "imu" else 20)
        R["ids"][name] = e

    # ICM: 11 float32 (+ reservado en el histórico)
    imu = [p for p in good if p["id"] == ids["imu"] and p["len"] == imu_fmt]
    if imu:
        nf = imu_fmt // 4
        F = np.array([struct.unpack(f"<{nf}f", p["payload"]) for p in imu], np.float64)
        a, w, q = F[:, 0:3], F[:, 3:6], F[:, 6:10]
        oa = OFF_A if imu_fmt == 44 else np.zeros(3, np.float32)
        og = OFF_G if imu_fmt == 44 else np.zeros(3, np.float32)
        I = {"n": len(imu), "no_finitos": int((~np.isfinite(F)).sum()),
             "offsets_supuestos": {"acel_g": oa.tolist(), "giro_dps": og.tolist()}}
        I["acel_min"], I["acel_max"] = a.min(0).tolist(), a.max(0).tolist()
        I["giro_min"], I["giro_max"] = w.min(0).tolist(), w.max(0).tolist()
        # dominio de códigos del productor: (v + offset) · divisor, en float32 como el productor
        ca = (a.astype(np.float32) + oa).astype(np.float64)
        cg = (w.astype(np.float32) + og).astype(np.float64)
        I["acel_cuantizacion_lsb_por_g"] = quant(ca, (2048, 4096, 8192, 16384))
        I["giro_cuantizacion_lsb_por_dps"] = quant(cg, (16.4, 32.8, 65.5, 131.0))
        codes_a = ca * 8192.0
        ext = np.abs(codes_a) >= 32768.0 - 1.5
        I["acel_candidatos_extremo_4g"] = {"n": int(ext.sum()),
                                          "valores": sorted({float(x) for x in a[ext]}),
                                          "por_eje": ext.sum(0).tolist(),
                                          "criterio": "|(v + offset)·8192| >= 32766.5"}
        codes_g = cg * 32.8
        extg = np.abs(codes_g) >= 32768.0 - 1.5
        I["giro_candidatos_extremo"] = {"n": int(extg.sum()), "por_eje": extg.sum(0).tolist()}
        I["fuera_de_dominio"] = int((np.abs(codes_a) > 32768.5).sum() + (np.abs(codes_g) > 32768.5).sum())
        qn = np.linalg.norm(q, axis=1)
        I["cuaternion_norma"] = [float(qn.min()), float(np.median(qn)), float(qn.max())]
        u, c = np.unique(F[:, 10], return_counts=True)
        I["indice10"] = {"min": float(u.min()), "max": float(u.max()), "enteros": bool(np.all(u == np.round(u))),
                         "mas_frecuente": float(u[np.argmax(c)]), "distintos": int(len(u)),
                         "significado": "accuracy del DMP (int16 de la FIFO, icm20948.c); escala sin confirmar"}
        I["indice11_no_cero"] = int((F[:, 11] != 0).sum()) if nf > 11 else None
        an = np.linalg.norm(a, axis=1)
        I["norma_acel_g_pct"] = dict(zip(("p1", "p50", "p99"), np.percentile(an, [1, 50, 99]).tolist()))
        # segmentos candidatos a reposo: ventanas de 1 s
        t = np.array([p["ts"] for p in imu], np.float64) / 1000.0
        segs = []
        t0 = t[0]
        while t0 < t[-1]:
            m = (t >= t0) & (t < t0 + 1.0)
            if m.sum() > 50:
                wn = np.linalg.norm(w[m], axis=1)
                segs.append({"t0_s": round(float(t0), 3), "n": int(m.sum()), "norma_acel_g_std": float(an[m].std()),
                             "giro_max_dps": float(wn.max()), "acel_media_g": a[m].mean(0).round(4).tolist()})
            t0 += 1.0
        quiet = [s for s in segs if s["giro_max_dps"] < 2.0 and s["norma_acel_g_std"] < 0.01]
        runs = []
        for s in quiet:
            if runs and abs(s["t0_s"] - runs[-1][1]) < 1e-6:
                runs[-1][1] = s["t0_s"] + 1.0
            else:
                runs.append([s["t0_s"], s["t0_s"] + 1.0])
        I["candidatos_reposo"] = {"criterio": "ventanas de 1 s con ‖ω‖ máx < 2 °/s y σ(‖a‖) < 0.01 g "
                                              "(sólo candidatos; hay que confirmar las condiciones del ensayo)",
                                  "tramos_s": [[round(x, 3) for x in r] for r in runs]}
        I["ventanas_1s"] = segs
        R["icm"] = I
    baro = [p for p in good if p["id"] == ids["baro"] and p["len"] == 12]
    if baro:
        B = np.array([struct.unpack("<3f", p["payload"]) for p in baro], np.float64)
        R["bmp"] = {"n": len(baro), "no_finitos": int((~np.isfinite(B)).sum()),
                    "altura_m": [float(B[:, 0].min()), float(B[:, 0].max())],
                    "presion_pa": [float(B[:, 1].min()), float(B[:, 1].max())],
                    "temperatura_c": [float(B[:, 2].min()), float(B[:, 2].max())],
                    "consecutivos_identicos": int(np.sum(np.all(B[1:] == B[:-1], axis=1)))}
        if imu_fmt == 48:  # histórico: altura ISA absoluta
            isa = 44330.0 * (1.0 - (B[:, 1] / 101325.0) ** 0.1903)
            R["bmp"]["semantica"] = "ISA absoluta (p0 = 101325 Pa), histórica"
            R["bmp"]["altura_menos_isa_m"] = [float(np.min(B[:, 0] - isa)), float(np.max(B[:, 0] - isa))]
        else:  # definitivo: relativa a p0/T0 del productor (500 muestras)
            t0k = B[0, 2] + 273.15
            p0 = B[:, 1] / (1.0 - B[:, 0] * 0.0065 / t0k) ** 5.255
            R["bmp"]["semantica"] = "relativa a p0/T0 del productor (bmp390.c)"
            R["bmp"]["p0_implicita_pa"] = [float(p0.min()), float(np.median(p0)), float(p0.max())]
    fsm = [p for p in good if p["id"] == ids["fsm"]]
    if fsm:
        R["fsm"] = [{"ts": p["ts"], "payload_hex": p["payload"].hex()} for p in fsm[:20]]
    R["errores_integridad"] = (R["crc_mal"] > 0 or bool(bad_len) or
                               any(i["tipo"] in ("truncado", "resincronizacion", "ceros_intermedios", "longitud")
                                   for i in issues))
    return R, pkts


def report(R, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    p(f"{R['archivo']}  formato {R['formato']}  {R['bytes_total']} bytes")
    p(f"  paquetes {R['paquetes']} ({R['bytes_paquetes']} bytes), relleno final de ceros {R['relleno_final_ceros']} bytes")
    p(f"  por identificador: {R['por_id']}  versiones {R['versiones']}")
    p(f"  CRC correctos {R['crc_ok']}, incorrectos {R['crc_mal']}; dropCounter no nulo {R['drop_counter']['no_nulos']}")
    if R["ids_desconocidos"]:
        p(f"  identificadores no configurados: {R['ids_desconocidos']}")
    for i in R["incidencias"][:20]:
        p(f"  INCIDENCIA {i}")
    if len(R["incidencias"]) > 20:
        p(f"  … {len(R['incidencias']) - 20} incidencias más")
    for b in R["longitudes_inesperadas"]:
        p(f"  LONGITUD INESPERADA {b}")
    for name, e in R["ids"].items():
        if not e["n"]:
            p(f"  {name}: sin paquetes")
            continue
        s, t = e["secuencia"], e["tiempos"]
        p(f"  {name}: {e['n']} paquetes, secuencia {e['seq_primera']}–{e['seq_ultima']}, saltos {s['saltos']} "
          f"(perdidos {s['perdidos']}), repetidos {s['repetidos']}, retrocesos {s['retrocesos']}")
        if t["n"] > 1:
            fr = t.get("frecuencia_sin_largos_hz")
            p(f"     marcas {t['t0_ms']}–{t['t1_ms']} ms, {t['frecuencia_media_hz']:.2f} Hz de media"
              + (f", {fr:.2f} Hz sin intervalos largos" if fr else "")
              + f"; no monótonas {t['no_monotonas']}, iguales {t['iguales']}")
            p(f"     intervalos (ms: n): {t['histograma_ms']}")
            if t["largos"]:
                p(f"     {len(t['largos'])} intervalos largos: " +
                  ", ".join(f"{x['dt_ms']} ms en t={x['t_ms']}" for x in t["largos"][:6]) +
                  (" …" if len(t["largos"]) > 6 else ""))
                if "paquetes_entre_largos" in t:
                    p(f"     separación en paquetes (de todos los ids): {t['paquetes_entre_largos']}, en ms: "
                      f"{t['ms_entre_largos']}")
                    if "periodos_que_faltan_por_hueco" in t:
                        p(f"     periodos que faltan por hueco (estimado): {t['periodos_que_faltan_por_hueco']}")
    if "icm" in R:
        I = R["icm"]
        p(f"  ICM: {I['n']} muestras, no finitos {I['no_finitos']}")
        p(f"     acel (g?) min {np.round(I['acel_min'], 4).tolist()} max {np.round(I['acel_max'], 4).tolist()}")
        p(f"     giro (°/s?) min {np.round(I['giro_min'], 2).tolist()} max {np.round(I['giro_max'], 2).tolist()}")
        p(f"     cuantización acel (fracción múltiplo de 1/LSB): {I['acel_cuantizacion_lsb_por_g']}")
        p(f"     cuantización giro: {I['giro_cuantizacion_lsb_por_dps']}")
        c = I["acel_candidatos_extremo_4g"]
        p(f"     formato {R['imu_formato']} B, offsets supuestos {I['offsets_supuestos']}; fuera del dominio de códigos "
          f"{I['fuera_de_dominio']}")
        p(f"     candidatos a saturación ({c['criterio']}): {c['n']} {c['valores']} por eje {c['por_eje']}; giro "
          f"{I['giro_candidatos_extremo']}")
        p(f"     cuaternión: norma min/med/max {I['cuaternion_norma']}; índice 10: {I['indice10']}; "
          f"índice 11 no cero: {I['indice11_no_cero']}")
        p(f"     candidatos a reposo: {I['candidatos_reposo']['tramos_s']}  ({I['candidatos_reposo']['criterio']})")
    if "bmp" in R:
        B = R["bmp"]
        extra = (f"altura − ISA(p) {np.round(B['altura_menos_isa_m'], 4).tolist()} m" if "altura_menos_isa_m" in B
                 else f"p0 implícita min/med/max {np.round(B['p0_implicita_pa'], 1).tolist()} Pa")
        p(f"  BMP ({B['semantica']}): {B['n']} muestras; altura {B['altura_m']}, presión {B['presion_pa']}, "
          f"T {B['temperatura_c']}; {extra}; consecutivos idénticos {B['consecutivos_identicos']}")
    if "fsm" in R:
        p(f"  FSM: {R['fsm']}")
    p("  RESULTADO: " + ("ERRORES DE INTEGRIDAD (ver arriba)" if R["errores_integridad"] else "sin errores de integridad"))


def write_csv(folder, pkts, imu_id, baro_id, imu_fmt):
    """imu.csv y baro.csv con los paquetes íntegros, en el orden del registro y las unidades publicadas."""
    import csv
    os.makedirs(folder, exist_ok=True)
    n_i = n_b = 0
    with open(os.path.join(folder, "imu.csv"), "w", newline="") as fi, \
            open(os.path.join(folder, "baro.csv"), "w", newline="") as fb:
        wi, wb = csv.writer(fi), csv.writer(fb)
        hi = ["seq", "ts_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "dmp_qw", "dmp_qx", "dmp_qy", "dmp_qz",
              "dmp_accuracy"]
        wi.writerow(hi + (["reservado"] if imu_fmt == 48 else []))
        wb.writerow(["seq", "ts_ms", "alt_rel_m", "p_pa", "t_c"])
        for p in pkts:
            if not (p["crc_ok"] and p["len_ok"]):
                continue
            if p["id"] == imu_id and p["len"] == imu_fmt:
                v = struct.unpack_from(f"<{imu_fmt // 4}f", p["payload"], 0)
                wi.writerow([p["seq"], p["ts"], *(repr(float(x)) for x in v)])
                n_i += 1
            elif p["id"] == baro_id and p["len"] == 12:
                v = struct.unpack_from("<3f", p["payload"], 0)
                wb.writerow([p["seq"], p["ts"], *(repr(float(x)) for x in v)])
                n_b += 1
    return n_i, n_b


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("archivo")
    ap.add_argument("--formato", choices=["bin", "hex", "xxd16", "xxd64"])
    ap.add_argument("--imu-id", type=lambda s: int(s, 0), default=8)
    ap.add_argument("--baro-id", type=lambda s: int(s, 0), default=4)
    ap.add_argument("--fsm-id", type=lambda s: int(s, 0), default=2)
    ap.add_argument("--out-id", type=lambda s: int(s, 0), help="KPID de la salida del servicio (sólo se cuenta)")
    ap.add_argument("--imu-formato", type=int, choices=[44, 48], help="44 definitivo, 48 histórico (por defecto: se deduce)")
    ap.add_argument("--json")
    ap.add_argument("--csv", metavar="CARPETA", help="escribe imu.csv y baro.csv (paquetes íntegros, unidades publicadas)")
    a = ap.parse_args()
    try:
        R, pkts = analyze(a.archivo, a.formato, {"imu": a.imu_id, "baro": a.baro_id, "fsm": a.fsm_id}, a.out_id,
                          a.imu_formato)
    except spp_wire.FormatError as e:
        print(f"ERROR de formato: {e}")
        return 2
    report(R)
    if a.csv:
        n_i, n_b = write_csv(a.csv, pkts, a.imu_id, a.baro_id, R["imu_formato"])
        print(f"  CSV: {n_i} IMU y {n_b} baro en {a.csv}")
    if a.json:
        R2 = dict(R)
        R2.get("icm", {}).pop("ventanas_1s", None)
        with open(a.json, "w") as f:
            json.dump(R2, f, indent=1, default=str)
    return 1 if R["errores_integridad"] else 0


if __name__ == "__main__":
    sys.exit(main())
