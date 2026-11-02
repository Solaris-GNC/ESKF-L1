#!/usr/bin/env python3
"""nav_out_decode.py — decodificador INDEPENDIENTE (Python) de la salida del servicio (esquema v4).

  nav_out_decode.py registro.bin --out-id 0x10 [--formato bin|hex|xxd16|xxd64] [--csv snaps.csv]
                    [--diag-csv d.csv] [--ops-csv o.csv] [--tolerante]

Lee los paquetes SPP del KPID de salida, reconstruye los snapshots (fragmentos 1–5), los
diagnósticos (tipo 6) y la salida operativa compacta (tipo 7, autónoma) y cuenta, sin ocultarlos:

  * lector SPP (capa EXTERIOR, cualquier KPID): paquetes rechazados por CRC SPP (y cuántos llevaban
    el KPID de salida), por versión, longitud imposible, resincronizaciones, ceros intermedios y
    final truncado; huecos y repeticiones en la secuencia SPP de la salida;
  * mensajes (capa INTERIOR): CRC interno incorrecto, snapshots incompletos (llega otro antes de
    completar, o sin cerrar al final), mezclados (misma sesión e id con CRC/banderas/máscara
    distintos) e inválidos.

Por defecto es una COMPROBACIÓN: código 1 ante cualquier corrupción o captura incompleta (también
escribe los CSV de lo que sí se pudo decodificar). --tolerante es el modo de RESCATE: escribe lo
decodificado, enumera lo descartado y devuelve 3 si se descartó algo (0 si la captura está limpia).

Una selección sin nada que comprobar NUNCA es un éxito: código 2 si el archivo no tiene datos, si el
KPID pedido no aparece (dice qué KPID hay) o si sus paquetes no dan ningún mensaje decodificable (si
además hay corrupción, manda el código 1). Código 0 sólo con al menos un mensaje comprobado.
Esquema: docs/SALIDA.md.
"""
import argparse
import csv
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spp_wire  # noqa: E402

VERSION = 4
HDR = 12
LEN = {1: 38, 2: 32, 3: 28, 4: 24, 5: 24, 6: 38, 7: 38}
FLAGS = {0x01: "marcas_ms", 0x02: "navega", 0x04: "experimental", 0x08: "estricto", 0x10: "aceptado"}
CAPS = {0x01: "actitud", 0x02: "ref_baro", 0x04: "nav_eskf", 0x08: "respaldo_baro", 0x10: "ref_lanzamiento"}
SOURCE = {0: "ninguna", 1: "eskf", 2: "baro"}
NAN = float("nan")


def crc_chain(chunks):
    c = 0xFFFF
    for b in chunks:
        c = spp_wire.binascii.crc_hqx(b, c)
    return c


def decode_snapshot(data, mask):
    s = {k: NAN for k in ("alt_ref",)}
    for k in ("p", "v", "sigma_th", "sigma_p", "sigma_v", "b_a", "b_g"):
        s[k] = (NAN,) * 3
    s["q"] = (NAN,) * 4
    if mask & 1:
        b = data[1]
        s["t_state_us"], s["t_eval_us"], s["t_proc_us"] = struct.unpack_from("<qqq", b, 0)
        (s["nav_reasons"], s["svc_reasons"], s["readiness"], s["why_code"], s["config_id"]) = struct.unpack_from("<5H", b, 24)
        s["nav_validity"], s["svc_validity"] = b[34] & 0x0F, b[34] >> 4
        s["phase"] = 2 if (b[35] >> 3) & 1 else b[35] & 1  # 0 rampa confirmada, 1 vuelo, 2 incierta (v4)
        s["at_rest"], s["heading_known"] = (b[35] >> 1) & 1, (b[35] >> 2) & 1
        s["profile"], s["time_source"] = (b[35] >> 4) & 3, (b[35] >> 6) & 3
        s["caps"], s["source"] = b[36], b[37]
        for k in ("t_state_us", "t_eval_us", "t_proc_us"):
            if s[k] == -(1 << 63):
                s[k] = None
    if mask & 2:
        f = struct.unpack_from("<7fI", data[2], 0)
        s["p"], s["v"], s["alt_ref"], s["alt_ref_seq"] = f[0:3], f[3:6], f[6], f[7]
    if mask & 4:
        f = struct.unpack_from("<7f", data[3], 0)
        s["q"], s["sigma_th"] = f[0:4], f[4:7]
    if mask & 8:
        f = struct.unpack_from("<6f", data[4], 0)
        s["sigma_p"], s["sigma_v"] = f[0:3], f[3:6]
    if mask & 16:
        f = struct.unpack_from("<6f", data[5], 0)
        s["b_a"], s["b_g"] = f[0:3], f[3:6]
    return s


def decode_diag(m):
    b = m[HDR:]
    d = dict(zip(("imu_rx", "baro_rx", "imu_lost", "baro_lost"), struct.unpack_from("<4I", b, 0)))
    (d["rejected"], d["sat_cand"]) = struct.unpack_from("<2H", b, 16)
    (d["last_imu_ret"], d["last_baro_ret"], d["last_tick_ret"]) = struct.unpack_from("<3h", b, 20)
    (d["nav_gaps"], d["out_fail"], d["skipped"]) = struct.unpack_from("<3H", b, 26)
    d["mailbox_hwm"], d["seq_resyncs"] = b[32], b[33]
    d["lat_rx_max_ms"], d["lat_proc_max_ms"] = struct.unpack_from("<2H", b, 34)
    d["session"], d["diag_id"] = struct.unpack_from("<HI", m, 4)
    d["banderas"] = [n for bit, n in FLAGS.items() if m[3] & bit]
    return d


def decode_ops(m):
    b = m[HDR:]
    o = {"session": struct.unpack_from("<H", m, 4)[0], "ops_id": struct.unpack_from("<I", m, 6)[0]}
    o["t_ms"], o["eskf_age_ms"], o["bb_age_ms"] = struct.unpack_from("<IHH", b, 0)
    (o["nav_reasons"], o["svc_reasons"], o["readiness"], o["config_id"], o["boot_id"]) = struct.unpack_from("<5H", b, 8)
    o["nav_validity"], o["svc_validity"], o["caps"] = b[18] & 0x0F, b[18] >> 4, b[19]
    o["source"], o["profile"] = b[20] & 3, (b[20] >> 4) & 3
    o["phase"] = 2 if (b[20] >> 3) & 1 else (b[20] >> 2) & 1  # 0 rampa confirmada, 1 vuelo, 2 incierta (v4)
    o["bb_valid"], o["bb_ref_frozen"] = (b[20] >> 6) & 1, (b[20] >> 7) & 1
    o["eskf_h_m"], o["eskf_vz_mps"], o["bb_h_m"], o["bb_vz_mps"] = struct.unpack_from("<4f", b, 22)
    o["banderas"] = [n for bit, n in FLAGS.items() if m[3] & bit]
    return o


class Reasm:
    def __init__(self):
        self.cur = None
        self.complete = self.incomplete = self.crc_bad = self.mixed = self.invalid = self.diag = self.ops = 0

    def push(self, m):
        """Devuelve ('snap', dict), ('diag', dict), ('ops', dict) o None."""
        if len(m) < HDR or m[0] != VERSION or m[1] not in LEN or len(m) != HDR + LEN[m[1]]:
            self.invalid += 1
            return None
        typ, mask, flags = m[1], m[2], m[3]
        session, ident, crc = struct.unpack_from("<HIH", m, 4)
        if typ in (6, 7):  # independientes: no tocan el snapshot en curso
            if mask != 1 << (typ - 1) or spp_wire.binascii.crc_hqx(bytes(m[HDR:]), 0xFFFF) != crc:
                self.crc_bad += 1
                return None
            if typ == 6:
                self.diag += 1
                return "diag", decode_diag(m)
            self.ops += 1
            return "ops", decode_ops(m)
        if not (mask >> (typ - 1)) & 1 or mask & ~0x1F:
            self.invalid += 1
            return None
        key = (session, ident)
        if self.cur and self.cur["key"] != key:
            self.incomplete += 1
            self.cur = None
        if not self.cur:
            self.cur = {"key": key, "crc": crc, "flags": flags, "mask": mask, "data": {}}
        elif crc != self.cur["crc"] or flags != self.cur["flags"] or mask != self.cur["mask"] or typ in self.cur["data"]:
            self.mixed += 1
            self.cur = None
            return None
        self.cur["data"][typ] = bytes(m[HDR:])
        have = sum(1 << (t - 1) for t in self.cur["data"])
        if have != mask:
            return None
        cur, self.cur = self.cur, None
        if crc_chain(cur["data"][t] for t in sorted(cur["data"])) != cur["crc"]:
            self.crc_bad += 1
            return None
        self.complete += 1
        s = decode_snapshot(cur["data"], mask)
        s.update({"session": session, "snap_id": ident, "mask": mask,
                  "banderas": [n for b, n in FLAGS.items() if flags & b]})
        return "snap", s


def flat(s):
    row = {}
    for k, v in s.items():
        if isinstance(v, (tuple, list)) and k != "banderas":
            for i, x in enumerate(v):
                row[f"{k}{i}"] = x
        elif k == "banderas":
            row[k] = "|".join(v)
        else:
            row[k] = v
    return row


def decode_capture(path, out_id, fmt=None):
    """Decodifica una captura entera. Devuelve (snaps, diags, ops, informe) con TODAS las incidencias."""
    data, _, _ = spp_wire.load(path, fmt)
    pkts, issues, _ = spp_wire.split_packets(data)
    r = Reasm()
    snaps, diags, ops = [], [], []
    rep = {"paquetes": len(pkts), "paquetes_salida": 0, "crc_spp": 0, "crc_spp_salida": 0, "version_spp": 0,
           "saltos_secuencia": 0, "repetidos": 0, "lector": {}, "descartados": [], "bytes": len(data),
           "kpid_presentes": sorted({p["id"] for p in pkts if p["crc_ok"]})}
    last_seq = None
    ver_off = {i["off"] for i in issues if i.get("tipo") == "version"}  # CRC bien y versión distinta
    for i in issues:
        t = i.get("tipo")
        if t in ("crc", "version"):
            continue  # se cuentan abajo, paquete a paquete
        rep["lector"][t] = rep["lector"].get(t, 0) + 1
        rep["descartados"].append(f"lector SPP: {t} en el byte {i.get('off')}" +
                                  (f" ({i['bytes']} B)" if "bytes" in i else "") +
                                  (f" ({i['bytes_saltados']} B saltados)" if "bytes_saltados" in i else ""))
    for p in pkts:
        if not p["crc_ok"]:
            kind = "version_spp" if p["off"] in ver_off else "crc_spp"
            rep[kind] += 1
            if kind == "crc_spp" and p["id"] == out_id:
                rep["crc_spp_salida"] += 1
            rep["descartados"].append(f"paquete {p['idx']} (byte {p['off']}): rechazado por "
                                      f"{'CRC SPP' if kind == 'crc_spp' else 'versión SPP'} (KPID leído 0x{p['id']:04x}, "
                                      f"secuencia {p['seq']})")
            continue
        if p["id"] != out_id:
            continue
        rep["paquetes_salida"] += 1
        if last_seq is not None:
            d = (p["seq"] - last_seq) & 0xFFFF
            if d == 0:
                rep["repetidos"] += 1
            elif d != 1:
                rep["saltos_secuencia"] += 1
                rep["descartados"].append(f"secuencia SPP de salida {last_seq} → {p['seq']}: {d - 1} paquetes ausentes")
        last_seq = p["seq"]
        k = r.push(p["payload"])
        if k:
            {"snap": snaps, "diag": diags, "ops": ops}[k[0]].append(k[1])
    if r.cur:
        r.incomplete += 1
        rep["descartados"].append(f"snapshot sesión {r.cur['key'][0]} id {r.cur['key'][1]} sin cerrar al final")
    rep.update({"incompletos": r.incomplete, "crc_interno": r.crc_bad, "mezclados": r.mixed, "invalidos": r.invalid})
    rep["limpia"] = not (rep["crc_spp"] or rep["version_spp"] or rep["lector"] or rep["saltos_secuencia"] or rep["repetidos"]
                         or r.incomplete or r.crc_bad or r.mixed or r.invalid)
    return snaps, diags, ops, rep


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("registro")
    ap.add_argument("--out-id", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--formato")
    ap.add_argument("--csv")
    ap.add_argument("--diag-csv")
    ap.add_argument("--ops-csv")
    ap.add_argument("--tolerante", action="store_true",
                    help="rescate: escribe lo decodificable y enumera lo descartado (código 3 si se descartó algo)")
    a = ap.parse_args()
    snaps, diags, ops, R = decode_capture(a.registro, a.out_id, a.formato)
    lector = ", ".join(f"{k} {v}" for k, v in sorted(R["lector"].items())) or "ninguna"
    print(f"{a.registro}: {len(snaps)} snapshots completos, {len(diags)} diagnósticos, {len(ops)} compactas")
    print(f"  capa SPP: {R['paquetes']} paquetes ({R['paquetes_salida']} válidos del KPID de salida); rechazados por CRC SPP "
          f"{R['crc_spp']} ({R['crc_spp_salida']} con el KPID de salida), por versión {R['version_spp']}; saltos de "
          f"secuencia de salida {R['saltos_secuencia']}, repetidos {R['repetidos']}; incidencias del lector: {lector}")
    print(f"  mensajes: CRC interno mal {R['crc_interno']}, incompletos {R['incompletos']}, mezclados {R['mezclados']}, "
          f"inválidos {R['invalidos']}")
    if snaps:
        s = snaps[-1]
        print(f"  último: sesión {s['session']} snapshot {s['snap_id']} validez núcleo {s.get('nav_validity')} servicio "
              f"{s.get('svc_validity')} motivos 0x{s.get('nav_reasons', 0):04x}/0x{s.get('svc_reasons', 0):04x} readiness "
              f"0x{s.get('readiness', 0):04x} why {s.get('why_code')} t_state {s.get('t_state_us')} banderas {s['banderas']}")
    if ops:
        o = ops[-1]
        print(f"  última compacta: t {o['t_ms']} ms sesión {o['session']} arranque {o['boot_id']} validez servicio "
              f"{o['svc_validity']} fuente {SOURCE.get(o['source'])} capacidades "
              f"{[n for b, n in CAPS.items() if o['caps'] & b]} ESKF h {o['eskf_h_m']:.2f} m vz {o['eskf_vz_mps']:.2f} m/s "
              f"(edad {o['eskf_age_ms']} ms); respaldo h {o['bb_h_m']:.2f} m vz {o['bb_vz_mps']:.2f} m/s "
              f"(válido {o['bb_valid']}, edad {o['bb_age_ms']} ms)")
    if a.ops_csv and ops:
        with open(a.ops_csv, "w", newline="") as fo:
            w = csv.DictWriter(fo, list(flat(ops[0])))
            w.writeheader()
            w.writerows(flat(o) for o in ops)
    if a.csv and snaps:
        rows = [flat(s) for s in snaps]
        keys = sorted({k for r_ in rows for k in r_})
        with open(a.csv, "w", newline="") as fo:
            w = csv.DictWriter(fo, keys)
            w.writeheader()
            w.writerows(rows)
    if a.diag_csv and diags:
        with open(a.diag_csv, "w", newline="") as fo:
            w = csv.DictWriter(fo, sorted(flat(diags[0])))
            w.writeheader()
            w.writerows(flat(d) for d in diags)
    n_msgs = len(snaps) + len(diags) + len(ops)
    vacia = None  # nada comprobable: nunca se presenta como validación satisfactoria
    if R["paquetes"] == 0 and R["crc_spp"] == 0 and R["version_spp"] == 0:
        vacia = ("archivo SIN DATOS: 0 paquetes SPP" + (f" ({R['bytes']} B de relleno)" if R["bytes"] else " (0 B)"))
    elif R["paquetes_salida"] == 0 and R["crc_spp_salida"] == 0:
        vacia = (f"el KPID 0x{a.out_id:04x} NO APARECE en la captura ({R['paquetes']} paquetes; KPID presentes: "
                 f"{', '.join(f'0x{k:04x}' for k in R['kpid_presentes']) or 'ninguno'})")
    elif n_msgs == 0:
        vacia = f"{R['paquetes_salida']} paquetes del KPID 0x{a.out_id:04x} y NINGÚN mensaje decodificable"
    if vacia:
        print(f"  SIN DATOS COMPROBABLES: {vacia}")
    if R["limpia"]:
        if vacia:
            return 2
        print(f"  RESULTADO: captura comprobada: {n_msgs} mensajes, completa y sin corrupción")
        return 0
    head = "MODO TOLERANTE (rescate): se ha DESCARTADO" if a.tolerante else "COMPROBACIÓN FALLIDA: captura corrupta o incompleta;"
    print(f"  {head} lo siguiente ({len(R['descartados'])} incidencias; mensajes internos descartados: CRC "
          f"{R['crc_interno']}, incompletos {R['incompletos']}, mezclados {R['mezclados']}, inválidos {R['invalidos']}):")
    for x in R["descartados"][:20]:
        print(f"    - {x}")
    if len(R["descartados"]) > 20:
        print(f"    ... {len(R['descartados']) - 20} más")
    if a.tolerante:
        return 2 if n_msgs == 0 else 3  # sin nada rescatado no hay rescate
    return 1


if __name__ == "__main__":
    sys.exit(main())
