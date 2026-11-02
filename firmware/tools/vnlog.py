#!/usr/bin/env python3
"""vnlog.py — decodificador del registro binario del Viradinha MKII (formato v1).

Mismas definiciones que firmware/viradinha_nav/src/vn/vn_logfmt.h (los tamaños se comprueban
aquí y en test_tools_fw.py contra el C). Biblioteca y herramienta:

  vnlog.py resumen registro.bin           cabecera, recuento de tramas, parámetros, registros
                                          releídos, textos, pérdidas
  vnlog.py csv registro.bin carpeta/      un CSV por tipo de registro

Trama: A5 5A | len u16 | tipo u8 | carga | crc16 (CCITT-FALSE sobre tipo+carga), little-endian.
"""
import binascii
import re
import struct
import sys
from collections import defaultdict

SYNC = b"\xA5\x5A"
FMT_VERSION = 1

REC = {
    1: "HEADER", 2: "PARAM", 3: "REG", 4: "IMU", 5: "BARO", 6: "NAVEV", 7: "NAVOUT",
    8: "STATS", 9: "TEXT", 10: "NAVCNT", 11: "BAROCAL", 12: "BENCH", 13: "TICK",
}
STATUS = {1: "confirmado", 2: "propuesto", 3: "provisional", 4: "pendiente"}
PROFILE = {0: "ADQUISICION", 1: "BANCO", 2: "VUELO"}

# nombre: (struct, campos)
LAYOUT = {
    "HEADER": ("<8sHHBBBBIqI16s24s40s24s24s24s",
               "magic fmt_version sizeof_eskf_real profile nav_enabled imu_irq baro_irq cpu_mhz t_us boot_id "
               "fw_version build compiler idf core board"),
    "PARAM": ("<40sdBB46s", "name value status used source"),
    "REG": ("<8B", "dev bank addr mask expected read ok reserved"),
    "IMU": ("<Iqq3h3hh3hBBBB",
            "seq t_drdy_us t_read_us ax ay az gx gy gz temp mx my mz mag_st1 mag_st2 sat flags"),
    "BARO": ("<IqqqfffBB", "seq t_prev_poll_us t_detect_us t_read_us pressure_pa temp_c alt_m status flags"),
    "NAVEV": ("<IBhBIH", "src_seq kind ret validity reasons exec_us"),
    "NAVOUT": ("<qIdddIIIBBBB25f",
               "t_us n_events t_state t_eval alt_ref reasons alt_ref_seq f_degraded validity phase at_rest "
               "heading_known px py pz vx vy vz qw qx qy qz spx spy spz svx svy svz sthx sthy sthz "
               "bax bay baz bgx bgy bgz"),
    "STATS": ("<q24I",
              "t_us imu_irq_lost imu_q_drop baro_q_drop ring_drop_recs ring_drop_bytes i2c_err_imu i2c_err_baro "
              "merge_imu_overflow merge_baro_overflow imu_events baro_events nav_events nav_exec_max_us "
              "nav_exec_sum_us acq_read_max_us log_write_max_us hwm_acq hwm_nav hwm_log hwm_loop heap_free_min "
              "ring_free_min sink_bytes sink_discard_bytes"),
    "TEXT": ("<96s", "text"),
    "NAVCNT": ("<q32I8h", None),
    "BAROCAL": ("<BB21s", "addr chip_id nvm"),
    "BENCH": ("<BBIfffIII", "what profile n min_us mean_us max_us over_dt hwm_after cpu_mhz"),
    "TICK": ("<qh", "t_us ret"),
}
SIZES = {"HEADER": 184, "PARAM": 96, "REG": 8, "IMU": 44, "BARO": 42, "NAVEV": 14, "NAVOUT": 152,
         "STATS": 104, "TEXT": 96, "NAVCNT": 152, "BAROCAL": 23, "BENCH": 30, "TICK": 10}

# nav_counters_t en orden de declaración (navigation.h); test_tools_fw.py lo compara con el C
NAV_COUNTERS = (
    "imu_rx imu_applied imu_time_nonfinite imu_time_dup imu_time_order imu_value_invalid imu_not_applied "
    "imu_acc_sat imu_gyro_sat baro_rx baro_used baro_gated baro_rejected time_future ref_updates ref_rejected "
    "ref_outliers ref_overflow rest_windows_tilt rest_breaks_gap zupt_used zupt_rejected grav_used grav_rejected "
    "inflate_fail rest_windows_ok rest_windows_bad align_ok align_fail gaps").split()
NAV_LAST = "imu predict baro zupt grav inflate align ref".split()
BENCH_WHAT = {1: "imu_reposo", 2: "imu_vuelo", 3: "baro_vuelo", 4: "alineacion"}

IMU_F = {"POLLED": 0x01, "I2C_ERR": 0x02, "IRQ_LOST": 0x04, "ACC_NEW": 0x08, "GYR_NEW": 0x10, "MAG": 0x20,
         "RACE": 0x40, "TIMEOUT": 0x80}
BARO_F = {"POLLED": 0x01, "I2C_ERR": 0x02, "IRQ": 0x04}

_STRUCTS = {k: struct.Struct(v[0]) for k, v in LAYOUT.items()}
for _k, _st in _STRUCTS.items():
    assert _st.size == SIZES[_k], f"{_k}: {_st.size} != {SIZES[_k]}"


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (polinomio 0x1021, inicial 0xFFFF): binascii.crc_hqx con inicial 0xFFFF."""
    return binascii.crc_hqx(bytes(data), crc)


def frames(data, stats=None):
    """Genera (tipo, inicio_carga, longitud) con resincronización.
    stats: dict con crc_err, bad_len, resync_bytes, truncated."""
    if stats is None:
        stats = {}
    for k in ("crc_err", "resync_bytes", "bad_len", "truncated"):
        stats.setdefault(k, 0)
    mv = memoryview(data)
    crc_hqx = binascii.crc_hqx
    sizes = {t: SIZES[n] for t, n in REC.items()}
    i, n = 0, len(data)
    while True:
        j = data.find(SYNC, i)
        if j < 0:
            stats["resync_bytes"] += n - i
            return
        stats["resync_bytes"] += j - i
        if j + 5 > n:
            stats["truncated"] += 1
            return
        ln = data[j + 2] | (data[j + 3] << 8)
        typ = data[j + 4]
        exp = sizes.get(typ)
        if exp is not None and ln != exp:
            stats["bad_len"] += 1
            i = j + 1
            continue
        end = j + 7 + ln
        if end > n:
            stats["truncated"] += 1
            return
        if crc_hqx(mv[j + 4:end - 2], 0xFFFF) != (data[end - 2] | (data[end - 1] << 8)):
            stats["crc_err"] += 1
            i = j + 1
            continue
        yield typ, j + 5, ln
        i = end


def _s(b):
    return b.split(b"\0", 1)[0].decode("utf-8", "replace")


def unpack(name, payload):
    v = _STRUCTS[name].unpack(payload)
    if name == "NAVCNT":
        return {"t_us": v[0], "cnt": dict(zip(NAV_COUNTERS, v[1:1 + len(NAV_COUNTERS)])),
                "last": dict(zip(NAV_LAST, v[33:41]))}
    d = dict(zip(LAYOUT[name][1].split(), v))
    for k, x in d.items():
        if isinstance(x, bytes) and k not in ("nvm",):
            d[k] = _s(x)
    return d


def _np_dtype(name):
    """dtype empaquetado de numpy equivalente a la estructura C"""
    import numpy as np
    fmt, names = LAYOUT[name]
    codes = {"I": "<u4", "q": "<i8", "h": "<i2", "B": "u1", "f": "<f4", "d": "<f8", "H": "<u2"}
    items = []
    k = 0
    for m in re.finditer(r"(\d*)([A-Za-z])", fmt[1:]):
        cnt = int(m.group(1) or 1)
        for _ in range(cnt):
            items.append(codes[m.group(2)])
    nm = names.split() if names else ["t_us"] + NAV_COUNTERS + ["_c%d" % i for i in range(32 - len(NAV_COUNTERS))] + \
        ["last_" + x for x in NAV_LAST]
    assert len(nm) == len(items), name
    dt = np.dtype(list(zip(nm, items)))
    assert dt.itemsize == SIZES[name]
    return dt


# Tipos frecuentes: se guardan como arrays estructurados de numpy
NUMERIC = ("IMU", "BARO", "NAVEV", "NAVOUT", "TICK", "STATS", "NAVCNT")


class Log:
    """Registro decodificado.
    log["IMU"], log["BARO"], ... (tipos de NUMERIC): array estructurado de numpy.
    log["PARAM"], log["TEXT"], ...: lista de dicts.
    log.order_type / log.order_idx: orden de aparición de todas las tramas (para reproducir
    el orden de vn_nav: TICK, NAVEV y NAVOUT)."""

    def __init__(self, path=None, data=None):
        import numpy as np
        if data is None:
            with open(path, "rb") as f:
                data = f.read()
        self.path = path
        self.nbytes = len(data)
        self.stats = {}
        chunks = defaultdict(list)
        small = defaultdict(list)
        otype, oidx = [], []
        counts = defaultdict(int)
        for typ, a, ln in frames(data, self.stats):
            name = REC.get(typ)
            if name is None:
                self.stats["unknown"] = self.stats.get("unknown", 0) + 1
                continue
            if name in NUMERIC:
                chunks[name].append(data[a:a + ln])
            else:
                small[name].append(unpack(name, data[a:a + ln]))
            otype.append(typ)
            oidx.append(counts[name])
            counts[name] += 1
        self.recs = {}
        for name in NUMERIC:
            dt = _np_dtype(name)
            self.recs[name] = np.frombuffer(b"".join(chunks.get(name, [])), dtype=dt)
        for name, lst in small.items():
            self.recs[name] = lst
        self.order_type = np.array(otype, dtype=np.uint8)
        self.order_idx = np.array(oidx, dtype=np.int64)

    def __getitem__(self, name):
        return self.recs.get(name, [])

    def count(self, name):
        return len(self.recs.get(name, []))

    @property
    def header(self):
        h = self["HEADER"]
        return h[-1] if h else None

    def params(self):
        """Último valor de cada parámetro: nombre → dict(value, status, used, source)"""
        return {p["name"]: p for p in self["PARAM"]}

    def is_sim(self):
        h = self.header
        return bool(h) and h["board"] == "PC-SIM"

    def boots(self):
        return sorted({h["boot_id"] for h in self["HEADER"]})

    def navcnt_last(self):
        c = self["NAVCNT"]
        if len(c) == 0:
            return None
        r = c[-1]
        return {k: int(r[k]) for k in NAV_COUNTERS}


def summary(log, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    h = log.header
    p(f"registro: {log.path}  ({log.nbytes} bytes)")
    if h is None:
        p("  SIN CABECERA (¿registro truncado al principio?)")
    else:
        p(f"  formato v{h['fmt_version']}  perfil {PROFILE.get(h['profile'], h['profile'])}  "
          f"navegación {'ACTIVA' if h['nav_enabled'] else 'desactivada'}  eskf_real {h['sizeof_eskf_real']} B")
        p(f"  placa {h['board']}  firmware {h['fw_version']}  compilado {h['build']}")
        p(f"  {h['compiler']} | IDF {h['idf']} | {h['core']} | CPU {h['cpu_mhz']} MHz | arranque 0x{h['boot_id']:08x}")
        p(f"  IMU por {'interrupción' if h['imu_irq'] else 'SONDEO'}, baro por "
          f"{'interrupción' if h['baro_irq'] else 'sondeo'}")
        if len(log.boots()) > 1:
            p(f"  ATENCIÓN: {len(log.boots())} arranques en el mismo archivo")
        if log.is_sim():
            p("  ATENCIÓN: registro SIMULADO en el PC (gen_vnlog), no de la placa")
    p("  tramas: " + ", ".join(f"{k} {len(v)}" for k, v in sorted(log.recs.items()) if len(v)))
    st = log.stats
    p(f"  integridad: crc {st.get('crc_err', 0)}, longitud {st.get('bad_len', 0)}, "
      f"bytes sin sincronismo {st.get('resync_bytes', 0)}, truncadas {st.get('truncated', 0)}")
    if len(log["STATS"]):
        s = log["STATS"][-1]
        p(f"  pérdidas (STATS final): registro {s['ring_drop_recs']} tramas/{s['ring_drop_bytes']} B, "
          f"irq IMU perdidas {s['imu_irq_lost']}, cola IMU {s['imu_q_drop']}, cola baro {s['baro_q_drop']}, "
          f"I2C IMU {s['i2c_err_imu']}, I2C baro {s['i2c_err_baro']}, sumidero descartó {s['sink_discard_bytes']} B")
    pars = log.params()
    if pars:
        cnt = defaultdict(int)
        for q in pars.values():
            cnt[q["status"]] += 1
        p("  parámetros: " + ", ".join(f"{STATUS.get(k, k)} {v}" for k, v in sorted(cnt.items())))
        pend = [k for k, q in pars.items() if q["status"] == 4]
        if pend:
            p("  pendientes: " + ", ".join(pend))
        prov = [k for k, q in pars.items() if q["status"] == 3]
        if prov:
            p("  PROVISIONALES (sólo banco): " + ", ".join(prov))
    regs = log["REG"]
    if regs:
        bad = [r for r in regs if not r["ok"]]
        p(f"  registros releídos: {len(regs)}, distintos de lo esperado: {len(bad)}")
        for r in bad[:10]:
            p(f"    {'IMU' if r['dev'] == 1 else 'BARO'} banco {r['bank']} 0x{r['addr']:02X}: "
              f"leído 0x{r['read']:02X} esperado 0x{r['expected']:02X} máscara 0x{r['mask']:02X}")
    for b in log["BENCH"][:4]:
        p(f"  tiempos ({BENCH_WHAT.get(b['what'], b['what'])}): n {b['n']} min {b['min_us']:.1f} "
          f"media {b['mean_us']:.1f} máx {b['max_us']:.1f} µs, > periodo {b['over_dt']}, pila libre {b['hwm_after']} B")
    texts = []
    for t in log["TEXT"]:
        if not texts or texts[-1] != t["text"]:
            texts.append(t["text"])
    for t in texts[:20]:
        p(f"  texto: {t}")


def to_csv(log, folder):
    import csv
    import os
    os.makedirs(folder, exist_ok=True)
    for name, rs in log.recs.items():
        if len(rs) == 0:
            continue
        path = os.path.join(folder, name.lower() + ".csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            if name in NUMERIC:
                cols = [c for c in rs.dtype.names if not c.startswith("_c")]
                w.writerow(cols)
                for r in rs:
                    w.writerow([r[c].item() for c in cols])
            else:
                cols = list(rs[0].keys())
                w.writerow(cols)
                for r in rs:
                    w.writerow([(r[c].hex() if isinstance(r[c], bytes) else r[c]) for c in cols])


def main(argv):
    if len(argv) >= 3 and argv[1] == "resumen":
        summary(Log(argv[2]))
        return 0
    if len(argv) == 4 and argv[1] == "csv":
        to_csv(Log(argv[2]), argv[3])
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
