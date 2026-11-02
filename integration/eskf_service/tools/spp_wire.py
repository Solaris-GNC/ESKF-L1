#!/usr/bin/env python3
"""spp_wire.py — lectura INDEPENDIENTE de registros SPP del Viradinha MKII.

No usa código de Solaris Software ni de SPP: implementa el formato OBSERVADO en la captura del
24-oct (spp_dump.bin) para poder comprobar los resultados por separado.

Paquete de 64 bytes, enteros y float en little-endian:
    0       versión (1 en la captura)
    1–2     identificador (APID/KPID)
    3–4     secuencia (por identificador)
    5–6     longitud del payload (<= 50)
    7–10    timestampMs
    11      dropCounter
    12–61   payload (hasta 50 bytes; el resto, relleno)
    62–63   CRC-16/CCITT-FALSE (polinomio 0x1021, inicial 0xFFFF) de los bytes 0–61, little-endian

Entradas admitidas (detección automática o --formato):
    bin       binario (el volcado de memoria: paquetes seguidos y relleno de ceros)
    hex       hexadecimal plano: 128 dígitos (64 bytes) por línea, como telemetry_example.hex.
              NO es Intel HEX (las líneas «:LLAAAATT…» se rechazan explícitamente).
    xxd16     volcado «00000000: 0102 0000 …  ASCII» (16 bytes por línea, grupos de 2 bytes),
              como spp_dump.txt
    xxd64     volcado «00000000: 01 02 00 …  ASCII» (64 bytes por línea), como telemetry_example.txt
En los volcados de texto la columna ASCII se ignora siempre: los bytes salen sólo de los grupos
hexadecimales anteriores al doble espacio, y los desplazamientos deben ser contiguos.
"""
import binascii
import re
import struct

PKT = 64
HDR = struct.Struct("<BHHHIB")  # versión, id, secuencia, longitud, timestampMs, dropCounter
PAYLOAD_MAX = 50
VERSION = 1


class FormatError(Exception):
    pass


def crc16(data):
    return binascii.crc_hqx(bytes(data), 0xFFFF)


# ---------------------------------------------------------------- formatos de entrada
_OFF = re.compile(r"^([0-9a-fA-F]{8}):")


def _parse_dump(text, report):
    """Volcados xxd16/xxd64. Devuelve bytes y el formato. Comprueba que los desplazamientos son
    contiguos y que cada línea tiene sólo grupos hexadecimales antes del doble espacio."""
    out = bytearray()
    fmt = None
    expected = 0
    for ln, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        m = _OFF.match(line)
        if not m:
            raise FormatError(f"línea {ln}: no empieza por un desplazamiento «xxxxxxxx:»")
        off = int(m.group(1), 16)
        if off != expected:
            raise FormatError(f"línea {ln}: desplazamiento 0x{off:x}, se esperaba 0x{expected:x} "
                              f"(líneas perdidas, repetidas o volcado comprimido)")
        rest = line[m.end():]
        if not rest.startswith(" "):
            raise FormatError(f"línea {ln}: falta el espacio tras el desplazamiento")
        rest = rest[1:]
        # grupos separados por UN espacio hasta el primer doble espacio (o fin de línea)
        hexpart = rest.split("  ", 1)[0]
        toks = hexpart.split(" ")
        if not toks or any(not re.fullmatch(r"[0-9a-fA-F]{2}|[0-9a-fA-F]{4}", t) for t in toks):
            raise FormatError(f"línea {ln}: grupos hexadecimales no válidos: {hexpart[:40]!r}")
        widths = {len(t) for t in toks}
        f = "xxd16" if widths == {4} or (widths == {4, 2} and len(toks[-1]) == 2) else "xxd64" if widths == {2} else None
        if f is None:
            raise FormatError(f"línea {ln}: mezcla de anchos de grupo")
        if fmt is None:
            fmt = f
        elif f != fmt and not (fmt == "xxd16" and widths == {2} and len(toks) == 1):
            raise FormatError(f"línea {ln}: cambia el formato de volcado ({fmt} → {f})")
        b = bytes.fromhex("".join(toks))
        per_line = 16 if fmt == "xxd16" else 64
        if len(b) > per_line:
            raise FormatError(f"línea {ln}: {len(b)} bytes (> {per_line})")
        out += b
        expected += len(b)
        if len(b) < per_line:
            report["dump_short_line"] = ln  # sólo admisible en la última línea
    if report.get("dump_short_line") is not None:
        report["dump_last_partial"] = True
    return bytes(out), fmt


def _parse_plain_hex(text):
    out = bytearray()
    for ln, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if not s:
            continue
        if s.startswith(":"):
            raise FormatError(f"línea {ln}: parece Intel HEX («:…»); este registro es hexadecimal plano")
        if not re.fullmatch(r"[0-9a-fA-F]+", s):
            raise FormatError(f"línea {ln}: caracteres no hexadecimales")
        if len(s) % 2:
            raise FormatError(f"línea {ln}: número impar de dígitos")
        if len(s) != 2 * PKT:
            raise FormatError(f"línea {ln}: {len(s) // 2} bytes; cada línea debe ser un paquete de 64")
        out += bytes.fromhex(s)
    return bytes(out)


def detect_format(raw):
    head = raw[:4096]
    try:
        txt = head.decode("ascii")
    except UnicodeDecodeError:
        return "bin"
    first = next((l for l in txt.splitlines() if l.strip()), "")
    if _OFF.match(first):
        return "dump"
    if first.strip().startswith(":"):
        return "intelhex"
    if re.fullmatch(r"[0-9a-fA-F\s]*", txt):
        return "hex"
    return "bin"


def load(path, fmt=None):
    """Devuelve (bytes, formato, informe_de_carga)."""
    with open(path, "rb") as f:
        raw = f.read()
    rep = {}
    f = fmt or detect_format(raw)
    if f == "intelhex":
        raise FormatError("el archivo parece Intel HEX; los registros SPP son hexadecimal plano o binario")
    if f == "bin":
        return raw, "bin", rep
    text = raw.decode("ascii", errors="strict")
    if f == "hex":
        return _parse_plain_hex(text), "hex", rep
    if f in ("dump", "xxd16", "xxd64"):
        data, df = _parse_dump(text, rep)
        if f in ("xxd16", "xxd64") and df != f:
            raise FormatError(f"se pidió {f} y el archivo es {df}")
        return data, df, rep
    raise FormatError(f"formato desconocido {f}")


# ---------------------------------------------------------------- paquetes
def packet_ok(p):
    if len(p) != PKT:
        return False
    v, _id, _seq, ln, _ts, _drop = HDR.unpack_from(p, 0)
    return v == VERSION and ln <= PAYLOAD_MAX and crc16(p[:62]) == struct.unpack_from("<H", p, 62)[0]


def split_packets(data):
    """Recorre los datos en bloques de 64 bytes. Devuelve (paquetes, incidencias, informe).
    paquete = dict(idx, off, ver, id, seq, len, ts, drop, payload, crc_ok, len_ok, filler).
    - El relleno final de ceros se reconoce y se mide; un bloque de ceros seguido de datos es
      una incidencia (no se oculta).
    - Un bloque con CRC o versión mal: incidencia; si los siguientes bloques alineados tampoco
      son válidos, se busca byte a byte el siguiente paquete válido (resincronización).
    - Un final de menos de 64 bytes no nulo: paquete truncado."""
    pkts, issues = [], []
    n = len(data)
    off = 0
    zero_run_start = None
    while off < n:
        blk = data[off:off + PKT]
        if len(blk) < PKT:
            if any(blk):
                issues.append({"tipo": "truncado", "off": off, "bytes": len(blk)})
            elif zero_run_start is None:
                zero_run_start = off
            break
        if not any(blk):
            if zero_run_start is None:
                zero_run_start = off
            off += PKT
            continue
        if zero_run_start is not None:
            issues.append({"tipo": "ceros_intermedios", "off": zero_run_start, "bytes": off - zero_run_start})
            zero_run_start = None
        v, ident, seq, ln, ts, drop = HDR.unpack_from(blk, 0)
        crc_ok = crc16(blk[:62]) == struct.unpack_from("<H", blk, 62)[0]
        if not crc_ok or v != VERSION:
            # ¿alineación perdida? si este bloque y el siguiente alineado no valen, resincronizar
            nxt = data[off + PKT:off + 2 * PKT]
            if not packet_ok(nxt) and off + PKT < n and any(nxt):
                j = off + 1
                while j + PKT <= n and not packet_ok(data[j:j + PKT]):
                    j += 1
                if j + PKT <= n:
                    issues.append({"tipo": "resincronizacion", "off": off, "bytes_saltados": j - off})
                    off = j
                    continue
            issues.append({"tipo": "crc" if crc_ok is False else "version", "off": off, "id": ident, "seq": seq,
                           "version": v})
        ln_ok = ln <= PAYLOAD_MAX
        if not ln_ok:
            issues.append({"tipo": "longitud", "off": off, "id": ident, "len": ln})
        pl = blk[12:12 + min(ln, PAYLOAD_MAX)]
        filler = blk[12 + min(ln, PAYLOAD_MAX):62]
        pkts.append({"idx": len(pkts), "off": off, "ver": v, "id": ident, "seq": seq, "len": ln, "ts": ts,
                     "drop": drop, "payload": pl, "crc_ok": crc_ok and v == VERSION, "len_ok": ln_ok,
                     "filler": filler})
        off += PKT
    rep = {"bytes_total": n, "bytes_paquetes": len(pkts) * PKT,
           "relleno_final_ceros": (n - zero_run_start) if zero_run_start is not None else 0}
    return pkts, issues, rep


def build_packet(ident, seq, ts_ms, payload, version=VERSION, drop=0, filler=0xFF):
    """Construye un paquete de 64 bytes (para pruebas y para la salida del servicio)."""
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("payload > 50 bytes")
    p = bytearray(HDR.pack(version, ident, seq & 0xFFFF, len(payload), ts_ms & 0xFFFFFFFF, drop))
    p += bytes(payload) + bytes([filler]) * (PAYLOAD_MAX - len(payload))
    p += struct.pack("<H", crc16(p))
    return bytes(p)


def to_xxd16(data):
    """Volcado con el formato de `xxd` (16 bytes por línea, grupos de 2): el de spp_dump.txt."""
    out = []
    for off in range(0, len(data), 16):
        b = data[off:off + 16]
        hx = " ".join(b[i:i + 2].hex() for i in range(0, len(b), 2))
        asc = "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)
        out.append(f"{off:08x}: {hx:<39}  {asc}\n")
    return "".join(out)


def to_xxd64(data):
    """Volcado de 64 bytes por línea con bytes sueltos (el de telemetry_example.txt)."""
    out = []
    for off in range(0, len(data), 64):
        b = data[off:off + 64]
        hx = " ".join(f"{c:02x}" for c in b)
        asc = "".join(chr(c) if 0x20 <= c < 0x7F else "." for c in b)
        out.append(f"{off:08x}: {hx}  {asc}\n")
    return "".join(out)


def to_plain_hex(data):
    return "".join(data[i:i + PKT].hex() + "\n" for i in range(0, len(data), PKT))
