#!/usr/bin/env python3
"""test_spp_tools.py — herramientas y servicio con la CAPTURA HISTÓRICA del 24-oct (fixture de 48 B),
con registros SINTÉTICOS del formato definitivo (44 B, SPP 9247d9e) y con escenarios de vuelo
sintéticos construidos con las series OpenRocket «Simulation 6» (test/data/openrocket_sim6_*.csv).

  test_spp_tools.py --svc build/svc_replay --nav-replay ../../c/build/nav_replay_float
                    [--router build/test_spp_router] [--svc-fma build/svc_replay_fma] [--dump-txt spp_dump.txt]
                    [--resultados docs/resultados]

La captura histórica sólo prueba la compatibilidad explícita con aquel formato y las herramientas;
no prueba el formato definitivo. Se guardan los 899 072 B de paquetes y la prueba REGENERA el
binario de 16 MiB y el volcado de texto y comprueba su SHA-256 con los recibidos.
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SVC = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(SVC, "tools"))
import nav_out_decode  # noqa: E402
import spp_decode  # noqa: E402
import spp_to_replay  # noqa: E402
import spp_wire  # noqa: E402

DATA = os.path.join(HERE, "data")
SHA_BIN = "b0be0ba8afad6314446b9e2c5c26130c68819e95deb7dfde38cba667a41f334e"
SHA_TXT = "1c0c70844d6eecd15cb8bc582df849968a2a557c6d91ec2dcc80d3cd05c2c6a9"
results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'OK' if ok else 'FALLO'}] {name}" + ("" if ok else f" — {detail}"))


def run(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)


def summary(svc, path, *args):
    r = run([svc, "--resumen", *args, path])
    return dict(re.findall(r"(\w+)=([^\s]+)", r.stdout)), r


def test_capture(tmp):
    print("A. captura histórica (fixture): conteos, CRC, formatos y relleno")
    pk = open(os.path.join(DATA, "spp_dump_paquetes.bin"), "rb").read()
    full = pk + bytes(16 * 1024 * 1024 - len(pk))
    check("binario regenerado = spp_dump.bin recibido (SHA-256)", hashlib.sha256(full).hexdigest() == SHA_BIN)
    txt = spp_wire.to_xxd16(full)
    check("volcado regenerado = spp_dump.txt recibido (SHA-256)", hashlib.sha256(txt.encode()).hexdigest() == SHA_TXT)
    fb = os.path.join(tmp, "spp_dump.bin")
    open(fb, "wb").write(full)
    R, pkts = spp_decode.analyze(fb, imu_fmt=48)
    ids = R["ids"]
    check("14 048 paquetes de 64 B, 899 072 B, ceros hasta 16 MiB",
          R["paquetes"] == 14048 and R["bytes_paquetes"] == 899072 and R["relleno_final_ceros"] == 16777216 - 899072)
    check("1 FSM 0x0002 (4 B), 4343 BMP 0x0004 (12 B), 9704 IMU 0x0008 (48 B)",
          R["longitudes"] == {"0x0002": {4: 1}, "0x0004": {12: 4343}, "0x0008": {48: 9704}}, R["longitudes"])
    check("CRC 14 048/14 048, versión 1, dropCounter 0", R["crc_ok"] == 14048 and R["crc_mal"] == 0 and
          R["versiones"] == [1] and R["drop_counter"]["no_nulos"] == 0)
    s_i, s_b = ids["imu"]["secuencia"], ids["baro"]["secuencia"]
    check("secuencias consecutivas", s_i["saltos"] == s_b["saltos"] == s_i["repetidos"] == s_i["retrocesos"] == 0)
    allts = [p["ts"] for p in pkts]
    check("marcas 1732–44862 ms (43,130 s), monótonas", min(allts) == 1732 and max(allts) == 44862 and
          all(b >= a for a, b in zip(allts, allts[1:])))
    ti, tb = ids["imu"]["tiempos"], ids["baro"]["tiempos"]
    check("BMP ≈ 100,69 Hz e IMU ≈ 225,16 Hz", abs(tb["frecuencia_media_hz"] - 100.69) < 0.01 and
          abs(ti["frecuencia_media_hz"] - 225.16) < 0.01)
    largos = [x["dt_ms"] for x in ti["largos"]]
    check("13 intervalos IMU de 33–35 ms cada 1104 paquetes (≈3,39 s)", len(largos) == 13 and min(largos) >= 33 and
          max(largos) <= 35 and ti["paquetes_entre_largos"] == [1104])
    check("faltan ~6,5–7 periodos por intervalo con la secuencia consecutiva",
          all(6.3 < x < 7.1 for x in ti["periodos_que_faltan_por_hueco"]))
    I = R["icm"]
    check("histórico: índice 11 siempre 0; índice 10 entero 295–453 (accuracy)", I["indice11_no_cero"] == 0 and
          I["indice10"]["enteros"] and I["indice10"]["min"] == 295 and I["indice10"]["max"] == 453)
    check("7 candidatos a saturación en el eje 3 (dominio de códigos sin offsets)",
          I["acel_candidatos_extremo_4g"]["n"] == 7 and I["acel_candidatos_extremo_4g"]["por_eje"] == [0, 0, 7])
    check("BMP histórico: altura ISA absoluta ±2 cm", max(abs(x) for x in R["bmp"]["altura_menos_isa_m"]) < 0.02)
    R44, _ = spp_decode.analyze(fb, imu_fmt=44)
    check("la captura NO pasa como formato definitivo: 48 B señalados como longitud inesperada",
          any(b["len"] == 48 for b in R44["longitudes_inesperadas"]) and R44["errores_integridad"])
    ex_bin = pk[:6400]
    for name, fmtn in (("telemetry_example.hex", "hex"), ("telemetry_example.txt", "xxd64")):
        d, f, _ = spp_wire.load(os.path.join(DATA, name))
        check(f"{name}: {fmtn} = primeros 100 paquetes", f == fmtn and d == ex_bin)
    ft = os.path.join(tmp, "dump.txt")
    open(ft, "w").write(txt)
    d, f, _ = spp_wire.load(ft)
    check("volcado de 16 B por línea completo = binario", f == "xxd16" and d == full)
    return fb, full


def test_robustness(tmp, full):
    print("B. robustez del lector: Intel HEX, columna ASCII, líneas, truncado, corrupción, longitudes")
    p = os.path.join(tmp, "x.hex")
    open(p, "w").write(":10010000214601360121470136007EFE09D2190140\n")
    try:
        spp_wire.load(p)
        ok = False
    except spp_wire.FormatError as e:
        ok = "Intel HEX" in str(e)
    check("Intel HEX rechazado", ok)
    pkt = spp_wire.build_packet(4, 0, 1000, b"0123456789abcdef0123456789abcdef0123456789ab")
    open(p, "w").write(spp_wire.to_xxd16(pkt))
    d, f, _ = spp_wire.load(p)
    check("columna ASCII con aspecto hexadecimal: ignorada", d == pkt and f == "xxd16")
    lines = spp_wire.to_xxd16(full[:6400]).splitlines(True)
    open(p, "w").write("".join(lines[:10] + lines[11:]))
    try:
        spp_wire.load(p)
        ok = False
    except spp_wire.FormatError as e:
        ok = "desplazamiento" in str(e)
    check("línea perdida: error con el desplazamiento", ok)
    pk = full[:899072]
    pkts, iss, _ = spp_wire.split_packets(pk[:6400 + 30])
    check("final truncado", any(i["tipo"] == "truncado" for i in iss) and len(pkts) == 100)
    bad = bytearray(pk[:6400])
    bad[64 * 10 + 20] ^= 0xFF
    pkts, _, _ = spp_wire.split_packets(bytes(bad))
    check("byte corrompido: un CRC malo", sum(not q["crc_ok"] for q in pkts) == 1)
    ins = pk[:64 * 10 + 5] + b"\x77" + pk[64 * 10 + 5:6400]
    pkts, iss, _ = spp_wire.split_packets(ins)
    check("byte insertado: resincronización", any(i["tipo"] == "resincronizacion" for i in iss) and
          sum(q["crc_ok"] for q in pkts) >= 98)
    zz = pk[:640] + bytes(128) + pk[640:1280]
    _, iss, _ = spp_wire.split_packets(zz)
    check("ceros intermedios ≠ relleno final", any(i["tipo"] == "ceros_intermedios" for i in iss))
    lp = bytearray(spp_wire.build_packet(8, 0, 1, b"x" * 50))
    lp[5] = 60
    c = spp_wire.crc16(lp[:62])
    lp[62], lp[63] = c & 0xFF, c >> 8
    _, iss, _ = spp_wire.split_packets(bytes(lp))
    check("longitud 60 con CRC correcto", any(i["tipo"] == "longitud" for i in iss))
    filler = bytearray(spp_wire.build_packet(4, 0, 1, b"abcd"))
    filler[40] = 0x00
    pkts, _, _ = spp_wire.split_packets(bytes(filler))
    check("relleno 0xFF dentro del CRC: alterarlo invalida el paquete", not pkts[0]["crc_ok"])


def test_service_capture(a, fb, tmp):
    print("C. servicio con la captura histórica (compatibilidad explícita, --historico)")
    kv, r = summary(a.svc, fb, "--historico", "--perfil", "adq")
    check("ADQ: mismas cuentas que el lector independiente; la FSM no está suscrita",
          (kv.get("imu_rx"), kv.get("baro_rx"), kv.get("seq_lost_imu"), kv.get("rejected_crc"), kv.get("rejected_apid"))
          == ("9704", "4343", "0", "0", "1"), r.stdout[-500:])
    kv, r = summary(a.svc, fb, "--historico", "--perfil", "banco", "--mensajes", os.path.join(tmp, "m.bin"))
    check("BANCO: 7 candidatos, 0 saturaciones confirmadas por hardware",
          kv.get("sat_cand_acc") == "7" and kv.get("sat_hw_confirmed") == "0")
    check("BANCO: no alinea (rest_breaks_gap > 0) — el resultado se conserva",
          kv.get("align_ok") == "0" and int(kv.get("rest_breaks_gap", 0)) > 0 and kv.get("nav_validez") == "0")
    cfg = os.path.join(tmp, "cfg.txt")
    open(cfg, "w").write(run([a.svc, "--dump-cfg", "banco"]).stdout)
    e = run([sys.executable, os.path.join(SVC, "tools", "explain_alignment.py"), fb, "--config", cfg])
    check("explicación sin tocar parámetros: 3,36 s < 5 s", "3.36 s < 5.0 s" in e.stdout, e.stdout[-300:])
    g = run([sys.executable, os.path.join(SVC, "tools", "gyro_dmp_check.py"), "--historico", fb])
    check("giro publicado coherente con el DMP en la captura (cociente ≈ 1; no es calibración)",
          "coherente" in g.stdout, g.stdout)
    d = run([sys.executable, os.path.join(SVC, "tools", "nav_out_decode.py"), os.path.join(tmp, "m.bin"), "--out-id", "0x40"])
    check("salida reconstruida por el decodificador Python independiente", d.returncode == 0 and "incompletos 0" in d.stdout,
          d.stdout)
    return cfg


def test_synthetic(a, tmp, cfg):
    print("D. formato definitivo (44 B, sintético) y reproducción")
    gen = os.path.join(SVC, "tools", "gen_spp.py")
    syn = os.path.join(tmp, "syn.bin")
    run([sys.executable, gen, syn, "--segundos", 16, "--movimiento", "12,14"])
    R, _ = spp_decode.analyze(syn, ids={"imu": 4, "baro": 8, "fsm": 2})
    I, B = R["icm"], R["bmp"]
    check("44 B: códigos (v + offset)·8192 enteros; ningún valor fuera de dominio",
          R["imu_formato"] == 44 and I["acel_cuantizacion_lsb_por_g"]["8192"] == 1.0 and I["fuera_de_dominio"] == 0)
    check("baro relativo: p0 implícita constante", B["p0_implicita_pa"][2] - B["p0_implicita_pa"][0] < 1.0)
    kv, r = summary(a.svc, syn)
    check("BANCO alinea con el formato definitivo y el baro tras la calibración",
          kv.get("align_ok") == "1" and int(kv.get("baro_rx", 0)) > 1000 and kv.get("seq_lost_imu") == "0", r.stdout[-400:])
    sg = os.path.join(tmp, "syng.bin")
    run([sys.executable, gen, sg, "--segundos", 16, "--hueco", "3389,34"])
    kv, _ = summary(a.svc, sg)
    check("parones de 34 ms cada 3,39 s (secuencia consecutiva): no alinea", kv.get("align_ok") == "0" and
          kv.get("seq_lost_imu") == "0")
    sl = os.path.join(tmp, "synl.bin")
    run([sys.executable, gen, sl, "--segundos", 8, "--perder-imu", 100])
    kv, _ = summary(a.svc, sl)
    Rl, _ = spp_decode.analyze(sl, ids={"imu": 4, "baro": 8, "fsm": 2})
    check("pérdidas de secuencia: servicio y lector cuentan lo mismo; motivo IMU_LOSS publicado",
          kv.get("seq_lost_imu") == str(Rl["ids"]["imu"]["secuencia"]["perdidos"]) and int(kv.get("motivos_svc", "0"), 16) & 0x2)
    print("E. las llamadas del servicio por nav_replay: comparación exacta")
    for path, extra, nm in ((syn, [], "sintético 44 B"), (os.path.join(tmp, "spp_dump.bin"), ["--historico"], "captura 48 B")):
        for every, lag in ((1, 0), (3, 2500), (7, 0)):
            fi, fo = os.path.join(tmp, "in.txt"), os.path.join(tmp, "out.txt")
            run([a.svc, *extra, "--entradas", fi, "--salidas", fo, "--consumir-cada", every, "--retardo-us", lag, path])
            r = run([a.nav_replay], stdin=open(fi))
            t1 = open(fo).read()
            check(f"{nm}, consumir cada {every}, retardo {lag} µs: idéntico ({len(t1.splitlines())} líneas)",
                  r.returncode == 0 and r.stdout == t1 and t1, r.stderr[:200])
        conv, _ = spp_to_replay.convert(open(path, "rb").read(), [l.strip() for l in open(cfg) if l.startswith("CFG")],
                                        *((0x8, 0x4) if extra else (0x4, 0x8)), hist=bool(extra))
        mine = sorted(l for l in conv if l.startswith(("IMU", "BARO")))
        svc_in = sorted(l.strip() for l in open(os.path.join(tmp, "in.txt")) if l.startswith(("IMU", "BARO")))
        check(f"{nm}: conversor Python = entradas del servicio ({len(mine)} eventos)", mine == svc_in)
    print("F. traza de validación: otro proceso reproduce el mismo calendario")
    for path, extra, nm in ((syn, [], "sintético"), (os.path.join(tmp, "spp_dump.bin"), ["--historico"], "captura")):
        tr, ma, mb = (os.path.join(tmp, x) for x in ("tr.bin", "ma.bin", "mb.bin"))
        run([a.svc, *extra, "--consumir-cada", 3, "--retardo-us", 2500, "--traza-val", tr, "--mensajes", ma, path])
        r = run([a.svc, *extra, "--reproducir", tr, "--mensajes", mb, path])
        A, Bm = open(ma, "rb").read(), open(mb, "rb").read()
        check(f"{nm}: mensajes de salida idénticos byte a byte ({len(A) // 64} paquetes; traza {os.path.getsize(tr)} B)",
              r.returncode == 0 and A == Bm and len(A) > 0, r.stderr)


def pkts64(path):
    d = open(path, "rb").read()
    return [d[i:i + 64] for i in range(0, len(d), 64)]


def spp_fix(p):
    """Recalcula el CRC SPP (bytes 62–63) de un paquete de 64 B."""
    p = bytearray(p)
    p[62:64] = spp_wire.crc16(p[:62]).to_bytes(2, "little")
    return bytes(p)


# posición (tipo de fragmento, desplazamiento en sus datos, nº de componentes) de cada campo del snapshot (esquema v4)
SNAP_FIELDS = {"p": (2, 0, 3), "v": (2, 12, 3), "alt_ref": (2, 24, 1), "q": (3, 0, 4), "sigma_th": (3, 16, 3),
               "sigma_p": (4, 0, 3), "sigma_v": (4, 12, 3), "b_a": (5, 0, 3), "b_g": (5, 12, 3)}


def snapshot_packets(pk, out_id=0x10):
    """Índices de los paquetes de cada snapshot completo: [(sesión, id, {tipo: índice})] en orden."""
    res, cur = [], None
    for i, q in enumerate(pk):
        if int.from_bytes(q[1:3], "little") != out_id or q[13] not in (1, 2, 3, 4, 5):
            continue
        key = (int.from_bytes(q[16:18], "little"), int.from_bytes(q[18:22], "little"))
        if q[13] == 1:
            cur = (key[0], key[1], {1: i})
        elif cur and (cur[0], cur[1]) == key:
            cur[2][q[13]] = i
            if q[13] == 5 and len(cur[2]) == 5:
                res.append(cur)
                cur = None
    return res


def patch_snapshot(pk, frags, changes):
    """Copia de la captura con campos del snapshot alterados {campo: {índice: valor}}; recalcula el CRC
    interno (en la cabecera de TODOS sus fragmentos) y el CRC SPP de cada paquete: sólo cambia el valor."""
    import struct
    pk = list(pk)
    data = {t: bytearray(pk[i][24:24 + pk[i][5] - 12]) for t, i in frags.items()}
    for f, comps in changes.items():
        t, off, n = SNAP_FIELDS[f]
        for k, v in comps.items():
            struct.pack_into("<f", data[t], off + 4 * k, v)
    c = 0xFFFF
    for t in sorted(data):
        c = spp_wire.binascii.crc_hqx(bytes(data[t]), c)
    for t, i in frags.items():
        q = bytearray(pk[i])
        q[24:24 + len(data[t])] = data[t]
        q[22:24] = c.to_bytes(2, "little")
        pk[i] = spp_fix(q)
    return pk


def patch_ops(pk, n, field, value):
    """Copia con el n-ésimo mensaje compacto alterado (campo f32), con su CRC interno y el de SPP."""
    import struct
    offs = {"eskf_h_m": 22, "eskf_vz_mps": 26, "bb_h_m": 30, "bb_vz_mps": 34}
    idx = [i for i, q in enumerate(pk) if int.from_bytes(q[1:3], "little") == 0x10 and q[13] == 7]
    pk = list(pk)
    q = bytearray(pk[idx[n]])
    struct.pack_into("<f", q, 24 + offs[field], value)
    q[22:24] = spp_wire.binascii.crc_hqx(bytes(q[24:24 + 38]), 0xFFFF).to_bytes(2, "little")
    pk[idx[n]] = spp_fix(q)
    return pk


def test_router_replay(a, tmp):
    print("H. router REAL: la traza del pegamento reproduce los paquetes completos; reproducción estricta")
    d = os.path.join(tmp, "cong")
    os.makedirs(d, exist_ok=True)
    r = run([a.router, "congestion", d])
    check("escenario de congestión con el router SPP real", r.returncode == 0, r.stdout[-400:])
    fi, fo, ft = (os.path.join(d, x) for x in ("congestion_entradas.bin", "congestion_salida_real.bin", "congestion_traza.bin"))
    rep = os.path.join(d, "rep.bin")
    r2 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--reproducir", ft, "--mensajes", rep, "--resumen", fi])
    real, again = open(fo, "rb").read(), open(rep, "rb").read()
    kv = dict(re.findall(r"(\w+)=([^\s]+)", r2.stdout))
    check(f"congestión: {len(again) // 64} paquetes reproducidos = {len(real) // 64} recibidos por el registro, idénticos "
          f"byte a byte (aplazados {kv.get('aplazados')})", r2.returncode == 0 and real == again and len(real) > 0 and
          int(kv.get("aplazados", 0)) > 0, r2.stderr)
    r3 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--reproducir", ft, "--mensajes", rep, "--telemetria", fi])
    check("una traza de OTRA configuración se detecta (código 6)", r3.returncode == 6 and "OTRA configuración" in r3.stderr,
          r3.stderr)
    r4 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--mensajes", rep, fi])
    cal = open(rep, "rb").read()
    check(f"el calendario propio (PC determinista) NO es el comportamiento del router: {len(cal) // 64} frente a "
          f"{len(real) // 64} paquetes", r4.returncode == 0 and cal != real)
    # reloj que avanza entre la construcción del paquete y la anotación
    d2 = os.path.join(tmp, "reloj")
    os.makedirs(d2, exist_ok=True)
    r = run([a.router, "reloj_avanza", d2])
    check("router real con un reloj que avanza en cada lectura", r.returncode == 0, r.stdout[-300:])
    fi2, fo2, ft2 = (os.path.join(d2, x) for x in ("reloj_entradas.bin", "reloj_salida_real.bin", "reloj_traza.bin"))
    rep2 = os.path.join(d2, "rep.bin")
    r5 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--reproducir", ft2, "--mensajes", rep2, fi2])
    A, B = pkts64(fo2), pkts64(rep2)
    pay_eq = len(A) == len(B) and all(x[12:12 + x[5]] == y[12:12 + y[5]] for x, y in zip(A, B))
    full_eq = A == B
    # el escenario cruza milisegundos: la cabecera no coincide con el último reloj del servicio anterior
    recs = open(ft2, "rb").read()
    last_clk, crossed, outs = None, 0, 0
    for i in range(0, len(recs), 16):
        typ, res = recs[i], recs[i + 1]
        t = int.from_bytes(recs[i + 8:i + 16], "little", signed=True)
        if typ in (1, 2):
            last_clk = t
        elif typ == 5 and res == 0:
            outs += 1
            crossed += last_clk is not None and t != last_clk // 1000
    check(f"igualdad de CARGAS ÚTILES: {len(B)} reproducidos frente a {len(A)} originales", r5.returncode == 0 and pay_eq,
          r5.stderr)
    check(f"igualdad de PAQUETES COMPLETOS (cabecera incluida): {sum(x == y for x, y in zip(A, B))}/{len(A)}; en {crossed} de "
          f"{outs} el timestampMs del paquete ≠ ms del último reloj del servicio", full_eq and crossed > 0)
    # reproducción estricta: ningún éxito silencioso
    t3 = os.path.join(d, "trunc.bin")
    open(t3, "wb").write(open(ft, "rb").read()[:-3])
    r6 = run([a.svc, "--perfil", "exp", "--reproducir", t3, "--mensajes", rep, fi])
    r2 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--reproducir", ft, "--mensajes", rep, fi])
    check(f"traza truncada 3 B: error (código {r6.returncode}) con el registro afectado — {r6.stderr.strip()[:110]}",
          r6.returncode == 5 and "TRUNCADA" in r6.stderr and "registro" in r6.stderr)
    t4 = os.path.join(d, "v1.bin")
    for ver in (1, 2, 3, 4):
        tr = bytearray(open(ft, "rb").read())
        tr[1] = ver
        open(t4, "wb").write(tr)
        r7 = run([a.svc, "--perfil", "exp", "--reproducir", t4, "--mensajes", rep, fi])
        check(f"traza de versión {ver}: rechazada (código 6) con un mensaje útil — {r7.stderr.strip()[:100]}",
              r7.returncode == 6 and f"versión {ver}" in r7.stderr and "END" in r7.stderr, r7.stderr)
    tr = bytearray(open(ft, "rb").read())
    k = next(i for i in range(16, len(tr), 16) if tr[i] == 1 and i > 16 * 200)
    tr[k + 4] ^= 0x01  # secuencia de una entrega
    t5 = os.path.join(d, "dif.bin")
    open(t5, "wb").write(end_fix(tr))  # con el END recalculado: lo detecta el contraste con las entradas
    r8 = run([a.svc, "--perfil", "exp", "--reproducir", t5, "--mensajes", rep, fi])
    check(f"entrega incoherente con las entradas: código 4 en el registro {k // 16} — {r8.stderr.strip()[:90]}",
          r8.returncode == 4 and f"registro {k // 16} (byte {k})" in r8.stderr, r8.stderr)
    trace_v3_checks(a, d, ft, fi, rep, r2)
    f6 = os.path.join(d, "mas.bin")
    open(f6, "wb").write(open(fi, "rb").read() + open(fi, "rb").read()[:640])
    r9 = run([a.svc, "--perfil", "exp", "--reproducir", ft, "--mensajes", rep, f6])
    check("entradas sobrantes tras la traza: código 4", r9.returncode == 4 and "quedan 10 paquetes" in r9.stderr, r9.stderr)
    f7 = os.path.join(d, "menos.bin")
    open(f7, "wb").write(open(fi, "rb").read()[:-64])
    r10 = run([a.svc, "--perfil", "exp", "--reproducir", ft, "--mensajes", rep, f7])
    check("entradas agotadas antes que la traza: código 4", r10.returncode == 4 and "agotadas" in r10.stderr, r10.stderr)
    f8 = os.path.join(d, "corta.bin")
    open(f8, "wb").write(open(fi, "rb").read()[:-5])
    r11 = run([a.svc, "--perfil", "exp", "--reproducir", ft, "--mensajes", rep, f8])
    check("entradas truncadas (no múltiplo de 64 B): código 5", r11.returncode == 5 and "TRUNCADAS" in r11.stderr, r11.stderr)


def end_fix(tr):
    """Recalcula el CRC del END (bytes 2–3 del último registro) sobre los registros anteriores."""
    tr = bytearray(tr)
    tr[-14:-12] = spp_wire.binascii.crc_hqx(bytes(tr[:-16]), 0xFFFF).to_bytes(2, "little")
    return tr


def trace_v3_checks(a, d, ft, fi, rep, r_full):
    """Cierre e integridad de la traza (v3+) y contraste de sus metadatos con la reproducción."""
    T = open(ft, "rb").read()
    n = len(T) // 16

    def replay(tr, *extra, name="x.bin"):
        path = os.path.join(d, name)
        open(path, "wb").write(bytes(tr))
        out = os.path.join(d, "x_out.bin")
        r = run([a.svc, "--perfil", "exp", "--reproducir", path, "--mensajes", out, *extra, fi])
        return r, (len(open(out, "rb").read()) // 64 if os.path.exists(out) else 0)
    r_full, full_pk = replay(T)
    check(f"traza completa del router real: {n} registros, termina en END con recuento y CRC; reproducción COMPLETA "
          f"(código 0, {full_pk} paquetes)", T[-16] == 7 and int.from_bytes(T[-8:], "little") == n - 1 and
          r_full.returncode == 0 and "COMPLETA" in r_full.stderr and full_pk > 0, r_full.stderr[-300:])
    # eliminación de registros COMPLETOS al final (antes: 395 → 394 paquetes con éxito)
    for k in (1, 2, 40):
        r, npk = replay(T[:-16 * k])
        check(f"REGRESIÓN: sin los {k} últimos registros completos (múltiplo de 16 B): código {r.returncode}, «sin cierre»",
              r.returncode == 5 and "SIN registro de cierre" in r.stderr, r.stderr)
    # truncamiento dentro de un registro, al final y con el END presente
    r, _ = replay(T[:-16 - 5])
    check(f"truncada dentro de un registro: código {r.returncode}", r.returncode == 5 and "TRUNCADA" in r.stderr, r.stderr)
    # un registro completo eliminado en medio, con el END original: recuento distinto
    k = next(i for i in range(n // 2, n) if T[16 * i] == 5)
    r, _ = replay(T[:16 * k] + T[16 * (k + 1):])
    check(f"un registro OUTPUT eliminado en medio (END intacto): código {r.returncode}, faltan registros",
          r.returncode == 5 and "faltan o sobran" in r.stderr, r.stderr)
    # metadatos de una salida: tipo 99 y longitud 1 (antes: éxito)
    alt = bytearray(T)
    alt[16 * k + 2:16 * k + 6] = (99).to_bytes(2, "little") + (1).to_bytes(2, "little")
    r, _ = replay(alt)
    check(f"OUTPUT alterado a tipo 99 / longitud 1 sin rehacer el END: código {r.returncode}, CRC distinto",
          r.returncode == 4 and "alterados" in r.stderr, r.stderr)
    r, _ = replay(end_fix(alt))
    check(f"REGRESIÓN: OUTPUT tipo 99 / longitud 1 con el END recalculado: código {r.returncode} en el registro {k} — el "
          f"mensaje producido es otro", r.returncode == 4 and f"registro {k} (byte {16 * k})" in r.stderr and "tipo 99" in r.stderr
          and "metadatos distintos" in r.stderr, r.stderr)
    # identidad, longitud y resultado de una entrada; resultado de un consume; reloj de una salida
    kd = next(i for i in range(n // 3, n) if T[16 * i] == 1)
    for label, pos, val in (("longitud de una entrada", 6, 49), ("resultado de una entrega", 1, 0xFD)):
        alt = bytearray(T)
        alt[16 * kd + pos] = val
        r, _ = replay(end_fix(alt))
        check(f"DELIVER con otra {label} (END recalculado): código {r.returncode} en el registro {kd}",
              r.returncode == 4 and f"registro {kd} " in r.stderr and "metadatos distintos" in r.stderr, r.stderr)
    kc = next(i for i in range(n // 3, n) if T[16 * i] == 2 and T[16 * i + 6] != 0)
    alt = bytearray(T)
    alt[16 * kc + 6] ^= 0x02
    r, _ = replay(end_fix(alt))
    check(f"CONSUME con otros mensajes construidos (END recalculado): código {r.returncode} en el registro {kc}",
          r.returncode == 4 and f"registro {kc} " in r.stderr, r.stderr)
    alt = bytearray(T)
    alt[16 * k + 8] ^= 0x01
    r, _ = replay(end_fix(alt))
    check(f"OUTPUT con otro timestampMs (END recalculado): código {r.returncode}", r.returncode == 4 and
          f"registro {k} " in r.stderr, r.stderr)
    # desbordamiento declarado y END en mitad
    alt = bytearray(T)
    alt[-12:-10] = (3).to_bytes(2, "little")
    r, _ = replay(alt)
    check(f"END que declara 3 registros perdidos por desbordamiento: código {r.returncode}",
          r.returncode == 5 and "desbordamiento" in r.stderr, r.stderr)
    r, _ = replay(T[:16 * k] + T[-16:] + T[16 * k:])
    check(f"END en mitad de la traza: código {r.returncode}", r.returncode == 4 and "END en mitad" in r.stderr, r.stderr)
    # reproducción PARCIAL: sólo si se pide; distinta de la completa
    cut = T[:16 * k] + T[16 * k:16 * k + 7]
    r, npk = replay(cut, "--parcial")
    check(f"--parcial con una traza sin cierre (cortada a mitad de registro): código {r.returncode}, «PARCIAL», {npk} paquetes "
          f"de {full_pk}", r.returncode == 3 and "PARCIAL" in r.stderr and "no completa" in r.stderr and 0 < npk < full_pk,
          r.stderr)
    r, _ = replay(T, "--parcial")
    check("--parcial con una traza cerrada: se comprueba entera y es COMPLETA (código 0)", r.returncode == 0 and
          "COMPLETA" in r.stderr, r.stderr)
    alt = bytearray(T[:16 * (k + 1)])
    alt[16 * k + 2:16 * k + 6] = (99).to_bytes(2, "little") + (1).to_bytes(2, "little")
    r, _ = replay(alt, "--parcial")
    check(f"--parcial también contrasta los metadatos: código {r.returncode}", r.returncode == 4 and "tipo 99" in r.stderr,
          r.stderr)


def test_flight(a, tmp):
    print("I. escenarios de vuelo sintéticos (series OpenRocket «Simulation 6»; datos simulados, no validación)")
    sys.path.insert(0, os.path.join(SVC, "tools"))
    import flight_check  # noqa: E402
    res = {}
    for nm in ("nominal", "adverso"):
        work = os.path.join(tmp, nm)
        os.makedirs(work, exist_ok=True)
        M = flight_check.run(a.svc, os.path.join(DATA, f"openrocket_sim6_{nm}.csv"), work)
        res[nm] = M
        F = M["fases"]
        check(f"{nm}: vuelo ENTERO simulado ({M['duracion_simulada_s']:.1f} s tras 20 s de rampa, hasta el impacto a "
              f"{M['eventos'].get('groundhit')} s)", M["duracion_simulada_s"] >= M["eventos"].get("groundhit", 1e9) - 0.01)
        check(f"{nm}: rampa sin NO VÁLIDA tras alinear ({F['rampa']['validez_servicio']}); VÁLIDA y fuente ESKF al final",
              "NO VÁLIDA" not in F["rampa"]["validez_servicio"] and F["rampa"]["validez_final"] == "VÁLIDA" and
              "nav_eskf" in F["rampa"]["capacidades_finales"] and F["rampa"]["fuente"][-1:] != [])
        check(f"{nm}: despegue detectado en ≤ 0.15 s ({M['despegue_detectado_s']:.2f} s)",
              M["despegue_detectado_s"] is not None and M["despegue_detectado_s"] <= 0.15)
        P = F["propulsión"]
        check(f"{nm}: propulsión recortada a ±4 g → DEGRADADA con ACC_SAT (nunca VÁLIDA tras el recorte), σv_z hasta "
              f"{P['eskf_sigma_vz_max_mps']:.1f} m/s", "DEGRADADA" in P["validez_servicio"] and "ACC_SAT" in P["motivos_nucleo"]
              and "SAT_CANDIDATE" in P["motivos_servicio"] and P["eskf_sigma_vz_max_mps"] > 10 and
              M["validas_tras_recorte"] == 0)
        check(f"{nm}: el error de altura ESKF en propulsión ({P['eskf_err_h_max_m']:.2f} m) y de velocidad "
              f"({P['eskf_err_vz_max_mps']:.2f} m/s) queda dentro de 3σ publicada",
              P["eskf_err_h_max_m"] < 2.0 and P["eskf_err_vz_max_mps"] < 3 * P["eskf_sigma_vz_max_mps"])
        ap_ = M["apogeo_verdad_m"]
        check(f"{nm}: apogeo ESKF {M['apogeo_eskf_m']:.1f} m y respaldo {M['apogeo_respaldo_m']:.1f} m frente a "
              f"{ap_:.1f} m de la serie (±2 m)", abs(M["apogeo_eskf_m"] - ap_) < 2 and abs(M["apogeo_respaldo_m"] - ap_) < 2)
        check(f"{nm}: respaldo barométrico válido en todo el vuelo hasta el aterrizaje; |Δh| máx en planeo "
              f"{F['planeo']['respaldo_err_h_max_m']:.2f} m, en el aterrizaje {F['aterrizaje']['respaldo_err_h_max_m']:.2f} m",
              all(F[k]["respaldo_valido_frac"] == 1.0 for k in ("propulsión", "planeo", "apogeo", "descenso", "aterrizaje"))
              and F["aterrizaje"]["respaldo_err_h_max_m"] < 1.0)
    Fn, Fa = res["nominal"]["fases"], res["adverso"]["fases"]
    check(f"nominal: despliegue a 3.7 g (sin recorte) → ESKF utilizable hasta el aterrizaje, fuente ESKF; |Δh| "
          f"{Fn['aterrizaje']['eskf_err_h_max_m']:.2f} m al aterrizar",
          Fn["descenso"]["fuente"] == ["eskf"] and Fn["aterrizaje"]["fuente"] == ["eskf"] and
          "NO VÁLIDA" not in Fn["descenso"]["validez_servicio"] + Fn["aterrizaje"]["validez_servicio"] and
          res["nominal"]["eskf_invalida_desde_s"] is None and Fn["aterrizaje"]["eskf_err_h_max_m"] < 1.0)
    t_inv = res["adverso"]["eskf_invalida_desde_s"]
    check(f"adverso: despliegue a 57 g fuera de la ventana del motor → SAT_OUT_OF_MODEL, ESKF NO VÁLIDA desde "
          f"{t_inv:.2f} s hasta el aterrizaje y fuente BARO", "SAT_OUT_OF_MODEL" in Fa["descenso"]["motivos_servicio"] and
          t_inv is not None and 9.0 < t_inv < 9.4 and "baro" in Fa["descenso"]["fuente"] and
          Fa["aterrizaje"]["validez_servicio"] == ["NO VÁLIDA"] and Fa["aterrizaje"]["fuente"] == ["baro"])
    if a.resultados:
        import json
        json.dump(res, open(os.path.join(a.resultados, "escenarios_vuelo.json"), "w"), indent=1, ensure_ascii=False)
    return res


def test_compare(a, tmp):
    print("J. comparación numérica placa–PC (otra aritmética: FMA), falsos éxitos y herramientas de salida")
    work = os.path.join(tmp, "nominal")
    log = os.path.join(work, "vuelo.bin")
    cmp_py = os.path.join(SVC, "tools", "compare_runs.py")
    tr, fm, pcm, pcl = (os.path.join(tmp, x) for x in ("fma_traza.bin", "fma.bin", "pc.bin", "pc.txt"))
    run([a.svc_fma, "--perfil", "exp", "--consumir-cada", 3, "--retardo-us", 1500, "--traza-val", tr, "--mensajes", fm, log])
    rr = run([a.svc, "--perfil", "exp", "--reproducir", tr, "--mensajes", pcm, log])
    r = r_base = run([sys.executable, cmp_py, "--placa", fm, "--pc-mensajes", pcm, "--out-id", "0x10"])
    check("«placa» compilada con FMA y otro calendario frente al PC con su traza: binario distinto, EQUIVALENTE dentro "
          "de tolerancia, todas las salidas emparejadas", rr.returncode == 0 and r.returncode == 0 and
          open(fm, "rb").read() != open(pcm, "rb").read() and "EQUIVALENTE" in r.stdout, r.stdout + rr.stderr)
    print("     " + r.stdout.strip().replace("\n", "\n     "))
    T = open(tr, "rb").read()
    open(tr + ".cut", "wb").write(T[:-16])
    rc = run([a.svc, "--perfil", "exp", "--reproducir", tr + ".cut", "--mensajes", pcm + ".cut", log])
    check(f"REGRESIÓN: traza del calendario propio (termina en salidas) sin su último registro completo: código {rc.returncode} "
          f"(antes: éxito con un paquete menos)", T[-32] == 5 and T[-16] == 7 and rc.returncode == 5 and
          "SIN registro de cierre" in rc.stderr, rc.stderr)
    # falso éxito 1: 48 snapshots completos en la placa y 47 en el PC (captura del PC acabada antes)
    A, P = pkts64(fm), pkts64(pcm)

    def cut_after(pk, n_snap):
        n = 0
        for i, q in enumerate(pk):
            if q[12 + 1] == 5:  # último fragmento (máscara completa)
                n += 1
                if n == n_snap:
                    return pk[:i + 1]
        return pk
    b48, p47 = os.path.join(tmp, "b48.bin"), os.path.join(tmp, "p47.bin")
    open(b48, "wb").write(b"".join(cut_after(A, 48)))
    open(p47, "wb").write(b"".join(cut_after(P, 47)))
    r = run([sys.executable, cmp_py, "--placa", b48, "--pc-mensajes", p47, "--out-id", "0x10"])
    check("REGRESIÓN: 47 snapshots emparejados + 1 completo de la placa SIN pareja → NO equivalente (código 1)",
          r.returncode == 1 and "SIN pareja en el PC" in r.stdout, r.stdout)
    # falso éxito 2: capturas incompletas de la placa
    gap = A[:300] + A[301:]
    g = os.path.join(tmp, "hueco.bin")
    open(g, "wb").write(b"".join(gap))
    r = run([sys.executable, cmp_py, "--placa", g, "--pc-mensajes", pcm, "--out-id", "0x10"])
    check("captura de la placa con un paquete perdido: NO equivalente", r.returncode == 1 and "secuencia SPP" in r.stdout, r.stdout)
    k = next(i for i in range(len(A) - 1, 0, -1) if A[i][13] == 3)
    tail = os.path.join(tmp, "cola.bin")
    open(tail, "wb").write(b"".join(A[:k + 1]))
    r = run([sys.executable, cmp_py, "--placa", tail, "--pc-mensajes", pcm, "--out-id", "0x10"])
    check("captura de la placa cortada a mitad de snapshot: NO equivalente", r.returncode == 1 and "sin cerrar" in r.stdout,
          r.stdout)
    other = os.path.join(tmp, "adverso", "salida.bin")
    r = run([sys.executable, cmp_py, "--placa", other, "--pc-mensajes", pcm, "--out-id", "0x10"])
    check("otro vuelo: NO equivalente (la herramienta discrimina)", r.returncode == 1, r.stdout)
    # valores no finitos: componente a componente, en posiciones que no son la primera
    run([a.svc, "--perfil", "exp", "--salidas", pcl, log])
    nan, inf = float("nan"), float("inf")
    snaps = snapshot_packets(A)
    dec = {}
    rr_ = nav_out_decode.Reasm()
    for q in A:
        k_ = rr_.push(q[12:12 + q[5]])
        if k_ and k_[0] == "snap":
            dec[(k_[1]["session"], k_[1]["snap_id"])] = k_[1]
    n_pre = sum(1 for x in dec.values() if x.get("t_state_us") is None)
    sel = next(x for x in snaps[300:] if dec[(x[0], x[1])].get("t_state_us") is not None and
               dec[(x[0], x[1])]["svc_validity"] >= 1 and dec[(x[0], x[1])]["alt_ref_seq"] > 0)
    check(f"comparación base: {n_pre} snapshots SIN estado (antes de alinear) con NaN legítimos en ambas, comparados sin "
          f"fallo; el snapshot alterado abajo es {sel[:2]} (con estado, validez {dec[sel[:2]]['svc_validity']})",
          n_pre > 0 and r_base.returncode == 0)

    def bad_board(changes, label, expect, pc_too=False, partial=False):
        b = os.path.join(tmp, "nf_b.bin")
        open(b, "wb").write(b"".join(patch_snapshot(A, sel[2], changes)))
        pcx = pcm
        if pc_too:
            pcx = os.path.join(tmp, "nf_p.bin")
            sp = next(x for x in snapshot_packets(P) if x[:2] == sel[:2])
            open(pcx, "wb").write(b"".join(patch_snapshot(P, sp[2], changes)))
        if partial:
            rx = run([sys.executable, cmp_py, "--parcial", "--placa", b, "--pc", pcl, "--out-id", "0x10"])
        else:
            rx = run([sys.executable, cmp_py, "--placa", b, "--pc-mensajes", pcx, "--out-id", "0x10"])
        check(f"{label}: código {rx.returncode}, «{expect}» señalado", rx.returncode == 1 and expect in rx.stdout and
              "EQUIVALENTE (" not in rx.stdout, rx.stdout[-600:])
    bad_board({"p": {1: nan}}, "REGRESIÓN: sólo la posición Y = NaN (antes: código 0, EQUIVALENTE, dif. 0)", "p[1]")
    bad_board({"v": {2: nan}}, "REGRESIÓN: sólo la velocidad Z = NaN (antes: código 0, EQUIVALENTE)", "v[2]")
    bad_board({"v": {1: inf}}, "velocidad Y = +inf", "v[1]")
    bad_board({"b_a": {2: nan}}, "sesgo de acelerómetro Z = NaN", "b_a[2]")
    bad_board({"b_g": {1: -inf}}, "sesgo de giróscopo Y = −inf", "b_g[1]")
    bad_board({"sigma_p": {2: nan}}, "σ_p Z = NaN", "sigma_p[2]")
    bad_board({"sigma_v": {1: inf}}, "σ_v Y = +inf", "sigma_v[1]")
    bad_board({"sigma_th": {2: nan}}, "σ_θ Z = NaN", "sigma_th[2]")
    bad_board({"alt_ref": {0: nan}}, "alt_ref = NaN con referencia publicada (alt_ref_seq > 0)", "alt_ref[0]")
    bad_board({"q": {3: nan}}, "cuaternión con q_z = NaN: diagnóstico explícito", "cuaternión no finito")
    q0 = dec[sel[:2]]["q"]
    bad_board({"q": {i: 2.0 * q0[i] for i in range(4)}}, "cuaternión finito de norma 2: diagnóstico explícito",
              "cuaternión de norma 2")
    bad_board({"p": {1: nan}}, "p[1] = NaN en AMBAS capturas con el estado publicado: no es «no disponible»", "p[1]",
              pc_too=True)
    bad_board({"v": {2: nan}}, "--parcial con v_z = NaN en la placa: fallo, no «parcial superada»", "v[2]", partial=True)
    bo = os.path.join(tmp, "nf_o.bin")
    n_ops = next(i for i, q in enumerate([q for q in A if q[13] == 7]) if i > 50 and q[24 + 20] & 0x40)
    open(bo, "wb").write(b"".join(patch_ops(A, n_ops, "bb_h_m", nan)))
    rx = run([sys.executable, cmp_py, "--placa", bo, "--pc-mensajes", pcm, "--out-id", "0x10"])
    check(f"compacta con respaldo VÁLIDO y altura del respaldo NaN: código {rx.returncode}", rx.returncode == 1 and
          "bb_h_m" in rx.stdout, rx.stdout[-500:])
    # modo parcial: explícito y marcado
    r = run([sys.executable, cmp_py, "--placa", fm, "--pc", pcl, "--out-id", "0x10"])
    check("líneas OUT sin --parcial: rechazado (no puede dar equivalencia)", r.returncode == 2, r.stdout)
    r = run([sys.executable, cmp_py, "--parcial", "--placa", fm, "--pc", pcl, "--out-id", "0x10"])
    check("--parcial: marcado PARCIAL, nunca «equivalente»; los t_state con varios estados posibles quedan sin comparar "
          "(no se elige el de menor error)", r.returncode in (2, 3) and "PARCIAL" in r.stdout and "ambigua" in r.stdout and
          "EQUIVALENTE" not in r.stdout, r.stdout)
    print("     " + r.stdout.strip())
    d = run([sys.executable, os.path.join(SVC, "tools", "nav_out_decode.py"), os.path.join(work, "salida.bin"), "--out-id",
             "0x10", "--ops-csv", os.path.join(tmp, "ops.csv")])
    check("decodificador Python v4: snapshots, diagnóstico y compacta sin incidencias",
          d.returncode == 0 and "incompletos 0" in d.stdout and "compactas" in d.stdout, d.stdout)
    cdir = os.path.join(tmp, "csv")
    c = run([sys.executable, os.path.join(SVC, "tools", "spp_decode.py"), log, "--imu-id", "4", "--baro-id", "8", "--csv", cdir])
    ni = sum(1 for _ in open(os.path.join(cdir, "imu.csv"))) - 1
    nb = sum(1 for _ in open(os.path.join(cdir, "baro.csv"))) - 1
    nin = len(pkts64(log))
    check(f"spp_decode --csv: imu.csv ({ni}) + baro.csv ({nb}) = {nin} paquetes", c.returncode == 0 and ni + nb == nin and nb > 0,
          c.stdout[-300:])


def test_decoder(a, tmp):
    print("K. decodificador de salida: CRC SPP (capa exterior) frente al CRC de los mensajes (interior); comprobación y rescate")
    work = os.path.join(tmp, "nominal")
    dec = os.path.join(SVC, "tools", "nav_out_decode.py")

    def bad_spp(pk, i):
        q = bytearray(pk[i])
        q[62] ^= 0x5A  # CRC SPP incorrecto; el contenido no cambia
        return pk[:i] + [bytes(q)] + pk[i + 1:]

    def dec_run(pk, *extra, name="d.bin"):
        path = os.path.join(tmp, name)
        open(path, "wb").write(b"".join(pk) if isinstance(pk, list) else pk)
        return run([sys.executable, dec, path, "--out-id", "0x10", *extra])

    def counts(out):
        m = re.search(r"(\d+) snapshots completos, (\d+) diagnósticos, (\d+) compactas", out)
        c = re.search(r"rechazados por CRC SPP (\d+) \((\d+) con el KPID de salida\)", out)
        i = re.search(r"CRC interno mal (\d+), incompletos (\d+), mezclados (\d+)", out)
        g = re.search(r"saltos de secuencia de salida (\d+)", out)
        return tuple(int(x) for x in m.groups()), tuple(int(x) for x in c.groups()), tuple(int(x) for x in i.groups()), \
            int(g.group(1))
    # el caso reproducido: 15 compactas (preajuste TELEMETRÍA) y el CRC SPP de una corrompido
    tel = os.path.join(tmp, "tel.bin")
    run([a.svc, "--perfil", "exp", "--telemetria", "--mensajes", tel, os.path.join(work, "vuelo.bin")])
    T = pkts64(tel)
    ops_i = [i for i, q in enumerate(T) if q[13] == 7]
    T15 = T[:ops_i[14] + 1]
    r = dec_run(T15)
    n0 = counts(r.stdout)
    check(f"captura de 15 compactas limpia: código {r.returncode}, {n0[0][2]} compactas, 0 rechazos",
          r.returncode == 0 and n0[0][2] == 15 and n0[1] == (0, 0) and "sin corrupción" in r.stdout, r.stdout)
    r = dec_run(bad_spp(T15, ops_i[7]))
    n = counts(r.stdout)
    check(f"REGRESIÓN: CRC SPP de 1 de las 15 compactas: código {r.returncode}; {n[0][2]} compactas, rechazados por CRC SPP "
          f"{n[1][0]}, CRC interno {n[2][0]} (antes: 14 compactas, «CRC mal 0» y éxito)",
          r.returncode == 1 and n[0][2] == 14 and n[1] == (1, 1) and n[2][0] == 0 and "COMPROBACIÓN FALLIDA" in r.stdout,
          r.stdout)
    r = dec_run(bad_spp(T15, ops_i[7]), "--tolerante")
    check(f"--tolerante: rescata las 14, dice qué descartó y devuelve {r.returncode} (≠ 0)", r.returncode == 3 and
          "MODO TOLERANTE" in r.stdout and "rechazado por CRC SPP" in r.stdout and counts(r.stdout)[0][2] == 14, r.stdout)
    r = dec_run(T15, "--tolerante")
    check("--tolerante con una captura limpia: código 0", r.returncode == 0, r.stdout)
    # registro completo: compactas, diagnósticos y fragmentos de snapshot, al principio y en medio
    P = pkts64(os.path.join(work, "salida.bin"))
    base = counts(dec_run(P).stdout)
    first = {t: next(i for i, q in enumerate(P) if q[13] == t) for t in (1, 3, 5, 6, 7)}
    mid = {t: next(i for i in range(len(P) // 2, len(P)) if P[i][13] == t) for t in (1, 3, 5, 6, 7)}
    names = {1: "fragmento 1 de snapshot", 3: "fragmento 3 de snapshot", 5: "fragmento 5 (último) de snapshot",
             6: "diagnóstico", 7: "compacta"}
    for where, idx in (("primer", first), ("en medio:", mid)):
        for t in (7, 6, 1, 3, 5):
            r = dec_run(bad_spp(P, idx[t]))
            n = counts(r.stdout)
            lost = {7: (0, 0, 1), 6: (0, 1, 0)}.get(t, (1, 0, 0))
            exp_counts = tuple(b - l for b, l in zip(base[0], lost))
            exp_inc = 1 if t in (1, 3, 5) else 0
            check(f"{where} {names[t]} (paquete {idx[t]}) con CRC SPP malo: código {r.returncode}, rechazados por CRC SPP "
                  f"{n[1][0]}, CRC interno {n[2][0]}, incompletos {n[2][1]}, salto de secuencia {n[3]}",
                  r.returncode == 1 and n[0] == exp_counts and n[1] == (1, 1) and n[2][0] == 0 and n[2][1] == exp_inc and
                  n[3] == (0 if idx[t] == 0 else 1), r.stdout)
    # el CRC interno se distingue del exterior
    q = bytearray(P[mid[3]])
    q[30] ^= 0x01
    r = dec_run(P[:mid[3]] + [spp_fix(q)] + P[mid[3] + 1:])
    n = counts(r.stdout)
    check(f"dato de un fragmento alterado con el CRC SPP rehecho: CRC interno {n[2][0]}, CRC SPP {n[1][0]}, código {r.returncode}",
          r.returncode == 1 and n[2][0] == 1 and n[1] == (0, 0), r.stdout)
    # captura incompleta: un paquete menos en medio, y final truncado
    r = dec_run(P[:mid[6]] + P[mid[6] + 1:])
    check(f"un paquete ausente en medio: salto de secuencia {counts(r.stdout)[3]}, código {r.returncode}",
          r.returncode == 1 and counts(r.stdout)[3] == 1, r.stdout)
    r = dec_run(b"".join(P)[:-10])
    check(f"captura con el último paquete truncado: incidencia del lector mostrada, código {r.returncode}",
          r.returncode == 1 and "truncado 1" in r.stdout, r.stdout)
    r = dec_run(P[:mid[7]] + [bytes(64)] * 3 + P[mid[7]:])
    check(f"ceros intermedios: mostrados, código {r.returncode}", r.returncode == 1 and "ceros_intermedios 1" in r.stdout,
          r.stdout)


def patch_caps(pk, frags, caps_or):
    """Copia con la capacidad caps_or añadida al fragmento 1 del snapshot (CRC interno y SPP rehechos)."""
    pk = list(pk)
    data = {t: bytearray(pk[i][24:24 + pk[i][5] - 12]) for t, i in frags.items()}
    data[1][36] |= caps_or
    c = 0xFFFF
    for t in sorted(data):
        c = spp_wire.binascii.crc_hqx(bytes(data[t]), c)
    for t, i in frags.items():
        q = bytearray(pk[i])
        q[24:24 + len(data[t])] = data[t]
        q[22:24] = c.to_bytes(2, "little")
        pk[i] = spp_fix(q)
    return pk


def test_unavailable_fields(a, tmp):
    print("L. comparación de capturas idénticas con campos legítimamente no disponibles; selección sin datos")
    gen = os.path.join(SVC, "tools", "gen_spp.py")
    cmp_py = os.path.join(SVC, "tools", "compare_runs.py")
    dec = os.path.join(SVC, "tools", "nav_out_decode.py")
    d = os.path.join(tmp, "L")
    os.makedirs(d, exist_ok=True)
    g14 = os.path.join(d, "g14.bin")
    run([sys.executable, gen, g14])  # 14 s, configuración por defecto (BMP tras su calibración de 5 s)
    g24 = os.path.join(d, "g24.bin")
    run([sys.executable, gen, g24, "--segundos", 24])
    nobmp = os.path.join(d, "sin_bmp.bin")
    open(nobmp, "wb").write(b"".join(q for q in pkts64(g24) if q[1] == 0x04))  # sólo la IMU

    def pair(log, nm):
        A, T, B = (os.path.join(d, f"{nm}_{x}.bin") for x in ("placa", "traza", "pc"))
        run([a.svc, "--perfil", "exp", "--mensajes", A, "--traza-val", T, log])
        r = run([a.svc, "--perfil", "exp", "--reproducir", T, "--mensajes", B, log])
        return A, B, r

    def snaps_of(path):
        r = nav_out_decode.Reasm()
        out = []
        for q in pkts64(path):
            k = r.push(q[12:12 + q[5]])
            if k and k[0] == "snap":
                out.append(k[1])
        return out
    for nm, log, what in (("g14", g14, "arranque normal, 14 s (gen_spp por defecto)"),
                          ("g24", g24, "arranque normal hasta la referencia barométrica (24 s)"),
                          ("sinbmp", nobmp, "sin BMP")):
        A, B, rr = pair(log, nm)
        S_ = snaps_of(A)
        n_noref = sum(1 for s in S_ if s.get("nav_validity") == 2 and not (s["caps"] & 0x02) and s["alt_ref"] != s["alt_ref"])
        n_ref = sum(1 for s in S_ if s["caps"] & 0x02)
        r = run([sys.executable, cmp_py, "--placa", A, "--pc-mensajes", B, "--out-id", "0x10"])
        ok_case = {"g14": n_noref > 0, "g24": n_noref > 0 and n_ref > 0, "sinbmp": n_noref > 0 and n_ref == 0}[nm]
        if nm == "g14":
            msgs = A  # la captura del caso reproducido (395 paquetes)
        check(f"REGRESIÓN {what}: mensajes idénticos byte a byte → EQUIVALENTE (código {r.returncode}); {n_noref} snapshots "
              f"válidos inercialmente con alt_ref NaN sin BARO_REF, {n_ref} con referencia",
              rr.returncode == 0 and open(A, "rb").read() == open(B, "rb").read() and r.returncode == 0 and
              "EQUIVALENTE (" in r.stdout and ok_case, r.stdout[-600:] + rr.stderr[-300:])
        if nm == "g24":
            P, Q = pkts64(A), pkts64(B)
            sp, sq = snapshot_packets(P), snapshot_packets(Q)
            dec_ = {(s["session"], s["snap_id"]): s for s in S_}
            with_ref = next(x for x in sp if dec_[x[:2]]["caps"] & 0x02)
            no_ref = next(x for x in sp if dec_[x[:2]].get("nav_validity") == 2 and not dec_[x[:2]]["caps"] & 0x02)
            # referencia declarada disponible y NaN: en una captura y en las dos
            b1 = os.path.join(d, "neg1.bin")
            open(b1, "wb").write(b"".join(patch_snapshot(P, with_ref[2], {"alt_ref": {0: float("nan")}})))
            r = run([sys.executable, cmp_py, "--placa", b1, "--pc-mensajes", B, "--out-id", "0x10"])
            check(f"BARO_REF declarada y alt_ref NaN en la placa: código {r.returncode}", r.returncode == 1 and "alt_ref" in r.stdout,
                  r.stdout[-500:])
            b2, b3 = os.path.join(d, "neg2.bin"), os.path.join(d, "neg3.bin")
            k_ = next(x for x in sq if x[:2] == with_ref[:2])
            open(b2, "wb").write(b"".join(patch_snapshot(P, with_ref[2], {"alt_ref": {0: float("nan")}})))
            open(b3, "wb").write(b"".join(patch_snapshot(Q, k_[2], {"alt_ref": {0: float("nan")}})))
            r = run([sys.executable, cmp_py, "--placa", b2, "--pc-mensajes", b3, "--out-id", "0x10"])
            check(f"BARO_REF declarada y alt_ref NaN en AMBAS capturas: código {r.returncode} (dos NaN no valen con la "
                  f"referencia disponible)", r.returncode == 1 and "alt_ref" in r.stdout, r.stdout[-500:])
            # sin BARO_REF, se añade la capacidad en las dos y alt_ref sigue NaN
            k2 = next(x for x in sq if x[:2] == no_ref[:2])
            open(b2, "wb").write(b"".join(patch_caps(P, no_ref[2], 0x02)))
            open(b3, "wb").write(b"".join(patch_caps(Q, k2[2], 0x02)))
            r = run([sys.executable, cmp_py, "--placa", b2, "--pc-mensajes", b3, "--out-id", "0x10"])
            check(f"intervalo NO_ALT_REF con BARO_REF añadida en ambas y alt_ref NaN: código {r.returncode}",
                  r.returncode == 1 and "alt_ref" in r.stdout, r.stdout[-500:])
            # sin BARO_REF: NaN frente a finito sigue fallando; infinito también
            for val, lab in ((12.5, "finito frente a NaN"), (float("inf"), "infinito")):
                open(b1, "wb").write(b"".join(patch_snapshot(P, no_ref[2], {"alt_ref": {0: val}})))
                r = run([sys.executable, cmp_py, "--placa", b1, "--pc-mensajes", B, "--out-id", "0x10"])
                check(f"intervalo NO_ALT_REF, alt_ref {lab}: código {r.returncode}", r.returncode == 1 and "alt_ref" in r.stdout,
                      r.stdout[-500:])
    # selección sin datos comprobables
    empty = os.path.join(d, "vacio.bin")
    open(empty, "wb").write(b"")
    zeros = os.path.join(d, "ceros.bin")
    open(zeros, "wb").write(bytes(64 * 8))
    n395 = len(pkts64(msgs))
    cases = ((empty, "0x10", 2, "SIN DATOS", "archivo vacío"), (zeros, "0x10", 2, "SIN DATOS", "sólo relleno de ceros"),
             (msgs, "0x40", 2, "NO APARECE", f"{n395} paquetes y un KPID inexistente"))
    for path, kp, code, txt, lab in cases:
        r = run([sys.executable, dec, path, "--out-id", kp])
        rt = run([sys.executable, dec, path, "--out-id", kp, "--tolerante"])
        check(f"REGRESIÓN nav_out_decode, {lab}: código {r.returncode} (tolerante {rt.returncode}), «{txt}» (antes: 0 y «sin "
              f"corrupción»)", r.returncode == code and rt.returncode == code and txt in r.stdout and
              "sin corrupción" not in r.stdout, r.stdout)
    P = pkts64(msgs)
    lone = [q for q in P if q[13] == 3][:1]  # un fragmento suelto: ningún mensaje completo
    f1 = os.path.join(d, "suelto.bin")
    open(f1, "wb").write(b"".join(lone))
    r = run([sys.executable, dec, f1, "--out-id", "0x10"])
    rt = run([sys.executable, dec, f1, "--out-id", "0x10", "--tolerante"])
    check(f"paquetes del KPID sin ningún mensaje decodificable (fragmento suelto): código {r.returncode} (incompleto), "
          f"tolerante {rt.returncode} (nada rescatado)", r.returncode == 1 and rt.returncode == 2 and
          "NINGÚN mensaje decodificable" in r.stdout, r.stdout)
    bad = bytearray(next(q for q in P if q[13] == 7))
    bad[12] = 9  # versión de mensaje imposible
    f2 = os.path.join(d, "invalido.bin")
    open(f2, "wb").write(spp_fix(bad))
    r = run([sys.executable, dec, f2, "--out-id", "0x10"])
    check(f"un paquete del KPID con un mensaje inválido: código {r.returncode}, «NINGÚN mensaje decodificable»",
          r.returncode == 1 and "NINGÚN mensaje decodificable" in r.stdout and "inválidos 1" in r.stdout, r.stdout)
    one = os.path.join(d, "una_compacta.bin")
    open(one, "wb").write(next(q for q in P if q[13] == 7))
    r = run([sys.executable, dec, one, "--out-id", "0x10"])
    check(f"captura mínima válida (una compacta): código {r.returncode}, «1 mensajes»", r.returncode == 0 and
          "captura comprobada: 1 mensajes" in r.stdout, r.stdout)
    i1 = next(i for i, q in enumerate(P) if q[13] == 1)
    snap = os.path.join(d, "un_snapshot.bin")
    open(snap, "wb").write(b"".join(P[i1:i1 + 5]))
    r = run([sys.executable, dec, snap, "--out-id", "0x10"])
    check(f"captura mínima válida (un snapshot de 5 fragmentos): código {r.returncode}", r.returncode == 0 and
          "1 snapshots completos" in r.stdout, r.stdout)


def phase_records(path):
    T = open(path, "rb").read()
    return [(T[i + 1], int.from_bytes(T[i + 2:i + 4], "little"), int.from_bytes(T[i + 4:i + 6], "little"),
             int.from_bytes(T[i + 8:i + 16], "little", signed=True), i // 16) for i in range(0, len(T), 16) if T[i] == 8]


def test_silence(a, tmp):
    print("M. silencio total: la rampa caduca por el reloj; transición en la traza y reproducción exacta")
    d = os.path.join(tmp, "M")
    os.makedirs(d, exist_ok=True)
    if a.router:
        r = run([a.router, "silencio", d])
        check("router REAL: silencio de IMU y BMP con consumeData en marcha, aviso de referencia durante el silencio",
              r.returncode == 0, r.stdout[-500:])
        fi, fo, ft = (os.path.join(d, x) for x in ("silencio_entradas.bin", "silencio_salida_real.bin", "silencio_traza.bin"))
        rep_ = os.path.join(d, "rep.bin")
        r2 = run([a.svc, "--perfil", "exp", "--kpid", "0x4,0x8,0x10", "--reproducir", ft, "--mensajes", rep_, fi])
        A, B = pkts64(fo), pkts64(rep_)
        ph = phase_records(ft)
        T = open(ft, "rb").read()
        baro_ref = [i // 16 for i in range(0, len(T), 16) if T[i] == 6]
        check(f"reproducción COMPLETA de la traza del router: {len(B)}/{len(A)} paquetes idénticos (cabeceras incluidas); "
              f"PHASE {ph[:1]} antes del aviso (registro {baro_ref[:1]})",
              r2.returncode == 0 and A == B and len(A) > 0 and len(ph) == 1 and ph[0][0] == 2 and ph[0][1] == 2 and
              baro_ref and ph[0][4] < baro_ref[0], r2.stderr[-300:])
        ops = [o for o in nav_out_decode.decode_capture(fo, 0x10)[2]]
        check(f"la compacta publica la fase incierta durante el silencio y después ({sum(o['phase'] == 2 for o in ops)} de "
              f"{len(ops)})", ops[-1]["phase"] == 2 and any(o["phase"] == 0 for o in ops))
    # calendario propio con 3 s sin ningún paquete: el consume siguiente detecta el silencio por el reloj
    gen = os.path.join(SVC, "tools", "gen_spp.py")
    g = os.path.join(d, "g24.bin")
    run([sys.executable, gen, g, "--segundos", 24])
    P = pkts64(g)
    ts = lambda q: int.from_bytes(q[7:11], "little")  # noqa: E731
    t0 = ts(P[0])
    sil = os.path.join(d, "silencio.bin")
    open(sil, "wb").write(b"".join(q for q in P if not (t0 + 15000 <= ts(q) < t0 + 18000)))
    A_, T_, B_ = (os.path.join(d, x) for x in ("cal_placa.bin", "cal_traza.bin", "cal_pc.bin"))
    run([a.svc, "--perfil", "exp", "--mensajes", A_, "--traza-val", T_, sil])
    r = run([a.svc, "--perfil", "exp", "--reproducir", T_, "--mensajes", B_, sil])
    cmp_py = os.path.join(SVC, "tools", "compare_runs.py")
    rc = run([sys.executable, cmp_py, "--placa", A_, "--pc-mensajes", B_, "--out-id", "0x10"])
    ph = phase_records(T_)
    check(f"registro de 24 s sin paquetes de 15 a 18 s: PHASE {ph} (silencio por el reloj); reproducción COMPLETA, mensajes "
          f"idénticos y EQUIVALENTE (código {rc.returncode})",
          r.returncode == 0 and open(A_, "rb").read() == open(B_, "rb").read() and rc.returncode == 0 and len(ph) == 1 and
          ph[0][0] == 2 and ph[0][1] == 2 and ph[0][2] >= 2900, r.stderr[-300:] + rc.stdout[-300:])
    alt = bytearray(open(T_, "rb").read())
    k = ph[0][4]
    alt[16 * k + 2] = 1  # causa alterada: hueco entre muestras en vez de silencio
    alt[-14:-12] = spp_wire.binascii.crc_hqx(bytes(alt[:-16]), 0xFFFF).to_bytes(2, "little")
    bad = os.path.join(d, "cal_traza_alt.bin")
    open(bad, "wb").write(alt)
    r = run([a.svc, "--perfil", "exp", "--reproducir", bad, "--mensajes", B_, sil])
    check(f"PHASE alterado (otra causa) con el END recalculado: código {r.returncode} en el registro {k}",
          r.returncode == 4 and f"registro {k} " in r.stderr and "PHASE" in r.stderr, r.stderr[-300:])


def test_docs(a):
    print("G. documentación")
    md = os.path.join(SVC, "docs", "PARAMETROS_SPP.md")
    table = run([a.svc, "--dump-md"]).stdout.strip()
    check("docs/PARAMETROS_SPP.md contiene la tabla actual de eskf_svc_params.c", os.path.exists(md) and table in open(md).read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--svc", required=True)
    ap.add_argument("--nav-replay", required=True)
    ap.add_argument("--dump-txt")
    ap.add_argument("--router")
    ap.add_argument("--svc-fma")
    ap.add_argument("--resultados")
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        fb, full = test_capture(tmp)
        test_robustness(tmp, full)
        cfg = test_service_capture(a, fb, tmp)
        test_synthetic(a, tmp, cfg)
        if a.router:
            test_router_replay(a, tmp)
        test_flight(a, tmp)
        if a.svc_fma:
            test_compare(a, tmp)
        test_decoder(a, tmp)
        test_unavailable_fields(a, tmp)
        test_silence(a, tmp)
        test_docs(a)
        if a.dump_txt:
            print("H. volcado de texto recibido")
            d, f, _ = spp_wire.load(a.dump_txt)
            check("spp_dump.txt recibido = binario regenerado", d == full and f == "xxd16")
    nf = results.count(False)
    print(f"herramientas SPP: {len(results) - nf}/{len(results)} OK")
    return 1 if nf else 0


if __name__ == "__main__":
    sys.exit(main())
