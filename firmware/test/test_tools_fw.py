#!/usr/bin/env python3
"""test_tools_fw.py — pruebas de las herramientas del firmware con registros SINTÉTICOS (gen_vnlog).

Comprueban que las herramientas leen bien el formato, detectan lo que se inyecta y que la
reproducción en el PC es EXACTA cuando la entrada es la misma. No sustituyen a las pruebas en
placa: ningún resultado de aquí es una medida de nuestra IMU ni de la placa.

  test_tools_fw.py --gen build/gen_vnlog --dump build/dump_params --nav-replay ../c/build/nav_replay
                   --nav-replay-float ../c/build/nav_replay_float [--gen-fma build/gen_vnlog_fma]
"""
import argparse
import json
import os
import random
import re
import subprocess
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(FW, "tools"))
import analyze_static  # noqa: E402
import axes_check  # noqa: E402,F401
import baro_updown  # noqa: E402
import capture_serial  # noqa: E402
import compare_board_pc  # noqa: E402
import log_to_replay  # noqa: E402
import six_position  # noqa: E402
import soak_report  # noqa: E402
import vnlog  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'OK' if ok else 'FALLO'}] {name}" + ("" if ok else f" — {detail}"))


class Gen:
    def __init__(self, exe, tmp):
        self.exe, self.tmp, self.n = exe, tmp, 0

    def __call__(self, *args, exe=None):
        self.n += 1
        out = os.path.join(self.tmp, f"g{self.n}.bin")
        tr = os.path.join(self.tmp, f"g{self.n}.json")
        r = subprocess.run([exe or self.exe, *map(str, args), out, tr], capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError(r.stderr)
        with open(tr) as f:
            return out, json.load(f)


def test_format(gen, tmp):
    print("formato y decodificador")
    h = open(os.path.join(FW, "viradinha_nav/src/vn/vn_logfmt.h")).read()
    c_sizes = {k: v for k, v in ((m.group(1), int(m.group(2))) for m in
                                 re.finditer(r"VN_STATIC_ASSERT\(sizeof\(vn_rec_(\w+)_t\) == (\d+)", h))}
    py = {k.lower(): v for k, v in vnlog.SIZES.items()}
    check("tamaños de registro iguales en C y Python", all(py.get(k) == v for k, v in c_sizes.items())
          and len(c_sizes) == len(py), f"C {c_sizes} Py {py}")
    nh = open(os.path.join(FW, "viradinha_nav/src/nav/navigation.h")).read()
    end = nh.index("} nav_counters_t;")
    body = nh[nh.rindex("typedef struct {", 0, end) + len("typedef struct {"):end]
    names = re.findall(r"\b([a-z_]+)\b(?=[,;])", re.sub(r"uint32_t", "", body))
    check("orden de nav_counters_t igual en C y Python", names == vnlog.NAV_COUNTERS, f"{names}")
    check("CRC-16/CCITT-FALSE", vnlog.crc16(b"123456789") == 0x29B1)
    path, tr = gen("--seconds", 20, "--seed", 2)
    data = open(path, "rb").read()
    log = vnlog.Log(data=data)
    check("decodifica sin errores", log.stats["crc_err"] == 0 and log.stats["resync_bytes"] == 0,
          str(log.stats))
    check("nº de IMU = verdad", log.count("IMU") == tr["imu_records"], f"{log.count('IMU')} vs {tr['imu_records']}")
    rnd = random.Random(7)
    bad = bytearray(data)
    for _ in range(200):
        bad[rnd.randrange(len(bad))] ^= 0xFF
    lb = vnlog.Log(data=bytes(bad))
    check("corrupción: cuenta CRC, se resincroniza y conserva casi todo",
          lb.stats["crc_err"] > 0 and lb.count("IMU") > 0.97 * log.count("IMU"),
          f"{lb.stats} IMU {lb.count('IMU')}")
    lt = vnlog.Log(data=data[: len(data) // 2 + 3])
    check("archivo truncado: decodifica la parte buena", lt.stats["truncated"] == 1 and lt.count("IMU") > 0)
    with tempfile.TemporaryDirectory(dir=tmp) as d:
        vnlog.to_csv(log, d)
        check("exporta CSV", os.path.exists(os.path.join(d, "imu.csv")))
    import io
    buf = io.StringIO()
    vnlog.summary(log, buf)
    check("resumen avisa de registro simulado", "SIMULADO" in buf.getvalue())


def test_static(gen):
    print("análisis estático")
    path, tr = gen("--seconds", 150, "--seed", 3, "--gyro-bias", "0.002,-0.001,0.0005", "--stall", "60,0.03",
                   "--lose-irq-every", 5000, "--i2c-err-every", 7001, "--ring-drop-every", 9001)
    R, _ = analyze_static.analyze(vnlog.Log(path), skip=5.0)
    check("frecuencia acel ±0.2 %", abs(R["acel_frecuencia_hz"] / tr["fs_acc"] - 1) < 2e-3, R["acel_frecuencia_hz"])
    check("frecuencia giro ±0.2 %", abs(R["giro_frecuencia_hz"] / tr["fs_gyro"] - 1) < 2e-3, R["giro_frecuencia_hz"])
    na = [R["allan"][f"acel_{a}"]["N"] for a in "xyz"]
    ng = [R["allan"][f"giro_{a}"]["N"] for a in "xyz"]
    check("densidad de ruido acel ±10 %", all(abs(x / tr["acc_noise"] - 1) < 0.1 for x in na), na)
    check("densidad de ruido giro ±10 %", all(abs(x / tr["gyro_noise"] - 1) < 0.1 for x in ng), ng)
    gb = R["giro_media_rads"]
    check("sesgo del giro ±2e-4 rad/s", all(abs(gb[i] - tr["gyro_bias"][i]) < 2e-4 for i in range(3)), gb)
    check("detecta el hueco inyectado", R["huecos_dt_max"]["n"] == 1, R["huecos_dt_max"])
    check("cuenta irq perdidas (marcas)", R["imu_flag_irq_lost"] == tr["injected"]["irq_lost"],
          f"{R['imu_flag_irq_lost']} vs {tr['injected']['irq_lost']}")
    check("cuenta errores I2C", R["imu_flag_i2c_err"] >= tr["injected"]["i2c_err"] - 1, R["imu_flag_i2c_err"])
    check("cuenta tramas perdidas del registro", R["imu_tramas_perdidas_registro"] >= tr["injected"]["ring_drop"] - 1,
          R["imu_tramas_perdidas_registro"])
    check("ruido del baro ±10 %", abs(R["baro_ruido_alt_m"] / tr["baro_noise"] - 1) < 0.1, R["baro_ruido_alt_m"])
    check("K no observable en 150 s sin paseo: lo dice", R["allan"]["giro_x"]["K"] is None and
          "K_nota" in R["allan"]["giro_x"])
    check("candidatos sin aplicar nada (sólo propuestas)", R["candidatos"]["eskf.sigma_a"]["propuesta"] is None)
    ca = R["candidatos"]["eskf.sigma_a"]
    check("σ_a por evento = N·√(E[dt]/E[dt²]) y distinto de la σ por muestra",
          abs(ca["sigma_eq_por_evento"] - ca["N_max"] * R["factor_sigma_eq"]) < 1e-12 and
          abs(ca["sigma_eq_por_evento"] / ca["std_por_muestra_max"] - 1) > 0.05, ca)


def test_replay(gen, a, tmp):
    print("reproducción placa/PC")
    path, tr = gen("--profile", "bench", "--seconds", 15, "--flight", "--seed", 4, "--updown", "1.5,4,1,7")
    log = vnlog.Log(path)
    res = compare_board_pc.compare(log, a.nav_replay_float, exact=True)
    check("misma entrada → resultado IDÉNTICO bit a bit", res["ok"] and res["events"] == tr["nav_calls"],
          res["errors"][:2])
    path2, tr2 = gen("--profile", "bench", "--seconds", 12, "--flight", "--seed", 5, "--lose-irq-every", 3001,
                     "--i2c-err-every", 4001, "--stall", "9,0.05")
    res2 = compare_board_pc.compare(vnlog.Log(path2), a.nav_replay_float, exact=True)
    check("con fallos inyectados (huecos, NaN, irq): idéntico", res2["ok"], res2["errors"][:2])
    # manipulación de un dato crudo con CRC válido → debe detectarse
    data = bytearray(open(path, "rb").read())
    imu_t = [t for t, n in vnlog.REC.items() if n == "IMU"][0]
    k = 0
    for typ, st, ln in vnlog.frames(bytes(data)):
        if typ == imu_t:
            k += 1
            if k == 3000:
                data[st + 20] ^= 0x10  # acel x, byte bajo
                crc = vnlog.crc16(bytes(data[st - 1:st + ln]))
                data[st + ln] = crc & 0xFF
                data[st + ln + 1] = crc >> 8
                break
    res3 = compare_board_pc.compare(vnlog.Log(data=bytes(data)), a.nav_replay_float, exact=True)
    check("un código crudo alterado se detecta", not res3["ok"])
    res4 = compare_board_pc.compare(log, a.nav_replay, exact=True)
    check("nav_replay en double frente a placa en float: NO coincide", not res4["ok"])
    path5, _ = gen("--profile", "bench", "--seconds", 12, "--seed", 6, "--ring-drop-every", 2000)
    lines, mapa, rep = log_to_replay.convert(vnlog.Log(path5))
    check("tramas perdidas del registro: se detiene y lo dice", rep["stopped_at_order"] is not None and
          rep["missing"] and rep["missing"][0][0] == "IMU", rep["missing"][:1])
    if a.gen_fma:
        p6, _ = gen("--profile", "bench", "--seconds", 15, "--flight", "--seed", 7, "--spin", "0.5,9,2",
                    exe=a.gen_fma)
        l6 = vnlog.Log(p6)
        r_ex = compare_board_pc.compare(l6, a.nav_replay_float, exact=True)
        r_tol = compare_board_pc.compare(l6, a.nav_replay_float, exact=False)
        check("FMA frente a sin FMA: difiere bit a bit pero dentro de tolerancia",
              (not r_ex["ok"]) and r_tol["ok"], r_tol["errors"][:2])
    # registro de adquisición con la configuración de banco
    p7, _ = gen("--seconds", 20, "--seed", 8)
    cfg = os.path.join(tmp, "bench_cfg.txt")
    with open(cfg, "w") as f:
        f.write(subprocess.run([a.dump, "--replay", "bench"], capture_output=True, text=True, check=True).stdout)
    lines, mapa, rep = log_to_replay.convert(vnlog.Log(p7), cfg)
    r = subprocess.run([a.nav_replay_float], input="\n".join(lines) + "\n", capture_output=True, text=True)
    outs = compare_board_pc.parse_out(r.stdout)
    check("adquisición + config de banco: nav_replay alinea", r.returncode == 0 and (outs[:, 2] == 2).any(),
          r.stderr[:200])
    rv = subprocess.run([a.nav_replay_float], input=subprocess.run([a.dump, "--replay", "vuelo"], capture_output=True,
                                                                   text=True).stdout + "INIT\n",
                        capture_output=True, text=True)
    check("config de VUELO actual: nav_replay la rechaza (pendientes)", rv.returncode in (2, 3), rv.stderr[:120])


def test_calibration(gen):
    print("seis posiciones, ejes y baro")
    bias, scale = [0.05, -0.03, 0.08], [0.002, -0.001, 0.003]
    ms = []
    for i, u in enumerate(["+x", "-x", "+y", "-y", "+z", "-z"]):
        p, _ = gen("--seconds", 25, "--seed", 40 + i, "--up", u, "--acc-bias", ",".join(map(str, bias)),
                   "--acc-scale", ",".join(map(str, scale)))
        ms.append(six_position.static_means(p, 3.0))
    R = six_position.solve(ms, 9.80665)
    check("seis posiciones: sesgo ±2e-3 m/s²", all(abs(R["sesgo_ms2"][i] - bias[i]) < 2e-3 for i in range(3)),
          R["sesgo_ms2"])
    check("seis posiciones: escala ±3e-4", all(abs(R["escala"][i] - scale[i]) < 3e-4 for i in range(3)), R["escala"])
    p1, _ = gen("--seconds", 15, "--seed", 50, "--up", "-z")
    p2, _ = gen("--seconds", 15, "--seed", 51, "--up", "+x")
    p3, _ = gen("--seconds", 15, "--seed", 52, "--up", "-z", "--spin", "1.5708,7,1")
    tool = os.path.join(FW, "tools", "axes_check.py")
    r = subprocess.run([sys.executable, tool, "--arriba", p1, "+x", "--arriba", p2, "+y", "--giro", p3, "+x"],
                       capture_output=True, text=True)
    check("ejes: montaje oculto recuperado y giro coherente", r.returncode == 0 and
          "cohete x = -sensor z, cohete y = +sensor x, cohete z = -sensor y" in r.stdout, r.stdout[-300:])
    r = subprocess.run([sys.executable, tool, "--arriba", p1, "+x", "--arriba", p2, "+y", "--giro", p3, "-x"],
                       capture_output=True, text=True)
    check("ejes: giro en sentido contrario → incoherente", r.returncode == 1)
    errs = []
    for seed in (60, 61, 62):
        p, tr = gen("--seconds", 70, "--seed", seed, "--updown", "2.0,3.5,15,10", "--baro-lag", 0.02)
        R = baro_updown.analyze(vnlog.Log(p))
        errs.append((R["latencia_s"] - tr["baro_latency_eff_mean"], R["sigma_latencia_s"], R["escala"]))
    check("baro: signo y escala ≈ +1", all(0.9 < e[2] < 1.1 for e in errs), errs)
    check("baro: latencia dentro de 3σ de la verdad", all(abs(e[0]) < 3 * e[1] for e in errs), errs)
    check("baro: σ de la latencia < 10 ms con 2 m y 15 ciclos", all(e[1] < 0.01 for e in errs), errs)


def test_soak_capture(gen, tmp):
    print("resistencia y captura")
    p, _ = gen("--profile", "bench", "--seconds", 90, "--seed", 70)
    R = soak_report.analyze(vnlog.Log(p), min_hours=0.02)
    check("resistencia: registro limpio → OK", R["resultado"] == "OK",
          [c for c in R["criterios"] if c["estado"] != "OK"])
    p2, _ = gen("--profile", "bench", "--seconds", 90, "--seed", 70, "--lose-irq-every", 20000)
    R2 = soak_report.analyze(vnlog.Log(p2), min_hours=0.02)
    warn = [c["criterio"] for c in R2["criterios"] if c["estado"] == "AVISO"]
    check("resistencia: irq perdida sin hueco → AVISO", "interrupciones IMU perdidas" in warn, warn)
    p3, _ = gen("--profile", "bench", "--seconds", 90, "--seed", 71, "--stall", "40,0.02")
    R4 = soak_report.analyze(vnlog.Log(p3), min_hours=0.02)
    bad = [c["criterio"] for c in R4["criterios"] if c["estado"] == "FALLO"]
    check("resistencia: hueco real de la IMU → FALLO", "huecos de la IMU > dt_max" in bad, bad)
    R3 = soak_report.analyze(vnlog.Log(p), min_hours=2.0)
    check("resistencia: duración insuficiente → FALLO", R3["resultado"] == "FALLO")
    try:
        import serial
    except ImportError:
        check("captura (pyserial no instalado: omitida)", True)
        return
    s = serial.serial_for_url("loop://", timeout=0.1)
    data = open(p, "rb").read()
    th = threading.Thread(target=lambda: [s.write(data[i:i + 3000]) for i in range(0, len(data), 3000)])
    th.start()
    out = os.path.join(tmp, "cap.bin")
    r = capture_serial.capture(s, out, idle_stop=1.0, report=lambda m: None)
    th.join()
    check("captura por puerto serie (bucle): bytes idénticos", open(out, "rb").read() == data and r["crc_err"] == 0)


def test_docs(a):
    print("documentación")
    md = os.path.join(FW, "PARAMETROS.md")
    if not os.path.exists(md):
        check("PARAMETROS.md existe", False)
        return
    table = subprocess.run([a.dump, "--md"], capture_output=True, text=True, check=True).stdout.strip()
    txt = open(md).read()
    check("PARAMETROS.md contiene la tabla actual de vn_params.c (make -C firmware docs)", table in txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gen", required=True)
    ap.add_argument("--gen-fma")
    ap.add_argument("--dump", required=True)
    ap.add_argument("--nav-replay", required=True)
    ap.add_argument("--nav-replay-float", required=True)
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        gen = Gen(a.gen, tmp)
        test_format(gen, tmp)
        test_static(gen)
        test_replay(gen, a, tmp)
        test_calibration(gen)
        test_soak_capture(gen, tmp)
        test_docs(a)
    nf = results.count(False)
    print(f"herramientas del firmware: {len(results) - nf}/{len(results)} OK")
    return 1 if nf else 0


if __name__ == "__main__":
    sys.exit(main())
