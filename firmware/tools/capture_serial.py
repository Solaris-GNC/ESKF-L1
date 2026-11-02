#!/usr/bin/env python3
"""capture_serial.py — guarda el registro binario que la placa envía por USB (sumidero por defecto).

  capture_serial.py --port /dev/ttyACM0 [--out registro.bin] [--seconds 7200]
  (Windows: --port COM5; macOS: /dev/cu.usbmodem…)

Escribe los bytes TAL CUAL llegan (el formato se resincroniza solo) y cada segundo muestra
bytes/s, tramas por tipo, errores de CRC y las pérdidas que declara la propia placa (STATS).
Ctrl+C para terminar. Necesita pyserial (pip install pyserial). Activa DTR: el USB CDC del
ESP32-S3 sólo envía con el puerto abierto (si no, la placa descarta y lo cuenta).
"""
import argparse
import datetime
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402


def capture(ser, out_path, seconds=None, idle_stop=None, report=print):
    t0 = time.monotonic()
    last_rx = t0
    last_rep = t0
    total = 0
    tail = b""
    counts = {}
    crc = 0
    last_stats = None
    with open(out_path, "wb") as f:
        try:
            while True:
                data = ser.read(65536)
                now = time.monotonic()
                if data:
                    f.write(data)
                    total += len(data)
                    last_rx = now
                    buf = tail + data
                    st = {}
                    end = 0
                    for typ, a, ln in vnlog.frames(buf, st):
                        name = vnlog.REC.get(typ, str(typ))
                        counts[name] = counts.get(name, 0) + 1
                        if name == "STATS":
                            last_stats = vnlog.unpack("STATS", buf[a:a + ln])
                        end = a + ln + 2
                    crc += st.get("crc_err", 0)
                    tail = buf[end:][-4096:]
                if now - last_rep >= 1.0:
                    msg = f"{now - t0:7.0f} s  {total / 1e6:8.2f} MB  " + " ".join(
                        f"{k}:{v}" for k, v in sorted(counts.items()) if k in ("IMU", "BARO", "NAVOUT", "STATS"))
                    if crc:
                        msg += f"  crc:{crc}"
                    if last_stats:
                        msg += (f"  pérdidas placa: reg {last_stats['ring_drop_recs']} irq {last_stats['imu_irq_lost']}"
                                f" i2c {last_stats['i2c_err_imu']}")
                    report(msg)
                    last_rep = now
                if seconds is not None and now - t0 >= seconds:
                    break
                if idle_stop is not None and now - last_rx >= idle_stop:
                    break
        except KeyboardInterrupt:
            pass
    return {"bytes": total, "frames": counts, "crc_err": crc}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--out")
    ap.add_argument("--seconds", type=float)
    a = ap.parse_args()
    try:
        import serial
    except ImportError:
        raise SystemExit("falta pyserial: pip install pyserial")
    out = a.out or datetime.datetime.now().strftime("vn_%Y%m%d_%H%M%S.bin")
    ser = serial.serial_for_url(a.port, baudrate=921600, timeout=0.2)
    try:
        ser.dtr = True
    except Exception:
        pass
    print(f"capturando {a.port} → {out} (Ctrl+C para terminar)")
    r = capture(ser, out, a.seconds)
    print(f"fin: {r['bytes']} bytes, tramas {r['frames']}, crc {r['crc_err']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
