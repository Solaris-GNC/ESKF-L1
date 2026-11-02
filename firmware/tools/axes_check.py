#!/usr/bin/env python3
"""axes_check.py — montaje de la IMU en el cohete: ejes y signos (imu.axis_map_*, imu.axis_sign_*).

  axes_check.py --arriba reposo1.bin +x --arriba reposo2.bin +y [--giro giro.bin +x] [--skip 3]

  --arriba REG EJE   registro estático con el eje EJE DEL COHETE apuntando hacia arriba
                     (dos registros con ejes distintos: p. ej. cohete vertical en la rampa = +x
                     si x es el eje longitudinal hacia la ojiva; y tumbado con +y hacia arriba).
  --giro REG EJE     registro girando a mano el cohete en sentido POSITIVO (regla de la mano
                     derecha, pulgar en +EJE) unos 90°, con reposo antes y después: comprueba el
                     signo del giróscopo con la conversión resultante.
La convención de ejes del cohete (qué es +x, +y, +z) la fija el equipo: PENDIENTE de documentar.
Salida: las líneas de vn_params.c / IMUC para la conversión, la comprobación de rotación propia
(det = +1) y la inclinación del montaje. NO marca fw.axes_confirmed: eso se hace a mano tras revisar.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vnlog  # noqa: E402

AX = {"x": 0, "y": 1, "z": 2}


def parse_axis(s):
    if len(s) != 2 or s[0] not in "+-" or s[1] not in AX:
        raise SystemExit(f"eje inválido: {s} (use +x, -y, ...)")
    return AX[s[1]], 1 if s[0] == "+" else -1


def load(path, skip):
    log = vnlog.Log(path)
    imu = log["IMU"]
    t = imu["t_drdy_us"].astype(np.int64)
    ok = ((imu["flags"] & vnlog.IMU_F["I2C_ERR"]) == 0) & (t >= t[0] + int(skip * 1e6))
    A = np.stack([imu["ax"], imu["ay"], imu["az"]], 1)[ok].astype(float)
    W = np.stack([imu["gx"], imu["gy"], imu["gz"]], 1)[ok].astype(float) / 16.4 * math.pi / 180
    return t[ok] * 1e-6, A, W, log.is_sim()


def up_axis(path, skip):
    _, A, _, sim = load(path, skip)
    m = A.mean(0)
    i = int(np.argmax(np.abs(m)))
    tilt = math.degrees(math.acos(min(1.0, abs(m[i]) / np.linalg.norm(m))))
    return i, (1 if m[i] > 0 else -1), tilt, sim


def rotation(path, skip, quiet_s=2.0):
    """Ángulo integrado por eje del sensor (sesgo quitado con el reposo inicial)."""
    t, _, W, _ = load(path, skip)
    q = t < t[0] + quiet_s
    b = W[q].mean(0) if q.sum() > 10 else np.zeros(3)
    dt = np.diff(t, prepend=t[0])
    return ((W - b) * dt[:, None]).sum(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arriba", nargs=2, action="append", metavar=("REG", "EJE"), required=True)
    ap.add_argument("--giro", nargs=2, metavar=("REG", "EJE"))
    ap.add_argument("--skip", type=float, default=3.0)
    a = ap.parse_args()
    if len(a.arriba) != 2:
        raise SystemExit("hacen falta dos registros --arriba con ejes del cohete distintos")
    # cohete[r] = sign[r] · sensor[map[r]]
    mp, sg = [None] * 3, [None] * 3
    sim = False
    for path, ax in a.arriba:
        r, rs = parse_axis(ax)
        i, s, tilt, sm = up_axis(path, a.skip)
        sim |= sm
        if mp[r] is not None:
            raise SystemExit("los dos registros --arriba usan el mismo eje del cohete")
        mp[r], sg[r] = i, s * rs
        print(f"{path}: cohete {ax} arriba → sensor {'+' if s > 0 else '-'}{'xyz'[i]} arriba "
              f"(inclinación {tilt:.2f}°)" + ("  [SIMULADO]" if sm else ""))
    used = [m for m in mp if m is not None]
    if len(set(used)) != 2:
        raise SystemExit("los dos registros dan el mismo eje del sensor: revise las posiciones")
    r3 = [k for k in range(3) if mp[k] is None][0]
    i3 = [k for k in range(3) if k not in used][0]
    mp[r3] = i3
    # signo del tercero para det = +1: filas e_{map[r]}·sign[r]
    Rm = np.zeros((3, 3))
    for r in range(3):
        if sg[r] is not None:
            Rm[r, mp[r]] = sg[r]
    Rm[r3, i3] = 1
    sg[r3] = 1 if np.linalg.det(Rm) > 0 else -1
    Rm[r3, i3] = sg[r3]
    det = round(float(np.linalg.det(Rm)))
    print(f"conversión cohete ← sensor: " + ", ".join(f"cohete {'xyz'[r]} = {'+' if sg[r] > 0 else '-'}sensor "
                                                         f"{'xyz'[mp[r]]}" for r in range(3)) + f"  (det {det:+d})")
    for r in range(3):
        print(f'  {{"imu.axis_map_{"xyz"[r]}", ... {mp[r]} ...}}, {{"imu.axis_sign_{"xyz"[r]}", ... {sg[r]} ...}}')
    ok = det == 1
    if a.giro:
        path, ax = a.giro
        r, rs = parse_axis(ax)
        ang_s = rotation(path, a.skip)
        ang_r = Rm @ ang_s
        k = int(np.argmax(np.abs(ang_r)))
        deg = math.degrees(ang_r[k])
        good = (k == r) and (np.sign(ang_r[k]) == rs)
        print(f"{path}: giro positivo pedido sobre cohete {ax}; medido {deg:+.1f}° sobre cohete "
              f"{'xyz'[k]} → {'COHERENTE' if good else 'INCOHERENTE: revisar ejes o el sentido del giro'}")
        ok &= good
    print("RESULTADO: " + ("coherente; revisar y, si procede, anotar en vn_params.c y fw.axes_confirmed = 1"
                           if ok else "NO coherente") + ("  [registros SIMULADOS]" if sim else ""))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
