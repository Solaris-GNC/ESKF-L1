# ESKF — Error-State Kalman Filter

Implementación en Python de un filtro de Kalman de estado de error (ESKF) de 15 estados para navegación inercial, con cuaterniones de Hamilton.

**Estado nominal:** posición `p`, velocidad `v`, orientación `q`, sesgo del acelerómetro `b_a` y sesgo del giróscopo `b_g`.

**Actualizaciones disponibles:** GPS (posición), magnetómetro, acelerómetro (gravedad) y barómetro (altura).

La formulación matemática completa está en [`docs/ESKF.pdf`](docs/ESKF.pdf).

## Estructura

```
eskf.py            # Clase ESKF: predicción y actualizaciones
quaternion.py      # Operaciones con cuaterniones y jacobianos
linalg_utils.py    # Utilidades de álgebra lineal (matriz antisimétrica)
test/              # Tests con pytest
docs/ESKF.pdf      # Documento con la derivación
c/                 # port a C99 (núcleo + módulo de navigation.c): ver c/README.md
firmware/          # integración en el Arduino Nano ESP32 del Viradinha MKII: ver firmware/README.md
```

## Instalación

```bash
pip install -r requirements.txt        # solo numpy
pip install -r requirements-dev.txt    # + pytest y scipy para los tests
```

## Uso

```python
import numpy as np
from eskf import ESKF

f = ESKF(sigma_a=0.05, sigma_w=0.005, sigma_aw=1e-4, sigma_ww=1e-5,
         mag_ref=np.array([0.21, 0.0, 0.43]))  # campo magnético en NED

dt = 0.01
f.predict(a_meas, w_meas, dt)               # IMU
f.update_gps(p_meas, sigma_gps=2.0)
f.update_mag(m_meas, sigma_mag=0.01)
f.update_accel(a_meas, sigma_a=0.1)
f.update_baro(z_meas, sigma_baro=0.5)

print(f.p, f.v, f.q)
```

## Tests

```bash
pytest
```
