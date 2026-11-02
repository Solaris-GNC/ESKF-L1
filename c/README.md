# ESKF en C

Navegación del vuelo experimental del Viradinha MKII (7-nov-2026, sin control ni aerofrenos): estimar, registrar junto a las medidas crudas y evaluar después.

**Estado (v4, 19-oct).**

- Núcleo equivalente a `eskf.py`.
- Módulo de navegación revisado: 13 puntos, todos con regresión.
- Integración en el firmware del Arduino Nano ESP32 en `../firmware`: compila para la placa, pero **no se ha probado en ella**.
- Lo que depende de la sensórica sigue como configuración pendiente (`NAV_PENDING`), y sin ello `nav_init` no arranca.

## Contenido

```
eskf.h / eskf.c               núcleo ESKF (solo <math.h>, <stdint.h>, <stddef.h>)
navigation.h / navigation.c   sesión de navegación: el módulo que usa el firmware
nav_events.h / nav_events.c   ordenación por tiempo de eventos IMU/baro (una sola dueña de nav_t)
imu_conv.h / imu_conv.c       códigos crudos → SI, ejes sensor → cohete, saturación por código
test/test_eskf.c              núcleo: unitarias, regresión y coherencia estadística del puente
test/test_nav.c               navigation.c: regresiones de las revisiones (v4-1…v4-11)
test/test_fw.c                nav_events.c e imu_conv.c
test/test_sim.c               simulación de vuelo que EJECUTA navigation.c
test/nav_replay.c             pasa un registro por navigation.c en el ordenador (también el de la placa)
test/replay.c, compare_with_python.py   equivalencia del núcleo con eskf.py
test/test_tools.py            pruebas de las herramientas
test/bench.c                  tiempos en el ordenador (no en la placa)
REVISION.md                   revisión v4; REVISION_v3.md y REVISION_v2.md, las anteriores
../firmware/                  sketch de Arduino, herramientas de registro y procedimientos de banco
```

## Convenciones

Idénticas a `eskf.py`:

- marco NED; cuaterniones de Hamilton `[w, x, y, z]`; `R(q)` de cuerpo a navegación; error de actitud local;
- fuerza específica en m/s², giro en rad/s, tiempo en s;
- `g = [0, 0, −9.81]` es la reacción gravitatoria y la predicción usa `v̇ = R(q)·(a_m − b_a) − g`;
- el barómetro entra como `z = −altura_relativa`.

Sin magnetómetro: `has_mag_ref = 0`, `heading_known = 0` y marco horizontal local con acimut arbitrario.

## Núcleo (`eskf.c`)

Contrato de errores, validación, fallo enclavado, Joseph en dos etapas, puente y `tau_max`: ver `REVISION_v2.md` y `REVISION_v3.md`.

En la v4 sólo cambian la ABI (`magic` primero) y el modo float automático en el ESP32 (`ARDUINO_ARCH_ESP32` / `ESP_PLATFORM`); `ESKF_FORCE_DOUBLE` lo evita. No compilar con `-ffast-math`: hay un `#error`.

**Unidades del ruido** (`predict_step`):

| parámetro | entra como | significado |
|---|---|---|
| `sigma_a`, `sigma_w` | `Q = σ²·dt²` | desviación **por evento** de navegación |
| `sigma_aw`, `sigma_ww` | `Q = σ²·dt` | densidad de paseo aleatorio del sesgo (K de Allan) |

Con muestras correladas por el filtro paso bajo de la IMU, o con valores retenidos, la σ por muestra no reproduce el paseo aleatorio real. Hay que usar σ_eq = N·√(E[dt]/E[dt²]), donde N es la densidad de Allan y dt son los intervalos reales entre eventos (`../firmware/tools/analyze_static.py`).

## Módulo de navegación (`navigation.c`)

**Tiempo.**

- `t_rx` es la última IMU aceptada, `t_valid` la última con valores válidos y `t_state` el instante que representa el estado.
- Marcas no finitas, repetidas o atrasadas se rechazan (`NAV_E_TIME_*`) sin tocar nada.
- Las marcas **futuras** (más de 1 ms por delante del reloj de `nav_tick`, o de la última IMU si no hay reloj) también se rechazan, sin estropear el orden posterior.
- `nav_tick(t_now)` se llama periódicamente aunque no lleguen muestras. Con él, la validez caduca (STALE) si el estado se atrasa más de `stale_max_s`.
- `navigation.c` recibe `t` en double, derivado de µs enteros.

**Huecos.** Un hueco > `dt_max` (`gap_max = 0`: sin puente) es NO VÁLIDO y queda enclavado. En la rampa se recupera con una ventana de reposo que empiece **después** del bloqueo (`t_blocked`). En vuelo sólo se recupera con `nav_init`.

**Reposo y alineación.** Cada muestra se comprueba contra `rest_acc_dev_max` y `rest_gyro_max`. Una ventana de `rest_window_s` además exige:

- continuidad: sin huecos ni NaN, que la parten;
- desviación de ‖a‖ ≤ `rest_acc_std_max`;
- cambio de dirección de la gravedad entre sus mitades ≤ `rest_tilt_change_max`, para no tomar un giro lento como sesgo.

Una ventana mala corta el reposo (sin ZUPT ni gravedad). La primera ventana buena alinea.

**Referencia de altura.** Media de los barómetros **válidos** de la ventana, sin atípicos (mediana/MAD, 5σ). Exige al menos `baro_ref_min_n` barómetros, y un salto mayor que `baro_ref_max_step` entre ventanas se rechaza y se cuenta. `alt_ref_seq` cuenta los cambios.

**Despegue.** `‖a‖ > liftoff_acc` de forma **continua** (sin huecos ni NaN) durante `liftoff_hold_s`.

**Saturación.** Por eje. Llega en los bits del driver; en el firmware, `imu_conv` la detecta por código crudo en los dos extremos. Por defecto `*_sat_max_s = 0`: cualquier saturación invalida.

**Barómetro.** `t` es el instante de lectura y `t − baro_latency_s` el de la medida. Se aplica con `eskf_update_baro_at` si `|τ| ≤ tau_max`, y `nav_config_check` exige `tau_max ≥ baro_latency_s + dt_max`. La latencia se mide con `../firmware/tools/baro_updown.py`.

**Estado publicado.**

| Validez | Cuándo |
|---|---|
| NO VÁLIDO | sin iniciar o sin alinear; fallo del núcleo; hueco no recuperable; estado atrasado (incluido por reloj); saturación no admitida; inflado fallido; rechazo interno |
| DEGRADADO | saturación admitida; hueco puenteado; en vuelo, sin baro aplicado durante `baro_timeout_s` |
| VÁLIDO | ninguno de los anteriores |

Retornos en `nav_t.last`, contadores en `nav_t.cnt`, bits del núcleo en `nav_t.f.degraded`. El firmware lo registra todo.

**Configuración.** La tabla única con estado y origen de cada parámetro y cómo medir lo pendiente está en `../firmware/PARAMETROS.md`.

## Compilar y probar

```bash
make test                     # C99 estricto double/float + C++ + pruebas + herramientas + fw-check
make compare REF=..           # equivalencia del núcleo con eskf.py
make bench; make stack        # en ESTE ordenador; para la placa: make -C ../firmware xtensa / arduino
make fw-sync                  # copia el núcleo al sketch (../firmware/viradinha_nav/src/nav)
```

Registro → navegación en el ordenador:

```bash
build/nav_replay < registro.txt > salida.txt
# CFG <campo> <valor> / INIT / IMU t ax ay az gx gy gz sat / BARO t alt / TICK t
# registros de la placa: ../firmware/tools/log_to_replay.py (IMUC, IMURAW, IMUNAN, BAROUS, TICKUS)
```

## Limitaciones conocidas

- Sólo IMU y barómetro. El magnetómetro y el GPS están en el núcleo, pero no se integran.
- Política estricta de tiempo: sin puente, perder una muestra en vuelo invalida el resto del vuelo (el registro sigue).
- Golpes en la rampa: una muestra de 3 g sin desplazamiento se integra como velocidad hasta el ZUPT (S9 de la simulación).
- Tiempos y pila sin medir en la placa (`../firmware/PROCEDIMIENTOS.md` §7).
