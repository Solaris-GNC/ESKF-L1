# Revisión v4 — 19-oct-2026

Base: v3 (commit `ac8497b`). Esta revisión revisa la lógica de navegación (13 puntos) e integra el estimador en el firmware del Viradinha MKII (`../firmware`).

Todo se ha ejecutado en un ordenador x86-64 (gcc 13.3, `-O2`). Además se ha compilado, sin ejecutarlo, para el ESP32-S3 con Xtensa GCC 8.4.0. **No se ha probado en la placa.** Las revisiones anteriores están en `REVISION_v3.md` y `REVISION_v2.md`.

## 1. Los 13 puntos de la revisión

Cada punto se reprodujo primero sobre la v3. Doce seguían presentes y el 12 ya estaba bien. Todos tienen ahora su regresión.

| # | fallo | v3 | v4 | regresión |
|---|---|---|---|---|
| 1 | ventanas de reposo que aceptan muestras separadas por un hueco o por valores inválidos | 2 muestras a 5 s de distancia alineaban; una ventana que atravesaba un NaN alineaba | la ventana exige continuidad de `t_valid`: un hueco > `dt_max` o un NaN la corta (`rest_breaks_gap`) | `test_nav` v4-1 |
| 2 | la recuperación reutiliza una ventana anterior al fallo | realineaba con datos previos al bloqueo | al bloquearse se guarda `t_blocked` y se descarta la ventana; sólo valen ventanas que empiecen **después** | v4-2 |
| 3 | una ventana rechazada por variabilidad mantiene `at_rest` y ZUPT | seguía en reposo | una ventana mala corta el reposo (`rest_windows_bad`) y no hay ZUPT ni gravedad | v4-3 |
| 4 | giros lentos aceptados como reposo y guardados como sesgo | cabeceo de 0,05 rad/s → `b_g` = 0,05 | se compara la dirección de la gravedad entre las dos mitades de la ventana (`rest_tilt_change_max`); un sesgo puro sigue alineando. Límite documentado: una guiñada lenta no cambia la gravedad y no es observable sin rumbo | v4-4 |
| 5 | persistencia de despegue que atraviesa huecos o datos inválidos | NaN y hueco entre muestras de más de 2 g → despegue | la persistencia exige continuidad; se reinicia con un hueco o un NaN | v4-5 |
| 6 | barómetros rechazados que contaminan la referencia | un baro rechazado por la puerta entraba en la referencia (106,67 en vez de 100) | la referencia sólo usa barómetros válidos de la ventana, con mediana/MAD y atípicos > 5σ fuera, un mínimo `baro_ref_min_n` y un salto máximo `baro_ref_max_step` entre ventanas (rechazo contado). `alt_ref_seq` cuenta los cambios | v4-6a–d |
| 7 | marcas futuras que estropean el orden posterior | un baro con t futuro dejaba atrasados los siguientes (−105) y el estado STALE | con reloj (`nav_tick`), o respecto a la IMU si no lo hay, una marca futura (> 1 ms) se rechaza (`NAV_E_TIME_FUTURE`) **sin tocar** el orden | v4-7 |
| 8 | la validez no caduca si dejan de llegar los sensores | seguía VÁLIDO indefinidamente | `nav_tick(t_now)`: atraso del estado > `stale_max_s` → STALE (NO VÁLIDO); un reloj que retrocede se ignora | v4-8 |
| 9 | toda la IMU antes del baro cuando llegan por lotes | el ejemplo v3 procesaba toda la IMU y luego el baro | `nav_events.c`: ordenación por tiempo de las dos colas con espera acotada (IMU primero en empate); desbordes contados. En el firmware, una sola tarea es dueña de `nav_t` | `test_fw` 9 |
| 10 | float/double incoherente entre unidades: acceso fuera de las estructuras | `nav_init` podía escribir con otro tamaño de estructura antes de comprobar nada | `magic` es el primer campo de `eskf_t` y de `nav_t`. Las macros pasan `sizeof` y todo se comprueba **antes** de tocar memoria. En el ESP32 se activa float de forma automática y coherente | v4-10 |
| 11 | saturación no detectada por cuantización | con ±16 g el código +32767 equivale a 15,9995 g, así que un límite físico de «16 g» no lo veía | `imu_conv.c`: saturación por **código crudo** en los dos extremos (con margen opcional), asignada al eje del cohete tras la rotación | `test_fw` 11, v4-11 |
| 12 | modelos de saturación o de puente sin validar | ya deshabilitados (`gap_max` = 0, `*_sat_max_s` = 0) | sin cambios; también en la tabla del firmware | `test_nav` (políticas), `firmware/test_vn` |
| 13 | el comparador da éxito sin ejecutar pruebas | `--seeds 0` → RESULTADO OK | exige ≥ 1 semilla y ≥ 1 paso, y falla si se comparan 0 órdenes | `test_tools.py` |

Las ecuaciones del núcleo (`eskf.c`) no cambian. La equivalencia con `eskf.py` sigue en 1,28e-13 (double) y 1,67e-4 (float).

## 2. Cambios por archivo

- **`eskf.h`**:
  - `ESKF_USE_FLOAT` automático con `ARDUINO_ARCH_ESP32` o `ESP_PLATFORM` (salvo `ESKF_FORCE_DOUBLE`);
  - `magic` es el primer campo.
- **`navigation.h` / `.c`** (`NAV_MAGIC` "NAV4"):
  - puntos 1–8 y 10;
  - `nav_tick`;
  - campos nuevos de configuración: `rest_tilt_change_max`, `baro_ref_min_n`, `baro_ref_max_step` y `stale_max_s`;
  - salida con `t_eval`, `alt_ref`, `alt_ref_seq`, `b_a` y `b_g`;
  - contadores nuevos: `time_future`, `ref_*`, `rest_windows_tilt` y `rest_breaks_gap`, y `last.ref`.
- **`nav_events.h` / `.c`** (nuevo): ordenación de eventos (punto 9).
- **`imu_conv.h` / `.c`** (nuevo): códigos crudos → m/s² y rad/s, rotación sensor→cohete (det = +1 comprobado) y saturación por código (punto 11).
- **`test/test_fw.c`** (nuevo): ordenación (orden, empates, espera, desborde, 20 s aleatorios) y conversión.
- **`test/nav_replay.c`**:
  - órdenes `IMUC`, `IMURAW`, `IMUNAN`, `BAROUS` y `TICKUS` para reproducir exactamente un registro de la placa;
  - salida con float `%.9g` y double `%.17g` (sin pérdida), más `alt_ref_seq` y `f_degraded`.
- **`test/compare_with_python.py`**: punto 13.
- **`Makefile`**: `fw-sync` / `fw-check` (copia del núcleo en el sketch, comprobada en `make test`).
- **Eliminados**: `examples/flight_loop_example.c`, sustituido por el firmware real, y `test/check_readme.py`, porque el README ya no contiene código compilable.

## 3. Integración en el firmware

Ver `../firmware/README.md`. Resumen:

- sketch para el Arduino Nano ESP32 con ICM-20948 (SparkFun 1.3.2) y BMP390 (API de Bosch 2.0.6);
- ISR que sólo marca el tiempo; una tarea dueña del I2C y otra dueña de `nav_t`;
- registro binario acotado con contadores de pérdidas;
- tabla única de parámetros con estado y origen, y perfiles ADQUISICIÓN / BANCO / VUELO;
- herramientas de análisis, de calibración y de comparación placa/PC.

## 4. Hallazgos que afectan a la configuración

- **Unidades de σ_a y σ_w.** Son desviaciones **por evento** (`Q_v = σ_a²·dt²`). El README de la v3 sugería usar la desviación por muestra o N·√BW, y eso subestima el paseo aleatorio de velocidad cuando las muestras están correladas por el DLPF o cuando se repiten valores retenidos. Lo correcto es σ_eq = N·√(E[dt]/E[dt²]), con los intervalos reales entre eventos. En simulación la varianza real coincide con σ_eq y la σ por muestra se queda ~40 % corta. `analyze_static.py` lo calcula.
- **Frecuencias distintas.** Con divisores 4, el giróscopo va a 220 Hz y el acelerómetro a 225 Hz. Se entregan todas las lecturas (`firmware/README.md`).
- **Errores I2C esporádicos.** Impiden alinear: un NaN corta la ventana de 5 s. Con un error cada 3,4 s no hay ninguna ventana completa (visto en simulación). La prueba de 2 h exige 0 errores.

## 5. Comandos y resultados

| orden | resultado |
|---|---|
| `make test` | test_eskf 1449/1449 (double), 1447/1447 (float); test_nav 192/192 ×2; test_fw 18/18 ×2; test_sim 81/81 ×2; herramientas 60/60; fw-check OK |
| `make compare REF=..` | 49 712 órdenes, peor 1,28e-13 (double); float 1,67e-4 |
| ASan + UBSan (test_eskf/nav/fw/sim, double y float) | limpios |
| `make -C ../firmware test` | test_vn 1331/1331 ×2; herramientas 43/43; docs-check OK |
| `make -C ../firmware xtensa`, `arduino` | sin avisos; tres perfiles compilan |

## 6. Pendiente

Todas las medidas de la placa y los datos del equipo están en `../firmware/PARAMETROS.md` y `../firmware/COMPILAR.md`.

Riesgo de tiempos sin medir: la llamada que cierra cada ventana de reposo ordena hasta 512 altitudes y hace 40 bisecciones en double, que es software en el ESP32-S3. Hay que medirla con la prueba de arranque (`firmware/PROCEDIMIENTOS.md` §7).
