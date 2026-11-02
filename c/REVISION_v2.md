# Revisión del port C — 18-oct-2026

Base revisada: commit `80b1d06` (versión entregada el 17-oct). Ninguna afirmación de este documento implica validación para vuelo: todo se ha ejecutado en un ordenador x86-64 (Intel Xeon 2.1 GHz, gcc 13.3, `-O2`), no en la placa.

## 1. Fallos reproducidos y clasificación

| # | Fallo | Reproducción con la versión antigua | Clase |
|---|---|---|---|
| 1 | NaN/Inf corrompen el estado | `baro(NaN)` con puerta 5 → ret 1, `p_z = nan`; `σ = inf` → P no finita; `sigma_a = NaN` aceptado por `init` | corrupción de estado |
| 1b | `quat_normalize` oculta fallos | alineación con acc NaN → ret 0 y `q = [1 0 0 0]` | corrupción silenciosa |
| 2 | `dt` sin validar | `dt` = −0.01, 0, NaN, Inf, 10 s aceptados; NaN/Inf → P no finita | corrupción de estado |
| 3 | comparador con NaN | todas las salidas del C a NaN → `RESULTADO: OK`, error 0.00 | herramienta: falso positivo |
| 4 | PSD insuficiente | P con autovalor −0.8 aceptada | herramienta: falso positivo |
| 5 | cancelación en float | P_zz 1e7, σ 0.5 → P⁺ = 0 (exacto 0.25); falla para P_zz ≥ 1e7 m² | numérico (float) |
| 6 | README no compila | 8 errores con `-Werror` (`float*` a `double*`, `phase` no declarado) | documentación |
| 7 | alineación | NaN en giróscopo → éxito y `b_g` NaN; \|a\| = 3 g aceptado; realinear conserva correlaciones | corrupción / semántica |
| 8 | umbral de norma | reposo + 2 m/s² horizontales → usado como gravedad | semántica |
| 9 | diagnósticos antiguos | accel rechazado o mag sin referencia conservan `last_nis` del baro anterior | diagnóstico |
| 10 | referencia Python | `make compare` en el ZIP aislado → `ModuleNotFoundError` | herramienta |
| 11 | parser de replay | `PRED 0`, `BAR abc 0.5`, `BAR 3` aceptados (0 de 12 casos rechazados) | herramienta |
| 12 | validación física | sólo vuelo ideal con IMU perfecta | cobertura |

Causa del 5: la expresión de Joseph era algebraicamente correcta, pero se sumaba en un solo paso `P += (−K bᵀ − c Kᵀ + r K Kᵀ)`; en float, `−1e7 + 0.25` pierde el 0.25 antes de cancelar con `P`. Se evalúa ahora en dos etapas, `M = (I − K h) P` y luego `P⁺ = M − (M hᵀ) Kᵀ + r K Kᵀ`, sin memoria adicional relevante (un vector de 15).

## 2. Cambios por archivo

**`eskf.h` / `eskf.c`**
- Códigos `eskf_status_t` y contrato único: entradas inválidas rechazadas antes de modificar el estado; fallo numérico enclavado (`f->fault`) hasta `eskf_init`/`eskf_align_static`; `eskf_check_health` al final de cada operación.
- Validación de parámetros (`eskf_validate_params`): ruidos de proceso e iniciales finitos y ≥ 0; medidas, umbrales y `dt_max` > 0; `has_mag_ref` ∈ {0,1}; `gap_max` coherente y ≤ 64·`dt_max` (tiempo de ejecución acotado). `init` fallido deja el filtro marcado como no inicializado.
- Comprobación ABI float/double: `eskf_init` y `eskf_default_params` son macros que pasan `sizeof`; si `ESKF_USE_FLOAT` difiere entre módulos devuelven `ESKF_E_CONFIG`. `#error` con `-ffast-math`.
- `quat_normalize` devuelve éxito/fallo y no modifica `q` si falla.
- `eskf_predict` devuelve `int`; contrato temporal con `dt_max` (obligatorio, sin valor por defecto) y `gap_max` (puente con subpasos o rechazo); bits `degraded` y contadores.
- Joseph en dos etapas (fallo 5). Inyección con comprobación de finitud previa.
- Diagnóstico `f->last` (sensor, estado, usadas, puerta, numéricas, NIS) reiniciado en toda llamada; sustituye a `last_nis`/`last_rejected`.
- Reposo confirmado por el llamador (`eskf_set_rest`): lo exigen alineación, ZUPT (`eskf_update_zupt`, nuevo) y acelerómetro como gravedad.
- Alineación: valida finitud, norma, giro y referencia; reinicia la estimación completa (realineación correcta); `heading_known` distingue rumbo geográfico de marco local; GPS, velocidad externa y magnetómetro exigen rumbo.
- Nuevos: `eskf_update_baro_at` (barómetro con desfase τ respecto al estado; ver §4), `eskf_inflate_block` (el llamador declara información perdida), `eskf_status_str`.
- Sin cambios: cuaterniones, Φ, Q, modelos de medida, integración de Euler, parametrización del ruido.

**`test/replay.c`**: número exacto de argumentos por comando, `strtod` con fin de token y rango, comandos nuevos (`REST`, `ZUPT`, `PARAMS` con `dt_max`/`gap_max`), errores con código 2 (formato) o 3 (configuración).

**`test/compare_with_python.py`**: valida líneas, campos, números y finitud antes de calcular errores; comprueba simetría y autovalor mínimo de P y ‖q‖ en cada comando; localiza la referencia por `--ref-dir`, `ESKF_PY_REF` o `../..`, informa de commit y SHA-256, y devuelve 3 si falta.

**Nuevos**: `test/test_util.h` (PSD por Cholesky de `P + δI`, suficiente: existe ⇔ λmin > −δ), `test/test_sim.c`, `test/test_tools.py`, `test/check_readme.py`, `test/bench.c`, `examples/adapter_example.c`.

**`test/test_eskf.c`**: reescrito; una prueba de regresión por fallo. **`README.md`**: contrato de integración; ejemplo = archivo compilado. **`Makefile`**: `test`, `compare`, `check`, `bench`, `stack`.

### Cambios de API

| Antes | Ahora | Motivo |
|---|---|---|
| `void eskf_init` / `void eskf_default_params` | devuelven `int`; macros con comprobación de tamaño | validación (1) y ABI (6) |
| `void eskf_predict` | `int` | contrato temporal (2) |
| accel rechazado por norma → 0; mag sin referencia → −1 | `ESKF_E_MOTION` (−8), `ESKF_E_NO_REF` (−9); `> 0` sigue siendo «usada» | diagnóstico (9) |
| `last_nis`, `last_rejected` | `f->last` | diagnóstico (9) |
| `eskf_update_velocity(v = 0)` como ZUPT | `eskf_update_zupt` (exige reposo); `eskf_update_velocity` exige rumbo | reposo (8) y marco (7) |
| alineación sin requisitos | exige `at_rest`, valida, reinicia P | (7) |
| `dt_max` no existía | obligatorio en `eskf_params_t` | (2) |

## 3. Pruebas que fallaban antes y pasan ahora

| Fallo | Prueba | Antes | Ahora |
|---|---|---|---|
| 1, 1b | `regresion_1_*`, `regresion_1b_*` | ver §1 | rechazo sin modificar el estado; `q` corrupto → `ESKF_E_NUMERIC`, enclavado, recuperable al realinear |
| 2 | `regresion_2_dt` | aceptados | `ESKF_E_DT`/`_GAP` sin modificar; puente idéntico a 4 subpasos |
| 3 | `test_tools.py`, envoltorio `nan` | OK con error 0 | código 1 (también truncado, línea de menos, perturbación de 1e-6) |
| 4 | `regresion_4_psd` | aceptada | rechazada por Cholesky; el comparador la rechaza por autovalores |
| 5 | `regresion_5_cancelacion` | float: 0 para P_zz ≥ 1e7 | error relativo ≤ 8.5e-8 (float) y 0 (double) de P_zz = 1 a 1e10 |
| 6 | `check_readme.py` + compilación del ejemplo | 8 errores | compila en double y float con `-Wconversion -Werror` |
| 7 | `regresion_7_alineacion` | ver §1 | `ESKF_E_INPUT/MOTION/NO_REF/DEGENERATE/NOT_AT_REST`; realineación limpia |
| 8 | `regresion_8_reposo` | usado como gravedad | `ESKF_E_NOT_AT_REST` en vuelo |
| 9 | `regresion_9_diagnosticos` | diagnóstico anterior | estado y contadores de la llamada actual |
| 10 | ZIP aislado | `ModuleNotFoundError` | mensaje y código 3; con `REF=` compara |
| 11 | `test_tools.py` (replay) | 0/12 rechazados | 12/12 |
| 12 | `test_sim.c` | — | 11 escenarios + Monte Carlo de 20 vuelos |

## 4. Hallazgo de integración: instante de las medidas

En la simulación con muestreo irregular, aplicar el barómetro en un instante distinto del estado (desfase medio 2.9 ms, hasta ~5 ms) a ~300 m/s produce ~1 m de error, del orden del ruido. El filtro lo atribuye a actitud y sesgos: 48 % de errores dentro de 3σ y 4.5° de error de inclinación; el sesgo del giróscopo estimado cambia de signo. Con el modelo `h = p_z + v_z·τ` (`eskf_update_baro_at`): 100 % dentro de 3σ y 1.7°. El mismo desfase había contaminado también mi primera métrica de error en la simulación; corregida y documentada en `test_sim.c`.

Se probó también sumar `½·a·dt²` a la posición: empeora (Φ deja de ser coherente). Euler se mantiene.

## 5. Resultados

```
make test        test_eskf 1435/1435 (double), 1433/1433 (float)
                 test_sim  120/120 (double), 120/120 (float)
                 herramientas 27/27 con referencia, 22/22 sin ella (extremo a extremo OMITIDO)
make compare     double: 8 escenarios × 6214 comandos, peor 1.28e-13 (tol 1e-10)
                 float:  3 escenarios × 6214 comandos, peor 1.67e-4 (tol 1e-2)
                 P semidefinida (autovalores) y ‖q‖ = 1 en todos los comandos
ASan + UBSan     todas las pruebas, double y float: sin errores
clang -Weverything (núcleo): 0 avisos; compila como C++11
```

Referencia Python: `eskf.py`, `quaternion.py`, `linalg_utils.py` del repositorio solaris-eskf sin cambios desde `bc8cc99` (el comparador informa del HEAD local `80b1d06`, que sólo añade `c/`). Sin equivalente en Python, cubiertos sólo por pruebas en C: `eskf_update_baro_at` con τ ≠ 0, ZUPT, inflado, reposo, rumbo, alineación.

Simulación (hipótesis de prueba: IMU 200 Hz ±10 % de jitter, baro 50 Hz σ 0.5 m, empuje 12 g durante 3 s, apogeo ~3.2 km):

| Escenario | err. alt. máx | en 3σ | inclinación máx | t apogeo |
|---|---|---|---|---|
| S1 nominal | 0.2 m | 100 % | 1.67° | 0.00 s |
| S1b con magnetómetro (actitud completa 1.83°) | 0.2 m | 100 % | 1.10° | 0.00 s |
| S1c sin compensar desfase (ref.) | 1.2 m | 48 % | 4.5° | −0.01 s |
| S2 baro: −60 m transónico, +80 m eyección, 1 s perdido | 0.3 m | 100 % | 1.67° | 0.00 s |
| S2' sin puerta (ref.) | 69.5 m | 9.5 % | 119° | +0.26 s |
| S3 hueco IMU 60 ms puenteado | 0.2 m | 100 % | 1.67° | 0.00 s |
| S4 hueco 0.5 s sin inflar (bloqueo de puerta) | 97 m (final 69 m) | 30 % | 4.0° | +0.90 s |
| S4' hueco 0.5 s inflando P | final 0.0 m | 100 % | 4.2° | 0.00 s |
| S5 accel ±8 g saturado | 2636 m | 0.4 % | 1.1° | −9.65 s |
| S5' saturado inflando v | 1.5 m | 99 % | 0.9° | 0.00 s |
| Monte Carlo 20 vuelos (sesgos aleatorios) | 0.3 m | > 95 % cada uno | — | — |

Tiempos y pila **en este ordenador** (`make bench`, `make stack`), no en la placa:

| | double | float |
|---|---|---|
| `eskf_predict` | 4.0 µs | 3.4 µs |
| `eskf_update_baro_at` | 1.1 µs | 1.1 µs |
| `eskf_update_gps` (6 comp.) | 5.2 µs | 5.7 µs |
| pila de `predict_step` (x86-64) | 4112 B | 2144 B |
| `sizeof(eskf_t)` (x86-64) | 2176 B | 1108 B |

## 6. Limitaciones y datos pendientes

Datos necesarios: modelo de IMU y rangos (el del acelerómetro por encima del empuje máximo, ver S5), Allan variance, frecuencias reales y origen de las marcas de tiempo, modelo de barómetro con su ruido y **latencia interna** (filtrado/sobremuestreo), si hay GPS o magnetómetro, microcontrolador (FPU simple o doble), compilador, pila disponible y presupuesto de tiempo.

Pendiente de código: no hay detector de reposo en la librería (es del adaptador); el bloqueo de la puerta sólo se evita si el llamador infla P; sin rumbo no hay uso horizontal del GPS; la alineación no comprueba la inclinación magnética; la simulación es simplificada (resistencia axial, sin viento ni vibración, sesgos constantes); tiempos y pila sin medir en la placa.

## 7. Siguiente paso mínimo

Con la placa real: registrar 10 min en reposo y unos minutos moviéndola a mano, con marcas de tiempo por muestra, y pasar ese registro por el **mismo** `eskf.c` en el ordenador (convertir el registro a comandos de `replay`). Eso valida unidades, ejes, signos, frecuencias, jitter y desfases, da la Allan variance y, en la placa, permite medir tiempo y pila con el compilador del firmware. El registro debe ir en una tarea o buffer independiente del filtro.
