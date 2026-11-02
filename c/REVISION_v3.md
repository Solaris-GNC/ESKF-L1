# Revisión v3 — 19-oct-2026

Base: v2 (commit `a7b7e99`). Todo se ha ejecutado en un ordenador x86-64 (Intel Xeon 2.1 GHz, gcc 13.3, clang, `-O2`), no en la placa. Pasar estas pruebas no valida el filtro para vuelo. La revisión anterior está en `REVISION_v2.md`.

## 1. Problemas reproducidos sobre la v2

Reproducción con `examples/adapter_example.c` y `eskf.c` de la v2, en double y float (resultados idénticos en ambos):

| # | Caso | v2 |
|---|---|---|
| 1 | muestra con t = NaN, luego 4 válidas (v_z = 10 m/s) | devuelve 0; `t_state` = NaN; p_z = 0.15 en lugar de 0.20 |
| 1 | t = 0.005, 0.003, 0.010 | las tres aceptadas; p_z = 0.12 en lugar de 0.10 |
| 1 | duplicado | aceptado sin diagnóstico |
| 2 | hueco t = 0.2 (rechazado) y luego 0.205 | `t_state` = 0.205 con p_z = 0.05 (el tiempo implica 2.05 m); un baro en 0.206 se aplicó como contemporáneo y llevó p_z a −1.99 |
| 2 | IMU con NaN | `t_state` avanza sin propagar |
| 3 | σ_a = 1, lectura retenida 0.04 s, puente de 4 subpasos | var(v) 0.0004 en lugar de 0.0016; Monte Carlo: empírica/P = 4.1 en v, 3.9 en θ |
| 4 | `nav_init()` tras vuelo | `aligned = 1`, `liftoff = 1` con el núcleo recién iniciado |
| 5 | giro recortado a 0.1 rad/s con alabeo de 2 rad/s | inclinación 22.9°, `n_sat = 0` |
| 6 | `f->fault = 1` con campos finitos | `eskf_check_health` = OK |
| 7 | una muestra de 2.5 g en rampa | despegue declarado |
| 7 | baro con `alt_ref` NaN o t NaN | solo el rechazo genérico del núcleo (−4) |

## 2. Cambios por archivo

**`eskf.c` / `eskf.h`** (cambios acotados; Φ, Q, modelos y Euler sin tocar):
- Puente: el ruido blanco de acelerómetro y giróscopo de una lectura repetida se trata como un solo error durante todo el hueco (`nsub·(σh)²` por subpaso = `(σ·dt)²`). Los random walk de los sesgos siguen siendo independientes. El paso normal no cambia (multiplicador 1), y la equivalencia con Python se mantiene.
- `eskf_check_health` devuelve `ESKF_E_FAULT` con un fallo enclavado; su documentación aclara que es numérica, no validez.
- `tau_max`: límite de desfase del barómetro, separado de `dt_max` (0 = `dt_max`, compatible con la v2).
- `eskf_align_static` borra también `degraded`.

**`navigation.c` / `navigation.h`** (nuevo; sustituye a `examples/adapter_example.c`): sesión completa en `nav_t`, reiniciada por `nav_init` (que también copia la configuración antes de borrar, por si se le pasa `&n->cfg`); tiempo con `t_rx` y `t_state` separados; política de huecos; reposo por ventana con corte instantáneo; despegue con persistencia; saturación por eje (bits del driver y límites); barómetro con latencia y referencia validada; validez VÁLIDO/DEGRADADO/NO VÁLIDO con motivos; registro de todos los retornos y contadores; configuración pendiente (`NAV_PENDING`) con `nav_config_check`.

**`examples/flight_loop_example.c`** (nuevo): adquisición → registro crudo → navegación → registro de salida, con desenrollado del contador de 32 bits. Si la configuración está pendiente, registra el campo y sigue registrando sin navegar.

**Pruebas:** `test/test_nav.c` (nuevo), `test/test_sim.c` (reescrito: ejecuta `navigation.c`), `test/nav_replay.c` (nuevo), `test/test_eskf.c` (prueba estadística del puente, `check_health`, `tau_max`), `test/test_tools.py` (pruebas de `nav_replay`), `test/bench.c` (tiempos de `navigation.c`).

**`README.md`**, **`Makefile`**: actualizados; `REVISION.md` de la v2 pasa a `REVISION_v2.md`.

**Cambios de API:** `eskf_check_health` puede devolver `ESKF_E_FAULT`; nuevo campo `eskf_params_t.tau_max` (el tamaño de la estructura cambia: recompilar todo, lo comprueba `eskf_init`). El adaptador de ejemplo desaparece; su función es ahora `navigation.h`.

## 3. Pruebas de regresión (una por fallo)

| Fallo | Prueba | v2 | v3 |
|---|---|---|---|
| 1 tiempos | `test_nav: regresion_1_tiempos` | p_z 0.12 / 0.15 | 0.100000 / 0.200000; `NAV_E_TIME_ORDER`, `_NONFINITE`, `_DUPLICATE` sin tocar nada |
| 2 hueco | `test_nav: regresion_2_hueco` | t_state 0.205, p_z 0.05 | `NAV_E_GAP`; t_state 0.000, p_z 0.0000, NO VÁLIDO; baro posterior `NAV_E_INVALID`; en rampa se realinea |
| 2 IMU inválida | ídem | t_state avanza | `NAV_E_VALUE`; t_state quieto; `NAV_R_STALE`; el siguiente `dt` se mide desde el estado |
| 3 puente | `test_eskf: prueba_puente_estadistica` (Monte Carlo 4000) | empírica/P v 4.14, θ 3.95 | v 1.03, θ 0.99, p 0.67 (conservadora); paso normal 1.04 / 1.01 |
| 4 reinicio | `test_nav: regresion_4_reinicio` | estáticas conservadas | arranque, reinicio tras vuelo y tras fallo: todo a cero, sin estimación publicada hasta alinear |
| 5 saturación | `test_nav: regresion_5_saturacion`, `test_sim: S6, S7` | n_sat 0, 22.9° | detectada por eje (driver y límite); inflado solo del eje de giro saturado; DEGRADADO dentro de lo admitido, NO VÁLIDO fuera |
| 6 validez | `test_nav: regresion_6_validez` | health OK con fallo | `ESKF_E_FAULT`; `nav_validity` NO VÁLIDO; STALE si la IMU se calla; DEGRADADO sin baro |
| 7 adaptador | `test_nav: regresion_7_adaptador`, `test_sim: S3, S3', S9` | despegue con 1 muestra | persistencia; retornos en `nav_t.last`; baro: t, alt, orden, referencia, estado atrasado, τ y latencia |

## 4. Comandos y resultados

```bash
make clean && make test        # C99 estricto double/float + C++ + pruebas + herramientas
make compare                   # (o REF=/ruta/solaris-eskf)
make bench; make stack
# sanitizadores (ASan + UBSan), double y float:
gcc -std=c99 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I. -Itest eskf.c navigation.c test/<prueba>.c -lm
```

| | double | float |
|---|---|---|
| test_eskf | 1449/1449 | 1447/1447 |
| test_nav | 126/126 | 126/126 |
| test_sim (13 escenarios + Monte Carlo de 20) | 80/80 | 80/80 |
| herramientas (comparador, `replay`, `nav_replay`) | 45/45 | — |
| ASan + UBSan en todas | sin errores | sin errores |
| clang `-Weverything` (núcleo, navegación, ejemplo) | 0 avisos | — |

Comparación con Python (`eskf.py` sin cambios desde `bc8cc99`; el comparador informa del HEAD `a7b7e99`): double 8 × 6214 comandos, peor 1.28e-13; float 3 × 6214, peor 1.67e-4. Iguales a la v2: el paso normal no ha cambiado.

Simulación sobre `navigation.c` (hipótesis: IMU 200 Hz ±10 %, baro 50 Hz σ 0.5 m, 12 g durante 3 s, apogeo ~3.2 km); resultados en double (float coincide salvo décimas):

| Escenario | err. alt. máx | en 3σ | incl. máx | Vuelo V/D/N | Final |
|---|---|---|---|---|---|
| S1 nominal | 0.14 m | 97 % | 0.76° | 100/0/0 % | VÁLIDO |
| S2 baro −60 m transónico, +80 m eyección, 0.8 s perdido | 0.28 m | 98 % | 0.88° | 93/7/0 % | VÁLIDO |
| S3 latencia baro 20 ms configurada | 0.16 m | 99 % | 0.79° | 100/0/0 % | VÁLIDO |
| S3' latencia 20 ms sin configurar (ref.) | 93 m | 0 % | 12.5° | 18/82/0 % | DEGRADADO |
| S4 hueco 60 ms en vuelo, sin puente | — | — | — | 29/0/71 % | NO VÁLIDO, estado congelado |
| S4' ídem con puente | 0.18 m | 94 % | 0.76° | 29/71/0 % | DEGRADADO |
| S5 hueco 0.3 s en rampa | 0.24 m | 100 % | 0.96° | 100/0/0 % | VÁLIDO (realineado) |
| S6 acc ±8 g, política por defecto | — | — | — | 0/0/100 % | NO VÁLIDO |
| S6' ídem, admitidos 5 s con exceso 4 g | 1.04 m | 99.5 % | 0.92° | 0/100/0 % | DEGRADADO |
| S7 giro recortado 0.1 rad/s | — | — | — | 0/0/100 % | NO VÁLIDO (5462 muestras detectadas) |
| S8 dup 1 %, desorden 0.5 %, NaN 0.2 %, sin puente | — | — | — | 0/0/100 % | NO VÁLIDO |
| S8' ídem con puente de 3 periodos | 0.18 m | 100 % | 1.07° | 0/100/0 % | DEGRADADO |
| S9 4 golpes de 3 g en rampa | 0.28 m | 98 % | 3.27° | 100/0/0 % | VÁLIDO, sin despegue falso |
| Monte Carlo 20 vuelos | ≤ 0.35 m | ≥ 96 % | — | 100 % válido | — |

En todos los escenarios el registro tiene una salida por muestra recibida, también tras la invalidación.

Tiempos y memoria **en este ordenador** (no en la placa):

| | double | float |
|---|---|---|
| `nav_on_imu` en vuelo | 4.0 µs | 3.4 µs |
| `nav_on_baro` | 1.2 µs | 1.1 µs |
| `sizeof(nav_t)` | 2848 B | 1696 B |
| pila de `predict_step` (`-fstack-usage`, x86-64) | 4112 B | 2144 B |

La pila en la placa es la de la cadena de llamadas `nav_on_imu → eskf_predict → predict_step`, con el compilador del firmware: hay que medirla allí.

## 5. Decisiones y pruebas descartadas

- **Puente desactivado** en la configuración de primera integración (`gap_max = 0`) aunque su ruido ya es correcto: el cambio real de aceleración durante el hueco no se modela. Con esta política, perder una muestra en vuelo invalida el resto de la estimación (S4, S8). Activarlo es una decisión a tomar con los registros reales (S4', S8').
- **Recuperación automática solo en rampa** (realineación). En vuelo, no.
- **S8 sin puente no llega a ser válido ni en rampa**: con un 0.2 % de muestras perdidas y un 0.5 % desordenadas, cada pérdida invalida la estimación antes de que se complete otra ventana de realineación de 5 s. O la adquisición no pierde muestras (es lo que debe mostrar el registro de banco), o hay que activar el puente.
- **Ventana corta para reanudar el reposo** tras un golpe (0.5 s): probada y descartada. Reducía la inclinación en S9 de 3.3° a 2.9°, pero bajaba la coherencia de la altura del 98 % al 60 % dentro de 3σ. Además, al actualizar la referencia barométrica con ventanas cortas, S1 caía al 46 %: el error de la referencia no está en P. Se mantiene una sola ventana de `rest_window_s`.
- **Una muestra con valor NaN no corta el reposo**: no indica movimiento, y cortarlo impedía alinear con una adquisición algo ruidosa.

## 6. Limitaciones y datos pendientes

Datos de placa y sensores necesarios para cerrar la configuración (`README.md`, «Configuración pendiente»): ruidos y Allan variance, periodo real y jitter de la IMU, sesgos de encendido, rangos y si el driver marca la saturación, umbrales de reposo del registro estático, curva de empuje, ruido y **latencia** del barómetro. También: microcontrolador y FPU, compilador, pila de la tarea, presupuesto de tiempo, y si hay magnetómetro o GPS (no integrados en `navigation.c`).

Limitaciones: política de tiempo estricta sin puente; golpes en rampa sin desplazamiento real contaminan la actitud (S9); sin detección de reposo en vuelo ni recuperación en vuelo; simulación simplificada (sin viento ni vibración); nada medido en la placa.

## 7. Siguiente paso mínimo

Registro de 10–30 min en reposo y unos minutos moviendo la placa, con la adquisición real y marcas de tiempo enteras, pasado por `build/nav_replay` con la configuración rellenada. Eso fija `dt_max`, el jitter real, los umbrales de reposo, σ y latencia del barómetro, y muestra si aparecen huecos (decisión sobre el puente). En la placa: medir `nav_on_imu` con el contador de ciclos y la pila de la cadena completa.
