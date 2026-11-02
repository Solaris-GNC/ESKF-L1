# Parámetros del firmware y del estimador

Fuente única: `viradinha_nav/src/vn/vn_params.c`. La tabla de abajo se **genera** desde ese archivo (`make -C firmware docs`) y `make -C firmware test` comprueba que coincide. Para cambiar un valor: editar `vn_params.c` (valor, estado y origen), `make docs`, `make test`.

## Estados

| estado | significado | dónde puede aparecer |
|---|---|---|
| 1 confirmado | por el hardware o por documentación (p. ej. sensibilidad nominal del rango elegido) | vuelo y banco |
| 2 propuesto | configuración elegida por el equipo, todavía sin comprobar en la placa | vuelo y banco |
| 3 provisional | estimación para poder ejercitar la navegación en el banco | **sólo banco** |
| 4 pendiente | falta la medida o la información del equipo; en vuelo el valor es NaN | **sólo vuelo** |

Reglas que comprueba `test_vn`:

- Todo lo que no está pendiente vale lo mismo en banco y en vuelo; el perfil de banco sólo sustituye lo pendiente por valores provisionales.
- En vuelo, un parámetro pendiente es NaN y `nav_config_check` impide arrancar la navegación. El registro crudo funciona igual.
- Cada campo de `nav_config_t` (incluido `eskf_params_t`), de `imu_conv_cfg_t` y de `vn_hw_t` está en la tabla con su tipo y su desplazamiento. Si se añade un campo y no se registra, la prueba falla.
- `baro_latency_s` sigue pendiente. Los tiempos de conversión del BMP390 (10,88 ms típico, 12,48 ms máximo) son documentación (`doc.*`) y no configuran nada.

Perfiles (`vn_profile.h`):

- **ADQUISICIÓN** (por defecto): registra los valores de vuelo de la tabla y usa los del hardware.
- **BANCO**: usa los provisionales. La cabecera y un texto periódico dicen «NO VOLAR».
- **VUELO**: sólo confirmado y propuesto. Hoy no navega porque quedan valores pendientes (ver abajo). El motivo queda en el registro.

## Tabla

<!-- TABLA-INICIO (generada: make -C firmware docs) -->
| parámetro | vuelo | estado (vuelo) | banco | estado (banco) | origen / cómo obtenerlo |
|---|---|---|---|---|---|
| `hw.imu_acc_fs_g` | 16 | propuesto | 16 | propuesto | propuesta; comprobar con OpenRocket |
| `hw.imu_gyro_fs_dps` | 2000 | propuesto | 2000 | propuesto | propuesta equipo |
| `imu.acc_lsb_per_g` | 2048 | confirmado | 2048 | confirmado | doc ±16 g (SparkFun 1.3.2 getAccMG) |
| `imu.gyro_lsb_per_dps` | 16.4 | confirmado | 16.4 | confirmado | doc ±2000 dps (SparkFun getGyrDPS) |
| `imu.g_si` | 9.80665 | confirmado | 9.80665 | confirmado | gravedad estándar (definición de g) |
| `hw.imu_acc_div` | 4 | propuesto | 4 | propuesto | propuesta: acel 1.125k/(1+4)=225 Hz (doc) |
| `hw.imu_gyro_div` | 4 | propuesto | 4 | propuesto | propuesta: giro 1.1k/(1+4)=220 Hz (doc) |
| `hw.imu_dlpf_cfg` | 3 | propuesto | 3 | propuesto | propuesta: 3dB 50.4 Hz acel/51.2 Hz giro |
| `imu.sat_margin_lsb` | 0 | propuesto | 0 | propuesto | politica: solo en el extremo; revisar DLPF |
| `imu.axis_map_x` | — | **pendiente** | 0 | provisional | montaje real del sensor en el cohete |
| `imu.axis_map_y` | — | **pendiente** | 1 | provisional | montaje real |
| `imu.axis_map_z` | — | **pendiente** | 2 | provisional | montaje real |
| `imu.axis_sign_x` | — | **pendiente** | 1 | provisional | montaje real |
| `imu.axis_sign_y` | — | **pendiente** | 1 | provisional | montaje real |
| `imu.axis_sign_z` | — | **pendiente** | 1 | provisional | montaje real |
| `hw.mag_div` | 8 | propuesto | 8 | propuesto | mag crudo cada 8 lecturas (no se fusiona) |
| `hw.baro_odr_hz` | 50 | propuesto | 50 | propuesto | propuesta: normal 50 Hz, IIR off |
| `hw.baro_osr_p` | 4 | propuesto | 4 | propuesto | propuesta: presion x4 |
| `hw.baro_osr_t` | 1 | propuesto | 1 | propuesto | propuesta: temperatura x1 |
| `doc.baro_conv_typ_ms` | 10.88 | confirmado | 10.88 | confirmado | doc conversion tipica (NO es latencia) |
| `doc.baro_conv_max_ms` | 12.48 | confirmado | 12.48 | confirmado | doc conversion maxima (NO es latencia) |
| `hw.baro_p0_pa` | 101325 | propuesto | 101325 | propuesto | ISA: altitud = 44330(1-(p/p0)^0.1903) |
| `hw.i2c_hz` | 400000 | propuesto | 400000 | propuesto | propuesta; segun compatibilidad electrica |
| `hw.pin_sda` | -1 | **pendiente** | -1 | provisional | -1: SDA por defecto de Wire (A4) |
| `hw.pin_scl` | -1 | **pendiente** | -1 | provisional | -1: SCL por defecto de Wire (A5) |
| `hw.pin_imu_int` | -1 | **pendiente** | -1 | provisional | -1: sondeo (marcas +-1 ms) |
| `hw.pin_baro_int` | -1 | **pendiente** | -1 | provisional | -1: sondeo del STATUS |
| `hw.imu_addr` | 0 | **pendiente** | 0 | provisional | 0: autodeteccion 0x69/0x68 (registrada) |
| `hw.baro_addr` | 0 | **pendiente** | 0 | provisional | 0: autodeteccion 0x77/0x76 (registrada) |
| `fw.merge_holdoff_us` | 10000 | propuesto | 10000 | propuesto | espera para ordenar IMU/baro |
| `fw.navout_hz` | 50 | propuesto | 50 | propuesto | salida completa al registro |
| `fw.telemetry_hz` | 10 | propuesto | 10 | propuesto | gancho de telemetria |
| `fw.stats_hz` | 1 | propuesto | 1 | propuesto | tiempos, pila, colas |
| `fw.header_period_s` | 10 | propuesto | 10 | propuesto | repite cabecera y config |
| `fw.log_navev` | 1 | propuesto | 1 | propuesto | retorno de cada evento |
| `fw.log_ticks` | 1 | propuesto | 1 | propuesto | reloj de nav_tick (reproduccion exacta) |
| `fw.stack_bytes` | 16384 | propuesto | 16384 | propuesto | reserva provisional; medir HWM |
| `fw.ring_bytes` | 49152 | propuesto | 49152 | propuesto | ~1 s de registro; segun almacenamiento |
| `fw.queue_depth` | 48 | propuesto | 48 | propuesto | cola adquisicion->navegacion (eventos) |
| `fw.i2c_timeout_ms` | 5 | propuesto | 5 | propuesto | Wire.setTimeOut; bus colgado = error contado |
| `fw.irq_timeout_ms` | 5 | propuesto | 5 | propuesto | sin irq IMU en el plazo: sondeo de respaldo |
| `fw.baro_poll_us` | 2000 | propuesto | 2000 | propuesto | intervalo minimo entre sondeos del baro |
| `fw.imu_reinit_errors` | 100 | propuesto | 100 | propuesto | errores I2C seguidos para reiniciar la IMU |
| `fw.boot_bench` | 1 | propuesto | 1 | propuesto | prueba de tiempos/pila al arrancar |
| `fw.core_acq` | 1 | propuesto | 1 | propuesto | nucleo de la tarea de adquisicion |
| `fw.core_nav` | 0 | propuesto | 0 | propuesto | nucleo de la tarea de navegacion |
| `fw.core_log` | 0 | propuesto | 0 | propuesto | nucleo de la tarea de registro |
| `fw.prio_acq` | 20 | propuesto | 20 | propuesto | prioridad FreeRTOS (loop de Arduino = 1) |
| `fw.prio_nav` | 15 | propuesto | 15 | propuesto | prioridad FreeRTOS |
| `fw.prio_log` | 5 | propuesto | 5 | propuesto | prioridad FreeRTOS |
| `fw.axes_confirmed` | 0 | **pendiente** | 0 | provisional | 1 cuando se confirme el montaje |
| `eskf.sigma_a` | — | **pendiente** | 0.05 | provisional | por evento: analyze_static (sigma_eq) |
| `eskf.sigma_w` | — | **pendiente** | 0.005 | provisional | por evento: analyze_static (sigma_eq) |
| `eskf.sigma_aw` | — | **pendiente** | 0.0001 | provisional | K de Allan: registro estatico largo |
| `eskf.sigma_ww` | — | **pendiente** | 1e-05 | provisional | K de Allan: registro estatico largo |
| `eskf.ba0_std` | — | **pendiente** | 0.5 | provisional | 6 posiciones + arranques; banco: provisional |
| `eskf.bg0_std` | — | **pendiente** | 0.01 | provisional | residual tras alinear; arranques repetidos |
| `eskf.dt_max` | 0.007 | propuesto | 0.007 | propuesto | propuesta 225 Hz; revisar periodo y jitter |
| `eskf.gap_max` | 0 | propuesto | 0 | propuesto | puente deshabilitado |
| `eskf.innov_gate` | 5 | propuesto | 5 | propuesto | propuesta |
| `eskf.tau_max` | — | **pendiente** | 0.03 | provisional | >= baro_latency_s + dt_max |
| `eskf.accel_threshold` | — | **pendiente** | 0.5 | provisional | registro estatico |
| `eskf.align_acc_tol` | — | **pendiente** | 0.6 | provisional | registro estatico (6 posiciones) |
| `eskf.align_gyro_max` | — | **pendiente** | 0.1 | provisional | registro estatico (sesgo giro) |
| `eskf.has_mag_ref` | 0 | propuesto | 0 | propuesto | magnetometro registrado, no fusionado |
| `eskf.mag_ref_n` | 0 | propuesto | 0 | propuesto | sin uso (has_mag_ref = 0) |
| `eskf.mag_ref_e` | 0 | propuesto | 0 | propuesto | sin uso (has_mag_ref = 0) |
| `eskf.mag_ref_d` | 0 | propuesto | 0 | propuesto | sin uso (has_mag_ref = 0) |
| `eskf.p0_std` | 1 | propuesto | 1 | propuesto | politica |
| `eskf.v0_std` | 0.1 | propuesto | 0.1 | propuesto | politica |
| `eskf.th0_std` | 0.05 | propuesto | 0.05 | propuesto | politica |
| `rest_window_s` | 5 | propuesto | 5 | propuesto | propuesta |
| `rest_update_period_s` | 0.05 | propuesto | 0.05 | propuesto | propuesta |
| `zupt_sigma` | 0.02 | propuesto | 0.02 | propuesto | politica provisional, comprobar |
| `grav_sigma` | 0.3 | propuesto | 0.3 | propuesto | politica provisional, comprobar |
| `rest_acc_dev_max` | — | **pendiente** | 0.6 | provisional | registro estatico |
| `rest_acc_std_max` | — | **pendiente** | 0.15 | provisional | registro estatico |
| `rest_gyro_max` | — | **pendiente** | 0.15 | provisional | registro estatico |
| `rest_tilt_change_max` | — | **pendiente** | 0.01 | provisional | registro estatico |
| `liftoff_acc` | — | **pendiente** | 30 | provisional | curva de empuje (OpenRocket) |
| `liftoff_hold_s` | 0.05 | propuesto | 0.05 | propuesto | propuesta |
| `acc_sat_limit` | 0 | propuesto | 0 | propuesto | 0: saturacion por codigo (imu_conv) |
| `gyro_sat_limit` | 0 | propuesto | 0 | propuesto | 0: saturacion por codigo (imu_conv) |
| `acc_sat_max_s` | 0 | propuesto | 0 | propuesto | modelo deshabilitado |
| `gyro_sat_max_s` | 0 | propuesto | 0 | propuesto | modelo deshabilitado |
| `acc_sat_excess` | 0 | propuesto | 0 | propuesto | solo si acc_sat_max_s > 0 |
| `gyro_sat_excess` | 0 | propuesto | 0 | propuesto | solo si gyro_sat_max_s > 0 |
| `baro_sigma` | — | **pendiente** | 0.5 | provisional | registro estatico |
| `baro_latency_s` | — | **pendiente** | 0 | provisional | medir; banco 0 = sin compensar |
| `baro_timeout_s` | 1 | propuesto | 1 | propuesto | politica provisional |
| `baro_ref_min_n` | 50 | propuesto | 50 | propuesto | politica |
| `baro_ref_max_step` | 2 | propuesto | 2 | propuesto | politica |
| `stale_max_s` | 0.05 | propuesto | 0.05 | propuesto | >= holdoff + dt_max |
<!-- TABLA-FIN -->

## Cómo obtener lo pendiente

Cada medida se anota en `vn_params.c` como **confirmada** y con su origen: el registro (fecha y `boot_id`) y la herramienta usada. Los candidatos que imprimen las herramientas son propuestas: hay que revisarlos antes de anotarlos.

| parámetro | cómo | herramienta / procedimiento |
|---|---|---|
| `imu.axis_map_*`, `imu.axis_sign_*`, `fw.axes_confirmed` | Dos posiciones estáticas con ejes del cohete conocidos hacia arriba y un giro positivo a mano. Antes hay que fijar la **convención de ejes del cohete**, que hoy está pendiente. | `axes_check.py`, PROCEDIMIENTOS §3 |
| `hw.pin_*`, `hw.imu_addr`, `hw.baro_addr` | Esquema real: módulos concretos, adaptación de niveles a 3,3 V, pines Arduino (Dx/Ax con remapeo) y si las INT están conectadas. Con −1 se usan los pines I2C por defecto (A4/A5) y sondeo. Con 0 se autodetecta la dirección y queda en el registro. | equipo |
| `eskf.sigma_a`, `eskf.sigma_w` | Son desviaciones **por evento**: `Q_v = σ_a²·dt²` (`eskf.c`, `predict_step`). Con el DLPF las muestras están correladas, y con interrupción de cualquier sensor la mitad de los eventos repiten un valor. Por eso se usa σ_eq = N·√(E[dt]/E[dt²]), donde N es la densidad de Allan y dt son los intervalos reales entre eventos. Esa σ_eq conserva el paseo aleatorio de velocidad N²·t. La σ por muestra lo subestima: en la simulación, ~40 % de la varianza. En reposo es una cota inferior; la vibración en vuelo está pendiente. | `analyze_static.py` |
| `eskf.sigma_aw`, `eskf.sigma_ww` | Son densidades: `Q_ba = σ_aw²·dt`, y corresponden a K de Allan. Un registro de 10–30 min **no** suele identificarlas, y la herramienta lo dice: da una cota superior y la duración orientativa necesaria. | `analyze_static.py` con un registro largo |
| `eskf.ba0_std` | Del orden del mayor sesgo de seis posiciones, más la dispersión entre arranques (varios registros con arranques distintos) y un margen por temperatura. La tolerancia de fábrica (±50 mg) **no** es una σ. | `six_position.py`, PROCEDIMIENTOS §4 |
| `eskf.bg0_std` | Residual tras alinear. La alineación toma el sesgo del giro en la ventana de reposo, así que se usa la dispersión de `b_g` (NAVOUT) entre arranques y entre ventanas. El ±5 °/s de fábrica **no** sirve. | registros de banco, `vnlog.py csv` |
| `rest_acc_dev_max`, `rest_acc_std_max`, `rest_gyro_max`, `rest_tilt_change_max`, `eskf.align_acc_tol`, `eskf.align_gyro_max`, `eskf.accel_threshold` | Máximos en ventanas de `rest_window_s` del registro estático, con margen (la herramienta imprime el margen que propone). Comprobar después en la rampa, con viento y con golpes. | `analyze_static.py` |
| `liftoff_acc` | Curva de empuje de OpenRocket (pendiente). Debe superar 9,81 + `rest_acc_dev_max` (lo comprueba `nav_config_check`) y ser compatible con el empuje mínimo en los primeros `liftoff_hold_s`. | equipo |
| `hw.imu_acc_fs_g` (16 g) | Comparar con la fuerza específica máxima prevista en ejes del sensor, más margen de vibración (OpenRocket, pendiente). La saturación se detecta por código crudo en los dos extremos (`imu_conv`). | equipo |
| `baro_sigma` | Ruido de altitud en reposo a 50 Hz. Faltan los efectos dinámicos en vuelo. | `analyze_static.py` |
| `baro_latency_s` | Latencia efectiva respecto a la marca de la IMU, con este firmware y esta configuración. Media de varias repeticiones. | `baro_updown.py`, PROCEDIMIENTOS §6 |
| `eskf.tau_max` | ≥ `baro_latency_s` + `dt_max` (lo comprueba `nav_config_check`). | — |
| `eskf.dt_max` (0,007 propuesto) | Comprobar con los intervalos reales. Con la interrupción por cualquier sensor llegan ~445 eventos/s con intervalos de 0 a 4,5 ms. Un hueco real (≥ 2 periodos de 225 Hz) debe seguir superándolo. | `analyze_static.py`, prueba de 2 h |

## Datos del vuelo que faltan (no se han usado datos de otros cohetes)

- Archivo de OpenRocket y configuración final de cohete y motor.
- Curvas de aceleración, velocidad y empuje; apogeo y tiempo de combustión.
- Fuerza específica máxima en ejes del sensor, con margen por vibración, para comprobar los ±16 g.
- Umbral y persistencia de despegue compatibles con esa curva.
- Convención de ejes del cohete y montaje físico de la placa.
- Medio real de almacenamiento y de telemetría, y el firmware existente del equipo (no se ha recibido).
