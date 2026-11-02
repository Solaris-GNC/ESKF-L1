# Parámetros del servicio

Fuente única: `src/eskf_svc_params.c`, separada de los parámetros de los drivers y de la
aplicación Arduino v4. La tabla se genera con `make docs` y `test_spp_tools.py` comprueba que
coincide. Los KPID no están aquí: los asigna el router al registrar.

Dos columnas: **ESTRICTO** (sólo lo confirmado; lo demás pendiente, NaN, y la navegación no
arranca) y **EXPERIMENTAL** (todo numérico y coherente para operar el 7-nov, cada valor con su
procedencia). Ningún valor experimental es una calibración ni una validación física; no hay
coeficientes de Allan.

| procedencia | significado |
|---|---|
| confirmado (código SPP) | hecho del código SPP 9247d9e (no es una calibración de nuestra placa) |
| observado (registro) | medido en un registro existente: la captura histórica del 24-oct (reposo 1,768–10,768 s) |
| simulado | de las series OpenRocket «Simulation 6» o del Monte Carlo RocketPy adjuntos |
| propuesto (política) | decisión de diseño razonada |
| provisional | valor razonable sin medida |
| **pendiente** | sin valor (sólo ESTRICTO) |

«grupo» es el bit de `readiness` que la fila mantiene levantado mientras no esté confirmada
(«(confirmar)»). La preparación se calcula con la configuración EXACTA en uso: cualquier valor
de un grupo distinto de esta tabla levanta además `RDY_OVERRIDE` (procedencia externa, p. ej.
aviónica); los cambios de política (salida, respaldo, límites) no lo levantan pero cambian
`config_id`. Reglas mantenidas: float; `gap_max = 0`; `innov_gate = 5`; sin magnetómetro ni GPS;
sin continuación del giróscopo; continuación del acelerómetro sólo en EXPERIMENTAL y con ventana
del motor; nada de 50 Hz/×4 de la propuesta Arduino para el baro; ningún pendiente rellenado
con 0. Una tolerancia de fabricante no es una σ medida y un tiempo de conversión no es una latencia.

`config_id` = CRC-16 de los valores de esta tabla leídos de la configuración + perfil: igual en
el PC (float) y en la placa si los valores coinciden. Va en la salida y en la traza.

<!-- TABLA-INICIO (generada: make docs) -->
| parámetro | unidad | estricto | estado estricto | experimental | procedencia experimental | grupo | fuente / cómo actualizarlo |
|---|---|---|---|---|---|---|---|
| `in.imu_format` | B | 44 | confirmado (código SPP) | 44 | confirmado (código SPP) | — | icm20948.c: 11 float32 |
| `in.baro_semantics` | - | 1 | confirmado (código SPP) | 1 | confirmado (código SPP) | — | bmp390.c: altura relativa a p0/T0 medios de 500 muestras |
| `in.acc_div` | LSB/g | 8192 | confirmado (código SPP) | 8192 | confirmado (código SPP) | — | icm20948.c: raw/8192 |
| `in.gyro_div` | LSB/(deg/s) | 32.8 | confirmado (código SPP) | 32.8 | confirmado (código SPP) | — | icm20948.c: raw/32.8 |
| `in.acc_off_x` | g | 0.004621 | confirmado (código SPP) | 0.004621 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.acc_off_y` | g | -0.001157 | confirmado (código SPP) | -0.001157 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.acc_off_z` | g | 0.010014 | confirmado (código SPP) | 0.010014 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.gyro_off_x` | deg/s | 0.681567 | confirmado (código SPP) | 0.681567 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.gyro_off_y` | deg/s | 0.428753 | confirmado (código SPP) | 0.428753 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.gyro_off_z` | deg/s | -0.32962 | confirmado (código SPP) | -0.32962 | confirmado (código SPP) | — | icm20948.h: ya restado |
| `in.acc_unit_si` | m/s2 por unidad | 9.80665 | propuesto (política) | 9.80665 | propuesto (política) | unidades (confirmar) | contrato: aceleracion publicada en g |
| `in.gyro_unit_si` | rad/s por unidad | — | **pendiente** | 0.01745329252 | propuesto (política) | unidades (confirmar) | contrato: deg/s; sin correccion (FS 2000 con /32.8 sin aclarar) |
| `in.r_sb_00` | - | — | **pendiente** | 1 | propuesto (política) | ejes (confirmar) | identidad: cuerpo = IMU (no es el eje del cohete) |
| `in.r_sb_01` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_02` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_10` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_11` | - | — | **pendiente** | 1 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_12` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_20` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_21` | - | — | **pendiente** | 0 | propuesto (política) | ejes (confirmar) | identidad |
| `in.r_sb_22` | - | — | **pendiente** | 1 | propuesto (política) | ejes (confirmar) | identidad |
| `in.sat_margin_codes` | LSB | 1.5 | propuesto (política) | 1.5 | propuesto (política) | rango (confirmar) | candidato si abs(codigo) >= 32768-1.5; rango util a la salida del DMP sin verificar |
| `in.sat_to_nav` | - | 1 | propuesto (política) | 1 | propuesto (política) | — | candidatos -> saturacion en navigation.c |
| `in.sat_motor_window_s` | s | — | **pendiente** | 1.5 | simulado | rango (confirmar) | SIM: >4 g de 0.028 a 0.949 s; burnout 1.076 s; ventana desde el despegue detectado |
| `in.time_source` | - | 1 | propuesto (política) | 1 | propuesto (política) | tiempo (confirmar) | timestampMs de packetData (construccion, 1 ms) |
| `in.future_tol_us` | us | 1000 | propuesto (política) | 1000 | propuesto (política) | — | marca truncada a ms: nunca posterior a la recepcion con el mismo reloj |
| `in.baro_h_abs_max_m` | m | 5000 | propuesto (política) | 5000 | propuesto (política) | — | altura relativa; apogeo maximo Monte Carlo 474.7 m |
| `in.baro_p_min_pa` | Pa | 30000 | propuesto (política) | 30000 | propuesto (política) | — | BMP390: 300 hPa |
| `in.baro_p_max_pa` | Pa | 125000 | propuesto (política) | 125000 | propuesto (política) | — | BMP390: 1250 hPa |
| `in.baro_t_min_c` | degC | -40 | propuesto (política) | -40 | propuesto (política) | — | BMP390: -40 C |
| `in.baro_t_max_c` | degC | 85 | propuesto (política) | 85 | propuesto (política) | — | BMP390: +85 C |
| `bb.alpha` | - | 0.25 | propuesto (política) | 0.25 | propuesto (política) | — | alfa-beta fijo a ~100 Hz |
| `bb.beta` | - | 0.02 | propuesto (política) | 0.02 | propuesto (política) | — | alfa-beta fijo a ~100 Hz |
| `bb.gate_m` | m | 10 | propuesto (política) | 10 | propuesto (política) | — | residuo maximo admitido |
| `bb.reseed_n` | muestras | 20 | propuesto (política) | 20 | propuesto (política) | — | rechazos seguidos que pasan a recuperacion |
| `bb.dt_max_s` | s | 0.1 | propuesto (política) | 0.1 | propuesto (política) | — | 10 periodos del BMP a 100 Hz sin medida aceptada: recuperacion |
| `bb.timeout_s` | s | 0.5 | propuesto (política) | 0.5 | propuesto (política) | — | antiguedad maxima de la ultima medida ACEPTADA |
| `bb.ref_tau_s` | s | 5 | propuesto (política) | 5 | propuesto (política) | — | referencia en rampa (media exponencial) |
| `bb.liftoff_m` | m | 10 | propuesto (política) | 10 | propuesto (política) | — | congela la referencia si el ESKF no detecta el despegue |
| `bb.min_samples` | muestras | 20 | propuesto (política) | 20 | propuesto (política) | — | ventana coherente de inicio y de recuperacion (0.2 s a 100 Hz) |
| `bb.coh_m` | m | 1 | provisional | 1 | provisional | — | residuo max. del ajuste lineal de la ventana: ~7 sigma del ruido observado (0.14 m) con margen de curvatura |
| `bb.flight_vz_mps` | m/s | 3 | provisional | 3 | provisional | — | vuelo reconocido por el respaldo sin atestacion inercial: descenso simulado 4.3-5.9 m/s; ruido de v ~0.4 m/s |
| `bb.hold_s` | s | 1 | provisional | 1 | provisional | — | duracion continua de esa |v_z| (>= 3 m de desplazamiento vertical) o del reposo (|v_z| <= 1 m/s) que fija la referencia de lanzamiento |
| `svc.pad_start_s` | s | 10 | propuesto (política) | 10 | propuesto (política) | — | contrato de inicio: ningun despegue en este plazo tras encender o reiniciar la sesion en rampa (arranque de productores: calibracion del BMP 500 muestras ~5 s) |
| `svc.pad_gap_max_s` | s | 0.5 | provisional | 0.5 | provisional | — | hueco max. entre evidencias de rampa: < empuje > 2.5 g (~1 s, simulado) - nav.liftoff_hold_s; > paron de SD de 34 ms |
| `svc.pad_lat_max_s` | s | 0.5 | propuesto (política) | 0.5 | propuesto (política) | — | latencia de entrega admitida antes de dar por caducada la rampa en silencio total: cola del router 128 paquetes / 325 paquetes/s de ICM+BMP = 0.39 s (codigo SPP) + margen |
| `svc.merge_holdoff_us` | us | 15000 | propuesto (política) | 15000 | propuesto (política) | anfitrión (confirmar) | espera de orden: baro cada ~10 ms |
| `svc.max_items` | paquetes | 16 | propuesto (política) | 16 | propuesto (política) | anfitrión (confirmar) | por consumeData |
| `svc.max_events` | llamadas | 24 | propuesto (política) | 24 | propuesto (política) | anfitrión (confirmar) | nav_on_* por consumeData |
| `svc.snapshot_period_us` | us | 200000 | propuesto (política) | 200000 | propuesto (política) | — | estado completo 5 Hz x 5 fragmentos (registro) |
| `svc.diag_period_us` | us | 1000000 | propuesto (política) | 1000000 | propuesto (política) | — | 1 Hz |
| `svc.ops_period_us` | us | 1000000 | propuesto (política) | 1000000 | propuesto (política) | — | compacta 1 Hz (LoRa SF7/125 kHz: ~160 ms por paquete) |
| `svc.snapshot_mask` | - | 31 | propuesto (política) | 31 | propuesto (política) | — | 5 fragmentos |
| `svc.trace_enable` | - | 0 | propuesto (política) | 0 | propuesto (política) | — | traza de validacion: activar en banco |
| `eskf.sigma_a` | m/s2 | — | **pendiente** | 0.12 | observado (registro) | ruido (confirmar) | OBS: 0.06 por eje en reposo (registro historico) x2; vibracion de vuelo no medida |
| `eskf.sigma_w` | rad/s | — | **pendiente** | 0.01 | observado (registro) | ruido (confirmar) | OBS: <=0.0046 por eje en reposo x2 |
| `eskf.sigma_aw` | m/s2/sqrt(s) | — | **pendiente** | 0.0001 | provisional | ruido (confirmar) | sin Allan |
| `eskf.sigma_ww` | rad/s/sqrt(s) | — | **pendiente** | 1e-05 | provisional | ruido (confirmar) | sin Allan |
| `eskf.ba0_std` | m/s2 | — | **pendiente** | 0.3 | provisional | ruido (confirmar) | residual tras los offsets del productor: sin medir |
| `eskf.bg0_std` | rad/s | — | **pendiente** | 0.01 | observado (registro) | ruido (confirmar) | OBS: media en reposo <=0.0095 (sin offsets) |
| `eskf.dt_max` | s | — | **pendiente** | 0.01 | observado (registro) | tiempo (confirmar) | OBS: intervalos normales 1-7 ms (marcas de 1 ms); 1 muestra perdida = 8.9 ms |
| `eskf.gap_max` | s | 0 | propuesto (política) | 0 | propuesto (política) | — | puente deshabilitado |
| `eskf.innov_gate` | sigma | 5 | propuesto (política) | 5 | propuesto (política) | — | puerta de innovacion |
| `eskf.tau_max` | s | — | **pendiente** | 0.03 | provisional | baro (confirmar) | >= baro_latency_s + dt_max + margen |
| `eskf.accel_threshold` | m/s2 | — | **pendiente** | 0.5 | observado (registro) | umbrales (confirmar) | OBS: abs(norma(a)-g) max 0.29 en reposo |
| `eskf.align_acc_tol` | m/s2 | — | **pendiente** | 0.6 | observado (registro) | umbrales (confirmar) | OBS: abs(norma(a media)-g) 0.06; margen x10 |
| `eskf.align_gyro_max` | rad/s | — | **pendiente** | 0.1 | observado (registro) | umbrales (confirmar) | OBS: norma(w media) 0.012 |
| `eskf.p0_std` | m | 1 | propuesto (política) | 1 | propuesto (política) | — | politica |
| `eskf.v0_std` | m/s | 0.1 | propuesto (política) | 0.1 | propuesto (política) | — | politica |
| `eskf.th0_std` | rad | 0.05 | propuesto (política) | 0.05 | propuesto (política) | — | politica |
| `eskf.has_mag_ref` | - | 0 | propuesto (política) | 0 | propuesto (política) | — | magnetometro no fusionado |
| `eskf.mag_ref_n` | uT | 0 | propuesto (política) | 0 | propuesto (política) | — | sin uso |
| `eskf.mag_ref_e` | uT | 0 | propuesto (política) | 0 | propuesto (política) | — | sin uso |
| `eskf.mag_ref_d` | uT | 0 | propuesto (política) | 0 | propuesto (política) | — | sin uso |
| `rest_window_s` | s | 5 | propuesto (política) | 5 | propuesto (política) | — | no se acorta por huecos; 5 s x 100 Hz = 500 < 512 |
| `rest_update_period_s` | s | 0.05 | propuesto (política) | 0.05 | propuesto (política) | — | politica |
| `zupt_sigma` | m/s | 0.02 | propuesto (política) | 0.02 | propuesto (política) | — | politica |
| `grav_sigma` | m/s2 | 0.3 | propuesto (política) | 0.3 | propuesto (política) | — | politica |
| `rest_acc_dev_max` | m/s2 | — | **pendiente** | 0.6 | observado (registro) | umbrales (confirmar) | OBS: max 0.29 en reposo x2 |
| `rest_acc_std_max` | m/s2 | — | **pendiente** | 0.15 | observado (registro) | umbrales (confirmar) | OBS: 0.061 en reposo x2.5 |
| `rest_gyro_max` | rad/s | — | **pendiente** | 0.15 | provisional | umbrales (confirmar) | OBS: max 0.027 en mesa; margen para viento en rampa |
| `rest_tilt_change_max` | rad | — | **pendiente** | 0.01 | provisional | umbrales (confirmar) | provisional |
| `liftoff_acc` | m/s2 | — | **pendiente** | 24.516625 | simulado | despegue (confirmar) | SIM: f > 4 g casi toda la combustion; 2.5 g < recorte de 4 g |
| `liftoff_hold_s` | s | — | **pendiente** | 0.05 | simulado | despegue (confirmar) | SIM: f > 2.5 g continuo ~0.9 s |
| `acc_sat_limit` | m/s2 | 0 | propuesto (política) | 0 | propuesto (política) | — | 0: solo candidatos del dominio de codigos |
| `gyro_sat_limit` | rad/s | 0 | propuesto (política) | 0 | propuesto (política) | — | 0: solo candidatos del dominio de codigos |
| `acc_sat_max_s` | s | 0 | propuesto (política) | 1.2 | simulado | rango (confirmar) | SIM: 0.92 s por encima de 4 g en la combustion, +30 % |
| `gyro_sat_max_s` | s | 0 | propuesto (política) | 0 | propuesto (política) | — | giroscopo: sin continuacion |
| `acc_sat_excess` | m/s2 | 0 | propuesto (política) | 110 | simulado | rango (confirmar) | SIM: pico 130.95 - 39.2 recortado = 91.8, +20 % |
| `gyro_sat_excess` | rad/s | 0 | propuesto (política) | 0 | propuesto (política) | — | giroscopo: sin continuacion |
| `baro_sigma` | m | — | **pendiente** | 0.5 | observado (registro) | ruido (confirmar) | OBS: 0.14 entre muestras en reposo; margen por efectos dinamicos no medidos |
| `baro_latency_s` | s | — | **pendiente** | 0.01 | provisional | baro (confirmar) | IIR coeficiente 1 (registro 0x02) a 100 Hz: ~1 muestra; no medido |
| `baro_timeout_s` | s | 1 | propuesto (política) | 1 | propuesto (política) | — | politica |
| `baro_ref_min_n` | muestras | 50 | propuesto (política) | 50 | propuesto (política) | — | politica |
| `baro_ref_max_step` | m | 2 | propuesto (política) | 2 | propuesto (política) | — | politica |
| `stale_max_s` | s | 0.05 | propuesto (política) | 0.05 | propuesto (política) | anfitrión (confirmar) | >= espera de orden + dt_max |
<!-- TABLA-FIN -->
