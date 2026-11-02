# Hipótesis usadas, campos que aviónica puede actualizar y lo que queda abierto

Decisión del equipo (31-oct): **no habrá nueva campaña** de calibración, seis posiciones, Allan
ni caracterización. Aviónica calibra los sensores en SPP. El servicio usa los datos existentes y,
para lo no medido, valores provisionales razonables con su procedencia explícita; la navegación
no se bloquea por ellos (perfil EXPERIMENTAL) y nada de ello se presenta como medido o validado.
Sin parches del anfitrión ni de SPP. Vuelo el 7-nov; sin trabajo del 2 al 6.

## Hechos extraídos del código (no hay que pedirlos)

| dato | valor | fuente |
|---|---|---|
| formato IMU | 44 B: a (g), ω (°/s), q DMP w,x,y,z, accuracy | `icm20948.c` |
| rango y divisores | ACCEL ±4 g, /8192; GYRO_FS ±2000 °/s, /32,8 | `icm20948.c` |
| filtros y ODR del ICM | acel sin DLPF; giro FCHOICE 1, DLPF_CFG 0; `SMPLRT_DIV = 4` (225 Hz) | `icm20948.c` |
| offsets aplicados por el productor | acel 0.004621, −0.001157, 0.010014 g; giro 0.681567, 0.428753, −0.329620 °/s | `icm20948.h` |
| publicación ICM | lee toda la FIFO y publica sólo el último paquete; reinicia la FIFO si > 512 B | `icm20948.c` |
| BMP390 | OSR ×1, ODR 100 Hz, IIR coeficiente 1 (registro 0x1F = 0x02); 500 muestras de calibración sin publicar; altura relativa | `bmp390.h/.c` |
| marca temporal | `timestampMs` de `packetData` al construir (1 ms) | `databank.c` |
| router, datalogger, SX1262 | docs/SERVICIO.md y docs/SALIDA.md | `pubsub.c`, `datalogger`, `sx1262` |

## Hipótesis del perfil EXPERIMENTAL (procedencia en docs/PARAMETROS_SPP.md)

| grupo | hipótesis | procedencia |
|---|---|---|
| unidades | aceleración publicada en g (×9,80665); giro en °/s reales (×π/180), sin ×2 | contrato (propuesto) |
| ejes | R = identidad: cuerpo del estimador = marco de la IMU; no se afirma cuál es el eje del cohete | provisional |
| ruido | σ_a 0,12 m/s² y σ_ω 0,01 rad/s (reposo observado × 2); σ de paseo de sesgos 1e-4 / 1e-5 (SIN Allan); σ iniciales de sesgo 0,3 m/s² / 0,01 rad/s | observado / provisional |
| tiempo | `dt_max` 10 ms (intervalos normales 1–7 ms; una muestra perdida 8,9 ms) | observado |
| umbrales de reposo | |a|−g ≤ 0,6, σ ≤ 0,15 m/s², ω ≤ 0,15 rad/s, giro de gravedad ≤ 0,01 rad | observado / provisional |
| despegue | ‖a‖ > 2,5 g durante 0,05 s (f > 4 g casi toda la combustión) | simulado |
| saturación | continuación del acelerómetro ≤ 1,2 s acumulados, exceso 110 m/s², sólo en la ventana de 1,5 s desde el despegue | simulado (+30 % / +20 %) |
| barómetro | σ 0,5 m (0,14 observado + margen); latencia 10 ms (IIR coeficiente 1, sin medir); `tau_max` 30 ms | observado / provisional |
| respaldo barométrico | α 0,25, β 0,02, puerta 10 m, caducidad 0,5 s, referencia τ 5 s, congelación a 10 m; ventana de 20 medidas con residuo ≤ 1 m para iniciar o recuperar | propuesto; la tolerancia de 1 m, provisional |
| fase (rampa confirmada o incierta) | contrato de inicio: sin despegue en los 10 s siguientes al inicio de la sesión; hueco máximo entre evidencias de rampa 0,5 s (< empuje > 2,5 g de ~1 s − 0,05 s de espera del detector); latencia de entrega admitida 0,5 s antes de caducar por el reloj en silencio total (cola del router de 128 paquetes ≈ 0,39 s) | propuesto (contrato, código SPP) / provisional (simulado) |
| vuelo por el respaldo (sin atestación inercial) | \|v_z\| ≥ 3 m/s durante 1 s; la referencia de lanzamiento exige 1 s de reposo continuo (\|v_z\| ≤ 1 m/s) en rampa | provisional (descenso simulado 4,3–5,9 m/s; ruido de v_z del respaldo ~0,4 m/s) |

Simulación usada (sólo sus resultados guardados; no se ha vuelto a simular): series OpenRocket
«Simulation 6 – Nominal (3,36 m/s, 3°)» y «– Adverse (8,9 m/s, 20°)» de `PUMEL_v3-7_Klima.ork` y el
resumen Monte Carlo de `Montecarlo_Viradinha.py` (RocketPy, `mc_resumen.json`/`determinista.json`:
nominal apogeo medio 399,5 m, máximo 474,7 m, determinista 401,1 m a 9,02 s; adverso medio 322,5 m,
máximo 386,4 m; despliegue a 9,05 s; descenso ≈ −4,3 m/s). Las series OpenRocket dan 390,8 m y
310,4 m. H165R, rail 2,5 m, combustión 1,05–1,08 s, f máx 13,35 g a 0,399 s,
despliegue 8 s tras el apagado (9,077 s; 57 g en el adverso). No se mezclan `rocket.ork` (otro
cohete), `SimuTWR.py` ni `Viradinha_flight_data.csv` (otras ejecuciones), ni Windy.

## Campos que aviónica puede fijar sin cambiar el algoritmo

Se rellenan en `eskf_svc_config_t` antes de `SPP_SERVICES_ESKF_configure`; cualquier valor
distinto de la tabla levanta `RDY_OVERRIDE` y cambia `config_id` (trazable en la salida):

- `acc_unit_si`, `gyro_unit_si` — escala real si se confirma (p. ej. el factor del giro).
- `r_sb[9]` — montaje de la IMU (cuerpo = R · sensor).
- `acc_off[3]`, `gyro_off[3]` — sólo si el productor cambia sus offsets (se usan para volver al
  dominio de códigos; NO se restan otra vez).
- `nav.eskf.sigma_a/sigma_w/sigma_aw/sigma_ww/ba0_std/bg0_std`, `nav.baro_sigma`,
  `nav.baro_latency_s` — si aviónica tiene valores mejores.
- `nav.liftoff_acc/liftoff_hold_s`, `nav.acc_sat_max_s/acc_sat_excess`, `sat_motor_window_s` —
  si cambia el motor o la simulación.
- `boot_id`, `ops_period_us`, `snapshot_period_us`, `diag_period_us` — identificación y salida.

## Lo que sigue abierto (no bloquea la navegación experimental)

| # | qué | efecto si es distinto de la hipótesis |
|---|---|---|
| 1 | escala efectiva del giro a la salida del DMP (¿/32,8 con FS 2000?) | si el real es el doble, actitud y gravedad proyectada mal en rotaciones; en la captura histórica el giro integrado coincide con el DMP (cociente 0,99), lo que apoya la hipótesis |
| 2 | montaje real de la IMU | nada mientras R = identidad se interprete como marco de la IMU; el eje del cohete se documenta aparte |
| 3 | rango útil del DMP y si recorta exactamente en ±32767 | el candidato de saturación usa el dominio de códigos con margen de 1,5 LSB |
| 4 | vibración del motor | no está en la simulación ni en los escenarios: la σ_a experimental puede quedarse corta en la combustión (el baro la compensa en altura) |
| 5 | jitter y parones con TODOS los servicios activos | huecos > 10 ms invalidan hasta realinear en rampa (en vuelo, hasta el final) |
| 6 | ejecución en placa, tiempos y pila reales | docs/VALIDACION.md; `RDY_BOARD` sigue levantado |
| 7 | integración de radio, su frecuencia y el ciclo de trabajo de 868 MHz | decidido para esta entrega: registro completo en SD como configuración principal; la radio es de Aviónica y no bloquea el servicio (docs/SALIDA.md) |

## Consecuencia para el 7-nov

Perfil EXPERIMENTAL operativo: navega en rampa y en vuelo, publica su validez dinámica, sus
capacidades y el respaldo barométrico; `readiness ≠ 0`, `NOT_VALIDATED` y `EXPERIMENTAL` en cada
mensaje, `flight_accept = 0`. ESTRICTO sigue sin navegar (`why_code` → `in.gyro_unit_si`). Con ±4 g
la combustión recorta: DEGRADADO con continuación; un recorte fuera de la ventana del motor o más
largo que el modelo invalida el ESKF y la fuente pasa al respaldo barométrico.
