# Comprobaciones pendientes en placa y comparación placa–PC

Nada de esto se ha ejecutado: el componente está probado en el PC y compilado para ESP-IDF con una
aplicación de prueba; no se ha integrado en el anfitrión real ni se ha ejecutado en placa ni en
vuelo. No se propone ninguna campaña nueva (calibración, seis posiciones, Allan): sólo lo que sale
del arranque, de la espera en rampa y de los registros que el datalogger guardará de todas formas.

## Comprobaciones pendientes en placa (lista para Aviónica)

Preparación: registro ICM, BMP y ESKF; datalogger suscrito a ICM | BMP | ESKF con el preajuste
REGISTRO; en banco, `trace_enable = 1`, la traza vaciada con `SPP_SERVICES_ESKF_readTrace` desde
la tarea del router (~10 kB/s) y, al terminar la captura, `SPP_SERVICES_ESKF_closeTrace()` (registro
END; si devuelve error, vaciar y repetir). Anotar commits (anfitrión, SPP, ESKF), `sdkconfig`,
`config_id` y `boot_id`.

| # | qué | cómo se ve | referencia propuesta |
|---|---|---|---|
| 1 | arranque | `getInitResult()`, `getRegistration()`, `getInitCalls()`, sesión, KPID de salida | `K_SPP_OK`, `REG_BOTH`, 3, sesión 1, KPID asignado; 10 arranques sin reinicio del ESP32 |
| 2 | adquisición y SD simultáneas | en la SD: secuencias de ICM, BMP y ESKF; `spp_decode.py` sobre el registro | sin saltos ni repetidos; IMU ≈ 225 Hz, BMP ≈ 100 Hz; intervalos IMU ≤ 10 ms salvo pérdidas explicadas |
| 3 | parones de la SD | histograma de intervalos de la IMU y del BMP (`spp_decode.py`) | ningún intervalo de IMU > 10 ms: si los hay (como los 34 ms de la captura histórica), la rampa no alinea y sólo queda el respaldo; NINGÚN hueco simultáneo de IMU y BMP > 0,5 s en rampa (dejaría la fase INCIERTA) |
| 4 | carga del router | `SPP_SERVICES_PUBSUB_queueDepth()` máxima, `mailbox_hwm`, `queue_busy`, `pool_empty` | cola < 64; `mailbox_hwm` < 16; aplazamientos sólo con la cola alta; `callConsumers` ≥ 352 Hz sostenidos |
| 5 | tiempos | duración de `consumeData` (máx. y p99), `lat_rx_max_us`, `lat_proc_max_us` | p99 < 1 ms; latencias de pocos ms |
| 6 | pila | marca de agua de la pila de la tarea del router | > 1 kB libres con la cadena medida de ≈ 3,1 kB del servicio |
| 7 | continuidad del registro | `diag` del ESKF en la SD: `imu_lost`, `seq_resyncs`, `out_fail`, `skipped` | 0 pérdidas no explicadas; `snapshots_skipped` < 1 % |
| 8 | rampa | validez, capacidades, fuente y fase publicadas | fase RAMPA (0) toda la espera; alinea a los 5 s de IMU quieta; `NO_ALT_REF` hasta la siguiente ventana con baro; después VÁLIDO, `ESKF_NAV` y `BARO_BACKUP`, fuente ESKF; respaldo con |h| < 0,5 m |
| 8b | arranque de los productores | instante del primer paquete de ICM y BMP tras `SPP_CORE_init` | ambos antes de `svc.pad_start_s` = 10 s; si no, pedir una sesión en rampa cuando ya publiquen |
| 8c | latencia y parones del router | `lat_rx_max_us`, `lat_proc_max_us` y el parón más largo de la tarea del router con todos los servicios | parones < `svc.pad_lat_max_s` = 0,5 s y latencia máxima < 0,5 s; `consumeData` se sigue llamando sin datos (la caducidad por el reloj depende de ello); fase 0 toda la espera |
| 9 | signo (con los datos de rampa) | aceleración media en reposo | el eje positivo que mira hacia ARRIBA mide ≈ +1 g (fuerza específica); con R = identidad, `q` lleva ese eje hacia −z NED |
| 10 | reproducción | `svc_replay --reproducir traza.bin --mensajes pc.bin entradas.bin` y `compare_runs.py` | traza v5 cerrada; código 0 de `svc_replay` («reproducción COMPLETA»); «EQUIVALENTE (dentro de tolerancia)» |
| 11 | vuelo reconocido o fase incierta en banco | registro de banco en reposo: `vuelo=0`, fase RAMPA con el ESKF alineado, respaldo con referencia ≈ 1,2 s después del primer baro | sin falsos reconocimientos de vuelo ni fase INCIERTA en rampa; `flight_detector` 0; `pad_lost_hole_us` 0 |
| 12 | decodificación en tierra | `nav_out_decode.py` sobre la salida de la SD | «captura comprobada: N mensajes» (código 0); código 2 = nada que comprobar (KPID equivocado o archivo vacío); si hay corrupción, `--tolerante` y anotar lo descartado |
| 13 | prolongado (opcional) | 30 min–2 h quieta con todos los servicios | 0 reinicios, validez estable, ninguna cola al límite, `vuelo=0` |

## Tres comparaciones distintas

1. **Determinista en el PC**: `svc_replay --entradas F --salidas G registro.bin` y
   `nav_replay_float < F` → G idéntico (lo hace `make test`).
2. **Comportamiento real del router**: con el registro de entradas (paquetes ICM/BMP en el orden
   del datalogger) y la traza: `svc_replay --reproducir traza.bin --mensajes pc.bin entradas.bin`.
   En el PC (misma aritmética) los paquetes salen idénticos, cabecera incluida (router real con
   congestión: 628 = 628; con un reloj que avanza en cada lectura: 216 = 216). `svc_replay` no da
   éxitos silenciosos: la traza v5 debe terminar en su registro END (recuento, CRC, sin
   desbordamiento) y la traza que regenera la reproducción se contrasta registro a registro con la
   original (tipo y longitud de cada mensaje producido, secuencia y marca de salida, identidad,
   longitud y resultado de cada entrada, consumes, reinicios). Traza sin cierre o con registros que
   faltan → 5; metadatos distintos, entradas sobrantes o agotadas → 4; otra versión o
   configuración → 6. Si la placa se apagó sin cerrar la traza, `--parcial` reproduce lo registrado
   y lo declara PARCIAL (código 3): no es una reproducción completa.
3. **Placa frente a PC (numérica, con tolerancias)**:
   `compare_runs.py --placa salida_eskf_de_la_sd.bin --pc-mensajes pc.bin --out-id 0x..` con el
   `pc.bin` del punto 2 hecho con la traza DE LA PLACA. Identidad determinista (sesión, id) para
   cada snapshot, diagnóstico y compacta; falla si alguna salida queda sin pareja en cualquiera de
   los dos lados o si alguna captura está incompleta (CRC, snapshot sin cerrar o mezclado, salto o
   repetición en la secuencia SPP de la salida).
   **Valores no finitos**: cada componente se valida ANTES de reducir las diferencias (en 52f01e3
   `max()` ocultaba un NaN según su posición: sólo p_y o sólo v_z en NaN daban «EQUIVALENTE»).
   Finito frente a NaN/inf, NaN frente a inf o infinito en cualquier lado → fallo con el campo y el
   índice; nunca se convierte en 0. Dos NaN sólo valen en campos no disponibles según el esquema y
   el estado publicado: fragmento ausente de la máscara, sin estado (`t_state` = INT64_MIN: antes
   de alinear o sin navegación), `alt_ref` sin la capacidad `BARO_REF` (la que describe la
   referencia vigente; alineado y VÁLIDO inercialmente con `NO_ALT_REF`, o sin BMP, es legítimo: en
   a26704a se exigía alt_ref con el núcleo VÁLIDO o `alt_ref_seq` > 0 y dos capturas idénticas de
   14 s daban NO EQUIVALENTE con 25 snapshots rechazados); en la compacta, antigüedad 0xFFFF o
   altura del respaldo sin referencia. Además, cada captura por separado: lo que una salida
   promete (estado publicado, DEGRADADO/VÁLIDO, `ESKF_NAV`, `BARO_REF`, fuente ESKF;
   respaldo válido o congelado) debe ser finito, σ ≥ 0 y el cuaternión finito y de norma 1 ± 1e-3
   (diagnóstico «cuaternión no finito» / «cuaternión de norma …»). Cubre posición, velocidad,
   sesgos, incertidumbres, referencia y actitud; las tolerancias no cambian.
   **Sin traza de la placa no hay comparación completa**: `--parcial --pc salidas.txt` sólo
   compara los snapshots cuyo `t_state` tiene un único estado posible en el PC, deja sin comparar
   los ambiguos (no elige el de menor error), no mira diagnósticos ni compactas y nunca dice
   «equivalente». En la prueba con el vuelo nominal, 518 de 543 son ambiguos: en la práctica hace
   falta la traza.

### Por qué no se exige igualdad binaria placa–PC

Xtensa (newlib) y x86-64 (glibc) no tienen las mismas `sqrtf`, `sinf`, `cosf`, `powf`; aunque el
componente se compila con `-ffp-contract=off` en ambos, otra contracción u otra libm cambian el
redondeo. Los campos discretos (marcas t_state/t_eval/t_proc, motivos, validez, fase,
capacidades, fuente, contadores del diagnóstico) sí deben coincidir exactamente.

### Tolerancias y su justificación

Experimento en el PC (`make test`, sección J): el vuelo nominal entero con la compilación normal y
con `-ffp-contract=fast -mfma` (otra aritmética, como podría pasar en la placa) y otro calendario,
reproducido con su traza. 763 salidas emparejadas (543 snapshots, 110 diagnósticos, 110
compactas); diferencias máximas:

| magnitud | diferencia medida | tolerancia |
|---|---|---|
| posición | 0,38 m (ejes horizontales, no observables, σ de kilómetros); vertical 0,15 mm | 0,01 m + 5 % de σ_p |
| velocidad | 0,017 m/s | 0,01 m/s + 5 % de σ_v |
| actitud | 3,9e-4 rad (q ≡ −q) | 1e-3 rad + 5 % de la mayor σ_θ |
| σ | 0,2 % relativo | 5 % relativo |
| sesgo acel. / giro | 3,9e-4 m/s² / 1,4e-5 rad/s | 0,01 m/s² (≈ 8 LSB) / 1e-4 rad/s (< 1 LSB) |
| compacta: alturas y v_z | | 0,01 m y 0,01 m/s |

Las tolerancias relativas a σ dicen que una diferencia es despreciable frente a la incertidumbre
que publica el propio estimador; los suelos absolutos quedan por debajo del ruido del baro
(0,14 m) y de un LSB del giro. Están justificadas con otra aritmética en el PC, no con resultados
de Xtensa: si la placa sale fuera, el informe JSON da la primera salida y los campos.

## Aceptación

`flight_accept` exige ESTRICTO, `readiness = 0` (todas las filas «confirmar» confirmadas y
`ESKF_SVC_BOARD_VALIDATED = 1`) y validez no NO VÁLIDA: hoy es 0. El perfil EXPERIMENTAL se usa
publicando su validez, motivos, capacidades y `NOT_VALIDATED`; superar estas pruebas de software
no es una validación física ni una aceptación para vuelo.
