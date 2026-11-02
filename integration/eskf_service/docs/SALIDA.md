# Salida SPP del servicio — esquema v4

Serialización explícita, campo a campo, little-endian (`src/eskf_svc_msg.c`); `nav_output_t` no
cabe en 50 B y su ABI no es un protocolo. Cada mensaje es el payload (≤ 50 B) de un paquete SPP
con el ÚNICO KPID asignado al productor ESKF; la cabecera SPP (secuencia, `timestampMs` =
construcción del paquete) la pone `packetData`. Un decodificador de otra versión la rechaza (byte 0).

Cambios v3 → v4: la fase tiene tres valores, 0 RAMPA confirmada, 1 VUELO, 2 INCIERTA
(docs/SERVICIO.md, «Fase»): bit «vuelo» y bit «incierta» en el byte de fase de los tipos 1 y 7. Un
decodificador v3 leería «rampa» donde ahora es «incierta»: por eso cambia la versión.

Cambios v2 → v3: tipo 1 de 36 a 38 B (capacidades y fuente); tipo 7 nuevo (salida operativa
compacta); banderas EXPERIMENTAL/ESTRICTO en lugar de BANCO/VUELO; en el diagnóstico,
«reinicios del productor» pasa a «resincronizaciones de secuencia».

## Cabecera común (12 B)

| byte | campo |
|---|---|
| 0 | versión del esquema = 3 |
| 1 | tipo: 1–5 fragmentos de snapshot, 6 diagnóstico, 7 compacta |
| 2 | máscara: fragmentos del snapshot (bit tipo−1); 0x20 en el diagnóstico; 0x40 en la compacta |
| 3 | banderas: 0x01 marcas en ms, 0x02 navega, 0x04 EXPERIMENTAL, 0x08 ESTRICTO, 0x10 aceptado para vuelo |
| 4–5 | sesión u16 |
| 6–9 | id u32 (snapshot, diagnóstico o compacta) |
| 10–11 | CRC-16/CCITT (0xFFFF, 0x1021) de los datos de TODOS los fragmentos de la máscara en orden de tipo |

## Snapshot: estado completo (tipos 1–5)

| tipo | B | campos (orden, tipo, unidad) |
|---|---|---|
| 1 ESTADO | 38 | `t_state_us` i64, `t_eval_us` i64, `t_proc_us` i64 (µs, época del reloj; INT64_MIN = no hay); motivos del núcleo u16 (`NAV_R_*`); motivos del servicio u16 (`ESKF_SVC_R_*`); readiness u16 (`ESKF_SVC_RDY_*`); why_code u16; config_id u16; u8 = validez núcleo (b0–3) \| validez servicio (b4–7) (0 NO VÁLIDO, 1 DEGRADADO, 2 VÁLIDO); u8 = fase vuelo (b0), reposo (b1), heading_known (b2), fase incierta (b3), perfil (b4–5), fuente de tiempo (b6–7); capacidades u8 (`ESKF_SVC_CAP_*`); fuente u8 (0 ninguna, 1 ESKF, 2 baro) |
| 2 PV | 32 | p[3] f32 (m, NED local), v[3] f32 (m/s), alt_ref f32 (m), alt_ref_seq u32 |
| 3 ACT | 28 | q[4] f32 (Hamilton w,x,y,z; cuerpo → NED), σθ[3] f32 (rad) |
| 4 SIG | 24 | σp[3] (m), σv[3] (m/s) f32 |
| 5 SESGO | 24 | b_a[3] (m/s²), b_g[3] (rad/s) f32 |

Sin navegación (ADQ o ESTRICTO con pendientes) la máscara es sólo el tipo 1: NO VÁLIDO, motivo y
`why_code`. Antes de alinear, p, v, q y σ son NaN. Un estado congelado conserva su `t_state_us`
(`t_proc_us` y `t_eval_us` avanzan; la caducidad aparece como `NAV_R_STALE`).

## Diagnóstico (tipo 6, 38 B, independiente)

imu_rx u32, baro_rx u32, IMU perdidas u32, baro perdidos u32, rechazados u16
(APID/versión/CRC/longitud), candidatos a saturación u16, último retorno de nav_on_imu /
nav_on_baro / nav_tick i16 ×3, huecos del núcleo u16, fallos de salida u16 (publish, pool vacío,
KPID), omitidos u16 (snapshots, diagnósticos y compactas sustituidos), ocupación máxima del buzón
u8, resincronizaciones de secuencia u8, latencia máx. recepción−construcción ms u16, latencia máx.
procesamiento−recepción ms u16. Acumulado por sesión.

**Pérdidas contadas una sola vez**: IMU perdidas = saltos de secuencia + longitud incorrecta + no
finitas + fuera de dominio + tardías. Los descartes del buzón, los CRC rechazados y las marcas
futuras ya aparecen como saltos de secuencia y NO se suman otra vez; los repetidos no son
pérdidas. (En 02fe8e4, 6 pérdidas reales se publicaban como 12.)

## Salida operativa compacta (tipo 7, 12 + 38 = 50 B, autónoma)

Pensada para telemetría: se interpreta sola, sin snapshot ni diagnóstico.

| bytes de datos | campo |
|---|---|
| 0–3 | `t_ms` u32: reloj del servicio al construirla (ms) |
| 4–5 | `eskf_age_ms` u16: `t_ms` − instante del estado ESKF (0xFFFF = no hay) |
| 6–7 | `bb_age_ms` u16: `t_ms` − último baro del respaldo (0xFFFF = no hay) |
| 8–17 | motivos del núcleo, motivos del servicio, readiness, config_id, boot_id (u16 ×5) |
| 18 | validez núcleo (b0–3) \| validez servicio (b4–7) |
| 19 | capacidades |
| 20 | fuente (b0–1) \| fase vuelo (b2) \| fase incierta (b3) \| perfil (b4–5) \| respaldo válido (b6) \| referencia de lanzamiento congelada (b7) |
| 21 | 0 |
| 22–37 | f32: altura ESKF (−p_z, m), v_z ESKF (−v_z, m/s), altura respaldo (m), v_z respaldo (m/s) |

Todas las alturas y velocidades verticales son POSITIVAS HACIA ARRIBA. La del ESKF es relativa a
su referencia de alineación; la del respaldo, a la referencia de lanzamiento (congelada al
despegar) y corresponde a su última medida aceptada (`bb_age_ms` dice cuánto hace). NaN si no
hay. La compacta se publica siempre con su validez real, también cuando no queda ninguna fuente
válida (fuente 0).

## Capacidades, fuente y validez (no son validación física)

| bit | capacidad | cuándo |
|---|---|---|
| 0x01 | ATTITUDE | inercial alineado y validez del servicio no NO VÁLIDA |
| 0x02 | BARO_REF | navigation.c tiene `alt_ref` (fijada en una ventana de reposo, no con un BMP suelto) |
| 0x04 | ESKF_NAV | ATTITUDE y BARO_REF |
| 0x08 | BARO_BACKUP | respaldo barométrico siguiendo medidas, con referencia y última aceptada a ≤ 0,5 s |
| 0x10 | LAUNCH_REF | referencia de lanzamiento del respaldo congelada |

Fuente: ESKF si ESKF_NAV; si no, BARO si BARO_BACKUP; si no, ninguna. Perder una capacidad no
borra los diagnósticos ni bloquea las demás salidas. Nunca se publica VÁLIDO sin referencia
barométrica: alineado sin `alt_ref` es DEGRADADO con `NO_ALT_REF`.

## Respaldo barométrico

Filtro alfa-beta de ganancias fijas (α = 0,25, β = 0,02 a ~100 Hz) sobre la altura PUBLICADA por
el BMP: altura y velocidad vertical (positiva hacia arriba), con su instante, antigüedad, validez
y estado (`ESKF_SVC_BB_NONE`, `_RECOVER`, `_TRACK`). No reconstruye p0 ni corrige nada del productor.

- **Estado y frescura**: el estado (h, v, `t_us`) es SIEMPRE el de la última medida ACEPTADA. Una
  medida rechazada (residuo > `bb.gate_m` = 10 m frente a la predicción) no propaga el estado ni
  renueva `t_us`: la antigüedad publicada (`age_us`, `bb_age_ms`) sigue creciendo y el respaldo
  caduca a los `bb.timeout_s` = 0,5 s sin medidas aceptadas. (En 1597c47 un rechazo renovaba la
  marca: tras 19 rechazos seguía válido con 410 ms cuando la última aceptada tenía 600 ms.)
- **Caducado = ventana nueva**: en cuanto llega una medida con la última aceptada a más de
  `bb.timeout_s` (antigüedad > 0,5 s; exactamente 0,5 s todavía vale), el respaldo pasa a
  `RECOVER` (`bb_expired`) y sólo vuelve a ser válido con una ventana coherente nueva. Una sola
  medida compatible con la predicción ya no lo revalida (en a26704a, 19 rechazos cada 60 ms y una
  medida de 0 m devolvían TRACK y válido con 1140 ms sin aceptadas). Los rechazos breves que no
  llegan a la caducidad siguen como antes (prueba 30: 19 rechazos en 380 ms; intervalos de 10 a
  99 ms).
- **Inicio y recuperación con ventana coherente**: hacen falta `bb.min_samples` = 20 medidas
  consecutivas (intervalos ≤ `bb.dt_max_s` = 0,1 s) cuyo ajuste lineal deja todos los residuos
  ≤ `bb.coh_m` = 1 m (provisional: ≈ 7σ del ruido observado más la curvatura de 120 m/s² en
  0,2 s). Altura y velocidad salen del ajuste. La ventana se desliza: una lectura anómala nunca
  fija la altura ni la referencia; sólo retrasa la ventana coherente. Mientras se reúne:
  `RECOVER`, NO válido.
- **Paso a recuperación**: tras un corte (> 0,1 s entre medidas RECIBIDAS), tras la caducidad
  (arriba) o tras `bb.reseed_n` = 20 rechazos seguidos. Veinte rechazos no «recuperan» nada: sólo abren la
  ventana, que exige medidas coherentes. Una ráfaga anómala de menos de 20 muestras se descarta y
  el respaldo sigue con su última aceptada (y su antigüedad real). Si un desplazamiento coherente
  dura más que 20 rechazos + una ventana (≈ 0,4 s), la ventana lo toma como el nuevo nivel: por eso
  un cambio de referencia del productor debe AVISARSE.
- **Medidas frente a referencia de lanzamiento**: tener medidas coherentes (estado `TRACK`, `bb.vz_mps`
  y su antigüedad) NO es tener referencia. La referencia de lanzamiento sólo se fija **en rampa
  CONFIRMADA** (docs/SERVICIO.md, «Fase»: contrato de inicio y evidencia continua; no basta que no
  se haya detectado el despegue) y con reposo continuo del respaldo
  (|v| ≤ 1 m/s) durante `bb.hold_s` = 1 s; su valor es la altura filtrada en ese instante (no una
  lectura suelta ni la ventana). Así no se fija con datos ya en vuelo: ni en el ascenso, ni en el
  descenso, ni en el paso por el apogeo (allí |v| ≤ 1 m/s dura ≈ 0,2 s), ni en un descenso lento
  (0,8 m/s cumple |v| ≤ 1 m/s, pero si el primer baro llega en vuelo la rampa ya no consta y en
  a26704a fijaba un cero en el descenso: −3,5 m válido a 140,4 m reales). En el caso normal (baro
  desde la rampa) la referencia aparece ≈ 1,2 s después del primer baro. No se usa ninguna
  referencia externa: el contrato del BMP390 sólo garantiza una altura relativa a su p0, no que
  ese p0 sea la rampa.
- **Sin referencia de rampa** (primer baro ya en vuelo): el respaldo sigue y registra las medidas
  (estado, `vz_mps`, antigüedad) pero `ref_m` y `h_m` son NaN, no es válido, no aporta
  `BARO_BACKUP` ni es fuente: una altura sobre un lanzamiento desconocido no se presenta como
  válida. La velocidad vertical sí sirve para reconocer el vuelo.
- **Deriva y congelación**: en rampa la referencia sigue la deriva lenta (media exponencial,
  τ = 5 s, sólo con medidas aceptadas y |v| ≤ 1 m/s); una recuperación en rampa reinicia altura y
  velocidad pero conserva la referencia (mismo marco del productor). Se CONGELA al reconocerse el
  vuelo o al quedar la fase INCIERTA. **Congelada no cambia nunca en la sesión**: recuperar el respaldo en vuelo no mueve el
  cero (prueba 24: bit a bit tras un corte de 0,6 s y una ráfaga de rechazos).
- **Aviso de cambio de referencia en vuelo**: el respaldo deja de recibir medidas y queda NO válido
  el resto de la sesión (no se reconstruye otra referencia). En rampa el aviso abre sesión nueva y
  el respaldo arranca de cero con datos POSTERIORES al aviso.
- Sin posición horizontal ni actitud: no las tiene.
- En los escenarios simulados (vuelo entero) el error de altura es ≤ 1,3 m y el retraso de v_z en
  la combustión llega a ~15 m/s (el alfa-beta no conoce la aceleración); en planeo < 3 m/s.

## Integridad

Un fragmento de otra (sesión, id) antes de completar la máscara: «incompleto» y se empieza de
nuevo. Misma (sesión, id) con CRC, banderas o máscara distintas, o fragmento repetido: «mezclado»
y se descarta. CRC final distinto: «CRC». Diagnóstico y compacta son independientes y no
interrumpen un snapshot en curso. Si un snapshot aún no ha salido entero cuando toca el siguiente,
el nuevo se omite (`snapshots_skipped`); un diagnóstico o una compacta pendientes se sustituyen
por la más reciente. Decodificadores: `eskf_msg_reasm_push_ops` (C) y `tools/nav_out_decode.py`
(Python, independiente; `--csv`, `--diag-csv`, `--ops-csv`).

`nav_out_decode.py` separa las dos capas y no oculta nada:

- **Exterior (lector SPP)**: paquetes rechazados por CRC SPP (y cuántos llevaban el KPID de
  salida), por versión, longitud imposible, resincronizaciones, ceros intermedios, final
  truncado; huecos y repeticiones en la secuencia SPP de la salida. Un paquete con el CRC SPP mal
  se descarta y se CUENTA (en 52f01e3 desaparecía: 14 compactas de 15, «CRC mal 0» y éxito).
- **Interior (mensajes)**: CRC interno de cada mensaje, snapshots incompletos (también el que
  queda sin cerrar al final), mezclados e inválidos.

Por defecto es una comprobación: código 1 ante cualquier corrupción o captura incompleta (escribe
igualmente los CSV de lo decodificado). `--tolerante` es el modo de RESCATE: escribe lo
decodificable, enumera cada descarte y devuelve 3 si hubo alguno (0 sólo con una captura limpia).
**Nada que comprobar no es un éxito**: código 2 y «SIN DATOS COMPROBABLES» si el archivo no tiene
paquetes (vacío o sólo relleno), si el KPID pedido no aparece (dice qué KPID hay) o si sus paquetes
no dan ningún mensaje (si además hay corrupción, manda el código 1; en el modo tolerante, nada
rescatado también es 2). Código 0 sólo con al menos un mensaje comprobado («captura comprobada: N
mensajes»). En a26704a un archivo vacío o una captura de 395 paquetes con un KPID inexistente
daban 0 y «captura completa y sin corrupción».

Prioridad de producción (un mensaje por `acquireData`): compacta, fragmentos del snapshot,
diagnóstico.

## Un solo KPID: registro prioritario, radio opcional (decisión de esta entrega)

El router da UN KPID por productor y la suscripción es por KPID: un consumidor no puede elegir
tipos de mensaje, y una máscara parcial rompería los snapshots. **No se puede enviar sólo la
compacta por radio y a la vez registrar todos los fragmentos** sin un filtrado en el anfitrión
que hoy no existe; no se modifica Solaris Software para conseguirlo.

Configuración principal: preajuste **REGISTRO** (`ESKF_OUT_LOG`, el de la tabla) y el datalogger
suscrito al KPID del ESKF. La compacta va incluida (1 Hz) en ese mismo registro. La integración
de radio y su frecuencia corresponden a Aviónica y no bloquean el cierre del servicio.

| preajuste | snapshot | diagnóstico | compacta | paquetes ESKF/s | router total (ICM 225 + BMP 100) |
|---|---|---|---|---|---|
| `ESKF_OUT_LOG` (principal) | 5 Hz × 5 = 25/s | 1 Hz | 1 Hz | 27 | 352 paq/s ≈ 22,5 kB/s (medido en `test_spp_router orden_a`) |
| `ESKF_OUT_TELEMETRY` (alternativa) | no | 0,1 Hz | 1 Hz | 1,1 | 326 paq/s ≈ 20,9 kB/s |

Hechos del anfitrión (SPP 9247d9e, leídos, no modificados):

- `callConsumers` reparte UN paquete por llamada: el anfitrión debe llamarlo a ≥ 352 Hz sostenidos
  (más GPS u otros) o la cola de 128 se llena y los sensores pierden publicaciones (escenario
  `congestion`). El productor ESKF no publica si la cola tiene ≥ 32 paquetes (aplaza).
- Datalogger: prioridad 0, sectores de 512 B (8 paquetes).
- SX1262: prioridad 3 (interrumpible), buzón de 100, como mucho 50 paquetes por llamada
  (descarta los más antiguos), SF7 / 125 kHz / CR 4/5 a 868 MHz: ≈ 159 ms en el aire por paquete
  de 64 B (≈ 6 paquetes/s como máximo físico).

Para Aviónica, si decide usar radio: suscribir el SX1262 al KPID del ESKF con REGISTRO le daría
27 paquetes/s (no caben); con TELEMETRÍA serían 1,1 paquetes/s, pero entonces el registro sólo
guardaría compactas y diagnósticos (el estado completo sigue en `getOutput`). En 868,0–868,6 MHz
(ERC/REC 70-03, banda h1.4, a confirmar) el ciclo de trabajo es ≤ 1 % por hora: a 1 Hz, ~3,8 min de
emisión por hora. El periodo de la compacta es `svc.ops_period_us` (≥ 100 ms; 0 la desactiva).

## Reproducción

Tres cosas distintas, que no se deben confundir:

1. **Reproducción determinista en el PC** (`svc_replay registro.bin`): calendario propio (reloj =
   marca + retardo, un consume cada K paquetes, ≤ 1 mensaje por consume). `--entradas F --salidas G`:
   F son exactamente las llamadas a navigation.c y `nav_replay_float < F` da G línea a línea.
2. **Reproducción del comportamiento REAL del router** (`svc_replay --reproducir TRAZA`). La traza
   (**versión 5**, `ESKF_SVC_TRACE_VERSION`) empieza con la configuración exacta (versión de traza,
   config_id, perfil y versión de esquema, boot_id y el **reloj del inicio**, del que depende el
   contrato de rampa), registra cada **transición de fase** (PHASE: a INCIERTA por hueco entre
   muestras o por silencio en el reloj, o a VUELO con su detector; la caducidad por silencio ocurre
   en un consume sin datos, que así queda registrado y se reproduce en el mismo instante), entregas (KPID, secuencia, longitud,
   resultado, recepción), consumes (paquetes, llamadas a navigation.c, mensajes construidos),
   reinicios (aceptados o rechazados), avisos de referencia y el resultado de cada publicación
   (publicado, perdido, aplazado por cola o pool) con el tipo y la longitud del mensaje y la
   **cabecera real del paquete** (secuencia y `timestampMs` que escribió `packetData`, leídos antes
   de `publish`), y **termina con un registro END** (`SPP_SERVICES_ESKF_closeTrace` /
   `eskf_svc_trace_close`): número de registros anteriores, su CRC-16 y los perdidos por
   desbordamiento. Así se rehacen los paquetes COMPLETOS (216/216 idénticos con el router real y un
   reloj que avanza en cada lectura) y se sabe si la captura está entera.
   - **Cierre**: sin END, con registros que faltan o sobran respecto al END, o con desbordamiento
     declarado → código 5 (en 52f01e3, quitar el último registro completo daba éxito con un
     paquete menos); END en mitad o CRC distinto → código 4; traza v1–v4 → código 6 con el motivo.
   - **Contraste de metadatos**: el servicio reproducido vuelve a generar su traza y se compara
     registro a registro con la original: tipo y longitud del mensaje realmente producido,
     secuencia de salida y `timestampMs`, identidad, longitud y resultado de cada entrada,
     paquetes/llamadas/mensajes de cada consume, reinicios aceptados o rechazados y, al final, el
     END regenerado. Cualquier diferencia → código 4 con el registro, el byte y ambos valores (en
     52f01e3 un OUTPUT con tipo 99 y longitud 1 pasaba).
   - **Parcial, sólo si se pide**: `--parcial` reproduce una traza SIN cierre (p. ej. la placa se
     apagó antes de cerrarla) hasta su último registro completo, con el mismo contraste, y devuelve
     3 diciendo «REPRODUCCIÓN PARCIAL (no completa)», cuántos registros y entradas cubre y cuántas
     entradas quedan sin reproducir. Una traza cerrada se comprueba entera aunque se pida
     `--parcial`.
   - Entradas truncadas → 5; entrega distinta, entradas agotadas o sobrantes, mensaje inexistente
     o secuencia distinta → 4; configuración distinta → 6.
3. **Comparación numérica placa–PC** (`tools/compare_runs.py`, docs/VALIDACION.md): completa sólo
   con la traza de la placa (identidad por sesión e id; ninguna salida sin pareja); sin traza, sólo
   un modo PARCIAL pedido explícitamente.

Carga de la traza: ~620 registros/s × 16 B ≈ 9,9 kB/s con un consume por paquete: para banco.
VNLOG (firmware Arduino v4) y SPP son formatos distintos: sus herramientas no se mezclan.
