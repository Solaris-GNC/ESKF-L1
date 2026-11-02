# Servicio: arquitectura, contrato SPP y políticas

Referencia: Solaris Packet Protocol, commit `9247d9e8df75ec62cd42fa5c42f5d0f660479a67`
(versión 2.0.0 en `core/version.h`), leído en su código. SPP y Solaris Software son de sólo
lectura: no se han modificado, parcheado ni copiado dentro del ESKF, y no se duplican sus drivers.

## Piezas

```
include/spp_services_eskf.h  src/spp_services_eskf.c   pegamento con los contratos REALES de SPP
include/eskf_service.h       src/eskf_service.c         núcleo del servicio (sin SPP; C99)
include/eskf_svc_msg.h       src/eskf_svc_msg.c         esquema de salida v4: codificador y decodificador
include/eskf_svc_params.h    src/eskf_svc_params.c      tabla única de parámetros (valor, unidad, procedencia, estado)
../../c/eskf.c navigation.c nav_events.c                núcleo matemático, una sola instancia, SIN CAMBIOS
```

El servicio no inicia drivers, buses, interrupciones, tareas, adquisición ni escritores de SD.

## Hechos del router que se respetan (core/pubsub/pubsub.c)

| hecho | consecuencia en el servicio |
|---|---|
| `registerProducer(contrato, &kpid)`: KPID = `1 << (n + 2)` por orden de registro; 0x0001 (log) y 0x0002 (FSM) reservados | `register` recibe los KPID de entrada y obtiene el de salida; se rechazan bits reservados, varios bits, repetidos y salida dentro de la suscripción |
| `registerConsumer(contrato, suscripción)`; entrega si `suscripción & apid ≠ 0` | suscripción = `kpid_imu \| kpid_baro`, nunca la salida propia |
| máximo 5 productores y 5 consumidores | el ESKF ocupa 1 + 1 (ver registro parcial) |
| `init` de consumidores y después de productores | dos funciones `init` distintas (`getInitCalls()` dice cuáles llegaron); la segunda no toca configuración, colas ni sesión |
| `deliverToMailbox(const SPP_Packet_t packet)` por valor, un paquete por `callConsumers` | se copia a 64 B y al buzón propio; no se guarda ningún puntero |
| `consumeData(void *)` con `NULL` | se ignora el argumento |
| `acquireData(SPP_Kpid_t)` con el KPID asignado | se publica sólo si coincide con `kpid_out` (si no, `kpid_mismatch`) |
| `publish` copia y devuelve el paquete al pool SIEMPRE | no se devuelve dos veces; cada resultado se anota (`eskf_svc_note_output`) |
| cola global de 128; pool de 50 | ≤ 1 mensaje por `acquireData` y sólo si `queueDepth() < 32`; si no, se aplaza |
| prioridad ascendente; `priority > 1U` se interrumpe con `signalProducerReady` | prioridad 1 (`_Static_assert` 0..1) |
| `init`/`acquireData` declarados con `()` | firmas comprobadas con `__builtin_types_compatible_p`; compilar en C11/gnu17, no C23 |
| `packetData`: relleno 0xFF, `dropCounter = 0`, CRC sobre 62 B, `timestampMs` = construcción | el CRC se verifica sobre los 62 B (relleno incluido) |

## Registro en el anfitrión

```c
SPP_Kpid_t icm, bmp, eskf;
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ICM20948_getProducerContract(), &icm);
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_BMP390_getProducerContract(), &bmp);
eskf_svc_config_t cfg;
eskf_svc_default_config(ESKF_SVC_PROFILE_EXP, &cfg);   /* perfil operativo experimental */
eskf_svc_output_preset(&cfg, ESKF_OUT_LOG);            /* registro completo en SD (principal) */
cfg.boot_id = ...;                                     /* opcional: identificador de arranque */
SPP_SERVICES_ESKF_configure(&cfg);                     /* antes de SPP_CORE_init */
SPP_SERVICES_ESKF_register(icm, bmp, &eskf);           /* consumidor y después productor */
/* datalogger suscrito a 'eskf'; la radio es opcional y de Aviónica (docs/SALIDA.md) */
SPP_CORE_init();
```

- **Orden del registro**: primero el consumidor (con `imu | baro`; la salida aún no tiene bit) y
  después el productor. Si no cabe el consumidor, no se registra nada y se devuelve el error del
  router. Si no cabe el productor, queda **sólo el consumidor**: navega y responde a consultas,
  sin salida SPP (`kpid_out = 0`), `getRegistration() == REG_CONSUMER`, y se devuelve el error
  (escenarios `registro_parcial`, `consumidores_llenos`).
- **Registro repetido**: `K_SPP_ERROR_ALREADY_INITIALIZED` sin tocar el router.
- Registro manual (ESKF productor antes que los sensores): `setKpids` y
  `registerConsumer(getConsumerContract(), getSubscription())` (escenario `orden_b`).
- Aviónica puede sustituir cualquier valor de la tabla en `cfg` antes de `configure`, sin tocar
  el algoritmo: se publica `RDY_OVERRIDE` y cambia `config_id`.

## Hilos y consultas

Todos los callbacks llegan desde la tarea del router; el servicio no añade tareas. `getOutput`,
`getDiagnostics`, `requestSessionRestart` y `notifyBaroRefChange` pueden llamarse desde otra
tarea. Las consultas copian la salida (256 B) y los diagnósticos (336 B) completos dentro de una
sección crítica breve (`portMUX`/`portENTER_CRITICAL` en ESP-IDF; cerrojo atómico en el PC): el
filtro NUNCA se ejecuta dentro, sólo `memcpy`. Sustituye al seqlock de 02fe8e4, que era una
carrera de datos (lecturas no atómicas de una estructura que otra tarea escribía). La prueba 21
lee ~200 000 veces mientras otra tarea navega: 0 copias incoherentes. Desde `init` la copia ya
contiene una salida coherente (sesión 1, NO VÁLIDO, NaN): una consulta anterior al primer
`consumeData` no devuelve una estructura a cero. `readTrace` sólo desde la tarea del router.

## Trabajo acotado por llamada

| callback | trabajo |
|---|---|
| `deliverToMailbox` | validación (versión, CRC, APID, longitud) y copia O(1) al buzón de 64 |
| `consumeData` | reinicio / aviso de referencia pedidos → ≤ 16 paquetes del buzón (marca futura, flujo, decodificación, respaldo barométrico) → `nav_tick(reloj)` SIEMPRE → ≤ 24 llamadas a navigation.c en orden temporal → ≤ 1 snapshot, ≤ 1 diagnóstico y ≤ 1 compacta → copia para consultas |
| `acquireData` | ≤ 1 mensaje (≤ 50 B) si hay pendiente, pool libre y cola global < 32; resultado anotado |

| cola | capacidad | al llenarse |
|---|---|---|
| buzón del servicio | 64 paquetes | se descarta el nuevo (`mailbox_full_*`); la pérdida se cuenta UNA vez, como salto de secuencia |
| cola de orden (`nav_merge`) | 64 IMU / 16 baro | contrapresión: el paquete se queda en el buzón (`merge_full_waits`) |
| cola global del router | 128 | el SENSOR pierde la publicación; el servicio lo ve como salto de secuencia |
| ventana baro de reposo | `NAV_REF_MAX` = 512 | 5 s a 100 Hz = 500; las sobrantes se cuentan (`ref_overflow`) |

## Perfiles

| perfil | navega | uso |
|---|---|---|
| ADQ | no | transporte, diagnóstico y respaldo barométrico; motivo publicado |
| EXPERIMENTAL | sí | **operativo para el 7-nov**: todos los valores numéricos y coherentes, cada uno con su procedencia (código, observado, simulado, propuesto, provisional). Publica su validez dinámica. Nada de ello es validación física |
| ESTRICTO | sólo sin pendientes | hoy no navega (primer pendiente `in.gyro_unit_si`): sin continuación en saturación, sin valores provisionales |

`ESKF_SVC_PROFILE_BENCH/FLIGHT` quedan como alias de EXP/STRICT por compatibilidad de código.
En todos: `ESKF_USE_FLOAT`, `gap_max = 0`, `innov_gate = 5`, sin magnetómetro ni GPS, sin
continuación del giróscopo; `eskf_svc_check_nav` se niega a navegar si alguno cambia, o si hay
continuación del acelerómetro sin ventana del motor.

## Tres cosas separadas: funcionamiento, calidad de entrada, validación física

- **Funcionamiento del estimador** (`nav_validity`, `svc_validity`, capacidades, fuente): dinámico,
  según los datos y la salud del filtro. VÁLIDO / DEGRADADO / NO VÁLIDO.
- **Calidad de la entrada** (motivos del servicio y diagnósticos): pérdidas, atrasos,
  resincronizaciones, saturación candidata, baro descartado, DMP inválido.
- **Validación física** (`readiness`): grupos de hipótesis sin confirmar (unidades, ejes, rango,
  tiempo, baro, ruido, umbrales, despegue, anfitrión), perfil no ESTRICTO, configuración distinta
  de la tabla (`RDY_OVERRIDE`) y **sin validación en placa** (`RDY_BOARD`, siempre: no se ha
  ejecutado en placa). Se calcula con la configuración EXACTA en uso. Una solución VÁLIDA no
  borra ninguno de estos bits, y `ESKF_SVC_R_NOT_VALIDATED`/`EXPERIMENTAL` van en los motivos.
- `flight_accept = ESTRICTO ∧ readiness = 0 ∧ svc_validity ≠ NO VÁLIDO`: hoy es siempre 0 y no es
  una afirmación de que el estimador funcione.

| motivo del servicio | efecto |
|---|---|
| `NAV_DISABLED`, `SAT_OUT_OF_MODEL` | NO VÁLIDO |
| `IMU_LOSS`, `LATE_EVENT`, `SEQ_RESYNC`, `NO_ALT_REF`, `BARO_REF_CHANGED` | DEGRADADO |
| `BARO_LOSS`, `NO_BARO`, `SAT_CANDIDATE`, `SAT_UNKNOWN`, `TIME_APPROX`, `DMP_INVALID`, `NOT_VALIDATED`, `EXPERIMENTAL`, `RESTART_REFUSED` | informativo |

Además manda la validez del núcleo (`NAV_R_*`: `TIME_GAP`, `STALE`, `ACC_SAT_LIMIT`… NO VÁLIDO;
`ACC_SAT`, `BARO_TIMEOUT`… DEGRADADO).

`heading_known = 0`: sin magnetómetro ni GPS, acimut arbitrario; IMU + barómetro no dan
posición horizontal absoluta: `p_x, p_y` son integración inercial relativa.

## Comportamiento ante fallos

Resumen en docs/FALLOS.md. Políticas de tiempo, secuencia, pérdidas de IMU y baro en
docs/CONTRATO_ENTRADA.md; respaldo barométrico (frescura, ventana coherente, recuperación,
referencia de lanzamiento) en docs/SALIDA.md. Guía de integración: docs/GUIA_AVIONICA.md.

## Fase: rampa CONFIRMADA, vuelo o INCIERTA

La fase publicada (`phase` en la salida, en el snapshot y en la compacta, esquema v4) tiene tres
valores. Sólo la rampa CONFIRMADA autoriza operaciones de rampa; la ausencia de despegue
detectado no la acredita.

| fase | cuándo | qué autoriza |
|---|---|---|
| RAMPA confirmada (0) | contrato de inicio y, después, evidencia continua (abajo) | alineación inicial, realineación, ZUPT y referencia de navigation.c; referencia de lanzamiento del respaldo y su deriva; borrar pérdidas enclavadas; reinicio; aviso de referencia → sesión nueva |
| VUELO (1) | un detector de vuelo (abajo); enclavada hasta el final de la sesión | nada de rampa; reinicio RECHAZADO |
| INCIERTA (2) | dejó de constar la rampa sin que se reconociera el vuelo; irreversible en la sesión | nada de rampa; reinicio aceptado (el anfitrión declara que está en rampa); aviso de referencia → baro fuera el resto de la sesión, como en vuelo |

**Qué confirma la rampa.**

1. **Contrato de inicio** (operativo, documentado): la sesión empieza con el vehículo en rampa
   (encendido de la placa, `requestSessionRestart()` o aviso de referencia aceptados en rampa) y no
   hay despegue antes de `svc.pad_start_s` = 10 s. Cubre el arranque de los productores (el BMP
   publica tras ~5 s de calibración) y los parones de la SD al arrancar.
2. **Evidencia continua** después: cada muestra que prueba que no ha habido despegue alarga el
   horizonte confirmado:
   - **IMU**: racha de muestras válidas entregadas a navigation.c con intervalos ≤ `eskf.dt_max`
     (10 ms) de al menos `nav.liftoff_hold_s` (0,05 s). Su detector de despegue vería un despegue
     ocurrido en el hueco anterior: el empuje > 2,5 g dura ~1 s (simulado), más que
     `svc.pad_gap_max_s` + 0,05 s. No cuenta si navigation.c no navega (perfil ADQ, ESTRICTO
     pendiente).
   - **Baro**: medida aceptada por el respaldo en seguimiento sin movimiento rápido
     (|v_z| < `bb.flight_vz_mps` = 3 m/s). Un despegue con el baro siguiendo se reconoce como vuelo
     (|v_z| ≥ 3 m/s durante 1 s), y uno ocurrido en un corte deja un hueco.
3. Un hueco entre evidencias mayor que `svc.pad_gap_max_s` = 0,5 s (provisional) deja la fase
   INCIERTA. Los parones de la SD de 34 ms o un hueco de IMU de 0,3 s quedan cubiertos; con el baro
   siguiendo en reposo, cualquier hueco de IMU.
4. **Silencio total (por el reloj)**: el hueco se mide con las marcas de las muestras cuando llegan
   y, si no llega ninguna, con el reloj de `consumeData` (el router lo llama aunque no haya datos).
   Si desde la LLEGADA de la última muestra procesada (o desde el fin del contrato de inicio) pasan
   más de `svc.pad_gap_max_s` + `nav.liftoff_hold_s` + `svc.pad_lat_max_s` = 0,5 + 0,05 + 0,5 =
   1,05 s, la rampa caduca (causa 2, «silencio»). `svc.pad_lat_max_s` es la latencia de entrega
   admitida: la cola del router guarda 128 paquetes ≈ 0,39 s de ICM + BMP, así que tras un parón
   del router más corto sus muestras llegan después con las marcas continuas y no hay transición
   falsa; uno más largo pierde datos y deja además un hueco de marcas. Con latencia constante (cola
   llena pero fluyendo) tampoco hay transición: cuenta la llegada, no la marca. La comprobación se
   hace al EMPEZAR cada consume, antes de atender reinicios y avisos de referencia: un aviso tras el
   silencio ve la fase INCIERTA y deja el baro fuera sin abrir sesión (en 8a9616c, tras 3 s de
   silencio un aviso abría sesión y concedía 10 s más de rampa asumida).

Nada de lo que llegue después confirma otra vez la rampa: una IMU con ~1 g y poco giro no distingue
la rampa de un descenso bajo paracaídas; el baro sólo separaría un descenso lento (0,8 m/s) con
ventanas largas y, aun así, no separa la rampa de un aterrizaje. No se intenta con otro umbral. En fase INCIERTA navigation.c deja de recibir
muestras y reloj (como con el vuelo reconocido en rampa), la referencia de lanzamiento existente
queda congelada y, si no la había, no se fija ninguna: la altura del respaldo es NaN y no es
fuente. Se publica `phase` 2, `NAV_DISABLED` y `why_code` 0x8009; capacidades sólo las que de
verdad hay (respaldo con la referencia de rampa, si la tenía). `pad_lost_hole_us` y
`pad_lost_cause` (1 hueco entre muestras, 2 silencio) dicen el hueco y la causa. Antes de cortar
navigation.c se le da un último `nav_tick` con el reloj de ese consume, y cada transición de fase
(a INCIERTA o a VUELO) queda en la traza como registro PHASE (traza v5): la reproducción es exacta
aunque los consumes vacíos del silencio no se registren.

**Vuelo**, el primer detector válido (`flight_detector`):

| detector | cuándo vale | código |
|---|---|---|
| navigation.c detecta el despegue (fase VUELO) | siempre | 1 |
| el respaldo sube más de `bb.liftoff_m` = 10 m sobre la referencia de lanzamiento | sólo si navigation.c no puede atestiguar el reposo (deshabilitado, sin alinear, NO VÁLIDO o cortado) | 2 |
| el respaldo mantiene \|v_z\| ≥ `bb.flight_vz_mps` = 3 m/s durante `bb.hold_s` = 1 s | idem; no necesita referencia | 3 |

También desde la fase INCIERTA (pasa a VUELO). Con el vuelo reconocido: reinicio rechazado; aviso
de referencia → sin baro el resto de la sesión; ninguna pérdida de IMU se da por recuperada; la
referencia del respaldo congelada (o ninguna); fase VUELO (`at_rest` 0). Si se reconoce con
navigation.c todavía en RAMPA, deja de recibir muestras y reloj (`nav_cut`): ESKF NO VÁLIDO
(`NAV_DISABLED`, `why_code` 0x8008). No se inventa un instante de despegue: `t_liftoff` sólo
existe si lo detectó navigation.c.

**Limitaciones declaradas** (físicamente indistinguibles con IMU y baro):

- Un reinicio de la placa (o una sesión pedida) EN VUELO incumple el contrato de inicio: el
  servicio lo tomaría por rampa. El anfitrión sólo debe pedir sesiones en rampa.
- En rampa, un hueco > 0,5 s en ambos sensores (o en la IMU sin baro siguiendo, o en el baro sin
  IMU navegando), o 1,05 s de silencio total, deja la fase INCIERTA y el ESKF fuera de servicio
  hasta que el anfitrión pida una sesión nueva: se prefiere a autorizar una alineación que podría
  ser en un descenso.
- Ventana residual: durante los primeros 1,05 s de un silencio total la rampa sigue confirmada (no
  se distingue de un parón del router con su cola llena); un aviso de referencia en esa ventana
  todavía abre sesión. Si el router se para más que `svc.pad_lat_max_s` sin perder datos (cola
  mayor que la de SPP 9247d9e), la rampa caducaría sin necesidad: ajustar el parámetro.
- Un desplazamiento del baro de más de 10 m, o de ≥ 3 m/s durante 1 s, con el ESKF sin atestiguar
  el reposo (p. ej. un cambio de referencia del productor sin avisar) se toma como vuelo hasta
  reiniciar la placa.

## Sesiones

Sesión 1 al iniciar. Sesión nueva sólo con `requestSessionRestart` (en rampa confirmada o en fase
INCIERTA: es una declaración del anfitrión de que el vehículo está en rampa) o con el aviso de
cambio de referencia del baro en rampa confirmada. Con el vuelo reconocido el reinicio se rechaza.
Una sesión reinicia navegación, cola de orden, motivos enclavados, respaldo, fase (contrato de
inicio desde su reloj) y diagnósticos (salvo los contadores de reinicio y avisos). Las salidas
llevan la sesión: nunca se mezclan.

**Frontera**: la sesión empieza en el reloj del `consumeData` que la abre (`t_session0_us`). Las
muestras con marca anterior — en el buzón, en la cola de orden o todavía en la cola del router —
se descartan (`session_discarded`) y sólo actualizan el seguimiento de secuencia de su flujo,
así que la primera muestra posterior no cuenta pérdidas ficticias. Así ni el aviso de cambio de
referencia ni un reinicio dejan que datos antiguos inicialicen la sesión nueva (prueba 25: buzón
con 21 baros antiguos, dos avisos seguidos y un reinicio posterior).

## Memoria y pila (medidas)

| recurso | valor | cómo |
|---|---|---|
| `eskf_svc_t` (float) | 19 592 B en .bss (`s_svc` en el ELF de ESP-IDF; 19 616 en el PC); +32 B frente a 8a9616c (caducidad de la rampa por el reloj) | `nm -S` / `sizeof` |
| otras estáticas | `s_cfg`, `s_probe`, `s_ref`: 504 B cada una (+8: `svc.pad_start_s`, `svc.pad_gap_max_s`) | `nm -S` |
| traza de validación | 256 × 16 B incluidos | `ESKF_SVC_TRACE_CAP` |
| `eskf_svc_consume` | 432 B de pila propia (+16 frente a 8a9616c) | `-fstack-usage`, Xtensa `-Os` |
| cadena más profunda | consume 432 → nav_on_imu 288 → … → `predict_step` 2032 (núcleo) ≈ 3,1 kB | idem |
| callbacks SPP | `deliverToMailbox` 96 B, `acquireData` 112 B, `consumeData` 32 B | idem |
| sin `malloc`; sin estructuras grandes en la pila | | revisión de código |

Tiempo de CPU por llamada: NO medido (no hay ejecución en placa). Ver docs/VALIDACION.md.
