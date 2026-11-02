# Guía breve para Aviónica: integrar el servicio ESKF en el anfitrión

Qué es: un servicio SPP (consumidor + productor) que lee los paquetes del ICM-20948 y del BMP390,
estima navegación con el ESKF y publica su salida con un KPID propio. Qué NO hace: no inicia
drivers, buses, interrupciones ni tareas; no escribe en la SD; no toca SPP ni Solaris Software.
Todo lo de esta guía lo hace el anfitrión con la API existente; no hay que modificarlo.

## 1. Compilación

- Añadir `integration/eskf_service` a `EXTRA_COMPONENT_DIRS`. Necesita un componente que exporte
  las cabeceras de SPP con raíz `spp/` (nombre por defecto `spp`; otro con `-DESKF_SPP_COMPONENT=...`).
- `ESKF_USE_FLOAT` en TODO el componente y en quien incluya sus cabeceras (el componente lo exporta;
  `eskf_service.h` da error en ESP-IDF si falta). C11/gnu17 (no C23), sin `-ffast-math`; el
  componente añade `-ffp-contract=off`.
- Comprobado sólo con `idf_test_app` (ESP-IDF v4.4.7, esp32s3): NO con el anfitrión real, cuya
  versión de ESP-IDF y `sdkconfig` no se conocen.

## 2. Registro y configuración (antes de `SPP_CORE_init`)

```c
#include "spp_services_eskf.h"

SPP_Kpid_t icm, bmp, eskf;
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ICM20948_getProducerContract(), &icm);
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_BMP390_getProducerContract(), &bmp);

eskf_svc_config_t cfg;
eskf_svc_default_config(ESKF_SVC_PROFILE_EXP, &cfg);  /* perfil operativo experimental */
eskf_svc_output_preset(&cfg, ESKF_OUT_LOG);           /* registro completo (principal) */
cfg.boot_id = n_arranque;                              /* opcional: identifica el arranque en la salida */
/* opcional, si Aviónica tiene valores mejores (docs/DEPENDENCIAS.md): unidades, montaje r_sb, ruido… */
SPP_SERVICES_ESKF_configure(&cfg);

if (SPP_SERVICES_ESKF_register(icm, bmp, &eskf) != K_SPP_OK) { /* ver abajo */ }
/* datalogger: incluir 'eskf.value' en su suscripción */
SPP_CORE_init();
```

- KPID: los asigna el router por orden de registro (`1 << (n + 2)`); no hay valores fijos. La
  suscripción del ESKF es `icm | bmp`; nunca su propia salida.
- `register` registra primero el consumidor y después el productor. Si no cabe el consumidor no se
  registra nada; si no cabe el productor, el ESKF navega y responde a consultas sin salida SPP
  (`getRegistration() == SPP_SERVICES_ESKF_REG_CONSUMER`). Llamarlo dos veces devuelve
  `K_SPP_ERROR_ALREADY_INITIALIZED`. Comprobar tras `SPP_CORE_init`: `getInitResult() == K_SPP_OK`
  y `getInitCalls() == 3` (init de consumidor y de productor).
- Cualquier valor distinto de la tabla de parámetros se publica (`RDY_OVERRIDE`, otro `config_id`).

## 3. Reloj

Por defecto el servicio usa `SPP_HAL_TIME_getTimeMs()` extendido a 64 bits: la MISMA fuente que
`packetData` usa para `timestampMs` (resolución 1 ms). Si el anfitrión prefiere otro reloj,
`SPP_SERVICES_ESKF_setClock(fn, ctx)` antes de `SPP_CORE_init`: µs, monótono y en la MISMA época
que `timestampMs × 1000` (con el puerto ESP32 de SPP sirve `SPP_SERVICES_ESKF_clockEspTimer`).
Una marca posterior a la recepción en más de 1 ms se descarta como futura.

## 4. Ejecución del router (lo que debe garantizar el anfitrión)

- Los callbacks (`init`, `deliverToMailbox`, `consumeData`, `acquireData`) llegan desde la tarea
  que mueve el router; el servicio no crea tareas.
- `callConsumers` reparte UN paquete por llamada: con ICM 225 Hz + BMP 100 Hz + ESKF 27 paquetes/s
  hay que llamarlo a **≥ 352 Hz sostenidos** (más lo que añadan GPS u otros). Si no, la cola de 128
  se llena, los sensores pierden publicaciones y el ESKF lo ve como pérdidas (DEGRADADO; NO VÁLIDO
  si un hueco de IMU supera 10 ms).
- `callProducers` con regularidad: el ESKF publica como mucho un mensaje por `acquireData` y sólo
  si la cola tiene < 32 paquetes (si no, aplaza).
- Prioridad del consumidor ESKF: 1 (no la interrumpe `signalProducerReady`).
- Pila de la tarea del router: el servicio usa ≈ 3,1 kB en su cadena más profunda (Xtensa `-Os`),
  además de lo del propio anfitrión.
- Memoria estática del servicio ≈ 19,4 kB (.bss); sin `malloc`.

## 5. Consultas y avisos (desde cualquier tarea)

- `SPP_SERVICES_ESKF_getOutput(&o)` / `getDiagnostics(&d)`: copia coherente bajo sección crítica.
- `SPP_SERVICES_ESKF_requestSessionRestart()`: declara que el vehículo está en rampa y abre una
  sesión nueva; se acepta en rampa y en fase INCIERTA, y se rechaza con el vuelo reconocido
  (docs/SERVICIO.md, «Fase»). Llamarlo SÓLO en rampa.
- **Contrato de inicio**: encender la placa (o pedir una sesión) con el vehículo en rampa y no
  lanzar en los 10 s siguientes (`svc.pad_start_s`). Después, la rampa sigue confirmada mientras
  la IMU o el BMP sigan llegando sin huecos > 0,5 s; si no llega nada, caduca por el reloj de
  `consumeData` a los 1,05 s (por eso el router debe seguir llamando a `callConsumers` aunque no
  haya datos, como hace SPP 9247d9e). Un aviso de referencia después de un silencio así deja el baro
  fuera el resto de la sesión, sin abrir otra; si en rampa se publica la fase INCIERTA
  (`phase` 2, why 0x8009), el ESKF no volverá a alinear en esa sesión: pedir una sesión nueva en
  rampa (o reiniciar la placa) y revisar los parones del registro.
- `SPP_SERVICES_ESKF_notifyBaroRefChange()`: llamar SÓLO si el BMP390 cambia su referencia
  (p0/T0), por ejemplo si se repite su calibración de 500 muestras sin reiniciar la placa. El
  servicio no lo deduce de los datos. En rampa: sesión nueva y referencia nueva con datos
  posteriores al aviso. En vuelo: el barómetro deja de usarse (ESKF y respaldo) el resto de la
  sesión. Con el código actual de SPP, p0/T0 sólo cambian al reiniciar, y entonces no hay que avisar.

## 6. Registro en la SD (configuración principal)

Preajuste REGISTRO: 5 snapshots/s (5 fragmentos cada uno), 1 diagnóstico/s y 1 compacta/s = 27
paquetes/s del ESKF; con ICM y BMP, ≈ 22,5 kB/s en total hacia el datalogger. Decodificación en
tierra: `tools/nav_out_decode.py registro.bin --out-id 0x.. --csv estado.csv --ops-csv compacta.csv`
y `tools/spp_decode.py registro.bin --csv carpeta/` para los sensores. `nav_out_decode.py` falla
(código 1) si la captura tiene paquetes con el CRC SPP mal, saltos de secuencia, mensajes con el CRC
interno mal o snapshots incompletos, y dice cuántos de cada; para rescatar lo legible de un
registro dañado, `--tolerante` (código 3, con la lista de lo descartado). Código 2 = nada que
comprobar (archivo vacío, KPID equivocado: dice qué KPID hay, o ningún mensaje decodificable).

## 7. Radio (opcional, de Aviónica)

Un único KPID: el SX1262 no puede quedarse sólo con la compacta mientras el datalogger guarda
todo. Con REGISTRO serían 27 paquetes/s (el enlace SF7/125 kHz no pasa de ~6/s); con TELEMETRÍA,
1,1 paquetes/s, pero el registro se queda sin el estado completo. Ciclo de trabajo de 868 MHz:
docs/SALIDA.md. No bloquea el servicio.

## 8. Banco (opcional)

`cfg.trace_enable = 1` y, desde la tarea del router, vaciar `SPP_SERVICES_ESKF_readTrace(buf, n)` a
un archivo (16 B por registro, ~10 kB/s). Al terminar la captura (antes de cerrar el archivo y con
el registro de entradas también cerrado): `SPP_SERVICES_ESKF_closeTrace()` y vaciar otra vez; si
devuelve `K_SPP_ERROR` (sin sitio), vaciar y repetir. Ese registro END (número de registros y CRC)
es lo que prueba que la traza está entera: sin él `svc_replay --reproducir` se niega (código 5) y
sólo `--parcial` reproduce lo registrado, marcado como PARCIAL. Con la traza y las entradas del
datalogger: `svc_replay --reproducir` y `tools/compare_runs.py` (docs/VALIDACION.md).

## 9. Leer la salida en tiempo real

| campo | significado |
|---|---|
| `svc_validity` | 2 VÁLIDO, 1 DEGRADADO, 0 NO VÁLIDO (estimación ESKF) |
| `caps` | `ATTITUDE`, `BARO_REF`, `ESKF_NAV`, `BARO_BACKUP`, `LAUNCH_REF` disponibles AHORA |
| `source` | 1 ESKF, 2 respaldo barométrico, 0 ninguna |
| `bb.h_m`, `bb.vz_mps`, `bb.age_us` | respaldo: altura y v_z (arriba +) de su última medida aceptada y su antigüedad; `h_m` y `ref_m` son NaN si no hubo referencia de rampa (primer baro ya en vuelo): entonces no es válido ni fuente |
| `phase` | 0 RAMPA confirmada, 1 VUELO (desde que cualquier detector lo reconoce, aunque navigation.c no viera el despegue), 2 INCIERTA (no consta que siga en rampa: sin operaciones de rampa) |
| `why_code` 0x8008 + `NAV_DISABLED` | vuelo reconocido con navigation.c aún en rampa (IMU perdida en el despegue o nunca alineada): el ESKF queda fuera de servicio el resto de la sesión, sin realinear; manda el respaldo |
| `why_code` 0x8009 + `NAV_DISABLED` | fase incierta: el ESKF queda fuera de servicio el resto de la sesión; el respaldo sólo vale con el cero que fijó en rampa |
| `svc_reasons` | por qué (docs/SERVICIO.md); `EXPERIMENTAL` y `NOT_VALIDATED` siempre presentes |
| `readiness`, `flight_accept` | hipótesis sin validar; `flight_accept` es 0: no hay validación física |

Comportamiento ante fallos: docs/FALLOS.md. Comprobaciones pendientes en placa: docs/VALIDACION.md.
