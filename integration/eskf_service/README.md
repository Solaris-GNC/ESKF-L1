# eskf_service — Solaris ESKF como servicio de Solaris Software (SPP 9247d9e)

Componente para el anfitrión: servicio DUAL de Solaris Packet Protocol que consume los paquetes
del ICM-20948 (44 B) y del BMP390 (12 B), ejecuta el núcleo canónico de `c/` (eskf, navigation,
nav_events; una sola instancia, sin cambios) y publica, con un único KPID, la navegación (estado
completo), un diagnóstico y una salida operativa compacta de 50 B con el respaldo barométrico.
SPP y Solaris Software no se han modificado ni copiado. El servicio no inicia drivers, buses,
interrupciones, tareas ni escritores.

## Estado (2-nov)

| nivel | estado |
|---|---|
| Software comprobado en PC | sí: servicio 275/275 (float) y 275/275 (double); router SPP REAL 13 escenarios (68/68); herramientas, vuelos simulados enteros, trazas v5, silencio total, comparación con tolerancias y decodificador 160/160 |
| Compilación para ESP-IDF | sí, con la aplicación de prueba `idf_test_app` (v4.4.7, esp32s3, xtensa-esp32s3-elf-gcc 8.4.0, SPP sin cambios): 0 avisos propios |
| Integración con el anfitrión real | **no** (faltan su versión de ESP-IDF y su `sdkconfig`; guía en docs/GUIA_AVIONICA.md) |
| Ejecución en placa | **no** (lista de comprobaciones en docs/VALIDACION.md) |
| Validación física / aceptación para vuelo | **no** (`RDY_BOARD`; `flight_accept = 0`) |

Perfil para el 7-nov: **EXPERIMENTAL** — navega con valores numéricos de procedencia explícita
(código, registro observado, simulación OpenRocket/RocketPy, propuesto, provisional), publica su
validez dinámica, capacidades y fuente, y marca `EXPERIMENTAL` y `NOT_VALIDATED` en cada mensaje.
ESTRICTO no navega mientras quede algo sin confirmar.

## Documentación

| documento | contenido |
|---|---|
| [docs/GUIA_AVIONICA.md](docs/GUIA_AVIONICA.md) | **para Aviónica**: compilación, registro, KPID, reloj, ejecución del router, avisos, registro y radio |
| [docs/SERVICIO.md](docs/SERVICIO.md) | router, registro, hilos, trabajo acotado, perfiles, validez/capacidades/preparación, **fase: rampa confirmada, vuelo o incierta**, sesiones y su frontera, memoria |
| [docs/FALLOS.md](docs/FALLOS.md) | comportamiento ante cada fallo, con la prueba que lo cubre |
| [docs/CONTRATO_ENTRADA.md](docs/CONTRATO_ENTRADA.md) | paquete, IMU, unidades, ejes, saturación, baro, tiempo, secuencia y pérdidas |
| [docs/SALIDA.md](docs/SALIDA.md) | esquema v4 (fase de tres valores), compacta, respaldo barométrico (**referencia de lanzamiento, caducidad**), decodificador, carga y enrutamiento del anfitrión, reproducción y **traza v5** |
| [docs/PARAMETROS_SPP.md](docs/PARAMETROS_SPP.md) | tabla con valor, unidad, procedencia y estado por perfil |
| [docs/DEPENDENCIAS.md](docs/DEPENDENCIAS.md) | hipótesis usadas, campos que aviónica puede fijar, lo que queda abierto |
| [docs/VALIDACION.md](docs/VALIDACION.md) | comprobaciones pendientes en placa y comparación placa–PC con tolerancias |
| [docs/RESULTADOS_PRUEBAS.md](docs/RESULTADOS_PRUEBAS.md) | resultados, regresiones, escenarios y archivos |
| [docs/RESULTADOS_CAPTURA.md](docs/RESULTADOS_CAPTURA.md) | captura del 24-oct como fixture histórico |

## Compilar y probar

```sh
# SPP 9247d9e descomprimido (sólo lectura) junto al repositorio, o SPP_ROOT=/ruta
make test        # PC: servicio float/double + C++ de cabeceras, router SPP real, herramientas y escenarios
make flight      # vuelos simulados ENTEROS (OpenRocket «Simulation 6», hasta el impacto), por fases
make xtensa      # compilador de la placa: avisos (-Werror) y pila por función
make idf         # ESP-IDF real con idf_test_app (componentes spp y eskf_service)
make docs        # regenera la tabla de parámetros
```

Variables: `SPP_ROOT`, `XTENSA_GCC`, `IDF_PATH`, `IDF_PYTHON`, `DUMP_TXT`, `RESULTADOS`. Si falta
algo, el objetivo dice «BLOQUEO» y qué falta. C11/gnu17 (no C23), `ESKF_USE_FLOAT` en todo el
componente, sin `-ffast-math`; el componente añade `-ffp-contract=off`. Python 3 + numpy.

## Integración en el anfitrión

```c
SPP_Kpid_t icm, bmp, eskf;
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ICM20948_getProducerContract(), &icm);
SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_BMP390_getProducerContract(), &bmp);
eskf_svc_config_t cfg;
eskf_svc_default_config(ESKF_SVC_PROFILE_EXP, &cfg);
eskf_svc_output_preset(&cfg, ESKF_OUT_LOG);   /* registro completo en SD: configuración principal */
/* aviónica puede fijar aquí unidades, montaje, ruido… (docs/DEPENDENCIAS.md) */
SPP_SERVICES_ESKF_configure(&cfg);
SPP_SERVICES_ESKF_register(icm, bmp, &eskf);  /* consumidor y después productor */
/* datalogger suscrito a 'eskf'; la radio es opcional y de Aviónica (docs/SALIDA.md) */
SPP_CORE_init();
/* desde cualquier tarea: SPP_SERVICES_ESKF_getOutput / getDiagnostics;
   SPP_SERVICES_ESKF_notifyBaroRefChange() sólo si el BMP cambia de referencia (aviso explícito) */
```

En CMake del anfitrión: `EXTRA_COMPONENT_DIRS` con esta carpeta y un componente que exporte las
cabeceras de SPP con raíz `spp/` (nombre configurable con `ESKF_SPP_COMPONENT`, por defecto `spp`).

## Herramientas (`tools/`)

| herramienta | uso |
|---|---|
| `spp_decode.py` | lector independiente de registros SPP (bin, hex, volcados); `--csv carpeta/` escribe imu.csv y baro.csv |
| `nav_out_decode.py` | decodificador independiente de la salida v4 (`--csv`, `--diag-csv`, `--ops-csv`); cuenta aparte el CRC SPP exterior y el CRC interno; comprobación por defecto (código 1 ante corrupción o captura incompleta; 2 si no hay nada que comprobar), `--tolerante` para rescatar (código 3) |
| `ork_extract.py` | serie de una simulación OpenRocket (.ork) → CSV con procedencia y comprobación de convenciones |
| `gen_flight.py` | registro SPP sintético de un vuelo a partir de esa serie (aritmética de los productores) |
| `flight_check.py` | pasa un vuelo sintético por el servicio y resume cada fase frente a la verdad |
| `compare_runs.py` | comparación numérica placa–PC con tolerancias: completa con la traza de la placa (identidad sesión/id, ninguna salida sin pareja); valida NaN/inf componente a componente (alt_ref exigida sólo con `BARO_REF`); `--parcial` sin traza, nunca «equivalente» |
| `gen_spp.py`, `spp_to_replay.py`, `explain_alignment.py`, `gyro_dmp_check.py`, `update_params_md.py` | sintéticos simples, conversor a nav_replay, diagnósticos, documentación |

`test/svc_replay` (C): anfitrión de PC; calendario propio determinista (el servicio arranca con el
primer paquete) o `--reproducir` una traza v5 del pegamento (paquetes completos; exige el registro de cierre END y contrasta cada registro con
la traza que regenera la reproducción; códigos 4/5/6; `--parcial` para una traza sin cierre,
código 3); `--entradas/--salidas`, `--mensajes`, `--traza-val`, `--telemetria`, `--estado-us`,
`--compacta-us`, `--dump-md`.

## Limitaciones reales

- Sin integración en el anfitrión real ni ejecución en placa: tiempos de CPU, pila de la tarea del
  router, jitter y parones de la SD con todos los servicios, NO medidos.
- Vuelos simulados (series OpenRocket, ruido blanco y sesgo fijo, sin vibración del motor, IMU
  supuesta en el eje del cohete, actitud congelada bajo paracaídas, sin choque de impacto):
  comprueban la lógica, no la precisión en vuelo.
- Hipótesis no confirmadas: escala del giro (sin ×2), montaje (R = identidad), ruido sin Allan,
  latencia del baro, umbrales, tolerancia de la ventana coherente del respaldo (1 m).
- Con ±4 g la combustión recorta: DEGRADADO y σ_v grande durante ~1 s; un recorte fuera de la
  ventana del motor o más largo que 1,2 s invalida el ESKF (la fuente pasa al respaldo).
- Un hueco de IMU > 10 ms invalida el ESKF hasta el final del vuelo (no se realinea en vuelo). Si
  persisten los parones de ~34 ms de la SD vistos en la captura histórica, la rampa no alinea.
- Si la IMU no ve el despegue (hueco en el ascenso o nunca alineada), el vuelo lo reconoce el
  respaldo (> 10 m sobre la referencia o |v_z| ≥ 3 m/s durante 1 s) y navigation.c queda fuera de
  servicio el resto de la sesión: no hay ESKF en ese vuelo, sólo el respaldo.
- La rampa sólo se considera confirmada por el contrato de inicio (sesión empezada en rampa, sin
  despegue en 10 s) y por evidencia continua después (IMU navegando o baro siguiendo, huecos
  ≤ 0,5 s; en silencio total, 1,05 s de reloj desde la última llegada). Un hueco mayor deja la fase INCIERTA: sin alineación, ZUPT ni referencias nuevas el
  resto de la sesión, aunque fuera en rampa (política conservadora; una sesión pedida por el
  anfitrión en rampa lo restablece). Un reinicio de la placa en vuelo incumple el contrato.
- La referencia de lanzamiento del respaldo sólo se fija en rampa confirmada (1 s de reposo
  continuo): si el primer baro llega ya en vuelo, no hay altura sobre el lanzamiento (NaN, no
  válida) en esa sesión. Caducado (> 0,5 s sin medidas aceptadas), sólo vuelve con una ventana.
- Respaldo barométrico: retraso de v_z durante la combustión (~15 m/s simulado); un
  desplazamiento coherente que dure más de ≈ 0,4 s se toma como el nuevo nivel al recuperar (por eso
  los cambios de referencia deben avisarse); efectos dinámicos de la toma de presión no modelados.
- Posición horizontal no observable (sin GPS ni magnetómetro): `p_x, p_y` sólo relativos.
- Un solo KPID: no se puede enviar sólo la compacta por radio mientras se registra todo; la radio y
  su ciclo de trabajo son de Aviónica.
- Sin traza de la placa la comparación placa–PC sólo es parcial; las tolerancias se justificaron
  con otra aritmética en el PC (FMA), no con resultados de Xtensa.
- Si el respaldo reconoce un vuelo con el ESKF sin atestiguar el reposo en rampa (desplazamiento
  del baro > 10 m o ≥ 3 m/s durante 1 s, p. ej. un cambio de referencia sin avisar), los reinicios
  se rechazan y el ESKF queda fuera de servicio hasta reiniciar la placa (se prefiere a volver a
  rampa en vuelo por error).
