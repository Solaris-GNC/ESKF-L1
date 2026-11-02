# Comportamiento ante fallos (resumen)

Lo que hace el servicio en cada caso, con la prueba que lo comprueba (`test_service` N,
`test_spp_router` escenario, `test_spp_tools` sección). Perfil EXPERIMENTAL salvo que se diga.

## Entradas, tiempo y sesiones

| fallo | qué hace | validez publicada | se recupera | prueba |
|---|---|---|---|---|
| paquete con CRC, versión, APID o longitud malos | rechazado y contado; el hueco aparece como salto de secuencia | — | — | 3 |
| IMU perdida con intervalo real ≤ 10 ms (`dt_max`) | contada una vez; navigation.c sigue | DEGRADADO (`IMU_LOSS`) | en rampa, tras una ventana de reposo completa posterior; en vuelo no | 7, 10, 26, `congestion` |
| hueco de IMU > 10 ms | `TIME_GAP`: no propaga a ciegas (sin puente, sin muestras inventadas) | ESKF NO VÁLIDO; fuente BARO si el respaldo vale | sólo en rampa CONFIRMADA, realineando; en vuelo o en fase incierta nunca | 10, 26, 29, `pool_vacio` |
| silencio total de IMU y BMP con el router llamando a `consumeData` | a los 1,05 s de la última llegada (0,5 + 0,05 + 0,5 de latencia admitida): fase INCIERTA por el reloj (causa 2, registro PHASE); un aviso de referencia posterior conserva la sesión y deja el baro fuera; al volver las muestras sigue INCIERTA (en 8a9616c seguía en RAMPA y el aviso abría sesión con 10 s más de rampa asumida) | ESKF NO VÁLIDO (`STALE` y después `NAV_DISABLED`, why 0x8009), `phase` 2 | sólo con una sesión pedida por el anfitrión en rampa | 31, M |
| parón del router ≤ 1,05 s con su cola continua, o latencia constante (cola llena fluyendo) | sin transición: cuenta la llegada de las muestras, no su marca | según los datos | — | 31, `congestion` |
| hueco de evidencia de rampa > 0,5 s (IMU sin baro siguiendo, baro sin IMU navegando, o ambos) pasado el plazo de inicio | fase INCIERTA: navigation.c no recibe nada más (ni alineación, ni realineación, ni ZUPT, ni referencia); el respaldo no fija cero nuevo y congela el que tenía | ESKF NO VÁLIDO (`NAV_DISABLED`, why 0x8009), `phase` 2; fuente BARO sólo con el cero de rampa | sólo con una sesión pedida por el anfitrión en rampa | 29 |
| IMU perdida en el despegue y primer baro ya en un descenso LENTO (2 m/s o 0,8 m/s, por debajo del detector de vuelo), con o sin alineación previa | fase INCIERTA desde el primer dato que revela el hueco; la IMU que vuelve con ~1 g no alinea; el baro casi quieto no fija cero (en a26704a: align_ok 1→2, ZUPT 129→242, ESKF −7,5 m VÁLIDO; o respaldo −3,5 m válido) | NO VÁLIDO; fuente NINGUNA; v_z del respaldo publicada | no | 29 |
| IMU sin datos (sensor o pool caído) | `nav_tick` sigue con el reloj real; el estado conserva su `t_state` | NO VÁLIDO (`STALE`) | al volver los datos (y, si hubo hueco, al realinear en rampa) | 8, `pool_vacio` |
| IMU con valores no finitos o imposibles | se entrega como NaN (navigation.c conserva el tiempo) | DEGRADADO | idem pérdida | 3 |
| marca futura (> recepción + 1 ms) | descartada ANTES de tocar el flujo (`ts_future`) | DEGRADADO (`LATE_EVENT`) si es IMU | idem pérdida | 14 |
| paquete antiguo o repetido | descartado y contado; no cambia fase ni sesión | — | — | 6, 15, `tardio` |
| secuencia reiniciada con reloj continuo | se procesa; `seq_resync` | DEGRADADO (`SEQ_RESYNC`) | idem pérdida | 15 |
| reinicio pedido en vuelo (vuelo reconocido por cualquier detector) | rechazado (`RESTART_REFUSED`) | sin cambio | — | 15, 27, `despegue` |
| IMU perdida en el despegue: navigation.c no ve el vuelo y sigue en RAMPA | el vuelo se reconoce por el respaldo (altura > 10 m sobre la referencia, o \|v_z\| ≥ 3 m/s durante 1 s); desde ahí navigation.c no recibe más muestras: ni alineación, ni realineación, ni ZUPT, ni referencia nueva por un falso reposo en el descenso | ESKF NO VÁLIDO (`NAV_DISABLED`, why 0x8008), fase publicada VUELO; fuente BARO si el respaldo vale | no en la sesión | 27 |
| ESKF nunca alineado en rampa y la IMU aparece en vuelo | igual: no hay alineación inicial en vuelo | ESKF NO VÁLIDO; fuente BARO o ninguna | no | 27 |
| reinicio o aviso de referencia en rampa con muestras pendientes | sesión nueva con frontera temporal: lo anterior se descarta (`session_discarded`), sin pérdidas ficticias | según la sesión nueva | alinea y fija referencia con datos posteriores | 25 |
| reinicio pedido en fase incierta | aceptado: el anfitrión declara que está en rampa (contrato de inicio) | según la sesión nueva | sí | 29 |
| aviso de referencia en fase incierta | como en vuelo: baro fuera el resto de la sesión, sin sesión nueva | — | no | 29 |

## Barómetro y respaldo

| fallo | qué hace | validez publicada | se recupera | prueba |
|---|---|---|---|---|
| una medida inválida (NaN, fuera de límites físicos) | descartada; el baro sigue en uso | — | inmediata | 16 |
| atípico finito o ráfaga breve (< 20 rechazos) | rechazado por las puertas (ESKF 5 σ, respaldo 10 m); el estado del respaldo NO avanza ni se renueva su frescura | respaldo válido mientras su última aceptada tenga ≤ 0,5 s | inmediata | 16, 23, 24 |
| rechazos continuados (≥ 20) o incoherentes | respaldo a recuperación: ventana de 20 medidas coherentes | respaldo NO válido (`RECOVER`) | con la ventana coherente | 23, 24 |
| corte del baro | el respaldo caduca a los 0,5 s; al volver, ventana coherente | respaldo NO válido; en vuelo el ESKF marca `BARO_TIMEOUT` | ventana coherente; el cero de lanzamiento NO cambia | 16, 24 |
| rechazos espaciados (< 20 y < 0,1 s entre medidas) más allá de la caducidad | al llegar una medida con antigüedad > 0,5 s: `RECOVER` (`bb_expired`); una sola medida compatible no lo revalida (en a26704a sí) | respaldo NO válido, sin `BARO_BACKUP` ni fuente BARO | ventana coherente; el cero no cambia | 30 |
| primera lectura anómala | no fija nada: hace falta una ventana coherente y después 1 s de reposo | respaldo NO válido hasta la referencia | sí | 24 |
| primer baro ya en vuelo (o en el apogeo) | sigue y registra las medidas; NO fija cero de lanzamiento (sólo en rampa, con 1 s de reposo continuo); su v_z sí reconoce el vuelo | respaldo NO válido, `ref_m`/`h_m` NaN, sin `BARO_BACKUP` ni fuente BARO | no en la sesión (las recuperaciones tampoco fijan cero) | 27, 28 |
| primer BMP sin referencia de navegación | ESKF alineado sin `alt_ref` | DEGRADADO (`NO_ALT_REF`), sin `ESKF_NAV` | siguiente ventana de reposo con baro | 9 |
| cambio de referencia del BMP sin aviso | NO se deduce; las puertas rechazan las medidas desplazadas | — | — | 9 |
| cambio de referencia avisado en rampa | sesión nueva con datos posteriores al aviso | según la sesión nueva | al fijar la nueva referencia | 9, 25 |
| cambio de referencia avisado en vuelo | sin baro en el ESKF ni en el respaldo el resto de la sesión; sin reinicio ni referencia nueva | ESKF DEGRADADO (`BARO_REF_CHANGED`, `BARO_TIMEOUT`); respaldo NO válido | no en la sesión | 17 |

## Saturación

| fallo | qué hace | validez publicada | se recupera | prueba |
|---|---|---|---|---|
| ±4 g en la combustión | continuación de navigation.c: σ_v crece, el baro sigue | DEGRADADO (`ACC_SAT`), enclavado | — (historial) | 18, I |
| saturación más larga que 1,2 s | `ACC_SAT_LIMIT` | NO VÁLIDO; fuente BARO si el respaldo vale | no | 18 |
| saturación tras la ventana del motor (despliegue) | `SAT_OUT_OF_MODEL` | NO VÁLIDO; fuente BARO | no | 18, I (adverso, 57 g) |
| saturación del giróscopo | sin continuación | NO VÁLIDO | no | (núcleo) |

## Fuentes, transporte y herramientas

| fallo | qué hace | validez publicada | prueba |
|---|---|---|---|
| sólo queda el respaldo | fuente BARO, capacidad `BARO_BACKUP` | ESKF NO VÁLIDO | 18, 26, I |
| no queda ninguna fuente | fuente NINGUNA; la compacta se sigue publicando con su estado y antigüedades | NO VÁLIDO | 26 |
| configuración de transporte inválida | `init` falla; consultas lo indican | — | 4, 22, `config_invalida` |
| configuración de navegación incompleta (ESTRICTO) | transporte y respaldo funcionan; motivo y `why_code` publicados | NO VÁLIDO (`NAV_DISABLED`) | 5, `estricto_pendiente` |
| consulta antes del primer `consumeData` | sesión 1, NO VÁLIDO, NaN (no una estructura a cero) | NO VÁLIDO | 5, 21 |
| productor sin sitio en el router | sólo consumidor: navega y responde, sin salida SPP | — | `registro_parcial` |
| cola del router alta / pool vacío para la salida | se aplaza (traza: `DEFER_*`); el servicio sigue consumiendo | — | `congestion`, `pool_vacio` |
| traza sin cierre (END), con registros que faltan o desbordada; truncada a mitad de registro | `svc_replay --reproducir` código 5; sólo con `--parcial` reproduce lo registrado y lo declara PARCIAL (código 3) | — | H, J |
| metadatos de la traza distintos de lo reproducido (tipo/longitud/secuencia/marca de una salida, longitud o resultado de una entrada, consume) | código 4 con el registro, el byte y ambos valores | — | H |
| comparación con salidas sin pareja o captura incompleta | `compare_runs.py` NO da equivalencia (código 1) | — | J |
| NaN o infinito en cualquier componente (posición, velocidad, sesgos, σ, alt_ref, actitud) | `compare_runs.py` código 1 con el campo y el índice; dos NaN sólo valen en campos no disponibles según estado y esquema (alt_ref: sin `BARO_REF`); cuaternión no finito o no unitario con diagnóstico propio | — | J, L |
| capturas idénticas con alt_ref NaN legítimo (alineado y válido inercialmente con `NO_ALT_REF`, o sin BMP) | `compare_runs.py` EQUIVALENTE (en a26704a: NO EQUIVALENTE, 25 snapshots rechazados) | — | L |
| captura vacía, KPID inexistente o sin ningún mensaje decodificable | `nav_out_decode.py` código 2 «SIN DATOS COMPROBABLES» (1 si además hay corrupción); nunca «captura comprobada» | — | L |
| paquete con CRC SPP incorrecto en la captura de salida | `nav_out_decode.py` lo cuenta aparte del CRC interno y falla (código 1); `--tolerante` rescata el resto y enumera lo descartado (código 3) | — | K |

Nunca: reinicio de sesión inferido, vuelta automática a rampa en vuelo, rampa deducida de no haber
detectado el despegue, alineación o realineación tras reconocer el vuelo o con la fase incierta,
ZUPT o referencia por un supuesto reposo en vuelo o con la fase incierta,
continuidad inventada en un hueco, frescura renovada por un rechazo, referencia tomada de una
lectura suelta o de datos en vuelo, cambio del cero de lanzamiento en vuelo, un instante de
despegue inventado cuando sólo se sabe que el vuelo ya empezó, sustitución de la lectura recortada
por la curva del motor, publicar VÁLIDO sin referencia barométrica.
