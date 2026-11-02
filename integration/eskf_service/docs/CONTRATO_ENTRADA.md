# Contrato de entrada (SPP 9247d9e)

Estados: **código** (hecho del código de SPP; no es una calibración validada de nuestra placa),
**observado** (captura histórica), **simulado** (OpenRocket/RocketPy), **hipótesis**, **pendiente**.
Aviónica calibra los sensores en SPP: el servicio no corrige nada de lo que ya hace el productor.

## Paquete (64 B, little-endian) — código

| bytes | campo | origen |
|---|---|---|
| 0 | versión = 1 | `K_SPP_PKT_VERSION` |
| 1–2 | APID/KPID (un bit por productor) | `registerProducer` |
| 3–4 | secuencia (la lleva cada productor; vuelve a 0 tras 65535) | driver |
| 5–6 | longitud del payload (≤ 50) | `packetData` |
| 7–10 | `timestampMs` = `SPP_HAL_TIME_getTimeMs()` al CONSTRUIR el paquete | `packetData` |
| 11 | `dropCounter` = 0 siempre | `packetData` |
| 12–61 | payload; el resto 0xFF (`memset(…, 0xFF, …)`) | `packetData` |
| 62–63 | CRC-16/CCITT (0x1021, inicial 0xFFFF) de los bytes 0–61, little-endian | `SPP_UTIL_crc16` |

Se rechazan y cuentan: versión, CRC (el relleno 0xFF está dentro), APID no suscrito (incluida la
salida propia), longitud > 50. Las longitudes de cada flujo se comprueban al decodificar.

## IMU definitiva: 44 B = 11 float32 — código (`icm20948.c`)

| índice | campo | uso |
|---|---|---|
| 0–2 | ax ay az = `raw/8192.0f − offset` (g) | entrada inercial |
| 3–5 | gx gy gz = `raw/32.8f − offset` (°/s) | entrada inercial |
| 6–9 | qw qx qy qz (DMP quat9; qw = √(1 − x² − y² − z²) ≥ 0) | sólo diagnóstico |
| 10 | accuracy (int16 de la FIFO del DMP) | sólo diagnóstico; NO es una σ en rad |

Offsets ya restados por el productor (`icm20948.h`): acel 0.004621, −0.001157, 0.010014 g;
giro 0.681567, 0.428753, −0.329620 °/s. Son hechos del código, no una calibración de nuestra
placa, y **no se vuelven a restar**: los sesgos del ESKF son los residuales de lo que recibe.

Conversión, una sola vez: `s = v · unidad` (g → 9.80665 m/s², °/s → π/180 rad/s) y después
`cuerpo = R · s` en float, `(r0·s0 + r1·s1) + r2·s2`, la MISMA R para acel y giro. R se comprueba:
ortonormal (1e-5) y det = +1. En EXPERIMENTAL R = identidad: el «cuerpo» del estimador ES el marco
de la IMU; no se afirma que ningún eje sea el eje longitudinal del cohete. En ESTRICTO, sin ejes
confirmados, la navegación no arranca. Si aviónica fija el montaje, basta con `in.r_sb_*`.

DMP y accuracy inválidos (no finitos o ‖q‖ lejos de 1) con a y ω válidos: la muestra se usa, se
cuenta `dmp_invalid` y se marca `ESKF_SVC_R_DMP_INVALID`. El cuaternión DMP no se fusiona, no es
una medida independiente (sale del mismo sensor) ni una verdad para validar el ESKF.

### Histórico de 48 B (sólo compatibilidad explícita)

`eskf_svc_config_historical()` / `svc_replay --historico`: los mismos 11 campos sin offsets +
reservado (0 en la captura), altura ISA absoluta. Un paquete de 48 B con la configuración
definitiva (o de 44 con la histórica) es `imu_bad_len` y pérdida. No es el antiguo formato con
magnetómetro y paquete de continuación de 16 B, que no se admite.

## Rango, escala y saturación

Del código: `ACCEL_FS_SEL = ±4 g` (8192 LSB/g, coherente con el divisor) y `GYRO_FS_SEL = ±2000 °/s`
con divisor **32,8 LSB/(°/s)**, que corresponde a ±1000 °/s. Si la FIFO del DMP entrega el giro en
la escala del registro (16,4 LSB/(°/s)), el valor publicado sería la MITAD del real. No se aplica
ningún ×2: EXPERIMENTAL usa la hipótesis del contrato (°/s publicados = °/s reales,
`in.gyro_unit_si = π/180`, PROPUESTO); ESTRICTO lo deja PENDIENTE (por eso no navega).
En la captura histórica (firmware anterior) el giro integrado coincide con el giro del cuaternión
DMP (cociente 0,99, `tools/gyro_dmp_check.py`): es una comprobación cruzada, no una calibración;
hay que repetirla con el firmware actual y confirmarla con giros de ángulo conocido. La aplicación
Arduino v4 proponía ±16 g: no se reutiliza.

Saturación, en el dominio de códigos y en ejes del SENSOR, antes de rotar:
`código = (v + offset) · divisor`; **candidato** si `|código| ≥ 32768 − 1,5`. Con offsets el
extremo publicado está desplazado (p. ej. `32767/8192 − 0.004621 = 3,995 g`), por eso no se mira
`|v| ≥ 4`. Un `|código| > 32768,5` es imposible para el productor: muestra inválida. Los
candidatos pasan a navigation.c en los ejes del cuerpo que reciben ese eje del sensor (|r| > 1e-6).

| categoría | estado |
|---|---|
| confirmada por hardware | no hay fuente: el payload no lleva banderas (`sat_hw_confirmed` = 0) |
| candidata por límites publicados | `sat_cand_acc/gyro`, `ESKF_SVC_R_SAT_CANDIDATE` |
| desconocida | `ESKF_SVC_R_SAT_UNKNOWN` siempre: la ausencia de banderas no prueba ausencia de saturación; el rango útil a la salida del DMP está por verificar |

Los rangos y divisores NO se cambian aquí: el vuelo simulado supera ±4 g (13,35 g a 0,399 s;
> 4 g de 0,028 a 0,949 s; 57 g en el despliegue adverso a 9,079 s) y el acelerómetro recortará.

- **ESTRICTO**: sin continuación (`acc_sat_max_s = 0`): un candidato hace NO VÁLIDO (`NAV_R_ACC_SAT_LIMIT`).
- **EXPERIMENTAL**: continuación del acelerómetro de navigation.c con hipótesis numéricas de la
  simulación: `acc_sat_max_s = 1,2 s` (0,92 s recortados + 30 %), `acc_sat_excess = 110 m/s²`
  (13,35 g − 4 g = 91,8 m/s², + 20 %). Mientras dura, la covarianza de velocidad crece
  (σ = exceso·√(T₁² − T₀²)), el baro se sigue aplicando, el estado se publica DEGRADADO con
  `NAV_R_ACC_SAT` y `ESKF_SVC_R_SAT_CANDIDATE` y el incidente queda enclavado. Superado el tiempo
  admitido: `NAV_R_ACC_SAT_LIMIT`, NO VÁLIDO. La lectura recortada NO se sustituye por la curva del
  motor ni por la simulación.
- **Ventana del motor**: el modelo sólo vale para la combustión. Un candidato posterior a
  `t_despegue + in.sat_motor_window_s` (1,5 s, simulado: apagado a 1,076 s) enclava
  `ESKF_SVC_R_SAT_OUT_OF_MODEL` → NO VÁLIDO (p. ej. el despliegue adverso de 57 g). El pico del
  motor no se usa como límite de todo el vuelo.
- Giróscopo: sin continuación en ningún perfil (`gyro_sat_max_s = 0`).

## Barómetro: 12 B = altura m, presión Pa, temperatura °C — código (`bmp390.c`)

- Configuración escrita: OSR = 0x00 (×1), ODR = 0x01 (100 Hz) e IIR = 0x02 en el registro
  `CONFIG` (0x1F). El campo `iir_filter` son los bits 3:1, así que 0x02 es el valor 1:
  **coeficiente 1** (y[n] = (y[n−1] + x[n]) / 2; retardo ≈ 1 muestra = 10 ms). En entregas
  anteriores se escribió «coeficiente 3»: era un error de lectura del registro. No se sustituye
  por la propuesta Arduino (50 Hz, ×4).
- Calibración del productor: promedia 500 muestras (≈ 5 s) de presión y temperatura (p0, T0),
  **sin publicarlas**. Después `altura = (T0+273,15)/0,0065 · (1 − (p/p0)^(1/5,255))`: altura
  RELATIVA a la rampa. El servicio **usa la altura publicada**: no reconstruye p0 (tampoco a partir
  de la primera temperatura), no suma la altura del sitio y no repite la calibración de 500 muestras.
- Validación de cada medida: longitud 12, valores finitos y límites físicos configurables
  (`in.baro_h_abs_max_m` 5000 m — apogeo máximo Monte Carlo 474,7 m —, presión 300–1250 hPa y
  temperatura −40…+85 °C del BMP390). Una medida fuera se descarta y se cuenta (`baro_bad_value`)
  **sin retirar el baro de la sesión**; la siguiente buena se usa. Además navigation.c aplica su
  puerta de innovación (5 σ) y el respaldo barométrico la suya (`bb.gate_m`).
- Fallo persistente: se ve en la antigüedad de la última medida ACEPTADA (`bb.age_us`,
  `bb_age_ms` de la compacta; los rechazos no la renuevan), en la validez del respaldo (caduca a los
  0,5 s, recupera con una ventana coherente de 20 medidas: docs/SALIDA.md) y en
  `NAV_R_BARO_TIMEOUT` del núcleo en vuelo.
- Referencia de navigation.c: la fija en una ventana de reposo (mediana/MAD, ≥ 50 muestras), no
  con un BMP suelto. Hasta entonces, alineado: `ESKF_SVC_R_NO_ALT_REF` (DEGRADADO) y capacidad
  `BARO_REF` ausente. En rampa la referencia sigue la deriva lenta con salto máximo de 2 m.
- **Cambio de referencia del productor**: NO se deduce de las muestras (un salto aislado lo
  rechazan las puertas). Sólo con aviso explícito del anfitrión, `SPP_SERVICES_ESKF_notifyBaroRefChange()`:
  - **en rampa** abre una sesión nueva con frontera temporal: todo lo construido antes del aviso
    (en el buzón, en la cola de orden o aún en la cola del router) se descarta sin contarlo como
    pérdida (`session_discarded`); navegación y respaldo fijan una referencia NUEVA sólo con datos
    posteriores (en 1597c47 21 baros antiguos pendientes inicializaban la sesión nueva: 96,3 m,
    despegue falso y reinicios rechazados);
  - **en vuelo** NO reinicia nada: deja de usar el baro el resto de la sesión, tanto en el ESKF
    (`baro_not_fed`) como en el respaldo (NO válido hasta acabar la sesión); enclava
    `ESKF_SVC_R_BARO_REF_CHANGED` (DEGRADADO). El ESKF sigue mientras cumpla su validez y muestra
    la falta de correcciones (`NAV_R_BARO_TIMEOUT` al segundo). No se reconstruye otra referencia.
  En el código actual p0/T0 sólo cambian al reiniciar la placa, lo que reinicia también este
  servicio. No hay bloqueo automático por «salto de p0».

## Tiempo

| instante | qué es | dónde |
|---|---|---|
| construcción | `timestampMs` del paquete (1 ms, truncado) | `packetData`, tras leer el sensor |
| recepción | reloj al entrar en `deliverToMailbox` | `rx_us`; `lat_rx_max_us` = recepción − construcción |
| procesamiento | reloj en `consumeData` | `t_proc_us`; `lat_proc_max_us` = procesamiento − recepción |
| estado | instante que representa el estado | `t_state_us` (de navigation.c) |
| evaluación | instante con que se evaluó la validez | `t_eval_us` (de navigation.c) |

`timestampMs` NO es la marca de adquisición de cada muestra: el driver del ICM lee toda la FIFO y
publica sólo el último paquete. Reloj del servicio: 64 bits en µs, inyectable; por defecto
`SPP_HAL_TIME_getTimeMs()` extendido a 64 bits (la misma fuente que `timestampMs`, resolución
1 ms). `SPP_HAL_TIME_getTimeUs()` es de 32 bits (vuelve a 0 cada 71,6 min) y no se usa. En ESP-IDF
existe `SPP_SERVICES_ESKF_clockEspTimer` (misma época sólo con el puerto ESP32 de SPP).
**Marca futura**: si la marca es posterior a la recepción más `in.future_tol_us` (1 ms: la marca
está truncada a ms), el paquete se descarta y se cuenta (`ts_future`, `clock_mismatch`) ANTES de
tocar el seguimiento del flujo; en la IMU enclava `LATE_EVENT` (DEGRADADO). Así una marca de 20 s
con el reloj en 13 s ya no hace rechazar las 225 IMU siguientes. Cada marca se extiende a 64 bits contra
la recepción (vuelta de 2³² ms incluida) y entra en navigation.c como `(double)t_us · 1e-6`.
Multiplicar ms por 1000 no crea precisión: la bandera `TIME_MS` va en cada mensaje.
Nunca se reconstruyen intervalos nominales ni se asigna la recepción de un lote a sus muestras.
`nav_tick` recibe el reloj real en cada `consumeData`, sin adelantarlo ni atrasarlo.

## Orden, duplicados, pérdidas y reinicios

El orden lo da el tiempo; la secuencia sólo sirve para contar pérdidas. Por flujo (IMU, baro):

| caso | criterio | acción |
|---|---|---|
| repetido | misma secuencia y marca no posterior | se descarta, `seq_dup`; NO es pérdida |
| antiguo | marca anterior a la última procesada (o igual con secuencia atrás) | se descarta, `seq_old`, `ts_back_*`; no toca el flujo, ni la fase, ni la sesión |
| salto | secuencia +k (k > 1) con marca posterior | se procesa; `seq_lost_* += k − 1` (pérdida real, una sola vez) |
| resincronización | secuencia atrás (o igual) con marca POSTERIOR: el productor reinició su contador con el reloj continuo | se procesa; `seq_resync`, `ESKF_SVC_R_SEQ_RESYNC` (DEGRADADO: no se sabe si faltó algo) |
| tardío entre flujos | marca anterior al último evento ya entregado a navigation.c | se descarta, `late_*`; IMU → `LATE_EVENT` |

- Los descartes del buzón, los CRC rechazados y las marcas futuras aparecen después como saltos de
  secuencia: se cuentan UNA vez (antes, 6 pérdidas se publicaban como 12).
- Ninguna sesión se reinicia por inferencia. Sesión nueva sólo con `requestSessionRestart()` (en
  rampa confirmada o en fase incierta: es una declaración del anfitrión de que está en rampa) o con
  el aviso de cambio de referencia del baro en rampa confirmada.
- **Contrato de inicio** (docs/SERVICIO.md, «Fase»): cada sesión empieza con el vehículo en rampa y
  sin despegue durante `svc.pad_start_s` = 10 s; después, la rampa sólo sigue confirmada con
  evidencia continua (IMU navegando sin huecos que oculten un despegue, o baro siguiendo sin
  movimiento rápido), con huecos ≤ `svc.pad_gap_max_s` = 0,5 s; si no llega nada, caduca por el
  reloj a los 1,05 s de la última llegada (latencia admitida `svc.pad_lat_max_s` = 0,5 s incluida).
  **Con el vuelo reconocido**
  (docs/SERVICIO.md, «Fase de vuelo enclavada») el reinicio se RECHAZA (`restarts_refused`,
  `ESKF_SVC_R_RESTART_REFUSED`): no hay vuelta automática a rampa.
- **Frontera de sesión**: la sesión nueva empieza en el reloj del `consumeData` que la abre. Las
  muestras con marca anterior (las del buzón, las que aún estén en la cola del router) se
  descartan y sólo actualizan el seguimiento de secuencia, para que la primera muestra posterior
  no cuente pérdidas ficticias; los eventos ya pendientes en la cola de orden se borran.
- Arranque completo del equipo: el servicio arranca de cero (sesión 1); el anfitrión puede
  distinguir arranques con `boot_id` (configuración), que va en la traza y en la salida compacta.
- **Pérdidas breves de IMU**: se continúa, DEGRADADO y enclavado (`IMU_LOSS`, `LATE_EVENT`,
  `SEQ_RESYNC`), SÓLO si el intervalo real entre muestras aplicadas es ≤ `eskf.dt_max` (10 ms en
  EXPERIMENTAL: una muestra perdida a 225 Hz da 8,9 ms). Si el hueco lo supera, navigation.c lo
  declara `NAV_R_TIME_GAP` y el ESKF queda NO VÁLIDO: la fuente pasa al respaldo barométrico si
  vale. No se amplían límites, no se inventan muestras (`gap_max = 0`, sin puente) y en vuelo no
  se reinicia ni se realinea: queda NO VÁLIDO hasta el final de la sesión. La recuperación por
  alineación es sólo de rampa (ventana de reposo posterior al bloqueo). Los motivos enclavados de
  pérdida se borran en rampa tras una ventana de reposo COMPLETA posterior al último fallo (el
  contador de ventanas buenas de navigation.c, no el de alineaciones); con el vuelo reconocido no
  se borran nunca. Si la IMU se pierde justo en el despegue, navigation.c sigue en RAMPA; el
  respaldo reconoce el vuelo y desde ese momento navigation.c no recibe más muestras, así que la
  IMU que vuelve en el descenso no puede completar un supuesto reposo ni realinear (prueba 27).
