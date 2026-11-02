# Firmware de navegación del Viradinha MKII (v4)

Integra el ESKF en un sketch de Arduino para el **Arduino Nano ESP32** con **ICM-20948** y **BMP390**. El vuelo experimental es el 7-nov-2026 y sólo hace adquisición, estimación, telemetría y registro, sin control.

**Estado.** Compila sin avisos (`-Wall -Wextra`) para `arduino:esp32:nano_nora` en los tres perfiles. Las pruebas del PC y la reproducción exacta con registros sintéticos pasan. **Nada se ha ejecutado en la placa**: no hubo hardware ni se recibió el firmware del equipo. Todo lo que dependa de la placa, los sensores o el montaje está pendiente y se mide con los procedimientos de [PROCEDIMIENTOS.md](PROCEDIMIENTOS.md).

## Contenido

```
viradinha_nav/                 sketch (abrir viradinha_nav.ino)
  vn_main.cpp                  setup()/loop() → vn_app_setup()/vn_app_loop()
  src/nav/                     COPIA de c/ (eskf, navigation, nav_events, imu_conv); make -C c fw-sync
  src/vn/vn_app.*              tareas, ISR, colas, registro, estadísticas, ganchos débiles
  src/vn/vn_imu.cpp            ICM-20948 (biblioteca SparkFun 1.3.2) + relectura de registros
  src/vn/vn_baro.cpp           BMP390 (API oficial de Bosch v2.0.6, en src/bmp3, sin modificar)
  src/vn/vn_params.*           TABLA ÚNICA de parámetros con estado y origen (PARAMETROS.md)
  src/vn/vn_profile.h          perfil: ADQUISICIÓN (por defecto) / BANCO / VUELO
  src/vn/vn_logfmt.*           formato binario del registro (C puro, también en el PC)
  src/vn/vn_bench.*            prueba de tiempos y pila de navigation.c al arrancar
tools/                         Python: vnlog, capture_serial, analyze_static, six_position,
                               axes_check, baro_updown, log_to_replay, compare_board_pc, soak_report
test/                          pruebas del PC: test_vn.c, gen_vnlog.c (registros sintéticos),
                               dump_params.c, test_tools_fw.py, fma_experiment.py
PARAMETROS.md  PROCEDIMIENTOS.md  COMPILAR.md  COMPARACION.md
```

El núcleo no está duplicado: `src/nav/` es una copia literal de `c/`, porque Arduino sólo compila lo que hay dentro del sketch. `make -C c test` y `make -C firmware test` fallan si las dos copias difieren.

## Arquitectura

```
INT IMU ─► ISR: t = esp_timer_get_time(), notifica          (sólo eso)
             │
vn_acq (núcleo 1, prio 20) ── ÚNICO dueño del I2C
   lee IMU en cada interrupción (o sondeo 1 ms si no hay INT), sondea el baro (>= 2 ms)
   ├─► registro crudo IMU/BARO ──────────────────────┐  (SIEMPRE: sin calibrar, sin navegar)
   └─► cola (48) ─► vn_nav (núcleo 0, prio 15) ── ÚNICO dueño de nav_t
                      nav_tick(reloj) · nav_merge (orden por tiempo) · nav_on_imu/baro
                      ├─► TICK, NAVEV (ret, validez, motivos, µs), NAVOUT 50 Hz, NAVCNT 1 Hz ─┤
                      └─► vn_telemetry_publish() 10 Hz (gancho)                              │
loop() (prio 1): cabecera + parámetros + relecturas cada 10 s, STATS 1 Hz ──────────────────┤
                                                                                              ▼
                                        búfer acotado de 48 KiB (xRingbuffer) ─► vn_log ─► vn_sink_write()
                                        lleno → la trama se descarta y se cuenta              (USB por defecto)
```

**Tiempos.** La base común es `esp_timer_get_time()` (µs, int64). Las diferencias del firmware se hacen en µs enteros. `navigation.c` recibe `t = µs·1e-6` en **double**, nunca en float. Durante más de 2 h la diferencia de dos marcas así convertidas tiene un error < 2·10⁻¹² s.

- IMU: `t` es el instante de la interrupción de dato listo (o del sondeo que lo detecta, marcado POLLED).
- Baro: `t` es el fin de la lectura. La latencia hasta el instante efectivo de medida la aplica `navigation.c` con `baro_latency_s`, que se mide con estas mismas marcas (PROCEDIMIENTOS §6).
- Retardo del DLPF de la IMU: no lo he podido comprobar en la hoja de datos (no se pudo descargar en este entorno), así que está **pendiente**. Afecta a acelerómetro y giróscopo, y la latencia del baro medida respecto a la IMU ya lo incluye.
- Sin FIFO: cada muestra lleva su propia marca de interrupción, y no se asigna a un lote el instante de lectura.

**Hallazgo de configuración.** Con divisores 4, el giróscopo va a 1100/5 = **220 Hz** y el acelerómetro a 1125/5 = **225 Hz** (fórmulas de la biblioteca, que citan la hoja de datos), así que no van a la misma frecuencia. La interrupción RAW_DATA_0_RDY no indica qué sensor cambió. El firmware:

- lee en cada interrupción;
- marca ACC_NEW/GYR_NEW comparando valores;
- **entrega todas las lecturas** a la navegación.

Descartar las lecturas «sin cambio» abriría huecos falsos: con el ruido real, tres ejes iguales por azar ocurren en ~0,05 % de los eventos (en la simulación, 34 de 130 000). Un valor retenido durante su propio periodo no sesga el paseo aleatorio si σ_a se calcula por evento (PARAMETROS.md). Que la interrupción salte por cualquier sensor o sólo por uno se confirma en el banco: `analyze_static.py` da la tabla «sensores nuevos por interrupción».

**Saturación.** Se detecta por código crudo en los dos extremos (`imu_conv`). Con ±16 g, el código máximo +32767 equivale a 15,9995 g, así que un límite de «16 g» en valor físico nunca vería el recorte positivo.

**Registro.** El formato está en `vn_logfmt.h`: trama `A5 5A | len | tipo | carga | CRC-16`, que se resincroniza sola. Incluye:

- cabecera con versiones, perfil, compilador, IDF, core y placa;
- todos los parámetros con estado y origen;
- relecturas de registros de los sensores y coeficientes NVM del BMP390;
- IMU cruda con dos marcas (dato listo y fin de lectura), temperatura, magnetómetro cada 8, saturación y banderas;
- baro con presión compensada (Pa), temperatura, altitud ISA y tres marcas;
- por evento, el retorno de `navigation.c` con validez, motivos y µs de ejecución;
- el reloj de `nav_tick`;
- la salida completa: `t_state`, `t_eval`, `alt_ref`, `alt_ref_seq`, validez, motivos, `nav.f.degraded`, p, v, q, σ, `b_a` y `b_g`;
- `nav.cnt` y `nav.last`, estadísticas de tiempos, pila, colas y pérdidas, y la prueba de tiempos del arranque.

Con TICK y NAVEV, la navegación de la placa se puede **reproducir exactamente** en el PC ([COMPARACION.md](COMPARACION.md)). Así cualquier magnitud interna (innovaciones, NIS) se puede recalcular después.

**Altitud.** El BMP390 usa la compensación oficial de Bosch en double, con presión en Pa y temperatura en °C. La altura se calcula con la ISA, `h = 44330·(1 − (p/p0)^0.1903)`, con `p0 = 101325 Pa`. La navegación sólo usa diferencias respecto a la referencia robusta de la rampa (`alt_ref`), así que p0 no afecta.

## Integración en el firmware del equipo

No se ha recibido el firmware actual, así que no se ha modificado. Para integrarlo:

1. Copiar `src/vn`, `src/nav` y `src/bmp3` al sketch del equipo y llamar a `vn_app_setup()` desde su `setup()` y a `vn_app_loop()` desde su `loop()`. Si su `loop()` ya tiene trabajo, se puede llamar a `vn_app_loop()` periódicamente: sólo emite cabecera y STATS.
2. Sustituir los **ganchos débiles** de `vn_app.h`:
   - `vn_sink_begin` y `vn_sink_write`: almacenamiento de vuelo (SD/flash). No deben bloquear.
   - `vn_telemetry_publish`: telemetría. Se llama desde vn_nav y no debe bloquear. Desde otra tarea se puede usar `vn_nav_snapshot()`.
3. Si el firmware ya tiene drivers del ICM-20948 o del BMP390, reimplementar las 6 funciones de `vn_sensors.h` sobre ellos. Una sola tarea debe ser la dueña del bus.
4. Rellenar pines y direcciones en `vn_params.c` (hoy pendientes: −1 = pines I2C por defecto y sondeo; 0 = autodetección).
5. Medir con la telemetría y el almacenamiento reales activos (prueba de 2 h).

## Correspondencia con lo pedido

| # | entregable | dónde |
|---|---|---|
| 1 | archivos de integración sin duplicar el núcleo | `viradinha_nav/` (núcleo copiado y comprobado, no bifurcado) |
| 2 | configuración centralizada con estado y origen | `vn_params.c`, [PARAMETROS.md](PARAMETROS.md) |
| 3 | modo de adquisición sin navegación completa | perfil ADQUISICIÓN (por defecto); en BANCO/VUELO incompletos el registro sigue |
| 4 | análisis del registro estático | `tools/analyze_static.py` |
| 5 | registro → nav_replay con el mismo módulo | `tools/log_to_replay.py` (órdenes IMURAW/BAROUS/TICKUS en `c/test/nav_replay.c`) |
| 6 | regresiones y comparación placa/PC | `c/test/test_nav.c` (v4-*), `test_fw.c`, `test_vn.c`; `tools/compare_board_pc.py`, [COMPARACION.md](COMPARACION.md) |
| 7 | tiempos y pila en ESP32 | `vn_bench.c` (CCOUNT de Xtensa), STATS (esp_timer, `uxTaskGetStackHighWaterMark`), `make -C firmware xtensa` / `arduino` (`-fstack-usage`) |
| 8 | ejes, seis posiciones, signos, baro | [PROCEDIMIENTOS.md](PROCEDIMIENTOS.md) §3–6 y sus herramientas |
| 9 | prueba de 2 horas | PROCEDIMIENTOS §9, `tools/soak_report.py` |
| 10 | compilación, versiones, pruebas y faltantes | [COMPILAR.md](COMPILAR.md) |
