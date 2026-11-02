# Procedimientos de banco

Todo lo que se mide aquí es de **nuestra unidad**: placa, sensores y montaje. Cada resultado se anota en `vn_params.c` con estado **confirmado** y su origen: herramienta, fecha y `boot_id` del registro (`vnlog.py resumen` lo muestra). Los candidatos que imprimen las herramientas son propuestas.

Orden recomendado, que cabe en ~3 días:

| día | qué |
|---|---|
| 1 | §0–§2: primer arranque, registro estático corto; y lanzar el registro largo de la noche |
| 2 | §3–§6: ejes, seis posiciones, signos de giro, baro |
| 3 | rellenar la tabla; perfil BANCO; §7–§8 tiempos y comparación placa/PC |
| 4 | §9 prueba de 2 h con el firmware completo (telemetría y almacenamiento reales) |

## §0. Preparación y captura

1. Perfil ADQUISICIÓN (`vn_profile.h`) y compilar ([COMPILAR.md](COMPILAR.md)).
2. Conectar por USB y ejecutar `python3 tools/capture_serial.py --port /dev/ttyACM0 --out arranque.bin` (en Windows, `COM5`). La placa sólo envía con el puerto abierto (DTR); sin anfitrión descarta y lo cuenta.
3. Ejecutar `python3 tools/vnlog.py resumen arranque.bin` y comprobar:
   - Integridad: CRC 0 y bytes sin sincronismo 0. Si no es así, revisar el cable o el puerto.
   - «registros releídos: …, distintos de lo esperado: 0». Cubre WHO_AM_I 0xEA, DMP y FIFO desactivados, bajo ruido, ±16 g, ±2000 °/s, DLPF 3 y divisores 4; en el BMP390, chip 0x60, modo normal, ×4/×1, 50 Hz e IIR desactivado.
   - Textos «IMU begin=0 dir=0x69/0x68» y «BARO begin=0 dir=0x77/0x76». La dirección detectada va a `hw.imu_addr` y `hw.baro_addr`.
   - Si no hay interrupción conectada, la IMU va por sondeo (marca POLLED, ±0,5 ms).
4. Esperar 10 min encendida antes de las medidas estáticas, para que se estabilice la temperatura (`analyze_static` informa del rango).

## §1. Interrupción y frecuencias reales

Con el registro de §2, `analyze_static.py` muestra:

- **frecuencia real** del acelerómetro y del giróscopo (por cambio de valor) y de las interrupciones;
- la tabla **sensores nuevos por interrupción** (sólo acelerómetro / sólo giróscopo / ambos);
- intervalos con jitter y percentiles, latencia de lectura y huecos > `dt_max`.

Así se confirma si RAW_DATA_0_RDY salta por cualquier sensor (~445 eventos/s) o sólo cuando están los dos, y se revisa `dt_max`.

**Criterios de aceptación:**

- frecuencias a ±1 % de 225 y 220 Hz;
- 0 huecos > `dt_max`, 0 interrupciones perdidas, 0 errores I2C;
- latencia de lectura (p99) < 1 ms.

## §2. Registro estático (10–30 min) y Allan

Procedimiento:

1. Placa sobre una superficie firme, sin vibraciones (nada de mesa con ordenador ni ventiladores) y lejos de corrientes de aire, para el baro.
2. Registrar 30 min con `capture_serial.py`.
3. Analizar con `python3 tools/analyze_static.py estatico.bin --json estatico.json --plot adev.png`.

Resultados: `σ_eq` por evento para `eskf.sigma_a` y `eskf.sigma_w` (las unidades de `eskf.c` están en PARAMETROS.md), ruido del baro (`baro_sigma`) y máximos de las ventanas de reposo (umbrales de reposo y alineación).

**K (`sigma_aw`, `sigma_ww`):** con 30 min la herramienta casi siempre dirá «K no observable» y dará una cota superior y una duración orientativa. Si hace falta, dejar un registro estático **largo** (una noche, ≥ 3–8 h, a temperatura estable) y repetir. Si no se puede, anotar la cota superior **como tal** (estado propuesto, con el origen «cota de un registro de N min»).

## §3. Ejes del cohete

Requisito previo: el equipo fija la **convención de ejes del cohete** (p. ej. +x hacia la ojiva) y la anota.

1. Registro A: cohete (o placa montada en su soporte) **vertical, ojiva arriba**, quieto 20 s. Con la convención del ejemplo, +x hacia arriba.
2. Registro B: cohete **tumbado** con un eje lateral conocido hacia arriba (p. ej. +y), quieto 20 s.
3. Registro C: partiendo de A, reposo 3 s, girar a mano **+90° alrededor de +x** (regla de la mano derecha, pulgar hacia la ojiva) en 1–2 s y reposo 3 s.
4. Ejecutar `python3 tools/axes_check.py --arriba A.bin +x --arriba B.bin +y --giro C.bin +x`.

La herramienta devuelve el mapa y los signos (con det = +1) y la coherencia del giro. Si sale «coherente», anotar `imu.axis_map_*` e `imu.axis_sign_*` como confirmados y `fw.axes_confirmed = 1`. La inclinación del montaje se informa: si supera ~2°, revisar el soporte.

## §4. Seis posiciones

Seis registros de 30 s cada uno, con la placa apoyada sobre cada cara de un bloque rectificado, de modo que cada eje del sensor quede arriba y abajo una vez. El orden da igual.

Ejecutar `python3 tools/six_position.py p1.bin … p6.bin [--g <gravedad local>]`.

Resultados:

- sesgo y error de escala por eje, acoplamiento entre ejes e inclinación de cada posición;
- residuo de ‖a‖ corregida − g;
- sesgo del giróscopo y su dispersión.

Con eso se fija `eskf.ba0_std`, sumando la dispersión entre arranques: repetir §4 o §2 en 3 arranques distintos. La corrección de escala o sesgo **no** se aplica en `imu_conv`; aplicarla es una decisión pendiente. La gravedad local de Huesca es un dato externo: si no se pasa, el error de escala incluye la diferencia con 9,80665.

## §5. Signos del giróscopo

Igual que §3-C, pero para los tres ejes del cohete: un registro por eje (+90° positivo) y `axes_check.py … --giro Ci.bin +eje`. Los tres deben salir «COHERENTE» con un ángulo cercano a +90°. Esto confirma también la conversión a rad/s: 16,4 LSB/(°/s).

## §6. Barómetro: signo y latencia

1. Reposo 10 s.
2. Subir y bajar la placa en vertical **1,5–2 m**, 10–15 veces con un periodo de 3–5 s, sin girarla. Mejor con una guía, una cuerda con polea o un ascensor de escalera.
3. Reposo 5 s.
4. Ejecutar `python3 tools/baro_updown.py ud.bin`.

**Criterios:**

- escala baro/IMU entre +0,9 y +1,1, con signo positivo: el baro sube cuando la placa sube;
- latencia ± σ, con σ < 10 ms.

Repetir 3–5 veces y anotar la media en `baro_latency_s`, y `eskf.tau_max` ≥ `baro_latency_s` + `dt_max`. La latencia medida es la efectiva **respecto a la marca de la IMU** e incluye conversión, sondeo, lectura y la diferencia de filtros. **No** usar los 10,88 / 12,48 ms de la hoja de datos.

En simulación, con 2 m y 15 ciclos, el estimador no tiene sesgo y su σ es ≈ 5 ms con 0,3 m de ruido de baro.

## §7. Tiempos y pila en la placa

Perfil BANCO, con la navegación activa:

- **Al arrancar**, `vn_bench.c` ejecuta navigation.c con datos sintéticos en la propia tarea vn_nav: alineación, despegue, vuelo con baro y planeo, caminos que el banco estático nunca recorre. Registra los µs mínimo, medio y máximo por tipo de llamada (contador de ciclos CCOUNT del Xtensa, `ESP.getCycleCount`) y la **pila libre mínima tras recorrer el código de vuelo** (`uxTaskGetStackHighWaterMark`; en ESP-IDF, bytes). `vnlog.py resumen` lo muestra.
- **En marcha**, STATS a 1 Hz da: µs máximos y suma por segundo de navigation.c (esp_timer), lectura de la IMU, escritura del registro, pila libre mínima de cada tarea, montón mínimo, búfer mínimo libre y pérdidas.
- **Estático**, `make -C firmware arduino` compila con `-fstack-usage`: los `.su` de `build/arduino/p*/sketch/` dan la pila por función. Las mayores son `predict_step` con 2032 B, `nav_task` con 720 y `nav_on_imu` con 288. La cadena más profunda de vn_nav se estima en < 5 KiB, frente a la reserva provisional de 16 KiB, a confirmar con la marca de agua.

**Criterios:**

- máximo de navigation.c < periodo medio de eventos (~2,3 ms);
- uso medio de vn_nav < 50 %;
- pila libre ≥ 25 % en cada tarea;
- 0 pérdidas.

Riesgo identificado sin medir: la llamada que cierra cada ventana de reposo ordena hasta 512 altitudes y hace 40 bisecciones en double (software en el ESP32-S3). Si en la prueba de arranque o en STATS supera ~4 ms, cambiar esa bisección a float. No afecta a la adquisición, que va en el otro núcleo.

## §8. Comparación placa/PC

Con un registro del perfil BANCO (5 min en reposo, y si es posible con movimientos a mano):

```
make -C c build/nav_replay_float
python3 firmware/tools/compare_board_pc.py banco.bin --nav-replay c/build/nav_replay_float
```

**Criterio:** ret, validez y motivos idénticos en **todos** los eventos, y estado dentro de las tolerancias de [COMPARACION.md](COMPARACION.md). Si el registro tiene pérdidas, la herramienta se detiene en la primera y lo dice.

## §9. Prueba de 2 horas

**Condiciones:**

- firmware **completo** del equipo con vn integrado, perfil BANCO (navegación activa, con los valores ya medidos);
- **telemetría y almacenamiento reales activos** a su ritmo de vuelo;
- placa quieta, con alimentación de vuelo si es posible;
- 2 h seguidas y capturando el registro.

Durante la prueba, provocar sin reiniciar:

- desconectar y reconectar el USB (si es el sumidero de pruebas);
- 3 golpes suaves en la mesa en momentos anotados;
- una ráfaga de telemetría si el protocolo lo permite.

Ejecutar `python3 tools/soak_report.py larga.bin`.

**Criterios** (FALLO bloquea la congelación del 30-oct):

- ≥ 2 h, un solo arranque, reloj monótono;
- ningún intervalo de la IMU > `dt_max`, 0 colas llenas, 0 desbordes de ordenación, 0 tramas perdidas del registro, 0 errores I2C;
- interrupciones «perdidas»: aviso. Con interrupción por cualquier sensor, dos pulsos casi simultáneos de acelerómetro y giróscopo cuentan aunque no se pierda dato; si aparecen, mirar con `analyze_static.py` si coinciden con huecos;
- tiempo máximo de navigation.c < `dt_max` y uso medio < 50 %;
- pila libre ≥ 25 % en cada tarea;
- montón sin descenso tras 10 min;
- frecuencia de eventos estable ±1 % y baro a 50 Hz;
- navegación VÁLIDA todo el tiempo tras alinear, sin huecos. Los golpes pueden cortar el reposo, pero no deben invalidar.

Si algo falla, el informe indica el criterio y el valor. Repetir tras corregir.
