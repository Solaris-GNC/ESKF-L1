# Compilar, versiones y pruebas realizadas

## Lo que se ha hecho y lo que no

| hecho (en un ordenador x86-64, 19-oct) | NO hecho |
|---|---|
| compilar el sketch para `arduino:esp32:nano_nora` en los 3 perfiles, 0 avisos con `-Wall -Wextra` | cargarlo en una placa |
| compilar el C propio con el compilador Xtensa, `-Werror -Wpedantic -Wshadow -Wdouble-promotion` | ejecutar nada en el ESP32-S3: tiempos, pila, interrupciones, I2C |
| pruebas del PC (abajo), sanitizadores ASan/UBSan, equivalencia con `eskf.py` | probar con sensores reales o con el firmware del equipo (no recibido) |
| reproducción exacta placa/PC **con registros sintéticos** | comparar con un registro real de la placa |

## Versiones usadas

| componente | versión |
|---|---|
| placa / FQBN | Arduino Nano ESP32 — `arduino:esp32:nano_nora` |
| paquete de placas | `arduino:esp32` 2.0.17 (repositorio `arduino/arduino-esp32`, etiqueta `v2.0.17-arduino.5`, commit `40cf8e55`) |
| ESP-IDF del paquete | v4.4.7 (`-DIDF_VER="v4.4.7-dirty"` en la línea de compilación) |
| compilador | `xtensa-esp32s3-elf-gcc/g++` 8.4.0 (crosstool-NG esp-2021r2-patch5) |
| arduino-cli | 1.1.1 |
| biblioteca IMU | «SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library» **1.3.2** (etiqueta `v1.3.2`, commit `32cf0614`), del gestor de bibliotecas |
| API del BMP390 | Bosch BMP3_SensorAPI **v2.0.6** (commit `db4cf8e4`), copiada sin modificar en `src/bmp3` (licencia BSD-3) |
| Wire | 2.0.0 (la del paquete) |
| PC | gcc 13.3.0, Python 3.13, numpy 2.5; scipy (opcional), matplotlib (opcional, `--plot`), pyserial 3.5 (captura) |

Si el gestor de placas del equipo muestra otra versión de «Arduino ESP32 Boards», hay que anotarla, recompilar y repetir `make -C firmware arduino` y las pruebas de placa: lo verificado es esta combinación.

## Opciones reales de compilación (perfil BANCO, de `build/arduino/p1.log`)

Para `navigation.c` (C), las que importan:

```
xtensa-esp32s3-elf-gcc -std=gnu99 -Os -mlongcalls -ffunction-sections -fdata-sections
 -fstack-protector -fstrict-volatile-bitfields -fno-jump-tables -fno-tree-switch-conversion
 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -ggdb
 -DESP_PLATFORM -DESP32 -DARDUINO_ARCH_ESP32 -DARDUINO_NANO_ESP32 -DF_CPU=240000000L
 -DBOARD_HAS_PIN_REMAP -DBOARD_HAS_PSRAM -DARDUINO_USB_MODE=0 -DARDUINO_USB_CDC_ON_BOOT=1
 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=1  [+ -fstack-usage -DVN_PROFILE=1, añadidas]
```

- C++: las mismas con `-std=gnu++11 -fexceptions -fno-rtti`.
- **Sin `-ffast-math`**: `eskf.h` tiene un `#error` si se activa.
- **`ESKF_USE_FLOAT`**: lo activa `eskf.h` porque el paquete pasa `-DARDUINO_ARCH_ESP32` por línea de órdenes a **todas** las unidades, .c y .cpp. No depende de un `#define` en el .ino. `vn_params.c` (C) y `vn_app.cpp` (C++) comprueban en compilación que `eskf_real` es float, y `nav_init` comprueba los tamaños en ejecución.
- **FMA**: con `-ffp-contract=fast` por defecto, el compilador usa `madd.s`/`msub.s` (54 y 17 en `eskf.o`), que redondean una sola vez. Por eso la comparación placa/PC usa tolerancias ([COMPARACION.md](COMPARACION.md)).
- `double` es software en el ESP32-S3: `navigation.c` llama a `__muldf3`, `__adddf3`, etc. para tiempos y ventanas. Su coste se mide en la placa (PROCEDIMIENTOS §7).

Memoria (compilación): 371 KB de programa (11 % de 3 MB) y 41 KB de RAM estática. En ejecución se reservan además 48 KiB de búfer de registro, 3 × 16 KiB de pila (provisional) y la cola.

## Compilar con el IDE de Arduino

1. Gestor de placas: «Arduino ESP32 Boards» (anotar la versión). Placa «Arduino Nano ESP32». USB Mode: «Normal mode (TinyUSB)», que es el valor por defecto.
2. Gestor de bibliotecas: «SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library», **versión 1.3.2**.
3. Abrir `firmware/viradinha_nav/viradinha_nav.ino`. Perfil en `src/vn/vn_profile.h` (por defecto ADQUISICIÓN).
4. Verificar / Subir.

## Compilar con arduino-cli

```bash
arduino-cli core install arduino:esp32          # anotar la versión instalada
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@1.3.2"
arduino-cli compile -b arduino:esp32:nano_nora --warnings all \
  --build-property "compiler.c.extra_flags=-fstack-usage" \
  --build-property "compiler.cpp.extra_flags=-fstack-usage" \
  firmware/viradinha_nav
arduino-cli upload -b arduino:esp32:nano_nora -p /dev/ttyACM0 firmware/viradinha_nav
```

`make -C firmware arduino` compila los tres perfiles con `-DVN_PROFILE=0/1/2` (variables `ARDUINO_CLI`, `SPARKFUN_LIB` y `CTAGS_PROP` para otro entorno).

En este entorno sin acceso a los servidores de Arduino:

- el paquete se clonó de GitHub y el compilador se colocó donde lo espera `platform.txt`;
- `esptool` 4.5.1 (el que fija el índice del paquete) se usó sólo para generar el `.bin`;
- se usó el `ctags` del sistema. El `.ino` no define funciones (`setup`/`loop` están en `vn_main.cpp`), así que da igual qué ctags se use.

Tras cambiar algo en `c/`: `make -C c fw-sync` copia el núcleo al sketch, y `make -C c test` / `make -C firmware test` fallan si no coinciden.

## Pruebas realizadas (PC) y resultado

| orden | qué comprueba | resultado |
|---|---|---|
| `make -C c test` | núcleo, navigation.c (regresiones v4), ordenación de eventos y conversión de la IMU, simulaciones, herramientas, en double, float y C++ | test_eskf 1449/1449 y 1447/1447; test_nav 192/192 ×2; test_fw 18/18 ×2; test_sim 81/81 ×2; herramientas 60/60 |
| `make -C c compare REF=..` | equivalencia del núcleo con `eskf.py` | error máximo 1,28e-13 (double), 1,67e-4 (float), sin cambios |
| `make -C firmware test` | formato del registro, tabla de parámetros (cobertura, estados, perfiles), prueba de arranque; herramientas con registros sintéticos; reproducción exacta; PARAMETROS.md al día | test_vn 1331/1331 ×2; herramientas 43/43; docs-check OK |
| `make -C firmware fma` | de dónde salen las tolerancias placa/PC | 9 escenarios, 0 eventos distintos, tolerancias ≥ 10× lo observado |
| `make -C firmware xtensa` | C propio con el compilador de la placa, `-Werror -Wpedantic -Wdouble-promotion` | sin avisos |
| `make -C firmware arduino` | sketch completo, 3 perfiles | 0 avisos con `-Wall -Wextra` |
| ASan + UBSan | test_eskf/nav/fw/sim (double y float), test_vn, gen_vnlog con fallos inyectados, nav_replay | limpios |

Las herramientas se han probado con registros **sintéticos** (`gen_vnlog`, que imita la ruta de datos del firmware). Que funcionen con los registros reales de la placa se comprobará en el primer arranque (PROCEDIMIENTOS §0).

## Datos y comprobaciones que faltan

- Firmware actual del equipo: bus, bibliotecas, telemetría y almacenamiento. No se recibió; la integración queda como ganchos ([README](README.md)).
- Esquema y módulos (adaptación de niveles a 3,3 V), pines, direcciones e INT de IMU y baro.
- Convención de ejes del cohete y montaje.
- OpenRocket y la curva de empuje: `liftoff_acc` y comprobación de ±16 g con vibración.
- Todas las medidas de PROCEDIMIENTOS §1–§9 y los parámetros pendientes de [PARAMETROS.md](PARAMETROS.md).
- Retardo del DLPF de la IMU: no comprobado en la hoja de datos.
- Si la interrupción RAW_DATA_0_RDY salta por cualquier sensor o por ambos (§1).
