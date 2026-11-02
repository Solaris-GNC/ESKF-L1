# Resultados de esta entrega (2-nov) y archivos

Sobre 8a9616c. Salidas completas en `docs/resultados/` (rutas locales sustituidas por `<repo>`,
`<spp>`, `<idf>`, `<fw_deps>`, `<tmp>`). Todo lo de aquí se ha EJECUTADO en esta entrega; los
números de 8a9616c (264, 264, 62, 154) son antecedentes, no resultados de esta versión.

## Comandos ejecutados y resultado

| comando (en `integration/eskf_service/`) | resultado | registro |
|---|---|---|
| `make test-c` | compilación estricta C99/C11 (`-Wconversion -Werror`) float y double sin avisos; cabeceras desde C++; `test_service` **275/275 float y 275/275 double** | `pc_test_c.log`, valores en `pc_test_service_detalle.log` |
| `make test-spp` | router SPP REAL, **13 escenarios (nuevo: `silencio`), 68/68** | `pc_test_spp.log` |
| `make test-tools` | **160/160** (antes de esta entrega: secciones A–L; nueva M: silencio total con el router real y con el calendario propio, reproducción exacta, comparación EQUIVALENTE, registro PHASE alterado detectado) | `pc_test_tools.log`, `escenarios_vuelo.json` |
| `make flight` | vuelos nominal (90,2 s) y adverso (72,2 s) enteros, por fases, sin cambios | `escenarios_vuelo.log` |
| `make -C ../../c test`, `make -C ../../firmware test` | todas correctas; `c/` y `firmware/` sin cambios (`git status`: 0 archivos) | `pc_nucleo_v4.log` |
| `make xtensa` | xtensa-esp32s3-elf-gcc 8.4.0, `-Os -Werror`, `ESKF_USE_FLOAT`, sin `-ffast-math`, sin avisos | `xtensa_pila.log` |
| `make idf` | aplicación de PRUEBA `idf_test_app` (ESP-IDF v4.4.7, esp32s3), no el anfitrión: 1107 pasos, 0 avisos propios (3 de SPP, sin tocar) | `esp_idf.log` |
| regresión 31 con el servicio de 8a9616c | falla donde debe (7 comprobaciones) | `reproduccion_8a9616c.log` |

Fallidas: ninguna. No ejecutadas: integración en el anfitrión real (faltan su versión de ESP-IDF
y su `sdkconfig`), ejecución en placa y todo lo de docs/VALIDACION.md.

## El defecto de 8a9616c y su regresión (`test_service` 31, router `silencio`, tools M)

| caso | antes (8a9616c) | ahora |
|---|---|---|
| navegación VÁLIDA en rampa hasta 15 s; IMU y BMP detenidos; `consume` cada 10 ms | ESKF NO VÁLIDO por antigüedad, pero `pad = 1` y fase RAMPA indefinidamente | NO VÁLIDO por antigüedad y rampa confirmada sólo hasta la latencia admitida; a los 1,05 s de la última llegada (0,5 + 0,05 + 0,5 s) fase INCIERTA por el reloj (causa 2, why 0x8009, navigation.c cortado) |
| límite exacto | — | 1,050 s sin muestras procesadas: todavía rampa; 1 µs más: INCIERTA |
| aviso de referencia a los 17 s (en silencio) | abría otra sesión con diez segundos nuevos de rampa asumida | conserva la sesión y deja el baro fuera el resto de la sesión (`BARO_REF_CHANGED`) |
| vuelven las muestras a los 18 s | rampa (nueva sesión) | sigue INCIERTA, sin realinear, baro sin usar; un reinicio pedido por el anfitrión en rampa la restablece (VÁLIDO de nuevo) |
| traza | sin registro de la transición | registro PHASE (res 2, causa 2, silencio en ms, reloj del consume) antes del aviso; con el router real: 513/513 paquetes idénticos al reproducir; con el calendario propio y 3 s sin paquetes: mensajes idénticos y EQUIVALENTE; un PHASE alterado (otra causa, END recalculado) → código 4 en ese registro |
| sin transiciones falsas | — | parón del router de 0,9 s con su cola continua y latencia constante de 0,9 s después: rampa confirmada; escenario `congestion` del router real sin cambios; 1,2 s (> latencia admitida): caduca |
| contrato de inicio | — | sin datos tras el inicio: rampa por contrato 10 s + 1,05 s; después INCIERTA |

Durante la prueba con el router apareció una diferencia de reproducción (`t_eval_us`) que no
detectaba el contraste de la traza: el último `nav_tick` antes de cortar navigation.c ocurría en
un consume vacío sin registrar. Ahora el corte da un último `nav_tick` con el reloj del consume
que lo decide (registrado); los paquetes reproducidos son idénticos.

## Memoria y pila

`s_svc` 19 592 B en .bss (+32 B frente a 8a9616c); `s_cfg`/`s_probe`/`s_ref` 504 B;
`eskf_svc_consume` 432 (+16) B de pila y `acquireData` 112 B. Binario de prueba 0x3e7f0 B (76 % de la partición libre).

## Archivos respecto a 8a9616c

Todo dentro de `integration/eskf_service/`; `c/` y `firmware/` sin cambios; SPP y Solaris Software
sin tocar.

- Modificados: `include/eskf_service.h` (`svc.pad_lat_max_s`, `pad_rx_us`, `now_us`,
  `pad_lost_cause`, registro PHASE, traza v5), `src/eskf_service.c` (caducidad por el reloj al
  empezar el consume, último `nav_tick` al cortar, registro de transiciones), `src/eskf_svc_params.c`
  (una fila), `test/test_service.c` (31), `test/test_spp_router.c` (escenario `silencio`),
  `test/svc_replay.c` (PHASE; v1–v4 rechazadas), `test/test_spp_tools.py` (M),
  `README.md`, `docs/SERVICIO.md`, `docs/CONTRATO_ENTRADA.md`, `docs/SALIDA.md`, `docs/FALLOS.md`,
  `docs/PARAMETROS_SPP.md`, `docs/DEPENDENCIAS.md`, `docs/VALIDACION.md`, `docs/GUIA_AVIONICA.md`,
  `docs/RESULTADOS_PRUEBAS.md`, `docs/resultados/*`.
- Nuevo: `docs/resultados/reproduccion_8a9616c.log`.
- Sin cambios: el esquema de salida (v4), `tools/`, `CMakeLists.txt`, `Makefile`,
  `src/spp_services_eskf.c`, `idf_test_app/`, `test/data/`.
