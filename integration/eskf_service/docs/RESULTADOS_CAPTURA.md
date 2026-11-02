# Captura del 24-oct: fixture HISTÓRICO

`spp_dump.bin` se usa como fixture de la compatibilidad histórica (IMU de 48 B, altura ISA), no
como prueba del formato definitivo de SPP 9247d9e (IMU de 44 B con offsets, altura relativa).
`make test-tools` regenera el binario de 16 MiB y el volcado de texto desde los 899 072 B de
paquetes guardados y comprueba su SHA-256 con los recibidos (`b0be0ba8…f334e`, `1c0c7084…c6a9`).

## Comprobado

| afirmación | resultado |
|---|---|
| 14 048 paquetes de 64 B: 1 FSM 0x0002 (4 B), 4343 BMP 0x0004 (12 B), 9704 IMU 0x0008 (48 B) | confirmado |
| CRC correctos, secuencias consecutivas, dropCounter 0 | confirmado |
| 899 072 B de paquetes + ceros hasta 16 MiB; cabeceras 1732–44862 ms = 43,130 s | confirmado |
| BMP ≈ 100,69 Hz, IMU ≈ 225,16 Hz (sin los intervalos largos: 101,33 y 227,20 Hz) | confirmado |
| 13 intervalos IMU de 33–35 ms, cada 1104 paquetes (~3,39 s) | confirmado |
| índice 11 = 0; índice 10 entero 295–453 | confirmado; el código actual identifica el 10 como accuracy del DMP |
| la captura como formato definitivo | rechazada: 48 B es «longitud inesperada» con `--imu-formato 44` |

## Lectura con el código actual de SPP

- **Parones**: la secuencia sigue consecutiva mientras faltan ~6,5–7 periodos IMU por parón. El
  driver actual lee todos los paquetes de la FIFO y publica sólo el último, lo que encaja con un
  bloqueo de ~34 ms (probablemente el escritor de la SD) seguido de una sola publicación. Es una
  explicación compatible, no verificada en placa (DEPENDENCIAS 7–8). Las secuencias consecutivas
  sólo describen paquetes PUBLICADOS.
- **Giro**: la cuantización 1/32,8 refleja el DIVISOR del productor, no el rango. El giro integrado
  coincide con el ángulo del cuaternión DMP (cociente 0,994, `tools/gyro_dmp_check.py --historico`)
  en el firmware de entonces. Con el firmware actual (FS 2000 °/s, /32,8) sigue siendo una
  hipótesis del contrato (sin ×2); no hay campaña prevista para confirmarlo.
- **Saturación**: 7 muestras del eje 3 en ±3,9998779 g = código ±32767 (sin offsets en aquel formato).

## Por qué no alinea con esta captura (se conserva, no se oculta)

`svc_replay --historico` (perfil EXPERIMENTAL, antes BANCO): `align_ok = 0`, `rest_breaks_gap > 0`.
`tools/explain_alignment.py`: los intervalos de 33–35 ms superan `dt_max` (10 ms en
EXPERIMENTAL) y parten la ventana; el mayor tramo continuo en reposo es 3,36 s < `rest_window_s` =
5 s. No se ha aumentado `dt_max` para tapar los parones, ni activado el puente, ni acortado la
ventana. Contraprueba sintética: el mismo reposo sin parones alinea; con parones de 34 ms cada
3,39 s y secuencia consecutiva, no. Si los parones de la SD persisten con el datalogger actual, la
rampa no alineará: se verá en la primera ejecución en placa (docs/VALIDACION.md 2).

## Segmentos de reposo y valores observados

1,768–10,768 s, 13,768–15,768 s y 42,768–43,768 s (ventanas de 1 s con ‖ω‖ < 2 °/s y
σ‖a‖ < 0,01 g). Del primero (9 s) salen los valores OBSERVADOS del perfil EXPERIMENTAL: ruido de
acelerómetro ≈ 0,06 m/s² por eje y de giro ≤ 0,0046 rad/s (se usan ×2), media del giro ≤ 0,0095
rad/s (`bg0_std` 0,01), |‖a‖ − g| máx 0,29 m/s², ruido del baro 0,14 m entre muestras, intervalos
IMU normales 1–7 ms. Son de otro firmware y de condiciones desconocidas: por eso llevan la
procedencia «observado» y no «confirmado», y no hay coeficientes de Allan (9 s no bastan y no se
inventan).
