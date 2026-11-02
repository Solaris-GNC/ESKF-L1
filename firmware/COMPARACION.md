# Comparación placa / ordenador

## Qué se compara

`tools/compare_board_pc.py registro.bin --nav-replay c/build/nav_replay_float` toma un registro de la placa con navegación (perfil BANCO o VUELO) y hace lo siguiente:

1. Reconstruye con `log_to_replay.py` **exactamente** lo que recibió `navigation.c` en la placa:
   - la configuración, de las tramas PARAM;
   - el orden de las llamadas, de las NAVEV, que se registran en el orden real de vn_nav;
   - el reloj de `nav_tick`, de las TICK;
   - la IMU como **códigos crudos** (IMURAW), convertidos en el PC con la misma `imu_conv` y la misma cuenta `t = µs·1e-6`;
   - el baro como el float registrado (BAROUS).
2. Ejecuta ese guion con `nav_replay`, que es el mismo `navigation.c` y `eskf.c`, compilado con el `eskf_real` que indica la cabecera (float en la placa).
3. Compara:
   - **en cada evento**, `ret`, validez y motivos deben ser **idénticos**. Es el criterio fuerte: cualquier error de configuración, de orden, de marcas o de conversión cambia alguna decisión;
   - **en cada NAVOUT**, `t_state`, validez, motivos, `alt_ref_seq` y `nav.f.degraded` deben ser idénticos; p, v, actitud, σ, `b_a`, `b_g` y `alt_ref` se comparan con tolerancia.

Si falta una trama necesaria (pérdidas del registro), la reconstrucción se detiene en ella y lo dice, porque desde ahí la comparación dejaría de ser exacta.

## Comprobación de la propia cadena (PC)

Con registros sintéticos de `gen_vnlog`, que reproduce la ruta de datos de `vn_app.cpp` y ejecuta `navigation.c` igual que la placa, y el mismo compilador sin FMA, la comparación `--exact` da **diferencia cero** en todos los campos y eventos. También la da con huecos, NaN, interrupciones perdidas y bloqueos inyectados (`test_tools_fw.py`). La prueba también comprueba que se detecta:

- un solo código crudo alterado;
- usar `nav_replay` en double contra un registro en float;
- las pérdidas del registro.

## Por qué no es bit a bit con la placa: FMA

El compilador Xtensa emplea `madd.s`/`msub.s` (multiplicación y suma con **un solo** redondeo) para `a·b + c`; en `eskf.o` hay 54 y 17. El PC x86-64 sin `-mfma` redondea dos veces. A esto se suman `sqrtf`/`sinf`/`cosf` de newlib frente a glibc, que pueden diferir en el último bit. Son diferencias de redondeo: no cambian las ecuaciones, pero se propagan por el filtro.

## Experimento de tolerancias (`make -C firmware fma`)

En el experimento:

- el mismo escenario se ejecuta con contracción FMA (`gen_vnlog_fma`, x86-64 con `-mfma -ffp-contract=fast`), que hace de «placa», y sin ella (`nav_replay_float`);
- las entradas son idénticas bit a bit y sólo cambia el redondeo de casi todas las operaciones, una perturbación del mismo tipo y orden que la esperada en la placa;
- son 9 escenarios: vuelo de 3 s de empuje y 4 de planeo, subir y bajar, giros, sesgos grandes, otro montaje e interrupción sólo del acelerómetro.

Diferencia máxima de cada grupo:

| escenario | p (m) | p/σ | v (m/s) | v/σ | actitud (rad) | act./σ | σ_p rel | σ_v rel | σ_θ rel | b_a | b_g | alt_ref | eventos distintos |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| vuelo 12 s | 0.0058 | 0.00054 | 0.0015 | 0.0016 | 2.1e-05 | 0.00016 | 0.0002 | 0.00021 | 0.0002 | 0.00019 | 9.3e-08 | 0 | 0 |
| vuelo + subir/bajar | 0.0021 | 0.0005 | 0.00034 | 0.00047 | 5.7e-06 | 3.4e-05 | 2e-05 | 9e-05 | 8.6e-06 | 2.3e-05 | 3.1e-07 | 0 | 0 |
| vuelo, otra semilla | 0.0009 | 0.0009 | 0.00018 | 0.0011 | 3.1e-06 | 2.1e-05 | 1.8e-05 | 0.00012 | 1.2e-05 | 2.2e-05 | 2.9e-08 | 0 | 0 |
| 30 s + giro + vuelo | 0.00031 | 0.00039 | 0.00012 | 0.00091 | 7e-06 | 2.9e-05 | 0.00022 | 0.00029 | 0.00024 | 2.3e-05 | 1.6e-07 | 0 | 0 |
| 30 s, semilla 5 | 0.00015 | 7.3e-05 | 3.6e-05 | 0.00089 | 4.9e-06 | 2.2e-05 | 0.00026 | 0.00021 | 0.00023 | 5.3e-06 | 2.4e-08 | 0 | 0 |
| 30 s, semilla 6 | 0.00081 | 0.0015 | 9.5e-05 | 0.0018 | 5.6e-06 | 1.8e-05 | 0.00016 | 0.00039 | 0.0006 | 1.9e-05 | 7.2e-08 | 0 | 0 |
| sesgos grandes | 0.0087 | 0.0028 | 0.0022 | 0.0043 | 0.00021 | 0.0013 | 0.0011 | 0.0011 | 0.0013 | 0.00029 | 8.6e-06 | 0 | 0 |
| eje −x arriba | 0.02 | 0.0057 | 0.0035 | 0.0054 | 0.00031 | 0.0018 | 0.00058 | 0.00075 | 0.00062 | 0.00052 | 1.4e-05 | 0 | 0 |
| modelo acel (una irq) | 0.13 | 0.015 | 0.042 | 0.016 | 0.00053 | 0.0027 | 0.0059 | 0.01 | 0.0059 | 0.0052 | 3.8e-06 | 0 | 0 |
| **máximo** | 0.13 | 0.015 | 0.042 | 0.016 | 0.00053 | 0.0027 | 0.0059 | 0.01 | 0.0059 | 0.0052 | 1.4e-05 | 0 | 0 |

Las diferencias crecen en vuelo con la incertidumbre, mientras el filtro integra sin observaciones fuertes. Por eso las tolerancias de p, v y actitud son **relativas a la σ que publica el propio filtro**. Ningún evento cambió de decisión en ninguno de los escenarios.

## Tolerancias (`TOL` en `compare_board_pc.py`)

Son 10 × el máximo observado, redondeado hacia arriba:

| grupo | criterio |
|---|---|
| p, v | \|Δ\| ≤ 0,2·σ (por eje) |
| actitud | ángulo ≤ 0,03·‖σ_θ‖ |
| σ_p, σ_v, σ_θ | \|Δ\| ≤ 10 % del valor |
| b_a | \|Δ\| ≤ 0,06 m/s² |
| b_g | \|Δ\| ≤ 2·10⁻⁴ rad/s |
| alt_ref | \|Δ\| ≤ 1·10⁻⁶ m |
| ret, validez, motivos, t_state, alt_ref_seq, degraded | idénticos |

Son una **hipótesis** hasta comparar con un registro real (PROCEDIMIENTOS §8). Si la placa las supera:

1. Mirar el primer evento distinto que informa la herramienta. Si es una decisión en el límite, por ejemplo una innovación justo en la puerta o un umbral de reposo, es la sensibilidad esperada y conviene ampliar el experimento con ese caso.
2. Si no lo es, sospechar de la configuración, de la versión del núcleo (`fw-check`) o de la conversión, que deberían dar diferencia **cero** en el primer evento.
