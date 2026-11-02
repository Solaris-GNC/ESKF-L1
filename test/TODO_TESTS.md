# Tests pendientes (temporal)

Convenciones: NED, `R(q)` lleva de cuerpo a NED, `g = [0, 0, -9.81]` (reacción a la gravedad, lo que mide el acelerómetro en reposo), error de actitud local `q_t = q ⊗ δq{δθ}`.

✅ = ya comprobado numéricamente fuera de pytest; solo falta pasarlo a test.

---

## 0. Tests existentes que hay que corregir

- [ ] **`predict` → `_predict`.** `test_rest`, `test_rotation`, `test_quat_norm`, `run_noisy`, `test_covariance_gyro_noise_only`, `test_transition_matrix_finite_differences` y `test_mag_corrects_yaw` llaman a `eskf.predict(...)`. Hay que actualizarlos, o volver a hacer pública `predict` (recomendado: los tests y el caso de solo IMU la usan).
- [ ] **`sensors.py` no se puede importar** (`last_ts` sin valor por defecto después de `flag`), así que pytest no recoge ningún test hasta arreglarlo.

## 1. Updates sin test

### `update_gps(p_meas, v_meas, sigma_p, sigma_v)`
- [ ] ✅ **Coincide con el Kalman lineal clásico.**
  `h(x) = [p; v]` es lineal y `H = [I₆ 0 0 0]` es exacto (ec. 67–68), así que el ESKF debe dar lo mismo que el KF de manual: `K = P Hᵀ (H P Hᵀ + V)⁻¹`, `δx = K (y − h)` y `P⁺ = G [(I − KH) P (I − KH)ᵀ + KVKᵀ] Gᵀ`.
  Comprobar `p`, `v` y `P`.
- [ ] **`V` tiene la estructura de la ec. 69:** `diag(σ_p², σ_p², σ_p², σ_v², σ_v², σ_v²)`. Posición y velocidad tienen unidades distintas (m y m/s), así que se comprueba que una sigma grande en `v` deja `v` casi sin corregir y `p` sí.
- [ ] **Corrige actitud y `b_a` a través de la velocidad.**
  Un error de actitud `δθ` proyecta mal la fuerza específica: `δv̇ = −R[a]× δθ`, el bloque `(v, θ)` de `Phi`. Tras propagar con aceleración, `P` correlaciona `v` con `θ` y `b_a`, y la innovación de velocidad corrige ambos. Con `a = 0` (solo gravedad) solo se observan roll y pitch; para observar el yaw hace falta aceleración horizontal.
- [ ] **`sigma` escalar y por eje dan lo mismo** cuando el vector por eje es constante.

### `update_accel`
- [ ] ✅ **`H` por diferencias finitas, las 15 columnas.**
  `h(x) = R(q)ᵀ g + b_a`: depende de la actitud (columnas `δθ`) y del bias (columnas `δb_a` = `I₃`). Perturbar el estado verdadero con `inject` y comparar `Δh/ε` con `H`.
- [ ] **Corrige roll y pitch, no yaw.**
  La gravedad es vertical. Girar alrededor de la vertical (yaw) no cambia `Rᵀg`, así que esa dirección es inobservable: `H` tiene una columna nula en esa dirección y `P[yaw]` no debe bajar. Partir con error en roll/pitch → convergen. Partir con error en yaw → no cambia.

## 2. `_update` genérico

- [ ] **`P` simétrica y definida positiva tras el update.**
  La forma de Joseph `(I−KH)P(I−KH)ᵀ + KVKᵀ` es una suma de formas cuadráticas, así que es definida positiva por construcción, aunque `K` no sea óptima.
- [ ] **El update no aumenta la incertidumbre:** `P⁺ ≤ P⁻` (diag o traza).
  Medir añade información; con `K` óptima, `P⁺ = P⁻ − K S Kᵀ`.
- [ ] **Quaternion unitario tras la inyección.**
- [ ] **Reset `G` por diferencias finitas.**
  Al inyectar `δθ`, el error que queda se expresa respecto al nuevo nominal: `δθ⁺ ≈ (I − [½δθ]×) δθ⁻`. Perturbar antes y después de la inyección y comparar con `G`.
- [ ] ✅ **Secuencial = apilado** (GPS + baro en el mismo instante; comprobado con el GPS de solo posición, hay que repetirlo con `[p; v]`).
  Con ruidos independientes (`V` diagonal por bloques), aplicar los updates de uno en uno equivale a uno apilado: el segundo parte del `P` ya reducido por el primero. Desmiente la justificación de la sección 4.5.3 del PDF.

## 3. Predicción

- [ ] ✅ **IMU inclinada en reposo no se mueve.**
  En reposo el acelerómetro mide `Rᵀ g` en cualquier orientación. Al rotarlo a NED, `R Rᵀ g − g = 0`. Fija a la vez el signo de `g` y la convención de `R` con `q ≠ identidad`; `test_rest` solo lo comprueba con la identidad.
- [ ] **Aceleración constante.**
  `a_meas = a + g` (fuerza específica). Con Euler (ec. 21–22): `v = a·t` exacta y `p = a·Δt²·N(N−1)/2` (0.495 para 1 m/s² en 1 s a 100 Hz, frente al 0.5 exacto). El test fija la integración que usa el código.
- [ ] **Bias conocido se compensa.**
  Si `b_a` y `b_g` del estado igualan el bias sumado a la medida, `a − b_a` y `ω − b_g` son las verdaderas → reposo.
- [ ] **Ruido de proceso de `sigma_a`.**
  Cada paso inyecta en `v` un impulso de varianza `σ_a²Δt²` (ruido blanco de aceleración integrado un `Δt`). Con `P₀ = 0` y el resto a cero: `var(v) = N·σ_a²·Δt²`.
- [ ] **Ruido de proceso de `sigma_aw` y `sigma_ww`.**
  El bias es un random walk: `ḃ = η`, `η ~ N(0, K²)`. Integrado un `Δt`, la varianza es `K²Δt` (lineal en `Δt`, no cuadrática), así que `var(b) = N·σ²·Δt`.

## 4. Convergencia de los bias

- [ ] **`b_g` converge con accel + mag en reposo.**
  Un bias del giróscopo hace derivar la actitud. Accel (roll/pitch) y mag (yaw) observan la actitud completa. La deriva se acumula en `δθ` y la correlación `θ–b_g` (bloque `−IΔt` en `Phi`) la atribuye al bias.
- [ ] **`b_a,z` converge con baro o GPS.**
  Un bias vertical integra dos veces en posición (`½b·t²`). El baro mide `z`, y la correlación `p–v–b_a` lleva la corrección al bias.

## 5. Para código que aún no existe

- [ ] **`run()` + `Sensor`:**
  - llama al `update_<kind>` correcto
  - cada medida se aplica una sola vez (`last_ts`)
  - `kind` desconocido → error
  - dos medidas consecutivas con el mismo valor se aplican las dos (se decide por timestamp, no por valor)
  - **GPS a través de `run()`:** ahora mismo `run()` llama a `update_<kind>(s.data, s.sigma)`, con dos argumentos, pero `update_gps` pide cuatro. Este test fallará hasta que una de las dos firmas cambie (por ejemplo, `update_gps(y, sigma)` con `y = [p; v]` y `sigma` de 6 elementos).
- [ ] **Umbral del acelerómetro (4.5.2).**
  Si `| ‖a‖ − g | ≥ k` hay aceleración lineal, el acelerómetro ya no mide solo gravedad, y usarlo inclinaría la actitud estimada. Se descarta la medida.
- [ ] **Inicialización (sección 5).**
  - `b_g0` = media del giróscopo en reposo (la velocidad angular verdadera es 0 si se ignora la rotación terrestre).
  - `q₀` recupera una `R` conocida. En reposo, `a₀` apunta hacia arriba en ejes cuerpo, así que `d = −a₀/‖a₀‖`, `e = (d × m₀)/‖·‖` y `n = e × d`. Las filas de `R₀` son `n`, `e`, `d`. La ec. 78 del PDF no normaliza y sale en NWU.

## 6. En la simulación, no en pytest

- [ ] **NEES con Monte Carlo.**
  Si `P` describe bien el error real, `eᵀ P⁻¹ e` sigue una χ² con 15 grados de libertad: media 15. Si sale mayor, el filtro es optimista (`Q` o `V` pequeñas); si sale menor, es pesimista.
