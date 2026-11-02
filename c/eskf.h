/*
 * eskf.h — Error-State Kalman Filter, 15 estados (port en C de eskf.py).
 *
 * Convenciones (idénticas a la versión Python, ver docs/ESKF.pdf):
 *   - Marco de navegación NED. z positivo hacia abajo.
 *   - Cuaterniones de Hamilton q = [w, x, y, z]; R(q) lleva de cuerpo a navegación.
 *   - Error de actitud local: q_t = q ⊗ δq{δθ}.
 *   - Estado nominal x = [p, v, q, b_a, b_g] (16).
 *   - Estado de error δx = [δp, δv, δθ, δb_a, δb_g] (15), en ese orden en P.
 *   - Acelerómetro: fuerza específica en m/s². Giróscopo: rad/s. dt: s.
 *   - g = [0, 0, -9.81]: reacción gravitatoria (lo que lee el acelerómetro en reposo,
 *     expresado en NED). La predicción usa v̇ = R(q)·(a_m − b_a) − g.
 *   - Barómetro: posición vertical NED, z = −altura_relativa.
 *
 * Diseño para microcontrolador:
 *   - C99, sin memoria dinámica ni recursión; todo el estado en eskf_t.
 *   - Tipo real configurable: -DESKF_USE_FLOAT para float32. La macro debe ser la MISMA
 *     en todas las unidades de compilación que incluyan este archivo. eskf_init() y
 *     eskf_default_params() comprueban en tiempo de ejecución que el tamaño de las
 *     estructuras coincide con el de la librería y devuelven ESKF_E_CONFIG si no.
 *   - Actualizaciones escalares secuenciales (V diagonal), covarianza en forma de Joseph.
 *
 * Contrato de errores:
 *   - Toda función pública que puede fallar devuelve int. Valor < 0: código eskf_status_t.
 *   - Las entradas inválidas (punteros nulos, NaN/Inf, sigma de medida <= 0, dt fuera de
 *     contrato, requisitos no cumplidos) se rechazan ANTES de modificar el estado.
 *   - Si un cálculo con entradas válidas produce un resultado no finito o una covarianza
 *     no válida, el filtro queda en fallo (f->fault = 1) y todas las llamadas devuelven
 *     ESKF_E_FAULT hasta eskf_init() o eskf_align_static(). El estado en fallo no debe
 *     usarse. El registro de datos debe ser independiente del filtro.
 */
#ifndef ESKF_H
#define ESKF_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ESP32 (FPU sólo de precisión simple): float en TODAS las unidades de compilación.
   El core Arduino-ESP32 pasa -DARDUINO_ARCH_ESP32 y -DESP_PLATFORM a C y a C++, así que la
   decisión no depende de un #define en el .ino (que no llega a los .c). Para forzar double
   en ESP32 (no recomendado) definir ESKF_FORCE_DOUBLE en build_opt.h. */
#if !defined(ESKF_USE_FLOAT) && !defined(ESKF_FORCE_DOUBLE) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM))
#define ESKF_USE_FLOAT
#endif

#ifdef ESKF_USE_FLOAT
typedef float eskf_real;
#else
typedef double eskf_real;
#endif

#define ESKF_N 15 /* dimensión del estado de error */

/* Índices de los bloques del estado de error dentro de P */
#define ESKF_IDX_P 0
#define ESKF_IDX_V 3
#define ESKF_IDX_TH 6
#define ESKF_IDX_BA 9
#define ESKF_IDX_BG 12

/* Máximo de subpasos con los que eskf_predict() puentea un hueco temporal */
#define ESKF_MAX_BRIDGE_STEPS 64

typedef enum {
    ESKF_OK = 0,
    ESKF_E_NULL = -1,         /* puntero nulo */
    ESKF_E_NOT_INIT = -2,     /* eskf_t sin inicializar con eskf_init() */
    ESKF_E_CONFIG = -3,       /* parámetro inválido o float/double incoherente entre módulos */
    ESKF_E_INPUT = -4,        /* medida no finita, o sigma de medida no finita o <= 0 */
    ESKF_E_DT = -5,           /* dt no finito o <= 0 */
    ESKF_E_DT_GAP = -6,       /* dt > dt_max y no puenteable (> gap_max) */
    ESKF_E_NOT_AT_REST = -7,  /* requiere reposo confirmado por el llamador (eskf_set_rest) */
    ESKF_E_MOTION = -8,       /* la medida contradice el reposo (norma o giro) */
    ESKF_E_NO_REF = -9,       /* falta mag_ref */
    ESKF_E_NO_HEADING = -10,  /* el marco horizontal no tiene rumbo conocido */
    ESKF_E_GATED = -11,       /* todas las componentes rechazadas por la puerta de innovación */
    ESKF_E_DEGENERATE = -12,  /* geometría de alineación degenerada */
    ESKF_E_NUMERIC = -13,     /* resultado no finito o covarianza no válida: filtro en fallo */
    ESKF_E_FAULT = -14        /* el filtro ya estaba en fallo */
} eskf_status_t;

typedef enum {
    ESKF_SENSOR_NONE = 0,
    ESKF_SENSOR_GPS,
    ESKF_SENSOR_VEL,
    ESKF_SENSOR_ZUPT,
    ESKF_SENSOR_MAG,
    ESKF_SENSOR_ACCEL,
    ESKF_SENSOR_BARO
} eskf_sensor_t;

/* Bits de f->degraded: el estimador sigue funcionando pero su resultado es menos fiable */
#define ESKF_DEG_GAP_BRIDGED 0x1u /* algún hueco de IMU se puenteó con retención de orden cero */
#define ESKF_DEG_GAP_LOST 0x2u    /* algún hueco de IMU se rechazó: se ha perdido tiempo */

typedef struct {
    /* Ruido de proceso de la IMU (desviaciones típicas, >= 0; 0 se admite en ensayos) */
    eskf_real sigma_a;  /* ruido blanco del acelerómetro (m/s²) */
    eskf_real sigma_w;  /* ruido blanco del giróscopo (rad/s) */
    eskf_real sigma_aw; /* random walk del bias del acelerómetro (m/s²/√s) */
    eskf_real sigma_ww; /* random walk del bias del giróscopo (rad/s/√s) */

    /* Campo magnético terrestre en NED en el sitio de lanzamiento (WMM/IGRF),
       en las mismas unidades que el magnetómetro calibrado. */
    eskf_real mag_ref[3];
    int has_mag_ref; /* 0 si no se usa magnetómetro */

    /* |‖a_m‖ - 9.81| máximo (m/s²) para usar el acelerómetro como medida de gravedad.
       Es un filtro de rechazo, NO prueba de reposo (ver eskf_set_rest). > 0. */
    eskf_real accel_threshold;

    /* Puerta de innovación (desviaciones típicas) por componente escalar.
       Se rechaza si innov² > gate²·S. 0 = desactivada (igual que Python). */
    eskf_real innov_gate;

    /* Contrato temporal. Ambos dependen de la frecuencia real de la IMU y NO tienen
       valor por defecto: eskf_init() devuelve ESKF_E_CONFIG si dt_max <= 0. */
    eskf_real dt_max;  /* dt máximo de una muestra normal (s), p. ej. 1.5–2 × periodo nominal */
    eskf_real gap_max; /* hueco máximo (s) que se puentea con subpasos; 0 = nunca */
    eskf_real tau_max; /* |τ| máximo (s) en eskf_update_baro_at; 0 = dt_max. Debe cubrir la
                          latencia REAL del barómetro (ver eskf_update_baro_at) */

    /* Comprobaciones groseras del vector medio en eskf_align_static. > 0. */
    eskf_real align_acc_tol;  /* |‖a_medio‖ − 9.81| máximo (m/s²) */
    eskf_real align_gyro_max; /* ‖ω_medio‖ máximo (rad/s); cota grosera, NO dato de la IMU */

    /* Covarianza inicial y tras alineación (desviaciones típicas, >= 0) */
    eskf_real p0_std;  /* posición (m) */
    eskf_real v0_std;  /* velocidad (m/s) */
    eskf_real th0_std; /* actitud (rad) */
    eskf_real ba0_std; /* bias acelerómetro (m/s²) */
    eskf_real bg0_std; /* bias giróscopo (rad/s) */
} eskf_params_t;

/* Diagnóstico de la ÚLTIMA llamada a una función eskf_update_*. Se reinicia al principio
   de cada llamada, también cuando se rechaza antes de procesar nada. */
typedef struct {
    int sensor;     /* eskf_sensor_t */
    int status;     /* el mismo valor que devolvió la función */
    int n_used;     /* componentes escalares aplicadas */
    int n_gated;    /* componentes rechazadas por la puerta */
    int n_numeric;  /* componentes con S <= 0 o no finitas */
    eskf_real nis;  /* suma de innov²/S de las componentes aplicadas */
} eskf_update_diag_t;

typedef struct {
    uint32_t n_predict;      /* predicciones aplicadas */
    uint32_t n_dt_rejected;  /* predicciones rechazadas por dt */
    uint32_t n_gaps_bridged; /* huecos puenteados */
} eskf_counters_t;

typedef struct {
    /* PRIMER campo: su desplazamiento (0) no depende de eskf_real, así que toda función
       puede comprobarlo sin leer fuera de una estructura de otro tamaño. */
    uint32_t magic; /* marca de inicialización */

    /* Estado nominal */
    eskf_real p[3];
    eskf_real v[3];
    eskf_real q[4];
    eskf_real b_a[3];
    eskf_real b_g[3];

    /* Covarianza del estado de error */
    eskf_real P[ESKF_N][ESKF_N];

    eskf_real g[3];
    eskf_params_t prm;

    /* Estado lógico */
    int fault;          /* 1: fallo numérico, ver contrato de errores */
    int at_rest;        /* reposo confirmado por el llamador (eskf_set_rest) */
    int heading_known;  /* 1: el eje N es el norte geográfico (alineado con mag + mag_ref) */
    unsigned degraded;  /* bits ESKF_DEG_* */

    eskf_update_diag_t last;
    eskf_counters_t cnt;
} eskf_t;

/* ---------------- Inicialización ---------------- */

/* Rellena parámetros por defecto. dt_max queda a 0 a propósito: hay que fijarlo según la
   frecuencia real de la IMU. Ruidos: valores MEMS genéricos, sustituir por Allan variance. */
#define eskf_default_params(prm) eskf_default_params_checked((prm), sizeof(eskf_params_t))
int eskf_default_params_checked(eskf_params_t *prm, size_t sizeof_params);

/* Valida prm e inicializa: estado en el origen, q = identidad, P diagonal de *_std,
   at_rest = 0, heading_known = 0, sin fallo. Si prm no es válido devuelve ESKF_E_CONFIG
   y deja f sin inicializar. */
#define eskf_init(f, prm) eskf_init_checked((f), (prm), sizeof(eskf_t), sizeof(eskf_real))
int eskf_init_checked(eskf_t *f, const eskf_params_t *prm, size_t sizeof_filter, size_t sizeof_real);

/* Comprueba los parámetros sin inicializar nada. */
int eskf_validate_params(const eskf_params_t *prm);

/* El llamador (adaptador de adquisición) confirma o retira el reposo. El reposo debe
   demostrarse sobre una ventana de muestras (p. ej. varianza de acelerómetro y giróscopo
   por debajo de un umbral durante N s) y retirarse en cuanto se detecte el despegue.
   Para el ensayo del 7-nov: no volver a ponerlo a 1 durante el vuelo.
   Requieren at_rest = 1: eskf_align_static, eskf_update_zupt, eskf_update_accel. */
int eskf_set_rest(eskf_t *f, int at_rest);

/* Alineación estática (sección 5 del PDF). Reinicia por completo la estimación:
   p = v = 0, b_a = 0, b_g = gyro_mean, q de gravedad (+ magnetómetro), P diagonal de *_std
   (se descartan las correlaciones anteriores) y borra el fallo y los bits degraded. Se puede volver a llamar
   en rampa para realinear. Requisitos:
     - at_rest = 1 (el llamador ha demostrado reposo durante la ventana de las medias);
     - vectores finitos; |‖acc_mean‖ − 9.81| < align_acc_tol; ‖gyro_mean‖ <= align_gyro_max;
     - mag_mean != NULL exige has_mag_ref (para pasar del norte magnético al geográfico).
   Con mag_mean: heading_known = 1 (N = norte geográfico).
   Sin mag_mean: heading_known = 0. El eje N del marco es la proyección horizontal de un eje
   de cuerpo (el más horizontal; a igualdad, el de menor índice): un marco LOCAL de
   lanzamiento con acimut arbitrario, no un rumbo geográfico. GPS, velocidad externa y
   magnetómetro se rechazan con ESKF_E_NO_HEADING en ese marco.
   Si falla, el filtro no se modifica. */
int eskf_align_static(eskf_t *f, const eskf_real acc_mean[3], const eskf_real gyro_mean[3],
                      const eskf_real *mag_mean);

/* ---------------- Predicción ---------------- */

/* Una muestra de IMU (equivale a ESKF._predict). Contrato temporal:
     - dt finito y > 0, si no ESKF_E_DT;
     - dt <= dt_max: paso normal;
     - dt_max < dt <= gap_max: se puentea con ceil(dt/dt_max) subpasos con esta misma
       muestra (retención de orden cero), se marca ESKF_DEG_GAP_BRIDGED y devuelve ESKF_OK.
       El ruido blanco de la lectura repetida se trata como UN solo error durante todo dt
       (no como nsub lecturas independientes); el cambio real de aceleración y giro durante
       el hueco NO se modela;
     - en otro caso ESKF_E_DT_GAP, sin modificar el estado, y se marca ESKF_DEG_GAP_LOST.
   dt debe salir de las marcas de tiempo de la IMU, no de un valor fijo. */
int eskf_predict(eskf_t *f, const eskf_real a_meas[3], const eskf_real w_meas[3], eskf_real dt);

/* Añade varianza std[i]² a la diagonal de un bloque de 3 del estado de error
   (block = ESKF_IDX_P, _V, _TH, _BA o _BG). Para que el llamador declare información
   perdida que el filtro no puede ver: tiempo perdido tras ESKF_E_DT_GAP, muestras
   saturadas, etc. Sin esto, la puerta de innovación puede rechazar indefinidamente las
   medidas tras un error grande (bloqueo de la puerta). std finito y >= 0. */
int eskf_inflate_block(eskf_t *f, int block, const eskf_real std[3]);

/* ---------------- Actualizaciones ----------------
   Devuelven el nº de componentes aplicadas (> 0) o un código < 0. Las sigma son
   desviaciones típicas de medida: finitas y > 0. Diagnóstico detallado en f->last. */

/* GPS: posición y velocidad en NED (ya convertidas desde LLA). Requiere heading_known. */
int eskf_update_gps(eskf_t *f, const eskf_real p_meas[3], const eskf_real v_meas[3],
                    const eskf_real sigma_p[3], const eskf_real sigma_v[3]);

/* Velocidad externa en NED. Requiere heading_known. Para v = 0 en reposo usar eskf_update_zupt. */
int eskf_update_velocity(eskf_t *f, const eskf_real v_meas[3], const eskf_real sigma_v[3]);

/* ZUPT: v = 0. Requiere at_rest (no depende del rumbo). */
int eskf_update_zupt(eskf_t *f, const eskf_real sigma_v[3]);

/* Magnetómetro calibrado en ejes cuerpo. Requiere has_mag_ref y heading_known. */
int eskf_update_mag(eskf_t *f, const eskf_real m_meas[3], const eskf_real sigma_mag[3]);

/* Acelerómetro como referencia de gravedad: requiere at_rest y además
   |‖a‖ − 9.81| < accel_threshold (si no, ESKF_E_MOTION). El umbral de norma por sí solo
   NO demuestra ausencia de aceleración lineal. */
int eskf_update_accel(eskf_t *f, const eskf_real a_meas[3], const eskf_real sigma_a[3]);

/* Barómetro: z NED (m, positivo hacia abajo) = −(altitud − altitud_ref).
   Supone que la medida corresponde al instante del estado (la última muestra de IMU). */
int eskf_update_baro(eskf_t *f, eskf_real z_meas, eskf_real sigma_baro);

/* Barómetro tomado en t_meas = t_estado + tau (tau en s, puede ser negativo).
   Modelo h = p_z + v_z·tau (velocidad constante durante tau), con su jacobiano.
   Requiere |tau| <= tau_max (o dt_max si tau_max = 0). t_meas es el instante al que
   corresponde la presión, no el de lectura: incluye la latencia de conversión y filtrado
   del sensor, que hay que medir. Error del modelo: ≤ ½·a_max·tau², que debe ser pequeño
   frente a sigma_baro (p. ej. 120 m/s² y 50 ms → 0.15 m). A 300 m/s, un desfase de 3 ms sin compensar son ~1 m de error,
   del orden del ruido del baro, y el filtro lo atribuye a actitud y sesgos.
   eskf_update_baro(f, z, s) equivale a tau = 0. */
int eskf_update_baro_at(eskf_t *f, eskf_real z_meas, eskf_real sigma_baro, eskf_real tau);

/* ---------------- Utilidades ---------------- */

/* Comprobación NUMÉRICA del núcleo: ESKF_E_FAULT si hay un fallo enclavado; si no,
   ESKF_OK cuando el estado es finito, ‖q‖ ≈ 1 y P es simétrica con diagonal >= 0.
   Es necesaria pero NO suficiente para declarar navegación válida: no sabe si el filtro
   está alineado, si el estado está al día ni si se han perdido datos. La validez
   operativa la publica navigation.h (nav_status). No demuestra que P sea semidefinida
   positiva (eso se comprueba en las pruebas, en el ordenador). */
int eskf_check_health(const eskf_t *f);

const char *eskf_status_str(int status);

void eskf_quat_to_euler(const eskf_real q[4], eskf_real rpy[3]); /* ZYX: roll, pitch, yaw */
void eskf_rotation_matrix(const eskf_real q[4], eskf_real R[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* ESKF_H */
