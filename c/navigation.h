/*
 * navigation.h — sesión de navegación: une la adquisición con el núcleo ESKF.
 *
 * Responsabilidades (lo que el núcleo eskf.c NO hace):
 *   - contrato temporal: orden de muestras, primera muestra, instante del estado;
 *   - política de huecos: un hueco no recuperable invalida la estimación;
 *   - reposo en rampa por ventana, alineación y referencia barométrica;
 *   - detección de despegue con persistencia;
 *   - saturación de acelerómetro y giróscopo por eje;
 *   - estado publicado: VÁLIDO, DEGRADADO o NO VÁLIDO, con sus motivos.
 *
 * Fuera de este módulo: la adquisición (drivers, marcas de tiempo) y el REGISTRO.
 * El registro debe guardar cada muestra cruda ANTES de llamar a nav_on_*, y después
 * nav_output_t, sea cual sea el estado del estimador. Nada aquí bloquea ni depende
 * del registro.
 *
 * Sensores: IMU + barómetro (la configuración mínima). El magnetómetro y el GPS están
 * en el núcleo, pero no se usan aquí hasta confirmar la sensórica.
 *
 * Memoria: todo en nav_t (sin asignación dinámica). C99.
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <math.h>
#include <stdint.h>

#include "eskf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Muestras (formato fijo, independiente de eskf_real) ---------------- */

/* Bits de saturación por eje. El driver los pone si el sensor lo indica; además el
   módulo los calcula si acc_sat_limit / gyro_sat_limit > 0. */
#define NAV_SAT_ACC_X 0x01u
#define NAV_SAT_ACC_Y 0x02u
#define NAV_SAT_ACC_Z 0x04u
#define NAV_SAT_GYRO_X 0x08u
#define NAV_SAT_GYRO_Y 0x10u
#define NAV_SAT_GYRO_Z 0x20u
#define NAV_SAT_ACC_MASK 0x07u
#define NAV_SAT_GYRO_MASK 0x38u

typedef struct {
    double t;      /* s. Instante de la medida, estrictamente creciente. Debe salir de un
                      contador entero desenrollado (sin desbordamiento) de la adquisición. */
    float acc[3];  /* fuerza específica en ejes cuerpo (m/s²) */
    float gyro[3]; /* velocidad angular en ejes cuerpo (rad/s) */
    uint8_t sat;   /* bits NAV_SAT_* que indica el driver; 0 si no lo sabe */
} nav_imu_sample_t;

typedef struct {
    double t;    /* s. Instante de LECTURA, en la misma base de tiempo que la IMU */
    float alt_m; /* altitud barométrica (m), ya convertida desde presión */
} nav_baro_sample_t;

/* ---------------- Configuración ---------------- */

/* Valor de los campos pendientes de datos reales en nav_default_config(). nav_init()
   rechaza la configuración mientras quede alguno (ver nav_config_check). */
#define NAV_PENDING ((double)NAN)

typedef struct {
    /* Núcleo. PENDIENTES: eskf.sigma_a, sigma_w, sigma_aw, sigma_ww (Allan variance),
       eskf.dt_max (periodo real de la IMU), eskf.ba0_std, bg0_std (hoja de datos).
       eskf.gap_max = 0 (sin puente) en la configuración de primera integración. */
    eskf_params_t eskf;

    /* Reposo en rampa. PENDIENTES: los tres umbrales (ruido, sesgo y error de escala). */
    double rest_window_s;    /* duración de la ventana de reposo (s) */
    double rest_acc_dev_max; /* |‖a‖ − 9.81| máximo de CADA muestra (m/s²) */
    double rest_acc_std_max; /* desviación típica máxima de ‖a‖ en la ventana (m/s²) */
    double rest_gyro_max;    /* ‖ω‖ máximo de CADA muestra (rad/s) */
    double rest_tilt_change_max; /* giro máximo (rad) de la dirección de la gravedad entre las
                                    dos mitades de la ventana: rechaza giros lentos que
                                    pasan los umbrales por muestra (se guardarían como sesgo) */
    double rest_update_period_s; /* periodo de ZUPT + gravedad en reposo (s); 0 = cada muestra */
    double zupt_sigma;       /* m/s */
    double grav_sigma;       /* m/s² */

    /* Despegue. PENDIENTE: liftoff_acc (curva de empuje). */
    double liftoff_acc;      /* ‖a‖ umbral (m/s²); debe superar 9.81 + rest_acc_dev_max */
    double liftoff_hold_s;   /* tiempo CONTINUO por encima del umbral (s) */

    /* Saturación. PENDIENTES: los dos límites (0 = sólo los bits del driver). */
    double acc_sat_limit;    /* |a_i| >= límite → eje saturado (m/s²) */
    double gyro_sat_limit;   /* |ω_i| >= límite → eje saturado (rad/s) */
    double acc_sat_max_s;    /* tiempo saturado acumulado admitido; superado → NO VÁLIDO */
    double gyro_sat_max_s;   /* idem; 0 = cualquier saturación invalida (por defecto) */
    double acc_sat_excess;   /* cota de la aceleración no medida (m/s²); necesaria si max_s > 0 */
    double gyro_sat_excess;  /* cota de la velocidad angular no medida (rad/s); idem */

    /* Barómetro. PENDIENTES: sigma y latencia (medir). */
    double baro_sigma;       /* m */
    double baro_latency_s;   /* t_medida = t_lectura − latencia (s), >= 0 */
    double baro_timeout_s;   /* sin baro aplicado durante más tiempo en vuelo → DEGRADADO; 0 = no */
    double baro_ref_min_n;   /* barómetros válidos mínimos (tras quitar atípicos) para fijar o
                                actualizar la referencia de altura en una ventana de reposo */
    double baro_ref_max_step; /* cambio máximo (m) de la referencia entre ventanas en rampa sin
                                 realinear; mayor → no se cambia y se cuenta */
    double stale_max_s;      /* atraso máximo del estado frente al reloj (nav_tick); mayor → STALE.
                                Debe cubrir la espera con la que el firmware ordena los eventos */
} nav_config_t;

/* Las funciones que reciben nav_config_t o nav_t comprueban, ANTES de tocar memoria, que el
   tamaño de las estructuras del llamador coincide con el de navigation.c (float/double
   coherente). Por eso son macros que pasan sizeof. */

/* Rellena la configuración: políticas con valor provisional y datos de sensores como
   NAV_PENDING. eskf.gap_max = 0 e innov_gate = 5. NAV_E_CONFIG si los tamaños no cuadran. */
#define nav_default_config(cfg) nav_default_config_checked((cfg), sizeof(nav_config_t))
int nav_default_config_checked(nav_config_t *cfg, size_t sizeof_cfg);

/* NULL si la configuración es completa y coherente; si no, el nombre del primer campo
   pendiente o inválido (para mostrarlo o registrarlo). */
#define nav_config_check(cfg) nav_config_check_checked((cfg), sizeof(nav_config_t))
const char *nav_config_check_checked(const nav_config_t *cfg, size_t sizeof_cfg);

/* ---------------- Estado publicado ---------------- */

typedef enum {
    NAV_INVALID = 0,  /* no usar la estimación */
    NAV_DEGRADED = 1, /* utilizable con cautela: se ha perdido información y P lo refleja */
    NAV_VALID = 2
} nav_validity_t;

/* Motivos (bits). Los de NAV_R_INVALID_MASK hacen NO VÁLIDO; los de NAV_R_DEGRADED_MASK,
   DEGRADADO. Los enclavados sólo se borran al realinear (en rampa) o con nav_init. */
#define NAV_R_NOT_INIT 0x0001u        /* nav_init no completado */
#define NAV_R_NOT_ALIGNED 0x0002u     /* aún no hay alineación */
#define NAV_R_CORE_FAULT 0x0004u      /* fallo numérico enclavado en el núcleo */
#define NAV_R_TIME_GAP 0x0008u        /* hueco no recuperable (enclavado) */
#define NAV_R_STALE 0x0010u           /* estado atrasado: > dt_max frente a la última muestra o > stale_max_s frente al reloj */
#define NAV_R_ACC_SAT_LIMIT 0x0020u   /* saturación del acelerómetro por encima de lo admitido */
#define NAV_R_GYRO_SAT_LIMIT 0x0040u  /* idem giróscopo */
#define NAV_R_INFLATE_FAIL 0x0080u    /* no se pudo declarar información perdida */
#define NAV_R_INTERNAL 0x0100u        /* el núcleo rechazó una operación que debía aceptar */
#define NAV_R_ACC_SAT 0x1000u         /* hubo saturación del acelerómetro (enclavado) */
#define NAV_R_GYRO_SAT 0x2000u        /* hubo saturación del giróscopo (enclavado) */
#define NAV_R_GAP_BRIDGED 0x4000u     /* se puenteó algún hueco (enclavado) */
#define NAV_R_BARO_TIMEOUT 0x8000u    /* en vuelo, sin baro aplicado durante baro_timeout_s */
#define NAV_R_INVALID_MASK 0x0FFFu
#define NAV_R_DEGRADED_MASK 0xF000u

typedef enum { NAV_PHASE_PAD = 0, NAV_PHASE_FLIGHT = 1 } nav_phase_t;

/* Registro de salida: formato fijo en float, sea cual sea eskf_real. Antes de alinear,
   p, v, q y sigmas son NaN. Tras una invalidación se publica el último estado congelado
   con su t_state real: NUNCA etiquetado con un instante posterior. */
typedef struct {
    double t_state;   /* instante que representa el estado (s); NaN si no hay */
    double t_rx;      /* última IMU aceptada (s); NaN si no hay */
    uint32_t reasons; /* bits NAV_R_* */
    uint8_t validity; /* nav_validity_t */
    uint8_t phase;    /* nav_phase_t */
    uint8_t at_rest;
    uint8_t heading_known; /* 0: marco local de lanzamiento, acimut arbitrario */
    float p[3], v[3], q[4];
    float sigma_p[3], sigma_v[3], sigma_th[3];
    double t_eval;        /* instante con el que se evaluó la validez (máx. de IMU, baro y reloj) */
    double alt_ref;       /* referencia de altura vigente (m); NaN si no hay */
    uint32_t alt_ref_seq; /* nº de veces que ha cambiado la referencia */
    float b_a[3], b_g[3]; /* sesgos estimados (para analizar sesgos) */
} nav_output_t;

/* ---------------- Códigos de retorno ----------------
   nav_on_imu / nav_on_baro devuelven: 0 (o > 0: componentes aplicadas por el núcleo) si
   la muestra se procesó, o un código < 0. Los códigos ESKF_E_* del núcleo (−1…−14) se
   devuelven tal cual; los propios del módulo van de −100 en adelante. */
#define NAV_OK 0
#define NAV_E_NULL (-100)
#define NAV_E_NOT_INIT (-101)
#define NAV_E_CONFIG (-102)
#define NAV_E_TIME_NONFINITE (-103) /* marca de tiempo NaN/Inf: muestra descartada */
#define NAV_E_TIME_DUPLICATE (-104) /* igual a la anterior aceptada: descartada */
#define NAV_E_TIME_ORDER (-105)     /* anterior a la última aceptada: descartada */
#define NAV_E_VALUE (-106)          /* valor no finito: descartada (el tiempo sí cuenta) */
#define NAV_E_INVALID (-107)        /* estimación no válida: muestra no aplicada */
#define NAV_E_GAP (-108)            /* esta muestra reveló un hueco no recuperable */
#define NAV_E_NO_ALT_REF (-109)     /* baro sin referencia de altura todavía */
#define NAV_E_STALE (-110)          /* baro sobre un estado atrasado: no aplicado */
#define NAV_E_BARO_TAU (-111)       /* |τ| del baro fuera del límite: no aplicado */
#define NAV_E_PAD (-112)            /* baro antes de alinear: sólo para la referencia */
#define NAV_E_TIME_FUTURE (-113)    /* marca posterior al reloj (nav_tick): descartada sin tocar el orden */

/* Tolerancia (s) para considerar futura una marca respecto al último nav_tick */
#define NAV_FUTURE_TOL_S 1e-3

/* Barómetros guardados por ventana para la referencia robusta (mediana / MAD) */
#define NAV_REF_MAX 512

typedef struct {
    uint32_t imu_rx, imu_applied, imu_time_nonfinite, imu_time_dup, imu_time_order;
    uint32_t imu_value_invalid, imu_not_applied, imu_acc_sat, imu_gyro_sat;
    uint32_t baro_rx, baro_used, baro_gated, baro_rejected, time_future;
    uint32_t ref_updates, ref_rejected, ref_outliers, ref_overflow, rest_windows_tilt, rest_breaks_gap;
    uint32_t zupt_used, zupt_rejected, grav_used, grav_rejected, inflate_fail;
    uint32_t rest_windows_ok, rest_windows_bad, align_ok, align_fail, gaps;
} nav_counters_t;

typedef struct { /* último código devuelto por cada operación (para el registro) */
    int imu, predict, baro, zupt, grav, inflate, align, ref;
} nav_last_t;

typedef struct {
    /* PRIMER campo: su desplazamiento no depende de eskf_real (ver nav_init) */
    uint32_t magic;
    nav_config_t cfg;
    eskf_t f;

    /* tiempo */
    int have_rx;    /* hay al menos una IMU aceptada en orden */
    double t_rx;    /* última IMU aceptada (incluidas las de valor inválido) */
    int have_valid; /* hay al menos una IMU con valores válidos */
    double t_valid; /* última IMU con valores válidos: continuidad de ventanas y despegue */
    int have_state; /* el estado representa un instante conocido (tras alinear) */
    double t_state; /* instante representado por el estado */
    int have_baro_rx;
    double t_baro_rx;
    int have_now;   /* hay reloj (nav_tick) */
    double t_now;

    /* fase, reposo y alineación */
    int phase;
    int aligned;
    double t_liftoff; /* inicio del tramo de empuje confirmado; NaN si no */
    int lift_armed;
    double lift_t0;
    double t_last_rest_upd;

    /* ventana de reposo */
    uint32_t win_n, win_h1_n, win_h2_n;
    double win_t0, win_a[3], win_w[3], win_an, win_an2, win_h1[3], win_h2[3];
    uint32_t win_alt_n;
    float win_alt[NAV_REF_MAX]; /* barómetros válidos de la ventana */
    int have_alt_ref;
    double alt_ref;
    uint32_t alt_ref_seq;

    /* saturación */
    double acc_sat_s, gyro_sat_s[3];

    /* validez */
    uint32_t latched;    /* motivos enclavados */
    int blocked;         /* la estimación está bloqueada (latched inválido o fallo) */
    double t_blocked;    /* instante en que se bloqueó: la recuperación usa ventanas posteriores */
    double t_last_baro_used;

    nav_counters_t cnt;
    nav_last_t last;
} nav_t;

/* Reinicia TODA la sesión (núcleo, tiempo, fase, ventanas, referencias, contadores).
   Si la configuración no es válida, o los tamaños no cuadran (float/double incoherente),
   devuelve NAV_E_CONFIG; en el segundo caso sin escribir nada en *n. */
#define nav_init(n, cfg) nav_init_checked((n), (cfg), sizeof(nav_t), sizeof(nav_config_t))
int nav_init_checked(nav_t *n, const nav_config_t *cfg, size_t sizeof_nav, size_t sizeof_cfg);

/* Reloj actual (s, misma base que las muestras). Llamarlo periódicamente aunque no lleguen
   muestras: así la validez caduca (STALE) cuando dejan de llegar IMU y baro, y las marcas
   futuras se rechazan sin estropear el orden. Un reloj que retrocede se ignora
   (NAV_E_TIME_ORDER). */
int nav_tick(nav_t *n, double t_now);

int nav_on_imu(nav_t *n, const nav_imu_sample_t *s);
int nav_on_baro(nav_t *n, const nav_baro_sample_t *s);

/* Validez actual y motivos (combina los enclavados con los calculados: no alineado,
   fallo del núcleo, estado atrasado, timeout de baro). */
nav_validity_t nav_validity(const nav_t *n, uint32_t *reasons);

/* Rellena el registro de salida (siempre: también si no es válido). */
void nav_get_output(const nav_t *n, nav_output_t *out);

const char *nav_status_str(int code);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H */
