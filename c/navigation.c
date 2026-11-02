/*
 * navigation.c — sesión de navegación (ver navigation.h).
 *
 * Política de tiempo
 *   - t_rx: última IMU ACEPTADA (marca finita, no futura y estrictamente mayor que la
 *     anterior). Las marcas no finitas, repetidas, atrasadas o futuras se descartan sin tocar
 *     nada más (tampoco el orden).
 *   - t_valid: última IMU con valores válidos. Las ventanas de reposo y la persistencia del
 *     despegue exigen continuidad: dos muestras válidas consecutivas separadas más de dt_max,
 *     o una muestra con valores no finitos entre ellas, reinician la ventana y el despegue.
 *   - t_state: instante que representa el estado del filtro. Sólo avanza cuando
 *     eskf_predict() se aplica.
 *   - La primera muestra sólo fija t_rx. t_state se establece al alinear: es el instante
 *     de la última muestra de la ventana de reposo.
 *   - Hueco no recuperable (eskf_predict → ESKF_E_DT_GAP): la estimación queda NO VÁLIDA
 *     (NAV_R_TIME_GAP enclavado), el estado no se mueve y conserva su t_state real, y no
 *     se aplica ninguna medida más. Cualquier bloqueo corta el reposo y descarta la ventana
 *     en curso. Salida: en rampa, una ventana de reposo completa POSTERIOR al bloqueo
 *     realinea; en vuelo, sólo nav_init(). El registro sigue igual en todos los casos.
 *   - nav_tick(t_now) da el reloj: la validez caduca (STALE) aunque no lleguen muestras,
 *     cuando el estado va más de stale_max_s por detrás del reloj.
 */
#include "navigation.h"

#include <math.h>
#include <string.h>

#define NAV_MAGIC 0x4E415634u /* "NAV4" */
#define G0 9.81

static int isfin(double x) { return isfinite(x) ? 1 : 0; }

static double norm3d(const double v[3]) { return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

/* ------------------------------------------------------------------------- */
/* Configuración                                                              */
/* ------------------------------------------------------------------------- */

int nav_default_config_checked(nav_config_t *c, size_t sizeof_cfg)
{
    if (sizeof_cfg != sizeof(nav_config_t)) return NAV_E_CONFIG; /* antes de tocar *c */
    if (c == NULL) return NAV_E_NULL;
    memset(c, 0, sizeof *c);
    if (eskf_default_params(&c->eskf) != ESKF_OK) return NAV_E_CONFIG;
    /* Datos de sensores: pendientes */
    c->eskf.sigma_a = (eskf_real)NAV_PENDING;
    c->eskf.sigma_w = (eskf_real)NAV_PENDING;
    c->eskf.sigma_aw = (eskf_real)NAV_PENDING;
    c->eskf.sigma_ww = (eskf_real)NAV_PENDING;
    c->eskf.dt_max = (eskf_real)NAV_PENDING;
    c->eskf.ba0_std = (eskf_real)NAV_PENDING;
    c->eskf.bg0_std = (eskf_real)NAV_PENDING;
    c->eskf.accel_threshold = (eskf_real)NAV_PENDING;
    c->eskf.align_acc_tol = (eskf_real)NAV_PENDING;
    c->eskf.align_gyro_max = (eskf_real)NAV_PENDING;
    /* Políticas de primera integración */
    c->eskf.gap_max = (eskf_real)0.0; /* sin puente */
    c->eskf.innov_gate = (eskf_real)5.0;
    c->eskf.tau_max = (eskf_real)0.0;

    c->rest_window_s = 5.0;
    c->rest_acc_dev_max = NAV_PENDING;
    c->rest_acc_std_max = NAV_PENDING;
    c->rest_gyro_max = NAV_PENDING;
    c->rest_tilt_change_max = NAV_PENDING;
    c->rest_update_period_s = 0.05;
    c->zupt_sigma = 0.02;
    c->grav_sigma = 0.3;

    c->liftoff_acc = NAV_PENDING;
    c->liftoff_hold_s = 0.05;

    c->acc_sat_limit = NAV_PENDING;
    c->gyro_sat_limit = NAV_PENDING;
    c->acc_sat_max_s = 0.0; /* cualquier saturación invalida */
    c->gyro_sat_max_s = 0.0;
    c->acc_sat_excess = 0.0;
    c->gyro_sat_excess = 0.0;

    c->baro_sigma = NAV_PENDING;
    c->baro_latency_s = NAV_PENDING;
    c->baro_timeout_s = 1.0;
    c->baro_ref_min_n = 50.0;
    c->baro_ref_max_step = 2.0;
    c->stale_max_s = 0.05;
    return NAV_OK;
}

#define CHK(cond, name)            \
    do {                           \
        if (!(cond)) return name;  \
    } while (0)

const char *nav_config_check_checked(const nav_config_t *c, size_t sizeof_cfg)
{
    if (sizeof_cfg != sizeof(nav_config_t)) return "tamaño de nav_config_t (float/double incoherente)";
    if (c == NULL) return "cfg (NULL)";
    const eskf_params_t *e = &c->eskf;
    CHK(isfin((double)e->sigma_a), "eskf.sigma_a");
    CHK(isfin((double)e->sigma_w), "eskf.sigma_w");
    CHK(isfin((double)e->sigma_aw), "eskf.sigma_aw");
    CHK(isfin((double)e->sigma_ww), "eskf.sigma_ww");
    CHK(isfin((double)e->dt_max), "eskf.dt_max");
    CHK(isfin((double)e->ba0_std), "eskf.ba0_std");
    CHK(isfin((double)e->bg0_std), "eskf.bg0_std");
    CHK(isfin((double)e->accel_threshold), "eskf.accel_threshold");
    CHK(isfin((double)e->align_acc_tol), "eskf.align_acc_tol");
    CHK(isfin((double)e->align_gyro_max), "eskf.align_gyro_max");
    CHK(eskf_validate_params(e) == ESKF_OK, "eskf (eskf_validate_params)");

    CHK(isfin(c->rest_window_s) && c->rest_window_s > 0.0, "rest_window_s");
    CHK(isfin(c->rest_acc_dev_max) && c->rest_acc_dev_max > 0.0, "rest_acc_dev_max");
    CHK(isfin(c->rest_acc_std_max) && c->rest_acc_std_max > 0.0, "rest_acc_std_max");
    CHK(isfin(c->rest_gyro_max) && c->rest_gyro_max > 0.0, "rest_gyro_max");
    CHK(isfin(c->rest_tilt_change_max) && c->rest_tilt_change_max > 0.0, "rest_tilt_change_max");
    CHK(isfin(c->rest_update_period_s) && c->rest_update_period_s >= 0.0, "rest_update_period_s");
    CHK(isfin(c->zupt_sigma) && c->zupt_sigma > 0.0, "zupt_sigma");
    CHK(isfin(c->grav_sigma) && c->grav_sigma > 0.0, "grav_sigma");

    CHK(isfin(c->liftoff_acc), "liftoff_acc");
    CHK(c->liftoff_acc > G0 + c->rest_acc_dev_max, "liftoff_acc (debe superar 9.81 + rest_acc_dev_max)");
    CHK(isfin(c->liftoff_hold_s) && c->liftoff_hold_s >= 0.0, "liftoff_hold_s");

    CHK(isfin(c->acc_sat_limit) && c->acc_sat_limit >= 0.0, "acc_sat_limit");
    CHK(isfin(c->gyro_sat_limit) && c->gyro_sat_limit >= 0.0, "gyro_sat_limit");
    CHK(isfin(c->acc_sat_max_s) && c->acc_sat_max_s >= 0.0, "acc_sat_max_s");
    CHK(isfin(c->gyro_sat_max_s) && c->gyro_sat_max_s >= 0.0, "gyro_sat_max_s");
    CHK(isfin(c->acc_sat_excess) && c->acc_sat_excess >= 0.0, "acc_sat_excess");
    CHK(isfin(c->gyro_sat_excess) && c->gyro_sat_excess >= 0.0, "gyro_sat_excess");
    CHK(c->acc_sat_max_s == 0.0 || c->acc_sat_excess > 0.0, "acc_sat_excess (necesario si acc_sat_max_s > 0)");
    CHK(c->gyro_sat_max_s == 0.0 || c->gyro_sat_excess > 0.0, "gyro_sat_excess (necesario si gyro_sat_max_s > 0)");

    CHK(isfin(c->baro_sigma) && c->baro_sigma > 0.0, "baro_sigma");
    CHK(isfin(c->baro_latency_s) && c->baro_latency_s >= 0.0, "baro_latency_s");
    CHK(isfin(c->baro_timeout_s) && c->baro_timeout_s >= 0.0, "baro_timeout_s");
    CHK(isfin(c->baro_ref_min_n) && c->baro_ref_min_n >= 1.0 && c->baro_ref_min_n <= (double)NAV_REF_MAX,
        "baro_ref_min_n (1..NAV_REF_MAX)");
    CHK(isfin(c->baro_ref_max_step) && c->baro_ref_max_step > 0.0, "baro_ref_max_step");
    CHK(isfin(c->stale_max_s) && c->stale_max_s >= (double)e->dt_max, "stale_max_s (>= dt_max)");
    double tau_lim = (e->tau_max > (eskf_real)0.0) ? (double)e->tau_max : (double)e->dt_max;
    CHK(c->baro_latency_s + (double)e->dt_max <= tau_lim * (1.0 + 1e-6),
        "eskf.tau_max (debe cubrir baro_latency_s + dt_max)");
    return NULL;
}

/* ------------------------------------------------------------------------- */
/* Sesión                                                                     */
/* ------------------------------------------------------------------------- */

static void win_reset(nav_t *n)
{
    n->win_n = n->win_h1_n = n->win_h2_n = 0u;
    n->win_alt_n = 0u;
    n->win_t0 = 0.0;
    n->win_an = n->win_an2 = 0.0;
    for (int i = 0; i < 3; i++) n->win_a[i] = n->win_w[i] = n->win_h1[i] = n->win_h2[i] = 0.0;
}

/* Se pierde el reposo: se descarta la ventana en curso (y sus barómetros) y el núcleo deja
   de aceptar ZUPT y gravedad. */
static void rest_break(nav_t *n)
{
    win_reset(n);
    (void)eskf_set_rest(&n->f, 0);
}

int nav_init_checked(nav_t *n, const nav_config_t *cfg, size_t sizeof_nav, size_t sizeof_cfg)
{
    /* Tamaños antes de tocar *n o *cfg: con float/double distinto entre unidades de
       compilación, el llamador tiene estructuras de otro tamaño. */
    if (sizeof_nav != sizeof(nav_t) || sizeof_cfg != sizeof(nav_config_t)) return NAV_E_CONFIG;
    if (n == NULL || cfg == NULL) return NAV_E_NULL;
    nav_config_t c = *cfg; /* cfg puede ser &n->cfg */
    memset(n, 0, sizeof *n);
    if (nav_config_check(&c) != NULL) return NAV_E_CONFIG;
    if (eskf_init(&n->f, &c.eskf) != ESKF_OK) return NAV_E_CONFIG;
    n->cfg = c;
    n->t_rx = n->t_valid = n->t_state = n->t_baro_rx = n->t_now = (double)NAN;
    n->t_liftoff = n->lift_t0 = n->t_last_rest_upd = n->t_last_baro_used = (double)NAN;
    n->alt_ref = n->t_blocked = (double)NAN;
    n->phase = NAV_PHASE_PAD;
    win_reset(n);
    n->magic = NAV_MAGIC;
    return NAV_OK;
}

static int estimation_blocked(const nav_t *n)
{
    return (n->latched & NAV_R_INVALID_MASK) != 0u || n->f.fault;
}

/* Instante más reciente conocido: IMU, baro o reloj */
static double latest_time(const nav_t *n)
{
    double tl = n->have_rx ? n->t_rx : -(double)INFINITY;
    if (n->have_baro_rx && n->t_baro_rx > tl) tl = n->t_baro_rx;
    if (n->have_now && n->t_now > tl) tl = n->t_now;
    return tl;
}

/* Al bloquearse la estimación (por cualquier motivo) se corta el reposo y se descarta la
   ventana en curso: la recuperación sólo puede usar datos posteriores al bloqueo. */
static void note_blocking(nav_t *n)
{
    if (!n->blocked && n->aligned && estimation_blocked(n)) {
        n->blocked = 1;
        n->t_blocked = latest_time(n);
        rest_break(n);
    }
}

static int is_future(const nav_t *n, double t)
{
    return n->have_now && t > n->t_now + NAV_FUTURE_TOL_S;
}

/* ---- referencia barométrica robusta ---- */

static void sort_floats(float *v, uint32_t n)
{
    for (uint32_t i = 1; i < n; i++) { /* inserción: n <= NAV_REF_MAX, una vez por ventana */
        float x = v[i];
        uint32_t j = i;
        while (j > 0 && v[j - 1] > x) {
            v[j] = v[j - 1];
            j--;
        }
        v[j] = x;
    }
}

static double median_sorted(const float *v, uint32_t n)
{
    return (n % 2u) ? (double)v[n / 2u] : 0.5 * ((double)v[n / 2u - 1u] + (double)v[n / 2u]);
}

/* Referencia = media de los barómetros de la ventana sin atípicos (|x − mediana| > 5·σ_MAD,
   σ_MAD = 1.4826·MAD, mínimo 1 cm). Devuelve 1 y *ref si hay al menos baro_ref_min_n. */
static int robust_ref(nav_t *n, double *ref)
{
    uint32_t m = n->win_alt_n;
    if (m == 0u || (double)m < n->cfg.baro_ref_min_n) return 0;
    float *v = n->win_alt;
    sort_floats(v, m);
    double med = median_sorted(v, m);
    /* MAD = mediana de |x − med|, por bisección sobre el valor (sin memoria extra) */
    double lo = 0.0, hi = fabs((double)v[m - 1u] - med) + fabs((double)v[0] - med) + 1e-6;
    for (int it = 0; it < 40; it++) {
        double mid = 0.5 * (lo + hi);
        uint32_t below = 0u;
        for (uint32_t i = 0; i < m; i++)
            if (fabs((double)v[i] - med) <= mid) below++;
        if (2u * below >= m) hi = mid;
        else lo = mid;
    }
    double sig = 1.4826 * hi;
    if (sig < 0.01) sig = 0.01;
    double sum = 0.0;
    uint32_t k = 0u;
    for (uint32_t i = 0; i < m; i++) {
        if (fabs((double)v[i] - med) <= 5.0 * sig) {
            sum += (double)v[i];
            k++;
        }
    }
    n->cnt.ref_outliers += m - k;
    if ((double)k < n->cfg.baro_ref_min_n) return 0;
    *ref = sum / (double)k;
    return 1;
}

/* Ventana de reposo completa y buena. Devuelve 1 si en esta muestra se ha (re)alineado. */
static int rest_window_good(nav_t *n, double t)
{
    n->cnt.rest_windows_ok++;
    int realign = !n->aligned || estimation_blocked(n);
    if (realign && n->blocked && !(n->win_t0 > n->t_blocked)) {
        /* defensa: nunca recuperar con una ventana que empezó antes del bloqueo */
        return 0;
    }
    (void)eskf_set_rest(&n->f, 1);

    double ref;
    int have_new_ref = robust_ref(n, &ref);
    if (!realign) {
        /* referencia al día durante la espera (deriva meteorológica), con salto limitado */
        if (have_new_ref) {
            if (!n->have_alt_ref || fabs(ref - n->alt_ref) <= n->cfg.baro_ref_max_step) {
                if (!n->have_alt_ref || ref != n->alt_ref) n->alt_ref_seq++;
                n->alt_ref = ref;
                n->have_alt_ref = 1;
                n->cnt.ref_updates++;
                n->last.ref = NAV_OK;
            } else {
                n->cnt.ref_rejected++;
                n->last.ref = NAV_E_PAD;
            }
        }
        return 0;
    }

    /* Primera alineación, o recuperación en rampa de una estimación invalidada */
    eskf_real am[3], wm[3];
    for (int i = 0; i < 3; i++) {
        am[i] = (eskf_real)(n->win_a[i] / (double)n->win_n);
        wm[i] = (eskf_real)(n->win_w[i] / (double)n->win_n);
    }
    int st = eskf_align_static(&n->f, am, wm, NULL);
    n->last.align = st;
    if (st != ESKF_OK) {
        n->cnt.align_fail++;
        (void)eskf_set_rest(&n->f, 0);
        return 0;
    }
    n->cnt.align_ok++;
    n->aligned = 1;
    n->have_state = 1;
    n->t_state = t;
    n->latched = 0u; /* estimación nueva: no hereda motivos */
    n->blocked = 0;
    n->acc_sat_s = 0.0;
    for (int i = 0; i < 3; i++) n->gyro_sat_s[i] = 0.0;
    n->t_last_rest_upd = t;
    n->t_last_baro_used = (double)NAN;
    /* nuevo marco: la referencia es la de esta ventana (sin límite de salto); si no hay
       barómetros suficientes, la anterior queda descartada */
    if (have_new_ref) {
        n->alt_ref = ref;
        n->have_alt_ref = 1;
        n->alt_ref_seq++;
        n->cnt.ref_updates++;
        n->last.ref = NAV_OK;
    } else {
        n->have_alt_ref = 0;
        n->alt_ref = (double)NAN;
        n->last.ref = NAV_E_NO_ALT_REF;
    }
    return 1;
}

static void inflate(nav_t *n, int block, const double std[3])
{
    eskf_real s3[3] = {(eskf_real)std[0], (eskf_real)std[1], (eskf_real)std[2]};
    int r = eskf_inflate_block(&n->f, block, s3);
    n->last.inflate = r;
    if (r != ESKF_OK) {
        n->cnt.inflate_fail++;
        n->latched |= NAV_R_INFLATE_FAIL;
    }
}

/* Respuesta a la saturación en el intervalo [t_state_anterior, t] de duración dt.
   La lectura recortada no se recupera: se declara la información perdida inflando P
   (error sistemático: crece linealmente con el tiempo saturado acumulado, σ = exceso·T)
   mientras el tiempo acumulado no supere el admitido; después, NO VÁLIDO. Por defecto el
   tiempo admitido es 0 (modelo deshabilitado). */
static void saturation_response(nav_t *n, unsigned sat, double dt)
{
    const nav_config_t *c = &n->cfg;
    if (sat & NAV_SAT_ACC_MASK) {
        double T0 = n->acc_sat_s, T1 = T0 + dt;
        n->acc_sat_s = T1;
        n->latched |= NAV_R_ACC_SAT;
        if (T1 > c->acc_sat_max_s) {
            n->latched |= NAV_R_ACC_SAT_LIMIT;
        } else {
            double sd = c->acc_sat_excess * sqrt(T1 * T1 - T0 * T0);
            double s3[3] = {sd, sd, sd};
            inflate(n, ESKF_IDX_V, s3);
        }
    }
    if (sat & NAV_SAT_GYRO_MASK) {
        double s3[3] = {0.0, 0.0, 0.0};
        int over = 0;
        n->latched |= NAV_R_GYRO_SAT;
        for (int i = 0; i < 3; i++) {
            if (!(sat & (NAV_SAT_GYRO_X << i))) continue;
            double T0 = n->gyro_sat_s[i], T1 = T0 + dt;
            n->gyro_sat_s[i] = T1;
            if (T1 > c->gyro_sat_max_s) over = 1;
            s3[i] = c->gyro_sat_excess * sqrt(T1 * T1 - T0 * T0);
        }
        if (over)
            n->latched |= NAV_R_GYRO_SAT_LIMIT;
        else
            inflate(n, ESKF_IDX_TH, s3);
    }
}

/* Ángulo entre dos vectores (rad), estable para ángulos pequeños */
static double angle_between(const double a[3], const double b[3])
{
    double cx = a[1] * b[2] - a[2] * b[1], cy = a[2] * b[0] - a[0] * b[2], cz = a[0] * b[1] - a[1] * b[0];
    return atan2(sqrt(cx * cx + cy * cy + cz * cz), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

static int imu_process(nav_t *n, const nav_imu_sample_t *s)
{
    const nav_config_t *c = &n->cfg;
    const double dt_max = (double)c->eskf.dt_max;

    /* 1. Tiempo: antes de modificar nada */
    if (!isfin(s->t)) {
        n->cnt.imu_time_nonfinite++;
        return NAV_E_TIME_NONFINITE;
    }
    if (is_future(n, s->t)) {
        n->cnt.time_future++;
        return NAV_E_TIME_FUTURE;
    }
    if (n->have_rx) {
        if (s->t == n->t_rx) {
            n->cnt.imu_time_dup++;
            return NAV_E_TIME_DUPLICATE;
        }
        if (s->t < n->t_rx) {
            n->cnt.imu_time_order++;
            return NAV_E_TIME_ORDER;
        }
    }
    n->have_rx = 1;
    n->t_rx = s->t;

    /* 2. Valores. Una muestra inválida rompe la continuidad: la ventana de reposo y el
          despegue no pueden atravesarla. */
    double a[3], w[3];
    for (int i = 0; i < 3; i++) {
        a[i] = (double)s->acc[i];
        w[i] = (double)s->gyro[i];
    }
    if (!isfin(a[0]) || !isfin(a[1]) || !isfin(a[2]) || !isfin(w[0]) || !isfin(w[1]) || !isfin(w[2])) {
        n->cnt.imu_value_invalid++;
        if (n->phase == NAV_PHASE_PAD) rest_break(n);
        n->lift_armed = 0;
        n->have_valid = 0; /* la siguiente válida empieza continuidad nueva */
        return NAV_E_VALUE;
    }

    /* Continuidad con la muestra válida anterior */
    int continuous = n->have_valid && (s->t - n->t_valid) <= dt_max;
    n->have_valid = 1;
    n->t_valid = s->t;
    if (!continuous) {
        if (n->phase == NAV_PHASE_PAD && (n->win_n > 0u || n->f.at_rest)) {
            n->cnt.rest_breaks_gap++;
            rest_break(n);
        }
        n->lift_armed = 0;
    }

    /* 3. Saturación por eje: bits del driver + límites configurados */
    unsigned sat = (unsigned)s->sat & (NAV_SAT_ACC_MASK | NAV_SAT_GYRO_MASK);
    for (int i = 0; i < 3; i++) {
        if (c->acc_sat_limit > 0.0 && fabs(a[i]) >= c->acc_sat_limit) sat |= NAV_SAT_ACC_X << i;
        if (c->gyro_sat_limit > 0.0 && fabs(w[i]) >= c->gyro_sat_limit) sat |= NAV_SAT_GYRO_X << i;
    }
    if (sat & NAV_SAT_ACC_MASK) n->cnt.imu_acc_sat++;
    if (sat & NAV_SAT_GYRO_MASK) n->cnt.imu_gyro_sat++;
    double an = norm3d(a), wn = norm3d(w);

    /* 4. Despegue: ‖a‖ > liftoff_acc de forma CONTINUA (sin huecos ni inválidas) */
    if (n->phase == NAV_PHASE_PAD) {
        if (an > c->liftoff_acc) {
            if (!n->lift_armed) {
                n->lift_armed = 1;
                n->lift_t0 = s->t;
            }
            if (s->t - n->lift_t0 >= c->liftoff_hold_s) {
                n->phase = NAV_PHASE_FLIGHT;
                n->t_liftoff = n->lift_t0;
                rest_break(n); /* en este ensayo no se vuelve a reposo */
            }
        } else {
            n->lift_armed = 0;
        }
    }

    /* 5. Reposo en rampa */
    int just_aligned = 0;
    if (n->phase == NAV_PHASE_PAD) {
        int inst_ok = fabs(an - G0) <= c->rest_acc_dev_max && wn <= c->rest_gyro_max && sat == 0u;
        if (!inst_ok) {
            rest_break(n);
        } else {
            if (n->win_n == 0u) n->win_t0 = s->t;
            n->win_n++;
            for (int i = 0; i < 3; i++) {
                n->win_a[i] += a[i];
                n->win_w[i] += w[i];
            }
            n->win_an += an;
            n->win_an2 += an * an;
            if (s->t - n->win_t0 < 0.5 * c->rest_window_s) {
                for (int i = 0; i < 3; i++) n->win_h1[i] += a[i];
                n->win_h1_n++;
            } else {
                for (int i = 0; i < 3; i++) n->win_h2[i] += a[i];
                n->win_h2_n++;
            }
            if (s->t - n->win_t0 >= c->rest_window_s && n->win_n >= 2u) {
                double m = n->win_an / (double)n->win_n;
                double var = n->win_an2 / (double)n->win_n - m * m;
                double tilt = 0.0;
                if (n->win_h1_n > 0u && n->win_h2_n > 0u) tilt = angle_between(n->win_h1, n->win_h2);
                if (sqrt(var > 0.0 ? var : 0.0) > c->rest_acc_std_max) {
                    n->cnt.rest_windows_bad++;
                    rest_break(n); /* variabilidad: se retira el reposo */
                } else if (!(tilt <= c->rest_tilt_change_max)) {
                    n->cnt.rest_windows_tilt++;
                    rest_break(n); /* la gravedad ha girado: no es reposo (giro lento) */
                } else {
                    just_aligned = rest_window_good(n, s->t);
                    win_reset(n); /* ventana siguiente, sin tocar el reposo confirmado */
                }
            }
        }
    }

    if (!n->aligned || just_aligned) {
        if (!n->aligned) n->cnt.imu_not_applied++;
        return NAV_OK; /* muestra usada sólo para el reposo / define el instante de alineación */
    }
    if (estimation_blocked(n)) {
        n->cnt.imu_not_applied++;
        return NAV_E_INVALID;
    }

    /* 6. Propagación desde el instante REAL del estado */
    double dt = s->t - n->t_state; /* > 0: t_state <= t_rx anterior < s->t */
    eskf_real ar[3], wr[3];
    for (int i = 0; i < 3; i++) {
        ar[i] = (eskf_real)a[i];
        wr[i] = (eskf_real)w[i];
    }
    int st = eskf_predict(&n->f, ar, wr, (eskf_real)dt);
    n->last.predict = st;
    if (st == ESKF_E_DT_GAP) {
        n->cnt.gaps++;
        n->latched |= NAV_R_TIME_GAP;
        return NAV_E_GAP;
    }
    if (st != ESKF_OK) {
        if (!n->f.fault) n->latched |= NAV_R_INTERNAL;
        return st;
    }
    n->t_state = s->t;
    n->cnt.imu_applied++;

    /* 7. Saturación */
    if (sat) saturation_response(n, sat, dt);

    /* 8. Reposo confirmado: ZUPT y gravedad */
    if (n->phase == NAV_PHASE_PAD && n->f.at_rest && !estimation_blocked(n) &&
        s->t - n->t_last_rest_upd >= c->rest_update_period_s) {
        n->t_last_rest_upd = s->t;
        eskf_real sz[3], sg[3];
        for (int i = 0; i < 3; i++) {
            sz[i] = (eskf_real)c->zupt_sigma;
            sg[i] = (eskf_real)c->grav_sigma;
        }
        int rz = eskf_update_zupt(&n->f, sz);
        n->last.zupt = rz;
        if (rz > 0) n->cnt.zupt_used++;
        else n->cnt.zupt_rejected++;
        if (!n->f.fault) {
            int rg = eskf_update_accel(&n->f, ar, sg);
            n->last.grav = rg;
            if (rg > 0) n->cnt.grav_used++;
            else n->cnt.grav_rejected++;
        }
    }
    return NAV_OK;
}

int nav_on_imu(nav_t *n, const nav_imu_sample_t *s)
{
    if (n == NULL || s == NULL) return NAV_E_NULL;
    if (n->magic != NAV_MAGIC) return NAV_E_NOT_INIT;
    n->cnt.imu_rx++;
    int r = imu_process(n, s);
    note_blocking(n);
    n->last.imu = r;
    return r;
}

static double tau_limit(const nav_t *n)
{
    const eskf_params_t *e = &n->cfg.eskf;
    return (e->tau_max > (eskf_real)0.0) ? (double)e->tau_max : (double)e->dt_max;
}

static int baro_process(nav_t *n, const nav_baro_sample_t *b)
{
    const nav_config_t *c = &n->cfg;
    if (!isfin(b->t)) return NAV_E_TIME_NONFINITE;
    /* Una marca futura se descarta SIN tocar el orden: si no, todas las posteriores
       válidas se rechazarían como atrasadas. Con reloj, futura = posterior al reloj; sin
       reloj, tras alinear, más allá de la IMU + la ventana de desfase admitida. */
    if (is_future(n, b->t) ||
        (!n->have_now && n->have_rx && b->t > n->t_rx + tau_limit(n) + c->baro_latency_s)) {
        n->cnt.time_future++;
        return NAV_E_TIME_FUTURE;
    }
    if (n->have_baro_rx) {
        if (b->t == n->t_baro_rx) return NAV_E_TIME_DUPLICATE;
        if (b->t < n->t_baro_rx) return NAV_E_TIME_ORDER;
    }
    n->have_baro_rx = 1;
    n->t_baro_rx = b->t;
    double alt = (double)b->alt_m;
    if (!isfin(alt)) return NAV_E_VALUE;

    int st;
    if (!n->aligned) st = NAV_E_PAD;
    else if (estimation_blocked(n)) st = NAV_E_INVALID;
    else if (!n->have_alt_ref) st = NAV_E_NO_ALT_REF;
    else if (n->t_rx - n->t_state > (double)c->eskf.dt_max) st = NAV_E_STALE;
    else {
        double t_meas = b->t - c->baro_latency_s;
        double tau = t_meas - n->t_state;
        if (!(fabs(tau) <= tau_limit(n))) {
            st = NAV_E_BARO_TAU;
        } else {
            double z = -(alt - n->alt_ref);
            st = eskf_update_baro_at(&n->f, (eskf_real)z, (eskf_real)c->baro_sigma, (eskf_real)tau);
            if (st > 0) n->t_last_baro_used = t_meas;
        }
    }

    /* Referencia: sólo barómetros aceptados (tiempo y valor válidos, no rechazados por la
       puerta ni por fallo numérico) durante una ventana de reposo en curso. La mediana/MAD
       de la ventana elimina además los atípicos. */
    int quality_reject = (st == ESKF_E_GATED || st == ESKF_E_NUMERIC || st == ESKF_E_FAULT);
    if (n->phase == NAV_PHASE_PAD && n->win_n > 0u && !quality_reject) {
        if (n->win_alt_n < NAV_REF_MAX) n->win_alt[n->win_alt_n++] = (float)alt;
        else n->cnt.ref_overflow++;
    }
    return st;
}

int nav_on_baro(nav_t *n, const nav_baro_sample_t *b)
{
    if (n == NULL || b == NULL) return NAV_E_NULL;
    if (n->magic != NAV_MAGIC) return NAV_E_NOT_INIT;
    n->cnt.baro_rx++;
    int r = baro_process(n, b);
    note_blocking(n);
    n->last.baro = r;
    if (r > 0) n->cnt.baro_used++;
    else if (r == ESKF_E_GATED) n->cnt.baro_gated++;
    else n->cnt.baro_rejected++;
    return r;
}

int nav_tick(nav_t *n, double t_now)
{
    if (n == NULL) return NAV_E_NULL;
    if (n->magic != NAV_MAGIC) return NAV_E_NOT_INIT;
    if (!isfin(t_now)) return NAV_E_TIME_NONFINITE;
    if (n->have_now && t_now < n->t_now) return NAV_E_TIME_ORDER;
    n->have_now = 1;
    n->t_now = t_now;
    return NAV_OK;
}

/* ------------------------------------------------------------------------- */
/* Estado publicado                                                           */
/* ------------------------------------------------------------------------- */

nav_validity_t nav_validity(const nav_t *n, uint32_t *reasons)
{
    uint32_t r;
    if (n == NULL || n->magic != NAV_MAGIC) {
        r = NAV_R_NOT_INIT;
    } else {
        r = n->latched;
        if (!n->aligned) r |= NAV_R_NOT_ALIGNED;
        if (eskf_check_health(&n->f) != ESKF_OK) r |= NAV_R_CORE_FAULT; /* incluye f.fault */
        if (n->f.degraded & ESKF_DEG_GAP_BRIDGED) r |= NAV_R_GAP_BRIDGED;
        if (n->aligned) {
            /* atraso del estado frente a la última muestra (IMU o baro) y frente al reloj: si
               todo se calla, sólo el reloj (nav_tick) lo revela */
            double ts = n->have_rx ? n->t_rx : -(double)INFINITY;
            if (n->have_baro_rx && n->t_baro_rx > ts) ts = n->t_baro_rx;
            if (ts - n->t_state > (double)n->cfg.eskf.dt_max) r |= NAV_R_STALE;
            if (n->have_now && n->t_now - n->t_state > n->cfg.stale_max_s) r |= NAV_R_STALE;
            double tl = latest_time(n);
            if (n->phase == NAV_PHASE_FLIGHT && n->cfg.baro_timeout_s > 0.0) {
                double tb = n->t_last_baro_used;
                if (!isfin(tb) || (isfin(n->t_liftoff) && tb < n->t_liftoff)) tb = n->t_liftoff;
                if (isfin(tb) && tl - tb > n->cfg.baro_timeout_s) r |= NAV_R_BARO_TIMEOUT;
            }
        }
    }
    if (reasons) *reasons = r;
    if (r & NAV_R_INVALID_MASK) return NAV_INVALID;
    if (r & NAV_R_DEGRADED_MASK) return NAV_DEGRADED;
    return NAV_VALID;
}

void nav_get_output(const nav_t *n, nav_output_t *o)
{
    if (o == NULL) return;
    const float nanf_ = (float)NAN;
    uint32_t r;
    nav_validity_t val = nav_validity(n, &r);
    o->validity = (uint8_t)val;
    o->reasons = r;
    o->t_state = o->t_rx = o->t_eval = o->alt_ref = (double)NAN;
    o->alt_ref_seq = 0u;
    o->phase = NAV_PHASE_PAD;
    o->at_rest = 0;
    o->heading_known = 0;
    for (int i = 0; i < 3; i++) {
        o->p[i] = o->v[i] = o->sigma_p[i] = o->sigma_v[i] = o->sigma_th[i] = nanf_;
        o->b_a[i] = o->b_g[i] = nanf_;
    }
    for (int i = 0; i < 4; i++) o->q[i] = nanf_;
    if (n == NULL || n->magic != NAV_MAGIC) return;

    o->phase = (uint8_t)n->phase;
    o->at_rest = (uint8_t)(n->f.at_rest ? 1 : 0);
    o->heading_known = (uint8_t)(n->f.heading_known ? 1 : 0);
    if (n->have_rx) o->t_rx = n->t_rx;
    double tl = latest_time(n);
    if (isfin(tl)) o->t_eval = tl;
    if (n->have_alt_ref) o->alt_ref = n->alt_ref;
    o->alt_ref_seq = n->alt_ref_seq;
    if (!n->aligned) return;
    o->t_state = n->t_state;
    for (int i = 0; i < 3; i++) {
        o->p[i] = (float)n->f.p[i];
        o->v[i] = (float)n->f.v[i];
        o->b_a[i] = (float)n->f.b_a[i];
        o->b_g[i] = (float)n->f.b_g[i];
        double pp = (double)n->f.P[ESKF_IDX_P + i][ESKF_IDX_P + i];
        double pv = (double)n->f.P[ESKF_IDX_V + i][ESKF_IDX_V + i];
        double pt = (double)n->f.P[ESKF_IDX_TH + i][ESKF_IDX_TH + i];
        o->sigma_p[i] = (pp >= 0.0) ? (float)sqrt(pp) : nanf_;
        o->sigma_v[i] = (pv >= 0.0) ? (float)sqrt(pv) : nanf_;
        o->sigma_th[i] = (pt >= 0.0) ? (float)sqrt(pt) : nanf_;
    }
    for (int i = 0; i < 4; i++) o->q[i] = (float)n->f.q[i];
}

const char *nav_status_str(int code)
{
    switch (code) {
    case NAV_E_NULL: return "puntero nulo";
    case NAV_E_NOT_INIT: return "sesión de navegación sin iniciar";
    case NAV_E_CONFIG: return "configuración incompleta o inválida (o float/double incoherente)";
    case NAV_E_TIME_NONFINITE: return "marca de tiempo no finita";
    case NAV_E_TIME_DUPLICATE: return "marca de tiempo repetida";
    case NAV_E_TIME_ORDER: return "marca de tiempo atrasada";
    case NAV_E_VALUE: return "valor no finito";
    case NAV_E_INVALID: return "estimación no válida: muestra no aplicada";
    case NAV_E_GAP: return "hueco no recuperable: estimación invalidada";
    case NAV_E_NO_ALT_REF: return "sin referencia de altura";
    case NAV_E_STALE: return "estado atrasado: baro no aplicado";
    case NAV_E_BARO_TAU: return "desfase del baro fuera de límite";
    case NAV_E_PAD: return "baro antes de alinear (sólo referencia)";
    case NAV_E_TIME_FUTURE: return "marca de tiempo futura";
    default: return eskf_status_str(code);
    }
}
