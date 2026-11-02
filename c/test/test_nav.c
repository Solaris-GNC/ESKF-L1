/*
 * test_nav.c — pruebas del módulo navigation.c (el que se integra en el firmware).
 * Las pruebas "regresion_N" reproducen los fallos de la revisión del 19-oct sobre
 * examples/adapter_example.c de la v2 (commit a7b7e99).
 *
 * La configuración de prueba es una HIPÓTESIS (IMU de 200 Hz), no la del vuelo.
 */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "navigation.h"
#include "test_util.h"

#define PER 0.005 /* periodo de la IMU hipotética */

static void cfg_test(nav_config_t *c)
{
    nav_default_config(c);
    c->eskf.sigma_a = (eskf_real)0.05;
    c->eskf.sigma_w = (eskf_real)0.005;
    c->eskf.sigma_aw = (eskf_real)1e-4;
    c->eskf.sigma_ww = (eskf_real)1e-5;
    c->eskf.dt_max = (eskf_real)(1.5 * PER);
    c->eskf.ba0_std = (eskf_real)0.2;
    c->eskf.bg0_std = (eskf_real)0.01;
    c->rest_window_s = 1.0;
    c->rest_acc_dev_max = 0.5;
    c->rest_acc_std_max = 0.2;
    c->rest_gyro_max = 0.2;
    c->liftoff_acc = 2 * 9.81;
    c->liftoff_hold_s = 0.05;
    c->acc_sat_limit = 0;  /* sólo bits del driver, salvo en las pruebas de límites */
    c->gyro_sat_limit = 0;
    c->baro_sigma = 0.5;
    c->baro_latency_s = 0;
    c->eskf.accel_threshold = (eskf_real)0.5;
    c->eskf.align_acc_tol = (eskf_real)0.5;
    c->eskf.align_gyro_max = (eskf_real)0.2;
    c->rest_tilt_change_max = 0.005;
}

static nav_imu_sample_t imu(double t, double ax, double ay, double az, double wx)
{
    nav_imu_sample_t s;
    s.t = t;
    s.acc[0] = (float)ax; s.acc[1] = (float)ay; s.acc[2] = (float)az;
    s.gyro[0] = (float)wx; s.gyro[1] = 0; s.gyro[2] = 0;
    s.sat = 0;
    return s;
}
static nav_imu_sample_t rest(double t) { return imu(t, 0, 0, -9.81, 0); }

/* Ventana de reposo de k = -200..0 (t = k·PER): alineación exactamente en t = 0. */
static int align_at_zero(nav_t *n, const nav_config_t *c, int with_baro)
{
    if (nav_init(n, c) != NAV_OK) return 0;
    for (int k = -200; k <= 0; k++) {
        nav_imu_sample_t s = rest(k * PER);
        nav_on_imu(n, &s);
        if (with_baro && k % 4 == 0 && k < 0) {
            nav_baro_sample_t b = {k * PER + 0.001, 100.0f};
            nav_on_baro(n, &b);
        }
    }
    return n->aligned && n->t_state == 0.0;
}

/* Alineado en t = 0 y puesto en vuelo con v_z = 10 m/s (aceleración de navegación nula) */
static int flight_v10(nav_t *n, const nav_config_t *c)
{
    if (!align_at_zero(n, c, 1)) return 0;
    n->phase = NAV_PHASE_FLIGHT;
    eskf_set_rest(&n->f, 0);
    n->f.v[2] = (eskf_real)10.0;
    n->t_liftoff = 0.0;
    return 1;
}

static int valid(const nav_t *n, uint32_t *r) { return nav_validity(n, r) == NAV_VALID; }

/* ---------------------------------------------------------------- 1 */
static void regresion_1_tiempos(void)
{
    printf("1. tiempos no finitos, repetidos y atrasados\n");
    nav_config_t c;
    nav_t n;
    cfg_test(&c);

    /* caso de la revisión: 0.005, 0.003, 0.010 → p_z = 0.10 */
    CHECK(flight_v10(&n, &c), "preparación");
    nav_imu_sample_t s = rest(0.005);
    CHECK(nav_on_imu(&n, &s) == NAV_OK, "0.005");
    s = rest(0.003);
    CHECK(nav_on_imu(&n, &s) == NAV_E_TIME_ORDER && n.last.imu == NAV_E_TIME_ORDER, "0.003 debe rechazarse");
    CHECK(n.t_state == 0.005 && n.t_rx == 0.005, "el rechazo no debe tocar los tiempos");
    s = rest(0.010);
    CHECK(nav_on_imu(&n, &s) == NAV_OK, "0.010");
    printf("   0.005, 0.003, 0.010: p_z = %.6f (v2: 0.12)\n", (double)n.f.p[2]);
    CHECK(fabs((double)n.f.p[2] - 0.10) < 1e-5 && n.t_state == 0.010, "p_z = %.6f", (double)n.f.p[2]);
    CHECK(n.cnt.imu_time_order == 1, "contador de atrasadas");

    /* NaN: rechazada, y las siguientes se propagan bien */
    CHECK(flight_v10(&n, &c), "preparación");
    s = rest((double)NAN);
    CHECK(nav_on_imu(&n, &s) == NAV_E_TIME_NONFINITE && n.t_state == 0.0 && n.t_rx == 0.0, "t = NaN");
    for (int k = 1; k <= 4; k++) {
        s = rest(k * PER);
        CHECK(nav_on_imu(&n, &s) == NAV_OK, "muestra %d tras NaN", k);
    }
    CHECK(fabs((double)n.f.p[2] - 0.20) < 1e-5 && valid(&n, NULL), "tras NaN p_z = %.6f (v2: 0.15)",
          (double)n.f.p[2]);
    s = rest((double)INFINITY);
    CHECK(nav_on_imu(&n, &s) == NAV_E_TIME_NONFINITE && n.cnt.imu_time_nonfinite == 2, "t = inf");

    /* duplicado */
    CHECK(flight_v10(&n, &c), "preparación");
    s = rest(0.005);
    nav_on_imu(&n, &s);
    eskf_real pz = n.f.p[2];
    CHECK(nav_on_imu(&n, &s) == NAV_E_TIME_DUPLICATE && n.f.p[2] == pz && n.cnt.imu_time_dup == 1, "duplicado");

    /* primera muestra y tiempo de alineación por separado */
    CHECK(nav_init(&n, &c) == NAV_OK, "init");
    nav_output_t o;
    nav_get_output(&n, &o);
    CHECK(isnan(o.t_rx) && isnan(o.t_state), "sin muestras: sin tiempos");
    s = rest(10.0);
    nav_on_imu(&n, &s);
    nav_get_output(&n, &o);
    CHECK(o.t_rx == 10.0 && isnan(o.t_state) && o.validity == NAV_INVALID && (o.reasons & NAV_R_NOT_ALIGNED),
          "la primera muestra sólo fija t_rx");
    for (int k = 1; k <= 200; k++) {
        s = rest(10.0 + k * PER);
        nav_on_imu(&n, &s);
    }
    nav_get_output(&n, &o);
    CHECK(n.aligned && o.t_state == 10.0 + 200 * PER, "t_state = última muestra de la ventana (%.6f)", o.t_state);
}

/* ---------------------------------------------------------------- 2 */
static void regresion_2_hueco(void)
{
    printf("2. hueco no recuperable y medida IMU inválida\n");
    nav_config_t c;
    nav_t n;
    nav_output_t o;
    cfg_test(&c);

    CHECK(flight_v10(&n, &c), "preparación");
    nav_imu_sample_t s = rest(0.2);
    CHECK(nav_on_imu(&n, &s) == NAV_E_GAP && n.cnt.gaps == 1, "t = 0.2 debe dar NAV_E_GAP");
    s = rest(0.205);
    CHECK(nav_on_imu(&n, &s) == NAV_E_INVALID, "t = 0.205 no debe aplicarse");
    nav_baro_sample_t b = {0.206, -1.95f};
    CHECK(nav_on_baro(&n, &b) == NAV_E_INVALID, "baro posterior no debe aplicarse");
    nav_get_output(&n, &o);
    printf("   tras el hueco: t_state = %.3f, p_z = %.4f, validez %d, motivos 0x%04x (v2: t_state 0.205, p_z 0.05)\n",
           o.t_state, (double)o.p[2], o.validity, (unsigned)o.reasons);
    CHECK(o.t_state == 0.0 && o.p[2] == 0.0f && o.t_rx == 0.205, "estado congelado con su t_state real");
    CHECK(o.validity == NAV_INVALID && (o.reasons & NAV_R_TIME_GAP), "debe publicarse NO VÁLIDO");
    /* en vuelo no hay recuperación automática aunque lleguen muestras de "reposo" */
    for (int k = 42; k < 42 + 400; k++) {
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_validity(&n, NULL) == NAV_INVALID && n.cnt.align_ok == 1, "en vuelo sigue NO VÁLIDO");

    /* en rampa: la siguiente ventana de reposo completa realinea */
    CHECK(align_at_zero(&n, &c, 1), "preparación rampa");
    s = rest(0.3);
    CHECK(nav_on_imu(&n, &s) == NAV_E_GAP && nav_validity(&n, NULL) == NAV_INVALID, "hueco en rampa");
    int k = 61;
    for (; k < 61 + 202 && nav_validity(&n, NULL) != NAV_VALID; k++) {
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_validity(&n, NULL) == NAV_VALID && n.cnt.align_ok == 2 && n.t_state == (k - 1) * PER,
          "en rampa se realinea (align_ok %u)", (unsigned)n.cnt.align_ok);

    /* IMU con valor no finito: el tiempo cuenta, el estado no avanza */
    CHECK(flight_v10(&n, &c), "preparación");
    s = rest(0.005);
    s.acc[0] = (float)NAN;
    CHECK(nav_on_imu(&n, &s) == NAV_E_VALUE && n.t_state == 0.0 && n.t_rx == 0.005, "valor NaN");
    CHECK(valid(&n, NULL), "una muestra perdida dentro de dt_max no invalida");
    s = rest(0.010);
    s.gyro[1] = (float)INFINITY;
    nav_on_imu(&n, &s);
    uint32_t r;
    CHECK(nav_validity(&n, &r) == NAV_INVALID && (r & NAV_R_STALE), "estado atrasado > dt_max: STALE");
    b.t = 0.011; b.alt_m = 100.0f;
    CHECK(nav_on_baro(&n, &b) == NAV_E_STALE, "baro sobre estado atrasado");
    s = rest(0.015);
    CHECK(nav_on_imu(&n, &s) == NAV_E_GAP, "la siguiente válida revela el hueco (dt = 0.015)");

    /* con dt_max = 2.5 periodos, una muestra perdida se absorbe con el tiempo correcto */
    c.eskf.dt_max = (eskf_real)(2.5 * PER);
    CHECK(flight_v10(&n, &c), "preparación");
    s = rest(0.005);
    s.acc[2] = (float)NAN;
    nav_on_imu(&n, &s);
    s = rest(0.010);
    CHECK(nav_on_imu(&n, &s) == NAV_OK && fabs((double)n.f.p[2] - 0.10) < 1e-5 && n.t_state == 0.010,
          "dt = 0.010 desde el estado: p_z = %.6f", (double)n.f.p[2]);
}

/* ---------------------------------------------------------------- 4 */
static void regresion_4_reinicio(void)
{
    printf("4. reinicio completo de la sesión\n");
    nav_config_t c;
    nav_t n;
    nav_output_t o;
    cfg_test(&c);

    /* arranque */
    CHECK(nav_init(&n, &c) == NAV_OK, "init");
    nav_get_output(&n, &o);
    CHECK(o.validity == NAV_INVALID && (o.reasons & NAV_R_NOT_ALIGNED) && isnan(o.p[0]) && isnan(o.q[0]) &&
          o.phase == NAV_PHASE_PAD, "arranque: NO VÁLIDO, sin estimación publicada");
    nav_imu_sample_t s;
    for (int k = 0; k < 100; k++) { /* media ventana */
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_validity(&n, NULL) == NAV_INVALID, "media ventana: aún NO VÁLIDO");

    /* tras funcionar en vuelo */
    CHECK(flight_v10(&n, &c), "preparación");
    for (int k = 1; k <= 20; k++) {
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_init(&n, &n.cfg) == NAV_OK, "reinicio con su propia configuración");
    CHECK(n.phase == NAV_PHASE_PAD && !n.aligned && !n.have_rx && !n.have_state && !n.have_alt_ref &&
          n.cnt.imu_rx == 0 && n.latched == 0 && !n.f.at_rest && isnan(n.t_liftoff),
          "todo el estado de la sesión debe reiniciarse");
    nav_get_output(&n, &o);
    CHECK(o.validity == NAV_INVALID && isnan(o.p[2]), "tras reinicio no se publica estimación");
    s = rest(-5.0); /* la base de tiempo puede reiniciarse también */
    CHECK(nav_on_imu(&n, &s) == NAV_OK && n.t_rx == -5.0, "tras reinicio se acepta un tiempo anterior");

    /* tras fallo del núcleo */
    CHECK(flight_v10(&n, &c), "preparación");
    n.f.q[0] = n.f.q[1] = n.f.q[2] = n.f.q[3] = 0; /* corrupción simulada */
    s = rest(0.005);
    int r = nav_on_imu(&n, &s);
    uint32_t why;
    CHECK(r == ESKF_E_NUMERIC && nav_validity(&n, &why) == NAV_INVALID && (why & NAV_R_CORE_FAULT), "fallo");
    CHECK(nav_init(&n, &c) == NAV_OK && !n.f.fault && !n.aligned, "reinicio tras fallo");
    CHECK(align_at_zero(&n, &c, 1) && valid(&n, NULL), "realineado tras reinicio: VÁLIDO");

    /* configuración inválida deja la sesión sin iniciar */
    nav_config_t bad = c;
    bad.baro_sigma = NAV_PENDING;
    CHECK(nav_init(&n, &bad) == NAV_E_CONFIG, "config pendiente");
    s = rest(1.0);
    CHECK(nav_on_imu(&n, &s) == NAV_E_NOT_INIT && nav_validity(&n, &why) == NAV_INVALID && (why & NAV_R_NOT_INIT),
          "sesión sin iniciar");
}

/* ---------------------------------------------------------------- 5 */
static void regresion_5_saturacion(void)
{
    printf("5. saturación de acelerómetro y giróscopo\n");
    nav_config_t c;
    nav_t n;
    uint32_t r;
    cfg_test(&c);

    /* bit del driver, política por defecto (0 s admitidos) → NO VÁLIDO */
    CHECK(flight_v10(&n, &c), "preparación");
    nav_imu_sample_t s = rest(0.005);
    s.sat = NAV_SAT_GYRO_Y;
    nav_on_imu(&n, &s);
    CHECK(n.cnt.imu_gyro_sat == 1 && nav_validity(&n, &r) == NAV_INVALID && (r & NAV_R_GYRO_SAT_LIMIT),
          "giro saturado (driver): motivos 0x%04x", (unsigned)r);

    /* límite configurado, por eje */
    c.gyro_sat_limit = 0.1;
    c.acc_sat_limit = 50.0;
    c.gyro_sat_max_s = 0.02;
    c.gyro_sat_excess = 2.0;
    c.acc_sat_max_s = 0.02;
    c.acc_sat_excess = 5.0;
    /* gemelo sin saturación: misma muestra con ω_x justo por debajo del límite. La
       diferencia de P entre ambos es sólo el inflado. */
    nav_t twin;
    CHECK(flight_v10(&n, &c) && flight_v10(&twin, &c), "preparación");
    s = imu(0.005, 0, 0, -9.81, 0.1); /* ω_x en el límite */
    nav_imu_sample_t s_twin = imu(0.005, 0, 0, -9.81, 0.0999999);
    nav_on_imu(&n, &s);
    nav_on_imu(&twin, &s_twin);
    double dth_x = (double)n.f.P[ESKF_IDX_TH][ESKF_IDX_TH] - (double)twin.f.P[ESKF_IDX_TH][ESKF_IDX_TH];
    double dth_y = (double)n.f.P[ESKF_IDX_TH + 1][ESKF_IDX_TH + 1] - (double)twin.f.P[ESKF_IDX_TH + 1][ESKF_IDX_TH + 1];
    CHECK(nav_validity(&n, &r) == NAV_DEGRADED && (r & NAV_R_GYRO_SAT) && n.cnt.imu_gyro_sat == 1,
          "dentro de lo admitido: DEGRADADO (0x%04x)", (unsigned)r);
    CHECK(fabs(dth_x - 1e-4) < 1e-6 && fabs(dth_y) < 1e-9, "inflado sólo del eje x, (2·0.005)²: Δx %.4g Δy %.3g",
          dth_x, dth_y);
    CHECK(n.last.inflate == ESKF_OK, "retorno del inflado registrado");
    for (int k = 2; k <= 5; k++) {
        s = imu(k * PER, 0, 0, -9.81, 0.1);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_validity(&n, &r) == NAV_INVALID && (r & NAV_R_GYRO_SAT_LIMIT), "más de 0.02 s: NO VÁLIDO");

    CHECK(flight_v10(&n, &c) && flight_v10(&twin, &c), "preparación");
    s = rest(0.005);
    s.sat = NAV_SAT_ACC_Z; /* bit del driver; el gemelo recibe la misma muestra sin el bit */
    s_twin = rest(0.005);
    nav_on_imu(&n, &s);
    nav_on_imu(&twin, &s_twin);
    CHECK(nav_validity(&n, &r) == NAV_DEGRADED && (r & NAV_R_ACC_SAT) && n.cnt.imu_acc_sat == 1, "acc saturado");
    for (int i = 0; i < 3; i++) {
        double dv = (double)n.f.P[ESKF_IDX_V + i][ESKF_IDX_V + i] - (double)twin.f.P[ESKF_IDX_V + i][ESKF_IDX_V + i];
        CHECK(fabs(dv - 6.25e-4) < 1e-7, "inflado isótropo de v, eje %d: %.6g (esperado (5·0.005)²)", i, dv);
    }
    CHECK(flight_v10(&n, &c), "preparación");
    s = imu(0.005, 0, 0, -60.0, 0); /* a_z más allá del límite configurado */
    nav_on_imu(&n, &s);
    CHECK(n.cnt.imu_acc_sat == 1 && (n.latched & NAV_R_ACC_SAT), "límite de acelerómetro detectado");

    /* el inflado imposible invalida en lugar de ignorarse */
    c.acc_sat_excess = 1e300;
    CHECK(flight_v10(&n, &c), "preparación");
    nav_on_imu(&n, &s);
    CHECK(n.last.inflate == ESKF_E_INPUT && n.cnt.inflate_fail == 1 && nav_validity(&n, &r) == NAV_INVALID &&
          (r & NAV_R_INFLATE_FAIL), "inflado fallido: NO VÁLIDO");

    /* en rampa una saturación corta el reposo */
    cfg_test(&c);
    CHECK(align_at_zero(&n, &c, 1) && n.f.at_rest, "rampa");
    s = rest(0.005);
    s.sat = NAV_SAT_ACC_X;
    nav_on_imu(&n, &s);
    CHECK(!n.f.at_rest, "saturación en rampa: sin reposo");
}

/* ---------------------------------------------------------------- 6 */
static void regresion_6_validez(void)
{
    printf("6. salud numérica frente a validez operativa\n");
    nav_config_t c;
    nav_t n;
    uint32_t r;
    cfg_test(&c);
    CHECK(flight_v10(&n, &c), "preparación");
    CHECK(eskf_check_health(&n.f) == ESKF_OK && valid(&n, NULL), "válido");
    n.f.fault = 1; /* campos finitos, fallo enclavado */
    CHECK(eskf_check_health(&n.f) == ESKF_E_FAULT, "check_health con fallo (v2: ESKF_OK)");
    CHECK(nav_validity(&n, &r) == NAV_INVALID && (r & NAV_R_CORE_FAULT), "nav con fallo: NO VÁLIDO");
    nav_imu_sample_t s = rest(0.005);
    CHECK(nav_on_imu(&n, &s) == NAV_E_INVALID, "con fallo no se aplica nada");

    /* baro timeout en vuelo → DEGRADADO */
    CHECK(flight_v10(&n, &c), "preparación");
    for (int k = 1; k <= 300; k++) { /* 1.5 s sin baro */
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(nav_validity(&n, &r) == NAV_DEGRADED && (r & NAV_R_BARO_TIMEOUT), "sin baro 1.5 s: 0x%04x", (unsigned)r);
    nav_baro_sample_t b = {300 * PER + 0.001, (float)(100.0 - (double)n.f.p[2])};
    CHECK(nav_on_baro(&n, &b) == 1 && valid(&n, NULL), "un baro aplicado lo devuelve a VÁLIDO");

    /* la IMU se calla y sólo llega baro: el estado se publica atrasado */
    nav_tick(&n, 300 * PER + 0.04);
    CHECK(nav_validity(&n, &r) == NAV_VALID, "reloj 0.04 s por delante (< stale_max_s 0.05): VÁLIDO");
    nav_tick(&n, 300 * PER + 0.1);
    CHECK(nav_validity(&n, &r) == NAV_INVALID && (r & NAV_R_STALE), "IMU y baro callados, reloj +0.1 s: STALE");
}

/* ---------------------------------------------------------------- 7 */
static void regresion_7_adaptador(void)
{
    printf("7. despegue, retornos y barómetro\n");
    nav_config_t c;
    nav_t n;
    cfg_test(&c);

    /* despegue con persistencia */
    CHECK(align_at_zero(&n, &c, 1) && n.f.at_rest, "rampa");
    uint32_t zupt0 = n.cnt.zupt_used;
    nav_imu_sample_t s = imu(0.005, 0, 0, -25, 0); /* golpe de una muestra */
    nav_on_imu(&n, &s);
    CHECK(n.phase == NAV_PHASE_PAD && !n.f.at_rest, "un golpe: sin despegue, sin reposo (v2: despegue)");
    int k = 2;
    for (; k < 2 + 210; k++) {
        s = rest(k * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(n.f.at_rest && n.cnt.align_ok == 1, "el reposo vuelve tras una ventana, sin realinear");
    for (int j = 0; j < 6; j++, k++) { /* 0.03 s < liftoff_hold_s */
        s = imu(k * PER, 0, 0, -25, 0);
        nav_on_imu(&n, &s);
    }
    s = rest(k++ * PER);
    nav_on_imu(&n, &s);
    CHECK(n.phase == NAV_PHASE_PAD, "0.03 s sobre el umbral: sin despegue");
    zupt0 = n.cnt.zupt_used;
    double t_on = k * PER;
    for (int j = 0; j < 12; j++, k++) {
        s = imu(k * PER, 0, 0, -30, 0);
        nav_on_imu(&n, &s);
    }
    CHECK(n.phase == NAV_PHASE_FLIGHT && fabs(n.t_liftoff - t_on) < 1e-12 && !n.f.at_rest,
          "despegue sostenido: t_liftoff %.3f", n.t_liftoff);
    CHECK(n.cnt.zupt_used == zupt0, "sin ZUPT durante la confirmación del despegue");
    nav_config_t bad = c;
    bad.liftoff_acc = 9.81 + 0.4;
    CHECK(nav_config_check(&bad) != NULL && strstr(nav_config_check(&bad), "liftoff_acc") != NULL, "umbral bajo");

    /* retornos registrados */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    for (int j = 1; j <= 10; j++) { /* hasta la primera actualización de reposo (0.05 s) */
        s = rest(j * PER);
        nav_on_imu(&n, &s);
    }
    CHECK(n.last.align == ESKF_OK && n.last.zupt == 3 && n.last.grav == 3 && n.last.imu == NAV_OK,
          "últimos códigos: align %d zupt %d grav %d", n.last.align, n.last.zupt, n.last.grav);

    /* barómetro */
    nav_baro_sample_t b;
    CHECK(nav_init(&n, &c) == NAV_OK, "init");
    b.t = 0.0; b.alt_m = 100.0f;
    CHECK(nav_on_baro(&n, &b) == NAV_E_PAD, "antes de alinear");
    b.t = (double)NAN;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_NONFINITE, "t NaN");
    b.t = 0.0;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_DUPLICATE, "duplicado");
    b.t = -1.0;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_ORDER, "atrasado");
    b.t = 0.1; b.alt_m = (float)NAN;
    CHECK(nav_on_baro(&n, &b) == NAV_E_VALUE, "alt NaN");
    CHECK(n.cnt.baro_rejected == 5 && n.last.baro == NAV_E_VALUE, "contadores de baro");

    CHECK(align_at_zero(&n, &c, 0), "alineado sin baro en la ventana");
    b.t = 0.002; b.alt_m = 100.0f;
    CHECK(nav_on_baro(&n, &b) == NAV_E_NO_ALT_REF, "sin referencia de altura");
    CHECK(align_at_zero(&n, &c, 1) && n.have_alt_ref && n.alt_ref == 100.0, "referencia = media de la ventana");
    b.t = 0.002; b.alt_m = 100.5f;
    CHECK(nav_on_baro(&n, &b) == 1, "baro aplicado");
    b.t = 0.02;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_FUTURE, "sin reloj, 0.02 s por delante de la IMU: futura");
    nav_tick(&n, 0.03);
    b.t = 0.021;
    CHECK(nav_on_baro(&n, &b) == NAV_E_BARO_TAU, "con reloj: |τ| = 0.021 > 0.0075");

    /* latencia: configurada y compensada */
    c.baro_latency_s = 0.02;
    CHECK(nav_config_check(&c) != NULL && strstr(nav_config_check(&c), "tau_max") != NULL,
          "latencia sin tau_max suficiente: configuración rechazada");
    c.eskf.tau_max = (eskf_real)0.03;
    CHECK(nav_config_check(&c) == NULL, "con tau_max = 0.03 vale");
    CHECK(flight_v10(&n, &c), "preparación");
    s = rest(0.005);
    nav_on_imu(&n, &s);
    eskf_real pz = n.f.p[2], vz = n.f.v[2];
    /* leído en 0.006 con 20 ms de latencia: corresponde a t = −0.014, z = −0.14 → alt 100.14 */
    b.t = 0.006; b.alt_m = (float)(100.0 + 0.14);
    CHECK(nav_on_baro(&n, &b) == 1, "baro con latencia aplicado");
    CHECK(fabs((double)(n.f.p[2] - pz)) < 1e-3 && fabs((double)(n.f.v[2] - vz)) < 1e-2,
          "medida coherente con su instante: no corrige (Δz %.2e)", (double)(n.f.p[2] - pz));
}

/* ---------------------------------------------------------------- configuración */
static void prueba_configuracion_pendiente(void)
{
    printf("configuración pendiente\n");
    nav_config_t c;
    nav_t n;
    nav_default_config(&c);
    const char *f = nav_config_check(&c);
    printf("   por defecto, primer campo pendiente: %s\n", f ? f : "(ninguno)");
    CHECK(f != NULL && nav_init(&n, &c) == NAV_E_CONFIG, "la configuración por defecto no debe arrancar");
    CHECK(c.eskf.gap_max == 0 && c.acc_sat_max_s == 0 && c.gyro_sat_max_s == 0, "políticas de primera integración");
    nav_config_t t;
    cfg_test(&t);
    CHECK(nav_config_check(&t) == NULL, "la configuración de prueba es completa");
    /* cada pendiente, por sí sola, bloquea y se nombra */
    struct { const char *name; eskf_real *e; double *d; } P[] = {
        {"eskf.sigma_a", &t.eskf.sigma_a, 0}, {"eskf.sigma_w", &t.eskf.sigma_w, 0},
        {"eskf.sigma_aw", &t.eskf.sigma_aw, 0}, {"eskf.sigma_ww", &t.eskf.sigma_ww, 0},
        {"eskf.dt_max", &t.eskf.dt_max, 0}, {"eskf.ba0_std", &t.eskf.ba0_std, 0},
        {"eskf.bg0_std", &t.eskf.bg0_std, 0}, {"eskf.accel_threshold", &t.eskf.accel_threshold, 0},
        {"eskf.align_acc_tol", &t.eskf.align_acc_tol, 0}, {"eskf.align_gyro_max", &t.eskf.align_gyro_max, 0},
        {"rest_acc_dev_max", 0, &t.rest_acc_dev_max}, {"rest_acc_std_max", 0, &t.rest_acc_std_max},
        {"rest_gyro_max", 0, &t.rest_gyro_max}, {"rest_tilt_change_max", 0, &t.rest_tilt_change_max},
        {"liftoff_acc", 0, &t.liftoff_acc}, {"acc_sat_limit", 0, &t.acc_sat_limit},
        {"gyro_sat_limit", 0, &t.gyro_sat_limit}, {"baro_sigma", 0, &t.baro_sigma},
        {"baro_latency_s", 0, &t.baro_latency_s}};
    nav_config_t dflt;
    nav_default_config(&dflt);
    for (size_t i = 0; i < sizeof P / sizeof P[0]; i++) {
        const char *g;
        if (P[i].e) {
            eskf_real keep = *P[i].e;
            *P[i].e = (eskf_real)NAV_PENDING;
            g = nav_config_check(&t);
            *P[i].e = keep;
        } else {
            double keep = *P[i].d;
            *P[i].d = NAV_PENDING;
            g = nav_config_check(&t);
            *P[i].d = keep;
        }
        CHECK(g != NULL && strcmp(g, P[i].name) == 0, "pendiente %s -> %s", P[i].name, g ? g : "NULL");
        /* y es NAV_PENDING en la configuración por defecto */
        size_t off = P[i].e ? (size_t)((char *)P[i].e - (char *)&t) : (size_t)((char *)P[i].d - (char *)&t);
        int pend = P[i].e ? isnan((double)*(eskf_real *)((char *)&dflt + off)) : isnan(*(double *)((char *)&dflt + off));
        CHECK(pend, "%s debe ser NAV_PENDING por defecto", P[i].name);
    }
}

/* ---------------------------------------------------------------- registro */
static void prueba_registro_continua(void)
{
    printf("el registro continúa con la estimación no válida\n");
    nav_config_t c;
    nav_t n;
    cfg_test(&c);
    CHECK(flight_v10(&n, &c), "preparación");
    int records = 0, invalid_records = 0, samples = 0;
    for (int k = 1; k <= 100; k++) {
        double t = (k < 50) ? k * PER : k * PER + 0.5; /* hueco de 0.5 s */
        nav_imu_sample_t s = rest(t);
        samples++;
        nav_on_imu(&n, &s); /* el retorno no condiciona el registro */
        nav_output_t o;
        nav_get_output(&n, &o);
        records++;
        if (o.validity == NAV_INVALID) invalid_records++;
    }
    CHECK(records == samples && invalid_records == 51, "registros %d/%d, no válidos %d", records, samples,
          invalid_records);
}

/* ================================================================ revisión 19-oct (lista de 13) */

static nav_imu_sample_t imu6(double t, double ax, double ay, double az, double wx, double wy, double wz)
{
    nav_imu_sample_t s = imu(t, ax, ay, az, wx);
    s.gyro[1] = (float)wy;
    s.gyro[2] = (float)wz;
    return s;
}

static void v4_ventanas_y_recuperacion(void)
{
    printf("v4-1/2/3/4. ventanas de reposo: continuidad, recuperación, variabilidad y giros lentos\n");
    nav_config_t c;
    static nav_t n;
    nav_imu_sample_t s;
    cfg_test(&c);

    /* 1a: dos muestras separadas por un hueco no forman ventana */
    nav_init(&n, &c);
    s = rest(0.0); nav_on_imu(&n, &s);
    s = rest(5.0); nav_on_imu(&n, &s);
    CHECK(!n.aligned && n.cnt.rest_breaks_gap == 1, "2 muestras a 5 s: no debe alinear (v3: alineaba)");
    /* 1b: una muestra NaN parte la ventana; hace falta una ventana limpia completa después */
    nav_init(&n, &c);
    int k;
    for (k = 0; k < 150; k++) { s = rest(k * PER); nav_on_imu(&n, &s); }
    s = rest(k++ * PER); s.acc[0] = (float)NAN; nav_on_imu(&n, &s);
    double t_after = k * PER;
    for (; k < 150 + 1 + 150; k++) { s = rest(k * PER); nav_on_imu(&n, &s); }
    CHECK(!n.aligned, "ventana atravesando un NaN: no debe alinear con 1 s total (v3: alineaba)");
    for (; k < 150 + 1 + 205; k++) { s = rest(k * PER); nav_on_imu(&n, &s); }
    CHECK(n.aligned && n.t_state >= t_after + c.rest_window_s - 1e-9, "alinea con una ventana completa posterior");

    /* 2: la recuperación en rampa nunca usa muestras anteriores al fallo */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    for (k = 1; k <= 150; k++) { s = rest(k * PER); nav_on_imu(&n, &s); } /* ventana en curso */
    double t_gap = 150 * PER + 0.2;
    s = rest(t_gap);
    CHECK(nav_on_imu(&n, &s) == NAV_E_GAP && n.blocked && n.win_n == 0 && fabs(n.t_blocked - t_gap) < 1e-12,
          "al bloquearse se descarta la ventana en curso (v3: seguía con win_n 176)");
    double t_realign = -1;
    for (k = 1; k <= 260 && t_realign < 0; k++) {
        s = rest(t_gap + k * PER);
        nav_on_imu(&n, &s);
        if (nav_validity(&n, NULL) == NAV_VALID) t_realign = n.t_state;
    }
    CHECK(t_realign >= t_gap + c.rest_window_s - 1e-9, "realineación sólo con datos posteriores (%.3f >= %.3f)",
          t_realign, t_gap + c.rest_window_s);

    /* 3: una ventana rechazada por variabilidad retira el reposo y el ZUPT */
    CHECK(align_at_zero(&n, &c, 1) && n.f.at_rest, "rampa");
    for (k = 1; k <= 202; k++) {
        s = rest(k * PER);
        s.acc[2] = (float)(-9.81 + ((k & 1) ? 0.4 : -0.4));
        nav_on_imu(&n, &s);
    }
    uint32_t z0 = n.cnt.zupt_used;
    for (int j = 0; j < 20; j++, k++) { s = rest(k * PER); nav_on_imu(&n, &s); }
    CHECK(n.cnt.rest_windows_bad == 1 && !n.f.at_rest && n.cnt.zupt_used == z0,
          "ventana ruidosa: sin reposo ni ZUPT (v3: at_rest = 1)");

    /* 4: cabeceo lento por debajo de los umbrales por muestra: no es reposo */
    nav_init(&n, &c);
    double wy = 0.05;
    for (k = 0; k <= 210; k++) {
        double th = wy * k * PER;
        s = imu6(k * PER, 9.81 * sin(th), 0, -9.81 * cos(th), 0, wy, 0);
        nav_on_imu(&n, &s);
    }
    CHECK(!n.aligned && n.cnt.rest_windows_tilt >= 1, "cabeceo 0.05 rad/s: rechazado (v3: b_g = 0.05)");
    /* en reposo real con sesgo de giro (sin rotación) sí alinea y guarda el sesgo */
    nav_init(&n, &c);
    for (k = 0; k <= 210; k++) { s = imu6(k * PER, 0, 0, -9.81, 0, 0.05, 0); nav_on_imu(&n, &s); }
    CHECK(n.aligned && fabs((double)n.f.b_g[1] - 0.05) < 1e-6, "sesgo puro de 0.05 rad/s: alinea");
    /* límite documentado: un giro lento alrededor de la vertical no cambia la gravedad y es
       indistinguible de un sesgo de z sin rumbo externo */
    nav_init(&n, &c);
    for (k = 0; k <= 210; k++) { s = imu6(k * PER, 0, 0, -9.81, 0, 0, 0.05); nav_on_imu(&n, &s); }
    printf("   (límite) guiñada lenta de 0.05 rad/s: alineado=%d, b_g_z=%.3f — no observable sin rumbo\n",
           n.aligned, (double)n.f.b_g[2]);
}

static void v4_despegue_y_baro(void)
{
    printf("v4-5/6/7/8. despegue, referencia barométrica, marcas futuras y reloj\n");
    nav_config_t c;
    static nav_t n;
    nav_imu_sample_t s;
    nav_baro_sample_t b;
    cfg_test(&c);
    int k;

    /* 5: la persistencia no atraviesa huecos ni datos inválidos */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    s = imu(1.10, 0, 0, -30, 0); nav_on_imu(&n, &s);
    s = imu(1.12, 0, 0, -30, 0); s.acc[1] = (float)NAN; nav_on_imu(&n, &s);
    s = imu(1.20, 0, 0, -30, 0); nav_on_imu(&n, &s);
    CHECK(n.phase == NAV_PHASE_PAD, "NaN y hueco entre muestras >2 g: sin despegue (v3: despegue)");
    for (k = 1; k <= 12; k++) { s = imu(1.20 + k * PER, 0, 0, -30, 0); nav_on_imu(&n, &s); }
    CHECK(n.phase == NAV_PHASE_FLIGHT && fabs(n.t_liftoff - 1.20) < 1e-9, "continuo 0.05 s: despegue desde 1.20");

    /* 6a: un barómetro atípico no contamina la referencia */
    nav_init(&n, &c);
    for (k = -200; k <= 0; k++) {
        s = rest(k * PER); nav_on_imu(&n, &s);
        if (k % 2 == 0 && k < 0) { /* 100 barómetros, uno atípico */
            b.t = k * PER + 1e-4;
            b.alt_m = (k == -100) ? 5000.0f : 100.0f;
            nav_on_baro(&n, &b);
        }
    }
    CHECK(n.aligned && n.have_alt_ref && fabs(n.alt_ref - 100.0) < 1e-6 && n.cnt.ref_outliers == 1,
          "atípico de 5000 m: alt_ref %.3f (v3: 208.89)", n.alt_ref);
    /* 6b: un baro rechazado por la puerta no entra en la siguiente referencia */
    for (k = 1; k <= 205; k++) {
        s = rest(k * PER); nav_on_imu(&n, &s);
        if (k % 2 == 0) {
            b.t = k * PER + 1e-4;
            b.alt_m = (k == 100) ? 400.0f : 100.0f;
            int r = nav_on_baro(&n, &b);
            if (k == 100) CHECK(r == ESKF_E_GATED, "400 m: puerta");
        }
    }
    CHECK(n.cnt.ref_updates == 2 && fabs(n.alt_ref - 100.0) < 1e-6, "el rechazado no entra: alt_ref %.3f (v3: 106.67)",
          n.alt_ref);
    /* 6c: pocos barómetros válidos: no hay referencia */
    nav_init(&n, &c);
    for (k = -200; k <= 0; k++) {
        s = rest(k * PER); nav_on_imu(&n, &s);
        if (k % 20 == 0 && k < 0) { b.t = k * PER + 1e-4; b.alt_m = 100.0f; nav_on_baro(&n, &b); }
    }
    CHECK(n.aligned && !n.have_alt_ref && n.last.ref == NAV_E_NO_ALT_REF, "10 barómetros < 50: sin referencia");
    /* 6d: un salto de referencia mayor que baro_ref_max_step en rampa no se aplica en silencio */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    uint32_t seq0 = n.alt_ref_seq;
    for (k = 1; k <= 205; k++) {
        s = rest(k * PER); nav_on_imu(&n, &s);
        if (k % 4 == 0) { b.t = k * PER + 1e-4; b.alt_m = 103.0f; nav_on_baro(&n, &b); }
    }
    CHECK(n.alt_ref == 100.0 && n.alt_ref_seq == seq0 && n.cnt.ref_rejected == 1, "salto de 3 m: rechazado y contado");

    /* 7: una marca barométrica futura no estropea el orden */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    nav_tick(&n, 0.001);
    b.t = 1000.0; b.alt_m = 100.0f;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_FUTURE && n.cnt.time_future == 1, "t = 1000 s con reloj 0.001: futura");
    b.t = 0.0005;
    CHECK(nav_on_baro(&n, &b) == 1, "el siguiente baro normal se aplica (v3: −105 atrasado)");
    uint32_t why;
    CHECK(nav_validity(&n, &why) == NAV_VALID, "y la validez no se ve afectada (v3: STALE)");
    /* sin reloj, respecto a la IMU */
    CHECK(align_at_zero(&n, &c, 1), "rampa");
    b.t = 1000.0;
    CHECK(nav_on_baro(&n, &b) == NAV_E_TIME_FUTURE, "sin reloj: futura respecto a la IMU");
    b.t = 0.0005;
    CHECK(nav_on_baro(&n, &b) == 1, "sin reloj: el siguiente se aplica");
    /* IMU futura */
    nav_tick(&n, 0.001);
    s = rest(50.0);
    CHECK(nav_on_imu(&n, &s) == NAV_E_TIME_FUTURE && n.t_rx == 0.0, "IMU futura descartada sin tocar t_rx");
    s = rest(0.0009);
    CHECK(nav_on_imu(&n, &s) == NAV_OK, "la siguiente IMU normal se aplica");

    /* 8: la validez caduca con el reloj aunque no llegue nada */
    CHECK(align_at_zero(&n, &c, 1) && nav_validity(&n, NULL) == NAV_VALID, "rampa");
    CHECK(nav_tick(&n, 10.0) == NAV_OK && nav_validity(&n, &why) == NAV_INVALID && (why & NAV_R_STALE),
          "10 s sin muestras: STALE (v3: VÁLIDO)");
    CHECK(nav_tick(&n, 5.0) == NAV_E_TIME_ORDER && n.t_now == 10.0, "un reloj que retrocede se ignora");
    CHECK(nav_tick(&n, (double)NAN) == NAV_E_TIME_NONFINITE, "reloj NaN");
    nav_output_t o;
    nav_get_output(&n, &o);
    CHECK(o.t_eval == 10.0 && o.validity == NAV_INVALID && o.t_state == 0.0, "salida: t_eval 10, t_state 0");
}

static void v4_abi_float_double(void)
{
    printf("v4-10. float/double incoherente: se comprueba antes de tocar memoria\n");
    CHECK(offsetof(nav_t, magic) == 0 && offsetof(eskf_t, magic) == 0, "magic en el desplazamiento 0");
    static unsigned char buf[sizeof(nav_t) + 64];
    memset(buf, 0xA5, sizeof buf);
    nav_config_t c;
    cfg_test(&c);
    /* un llamador con otro eskf_real pasa otros tamaños */
    CHECK(nav_init_checked((nav_t *)(void *)buf, &c, sizeof(nav_t) - 8, sizeof(nav_config_t)) == NAV_E_CONFIG,
          "tamaño de nav_t distinto");
    CHECK(nav_init_checked((nav_t *)(void *)buf, &c, sizeof(nav_t), sizeof(nav_config_t) + 8) == NAV_E_CONFIG,
          "tamaño de nav_config_t distinto");
    CHECK(nav_default_config_checked((nav_config_t *)(void *)buf, sizeof(nav_config_t) - 4) == NAV_E_CONFIG,
          "default_config con tamaño distinto");
    int untouched = 1;
    for (size_t i = 0; i < sizeof buf; i++) untouched &= buf[i] == 0xA5;
    CHECK(untouched, "no se ha escrito nada");
    CHECK(strstr(nav_config_check_checked(&c, sizeof(nav_config_t) + 4), "float/double") != NULL, "config_check");
    static nav_t z; /* sin iniciar: todas las funciones lo rechazan leyendo sólo magic */
    nav_imu_sample_t s = rest(0.0);
    nav_baro_sample_t b = {0.0, 0.0f};
    CHECK(nav_on_imu(&z, &s) == NAV_E_NOT_INIT && nav_on_baro(&z, &b) == NAV_E_NOT_INIT &&
          nav_tick(&z, 0.0) == NAV_E_NOT_INIT, "sesión sin iniciar");
}

static void v4_saturacion_cuantizada(void)
{
    printf("v4-11. saturación por cuantización (límite en valor físico)\n");
    nav_config_t c;
    static nav_t n;
    cfg_test(&c);
    c.acc_sat_limit = 16 * 9.80665;
    nav_init(&n, &c);
    double amax = 32767.0 / 2048.0 * 9.80665;
    nav_imu_sample_t s = imu(0.0, amax, 0, 0, 0);
    nav_on_imu(&n, &s);
    CHECK(n.cnt.imu_acc_sat == 0, "límite «16 g» en valor físico no ve +32767 LSB (por eso se detecta por código)");
}

int main(void)
{
    regresion_1_tiempos();
    regresion_2_hueco();
    regresion_4_reinicio();
    regresion_5_saturacion();
    regresion_6_validez();
    regresion_7_adaptador();
    prueba_configuracion_pendiente();
    prueba_registro_continua();
    v4_ventanas_y_recuperacion();
    v4_despegue_y_baro();
    v4_abi_float_double();
    v4_saturacion_cuantizada();
    return report("test_nav");
}
