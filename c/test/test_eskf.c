/*
 * test_eskf.c — pruebas unitarias y de regresión del ESKF en C.
 *
 * Cada prueba "regresion_N_*" reproduce el fallo N de la revisión del 18-oct; todas
 * fallaban con la versión entregada (commit 80b1d06) y pasan con esta.
 * La equivalencia con eskf.py se comprueba aparte (compare_with_python.py) y la
 * simulación física en test_sim.c.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "eskf.h"
#include "test_util.h"

static const double G_NED[3] = {0, 0, -9.81};
static const eskf_real SIG3[3] = {(eskf_real)0.3, (eskf_real)0.3, (eskf_real)0.3};

/* Filtro inicializado, alineado sin magnetómetro y en reposo confirmado */
static void ready_filter(eskf_t *f, const eskf_params_t *prm)
{
    eskf_real a[3] = {0, 0, (eskf_real)-9.81}, w[3] = {0, 0, 0};
    CHECK(eskf_init(f, prm) == ESKF_OK, "init");
    CHECK(eskf_set_rest(f, 1) == ESKF_OK, "set_rest");
    CHECK(eskf_align_static(f, a, w, NULL) == ESKF_OK, "align");
}

/* ------------------------------------------------------------------ 1 */
static void regresion_1_entradas_no_finitas(void)
{
    printf("1. entradas y parámetros no finitos\n");
    eskf_params_t prm;
    eskf_t f;
    snapshot_t s;
    test_params(&prm);
    prm.innov_gate = 5;
    ready_filter(&f, &prm);
    const eskf_real nan = (eskf_real)NAN, inf = (eskf_real)INFINITY;

    /* baro */
    struct { eskf_real z, s; } baro_cases[] = {{nan, 0.5f}, {inf, 0.5f}, {1, nan}, {1, inf},
                                               {1, 0}, {1, -0.5f}, {1, (eskf_real)1e-30}};
    for (size_t k = 0; k < sizeof baro_cases / sizeof baro_cases[0]; k++) {
        snap(&f, &s);
        int r = eskf_update_baro(&f, baro_cases[k].z, baro_cases[k].s);
        /* 1e-30² no es representable en float (underflow a 0): se rechaza ahí; en double es
           una varianza válida y la medida se aplica. */
        int expect_reject = !(k == 6 && sizeof(eskf_real) == 8);
        if (expect_reject) {
            CHECK(r == ESKF_E_INPUT && unchanged(&f, &s), "baro caso %zu: ret %d", k, r);
            CHECK(f.last.status == ESKF_E_INPUT && f.last.sensor == ESKF_SENSOR_BARO &&
                  f.last.n_used == 0, "baro caso %zu: diagnóstico", k);
        } else {
            CHECK(r == 1 && P_ok(&f), "baro sigma 1e-30 en double: ret %d", r);
            ready_filter(&f, &prm);
        }
    }

    /* un NaN en cada tipo de medida o sigma */
    eskf_real ok3[3] = {0, 0, (eskf_real)-9.81}, bad3[3] = {0, nan, 0}, sig_bad[3] = {1, 1, inf};
    f.heading_known = 1; /* para llegar a la validación de GPS/vel/mag */
    f.prm.has_mag_ref = 1;
    f.prm.mag_ref[0] = 27; f.prm.mag_ref[1] = 0; f.prm.mag_ref[2] = 34;
    snap(&f, &s);
    CHECK(eskf_update_gps(&f, bad3, ok3, SIG3, SIG3) == ESKF_E_INPUT && unchanged(&f, &s), "gps p NaN");
    CHECK(eskf_update_gps(&f, ok3, ok3, SIG3, sig_bad) == ESKF_E_INPUT && unchanged(&f, &s), "gps sigma inf");
    CHECK(eskf_update_velocity(&f, bad3, SIG3) == ESKF_E_INPUT && unchanged(&f, &s), "vel NaN");
    CHECK(eskf_update_zupt(&f, sig_bad) == ESKF_E_INPUT && unchanged(&f, &s), "zupt sigma inf");
    CHECK(eskf_update_mag(&f, bad3, SIG3) == ESKF_E_INPUT && unchanged(&f, &s), "mag NaN");
    CHECK(eskf_update_accel(&f, bad3, SIG3) == ESKF_E_INPUT && unchanged(&f, &s), "accel NaN");
    CHECK(eskf_predict(&f, bad3, ok3, (eskf_real)0.01) == ESKF_E_INPUT && unchanged(&f, &s), "predict a NaN");
    CHECK(eskf_predict(&f, ok3, bad3, (eskf_real)0.01) == ESKF_E_INPUT && unchanged(&f, &s), "predict w NaN");
    CHECK(eskf_update_gps(&f, NULL, ok3, SIG3, SIG3) == ESKF_E_NULL && unchanged(&f, &s), "gps NULL");
    CHECK(eskf_predict(NULL, ok3, ok3, (eskf_real)0.01) == ESKF_E_NULL, "predict NULL");

    /* parámetros: proceso >= 0 (0 admitido), medida/umbral > 0, todo finito */
    struct { const char *name; void (*mut)(eskf_params_t *); } bad_prm[] = {
        {"sigma_a NaN", NULL}, {"sigma_w < 0", NULL}, {"has_mag 2", NULL}, {"mag_ref 0", NULL},
        {"mag_ref inf", NULL}, {"threshold 0", NULL}, {"gate < 0", NULL}, {"dt_max 0", NULL},
        {"gap_max < dt_max", NULL}, {"gap_max > 64 dt_max", NULL}, {"p0_std NaN", NULL},
        {"align_gyro_max 0", NULL}};
    for (size_t k = 0; k < sizeof bad_prm / sizeof bad_prm[0]; k++) {
        eskf_params_t b;
        test_params(&b);
        switch (k) {
        case 0: b.sigma_a = nan; break;
        case 1: b.sigma_w = (eskf_real)-1e-3; break;
        case 2: b.has_mag_ref = 2; break;
        case 3: b.has_mag_ref = 1; break;
        case 4: b.has_mag_ref = 1; b.mag_ref[0] = inf; break;
        case 5: b.accel_threshold = 0; break;
        case 6: b.innov_gate = -1; break;
        case 7: b.dt_max = 0; break;
        case 8: b.gap_max = (eskf_real)0.01; break;
        case 9: b.gap_max = (eskf_real)(0.015 * 65); break;
        case 10: b.p0_std = nan; break;
        case 11: b.align_gyro_max = 0; break;
        }
        eskf_t g;
        CHECK(eskf_init(&g, &b) == ESKF_E_CONFIG, "init acepta %s", bad_prm[k].name);
        CHECK(eskf_predict(&g, ok3, ok3, (eskf_real)0.01) == ESKF_E_NOT_INIT, "filtro usable tras init fallido (%s)",
              bad_prm[k].name);
    }
    eskf_params_t z;
    test_params(&z);
    z.sigma_a = z.sigma_w = z.sigma_aw = z.sigma_ww = 0; /* ensayo determinista */
    z.p0_std = 0;
    CHECK(eskf_init(&f, &z) == ESKF_OK, "ruidos de proceso y P0 nulos deben admitirse");
    CHECK(eskf_default_params_checked(&z, sizeof(eskf_params_t) + 1) == ESKF_E_CONFIG, "ABI params");
    CHECK(eskf_init_checked(&f, &prm, sizeof(eskf_t), sizeof(eskf_real) == 8 ? 4 : 8) == ESKF_E_CONFIG,
          "ABI float/double incoherente no detectada");
}

/* quat_normalize ya no sustituye en silencio un cuaternión inválido por la identidad */
static void regresion_1b_cuaternion_invalido(void)
{
    printf("1b. cuaternión inválido -> fallo explícito, recuperación por realineación\n");
    eskf_params_t prm;
    eskf_t f;
    test_params(&prm);
    ready_filter(&f, &prm);
    eskf_real a[3] = {0, 0, (eskf_real)-9.81}, w[3] = {0, 0, 0};
    f.q[0] = f.q[1] = f.q[2] = f.q[3] = 0; /* corrupción simulada (p. ej. memoria) */
    int r = eskf_predict(&f, a, w, (eskf_real)0.01);
    CHECK(r == ESKF_E_NUMERIC && f.fault == 1, "q = 0 debe dar ESKF_E_NUMERIC y fallo (ret %d)", r);
    CHECK(f.q[0] == 0 && f.q[1] == 0, "q no debe sustituirse por la identidad");
    CHECK(eskf_predict(&f, a, w, (eskf_real)0.01) == ESKF_E_FAULT, "el fallo debe quedar enclavado");
    CHECK(eskf_check_health(&f) == ESKF_E_FAULT, "check_health con fallo enclavado debe dar ESKF_E_FAULT");
    {
        eskf_t g2;
        ready_filter(&g2, &prm);
        g2.fault = 1; /* campos finitos, fallo enclavado: en la v2 devolvía ESKF_OK */
        CHECK(eskf_check_health(&g2) == ESKF_E_FAULT, "fallo con campos finitos");
    }
    CHECK(eskf_update_baro(&f, 0, 1) == ESKF_E_FAULT && f.last.status == ESKF_E_FAULT, "update en fallo");
    CHECK(eskf_align_static(&f, a, w, NULL) == ESKF_OK && f.fault == 0 && P_ok(&f), "realinear recupera");
    CHECK(eskf_predict(&f, a, w, (eskf_real)0.01) == ESKF_OK, "predict tras recuperar");

    /* P indefinida -> S <= 0 -> fallo numérico, no inyección */
    ready_filter(&f, &prm);
    f.P[2][2] = -10;
    r = eskf_update_baro(&f, 1, (eskf_real)0.5);
    CHECK(r == ESKF_E_NUMERIC && f.fault && f.last.n_numeric == 1, "S <= 0 debe enclavar fallo (ret %d)", r);
}

/* ------------------------------------------------------------------ 2 */
static void regresion_2_dt(void)
{
    printf("2. contrato temporal de eskf_predict\n");
    eskf_params_t prm;
    eskf_t f;
    snapshot_t s;
    test_params(&prm);
    ready_filter(&f, &prm);
    eskf_real a[3] = {(eskf_real)0.2, 0, (eskf_real)-9.81}, w[3] = {(eskf_real)0.1, 0, 0};
    const double bad[] = {-0.01, 0.0, NAN, INFINITY, -INFINITY};
    for (int k = 0; k < 5; k++) {
        snap(&f, &s);
        int r = eskf_predict(&f, a, w, (eskf_real)bad[k]);
        CHECK(r == ESKF_E_DT && unchanged(&f, &s), "dt=%g: ret %d", bad[k], r);
    }
    snap(&f, &s);
    int r = eskf_predict(&f, a, w, (eskf_real)10.0);
    CHECK(r == ESKF_E_DT_GAP && unchanged(&f, &s) && (f.degraded & ESKF_DEG_GAP_LOST),
          "dt=10 sin puente: ret %d", r);
    CHECK(f.cnt.n_dt_rejected == 6, "contador de rechazos %u", (unsigned)f.cnt.n_dt_rejected);

    /* hueco puenteable: 0.05 s con dt_max 0.015 -> 4 subpasos de 0.0125 */
    prm.gap_max = (eskf_real)0.1;
    eskf_t g, h;
    ready_filter(&g, &prm);
    ready_filter(&h, &prm);
    r = eskf_predict(&g, a, w, (eskf_real)0.05);
    for (int k = 0; k < 4; k++) eskf_predict(&h, a, w, (eskf_real)0.0125);
    CHECK(r == ESKF_OK && (g.degraded & ESKF_DEG_GAP_BRIDGED) && g.cnt.n_gaps_bridged == 1, "puente: ret %d", r);
    /* el estado nominal es el de 4 pasos con la misma muestra; P no (ver prueba_puente_estadistica) */
    CHECK(memcmp(g.p, h.p, sizeof g.p) == 0 && memcmp(g.v, h.v, sizeof g.v) == 0 && memcmp(g.q, h.q, sizeof g.q) == 0,
          "nominal del puente = 4 pasos con la misma muestra");
    CHECK(g.P[ESKF_IDX_V][ESKF_IDX_V] > h.P[ESKF_IDX_V][ESKF_IDX_V], "el puente no debe ser más confiado que 4 lecturas");
    CHECK(eskf_set_rest(&g, 1) == ESKF_OK && eskf_align_static(&g, a, w, NULL) == ESKF_OK && g.degraded == 0u,
          "realinear borra los bits degraded");
    CHECK(eskf_predict(&g, a, w, (eskf_real)0.2) == ESKF_E_DT_GAP, "0.2 s > gap_max debe rechazarse");
}

/* ------------------------------------------------------------------ 4 */
static void regresion_4_psd(void)
{
    printf("4. comprobación de semidefinición positiva\n");
    eskf_real P[ESKF_N][ESKF_N];
    for (int i = 0; i < ESKF_N; i++)
        for (int j = 0; j < ESKF_N; j++) P[i][j] = (i == j) ? 1 : 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i != j) P[i][j] = (eskf_real)-0.9; /* autovalores 1.9, 1.9, -0.8 */
    int pairwise = 1;
    for (int i = 0; i < ESKF_N; i++)
        for (int j = 0; j < ESKF_N; j++)
            if (P[i][j] * P[i][j] > P[i][i] * P[j][j]) pairwise = 0;
    CHECK(pairwise == 1, "el caso debe pasar la comprobación antigua por parejas");
    CHECK(!matrix_is_psd(P, PSD_REL, NULL), "Cholesky debe rechazar λmin = -0.8");
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i != j) P[i][j] = (eskf_real)0.4999; /* autovalores > 0 */
    CHECK(matrix_is_psd(P, PSD_REL, NULL), "Cholesky debe aceptar una P definida positiva");
    P[0][0] = 0; /* semidefinida con autovalor 0 exacto en la frontera */
    for (int j = 1; j < ESKF_N; j++) P[0][j] = P[j][0] = 0;
    CHECK(matrix_is_psd(P, PSD_REL, NULL), "debe aceptar autovalor 0 (semidefinida)");
}

/* ------------------------------------------------------------------ 5 */
static void regresion_5_cancelacion(void)
{
    printf("5. cancelación en la actualización de covarianza\n");
    eskf_params_t prm;
    eskf_t f;
    test_params(&prm);
    double worst = 0, worst_ratio = 0;
    for (int e = 0; e <= 10; e++) { /* P_zz/r de 4 a 4e10 */
        double Pzz = pow(10.0, e), r = 0.25, exact = Pzz * r / (Pzz + r);
        ready_filter(&f, &prm);
        f.P[2][2] = (eskf_real)Pzz;
        int ret = eskf_update_baro(&f, 0, (eskf_real)0.5);
        double rel = fabs((double)f.P[2][2] - exact) / exact;
        if (rel > worst) { worst = rel; worst_ratio = Pzz / r; }
        CHECK(ret == 1 && P_ok(&f), "P_zz=%g: ret %d o P no PSD", Pzz, ret);
    }
    printf("   error relativo máx de la varianza posterior: %.2e (P_zz/r = %.0e)\n", worst, worst_ratio);
    CHECK(worst < 1e-5, "varianza posterior con error relativo %.2e", worst);
    ready_filter(&f, &prm);
    f.P[2][2] = (eskf_real)1e7;
    eskf_update_baro(&f, 0, (eskf_real)0.5);
    printf("   caso informado: P_zz 1e7, sigma 0.5 -> %.6g (antes en float: 0)\n", (double)f.P[2][2]);
    CHECK(fabs((double)f.P[2][2] - 0.25) < 1e-6, "caso informado: %.9g", (double)f.P[2][2]);

    /* Caso realista con correlaciones: P0 grande en posición y velocidad, varias updates */
    prm.p0_std = (eskf_real)3000;
    prm.v0_std = (eskf_real)300;
    ready_filter(&f, &prm);
    eskf_real a[3] = {0, 0, (eskf_real)-9.81}, w[3] = {0, 0, 0};
    for (int k = 0; k < 2000; k++) {
        eskf_predict(&f, a, w, (eskf_real)0.01);
        if (k % 5 == 0) eskf_update_baro(&f, 0, (eskf_real)0.5);
        if (k % 100 == 0) CHECK(P_ok(&f), "P no PSD en el paso %d", k);
    }
    CHECK(f.P[2][2] > 0 && f.P[2][2] < (eskf_real)0.25, "P_zz final %.3g", (double)f.P[2][2]);
}

/* ------------------------------------------------------------------ 7 */
static void regresion_7_alineacion(void)
{
    printf("7. alineación estática: validación, marco y realineación\n");
    eskf_params_t prm;
    eskf_t f;
    snapshot_t s;
    test_params(&prm);
    prm.has_mag_ref = 1;
    prm.mag_ref[0] = 27; prm.mag_ref[1] = (eskf_real)-0.5; prm.mag_ref[2] = 34;
    eskf_real a[3] = {0, 0, (eskf_real)-9.81}, w[3] = {0, 0, 0}, m[3] = {27, (eskf_real)-0.5, 34};
    eskf_real a_nan[3] = {(eskf_real)NAN, 0, (eskf_real)-9.81}, w_nan[3] = {(eskf_real)NAN, 0, 0};
    eskf_real m_nan[3] = {1, (eskf_real)NAN, 0}, a3g[3] = {0, 0, (eskf_real)-29.43};
    eskf_real w_big[3] = {(eskf_real)0.5, 0, 0}, m_vert[3] = {0, 0, 40}, m_zero[3] = {0, 0, 0};

    CHECK(eskf_init(&f, &prm) == ESKF_OK, "init");
    snap(&f, &s);
    CHECK(eskf_align_static(&f, a, w, m) == ESKF_E_NOT_AT_REST && unchanged(&f, &s), "sin reposo confirmado");
    eskf_set_rest(&f, 1);
    snap(&f, &s);
    CHECK(eskf_align_static(&f, a_nan, w, NULL) == ESKF_E_INPUT && unchanged(&f, &s), "acc NaN");
    CHECK(eskf_align_static(&f, a, w_nan, NULL) == ESKF_E_INPUT && unchanged(&f, &s), "gyro NaN");
    CHECK(eskf_align_static(&f, a, w, m_nan) == ESKF_E_INPUT && unchanged(&f, &s), "mag NaN");
    CHECK(eskf_align_static(&f, a, w, m_zero) == ESKF_E_INPUT && unchanged(&f, &s), "mag nulo");
    CHECK(eskf_align_static(&f, a3g, w, NULL) == ESKF_E_MOTION && unchanged(&f, &s), "|a| = 3 g");
    CHECK(eskf_align_static(&f, a, w_big, NULL) == ESKF_E_MOTION && unchanged(&f, &s), "giro 0.5 rad/s");
    CHECK(eskf_align_static(&f, a, w, m_vert) == ESKF_E_DEGENERATE && unchanged(&f, &s), "mag vertical");
    f.prm.has_mag_ref = 0;
    CHECK(eskf_align_static(&f, a, w, m) == ESKF_E_NO_REF && unchanged(&f, &s), "mag sin mag_ref");
    f.prm.has_mag_ref = 1;

    /* Marco: con mag -> rumbo geográfico; sin mag -> marco local, GPS/vel/mag rechazados */
    CHECK(eskf_align_static(&f, a, w, m) == ESKF_OK && f.heading_known == 1, "con mag");
    CHECK(eskf_align_static(&f, a, w, NULL) == ESKF_OK && f.heading_known == 0, "sin mag");
    eskf_real z3[3] = {0, 0, 0};
    CHECK(eskf_update_gps(&f, z3, z3, SIG3, SIG3) == ESKF_E_NO_HEADING, "GPS sin rumbo");
    CHECK(eskf_update_velocity(&f, z3, SIG3) == ESKF_E_NO_HEADING, "velocidad sin rumbo");
    CHECK(eskf_update_mag(&f, m, SIG3) == ESKF_E_NO_HEADING, "mag sin rumbo");
    CHECK(eskf_update_zupt(&f, SIG3) == 3, "ZUPT no depende del rumbo");

    /* Realineación: reinicio completo, sin correlaciones heredadas */
    eskf_real am[3] = {(eskf_real)0.3, 0, (eskf_real)-9.81};
    for (int k = 0; k < 500; k++) eskf_predict(&f, am, w, (eskf_real)0.01);
    CHECK(fabs((double)f.P[0][3]) > 1, "debe haber correlación p-v antes de realinear");
    CHECK(eskf_align_static(&f, a, w, m) == ESKF_OK, "realineación");
    int clean = 1;
    for (int i = 0; i < ESKF_N; i++)
        for (int j = 0; j < ESKF_N; j++)
            if (i != j && f.P[i][j] != 0) clean = 0;
    CHECK(clean && f.P[0][0] == prm.p0_std * prm.p0_std && f.v[0] == 0 && f.p[0] == 0,
          "la realineación debe reiniciar estado y P");

    /* Precisión de la alineación con rumbo (1000 orientaciones aleatorias) */
    unsigned st = 7u;
    double worst = 0;
    for (int t = 0; t < 1000; t++) {
        eskf_real q[4], aa[3], mm[3];
        double ad[3], md[3], mref[3] = {27, -0.5, 34};
        random_quat(&st, q);
        rotT(q, G_NED, ad);
        rotT(q, mref, md);
        for (int i = 0; i < 3; i++) { aa[i] = (eskf_real)ad[i]; mm[i] = (eskf_real)md[i]; }
        if (eskf_align_static(&f, aa, w, mm) != ESKF_OK) { worst = 1e9; break; }
        double e = rot_angle_between(q, f.q);
        if (e > worst) worst = e;
    }
    printf("   alineación con mag: error angular máx %.2e rad (1000 orientaciones)\n", worst);
    CHECK(worst < 1e-4, "error de alineación %.3e", worst);

    /* Cohete vertical (eje x de cuerpo hacia arriba) sin mag */
    eskf_real av[3] = {(eskf_real)9.81, 0, 0}, rpy[3];
    CHECK(eskf_align_static(&f, av, w, NULL) == ESKF_OK, "vertical sin mag");
    eskf_quat_to_euler(f.q, rpy);
    CHECK(fabs(fabs((double)rpy[1]) - 1.5707963267948966) < 1e-3, "pitch vertical %.4f", (double)rpy[1]);
}

/* ------------------------------------------------------------------ 8 */
static void regresion_8_reposo(void)
{
    printf("8. gravedad y ZUPT sólo con reposo confirmado\n");
    eskf_params_t prm;
    eskf_t f;
    snapshot_t s;
    test_params(&prm);
    ready_filter(&f, &prm);
    eskf_real ah[3] = {(eskf_real)2.0, 0, (eskf_real)-9.81}; /* reposo + 2 m/s² horizontal */
    double dn = sqrt(4.0 + 9.81 * 9.81) - 9.81;
    eskf_set_rest(&f, 0); /* vuelo: el llamador retira el reposo */
    snap(&f, &s);
    int r = eskf_update_accel(&f, ah, SIG3);
    CHECK(r == ESKF_E_NOT_AT_REST && unchanged(&f, &s), "en vuelo el accel no debe usarse como gravedad (%d)", r);
    CHECK(eskf_update_zupt(&f, SIG3) == ESKF_E_NOT_AT_REST && unchanged(&f, &s), "ZUPT en vuelo");
    eskf_real a[3] = {0, 0, (eskf_real)-9.81}, w[3] = {0, 0, 0};
    CHECK(eskf_align_static(&f, a, w, NULL) == ESKF_E_NOT_AT_REST, "alinear en vuelo");
    CHECK(eskf_predict(&f, ah, w, (eskf_real)0.01) == ESKF_OK, "la predicción SÍ usa el accel en vuelo");

    /* Documenta el límite del umbral de norma: si el llamador afirma reposo por error,
       |‖a‖−g| = 0.20 < 0.5 y la medida se acepta. La responsabilidad es del llamador. */
    eskf_set_rest(&f, 1);
    r = eskf_update_accel(&f, ah, SIG3);
    printf("   |‖a‖−g| con 2 m/s² horizontales = %.3f m/s² < umbral 0.5: el umbral solo no basta\n", dn);
    CHECK(r == 3, "con reposo afirmado la norma no lo detecta (esperado, ret %d)", r);
    eskf_real a3g[3] = {0, 0, (eskf_real)-29.43};
    CHECK(eskf_update_accel(&f, a3g, SIG3) == ESKF_E_MOTION, "la norma sí rechaza 3 g");
}

/* ------------------------------------------------------------------ 9 */
static void regresion_9_diagnosticos(void)
{
    printf("9. diagnósticos de la última actualización\n");
    eskf_params_t prm;
    eskf_t f;
    test_params(&prm);
    prm.innov_gate = 3;
    ready_filter(&f, &prm);
    CHECK(eskf_update_baro(&f, (eskf_real)0.5, (eskf_real)0.5) == 1 && f.last.n_used == 1 && f.last.nis > 0,
          "baro usado");
    eskf_real a3g[3] = {0, 0, (eskf_real)-29.43};
    CHECK(eskf_update_accel(&f, a3g, SIG3) == ESKF_E_MOTION, "rechazo por movimiento");
    CHECK(f.last.sensor == ESKF_SENSOR_ACCEL && f.last.status == ESKF_E_MOTION && f.last.n_used == 0 &&
          f.last.nis == 0, "diagnóstico tras rechazo por movimiento");
    CHECK(eskf_update_mag(&f, a3g, SIG3) == ESKF_E_NO_REF && f.last.sensor == ESKF_SENSOR_MAG &&
          f.last.status == ESKF_E_NO_REF && f.last.nis == 0, "diagnóstico sin referencia");
    CHECK(eskf_update_baro(&f, 500, (eskf_real)0.5) == ESKF_E_GATED && f.last.n_gated == 1 &&
          f.last.n_used == 0 && f.last.status == ESKF_E_GATED, "diagnóstico de puerta");
    CHECK(eskf_update_baro(&f, (eskf_real)NAN, (eskf_real)0.5) == ESKF_E_INPUT && f.last.n_gated == 0 &&
          f.last.status == ESKF_E_INPUT, "diagnóstico de entrada inválida");
    eskf_set_rest(&f, 0);
    CHECK(eskf_update_zupt(&f, SIG3) == ESKF_E_NOT_AT_REST && f.last.sensor == ESKF_SENSOR_ZUPT,
          "diagnóstico sin reposo");
    CHECK(strcmp(eskf_status_str(ESKF_E_GATED), "rechazada por la puerta de innovación") == 0, "status_str");
}

/* ------------------------------------------------------------------ otros */
static void prueba_zupt_rampa(void)
{
    printf("ZUPT + baro en rampa\n");
    eskf_params_t prm;
    eskf_t f, g;
    test_params(&prm);
    ready_filter(&f, &prm);
    ready_filter(&g, &prm);
    eskf_real a[3] = {(eskf_real)0.05, (eskf_real)-0.03, (eskf_real)(-9.81 + 0.04)}, w[3] = {0, 0, 0};
    eskf_real sv[3] = {(eskf_real)0.02, (eskf_real)0.02, (eskf_real)0.02};
    for (int k = 0; k < 6000; k++) {
        eskf_predict(&f, a, w, (eskf_real)0.01);
        eskf_predict(&g, a, w, (eskf_real)0.01);
        if (k % 10 == 0) {
            CHECK(eskf_update_zupt(&g, sv) == 3, "zupt");
            CHECK(eskf_update_baro(&g, 0, (eskf_real)0.5) == 1, "baro");
        }
    }
    double vf = sqrt((double)(f.v[0] * f.v[0] + f.v[1] * f.v[1] + f.v[2] * f.v[2]));
    double vg = sqrt((double)(g.v[0] * g.v[0] + g.v[1] * g.v[1] + g.v[2] * g.v[2]));
    printf("   60 s en rampa: |v| solo IMU = %.2f m/s, con ZUPT+baro = %.4f m/s\n", vf, vg);
    CHECK(vg < 0.05, "ZUPT no acota la velocidad: %.3f", vg);
    CHECK(P_ok(&g), "P no PSD tras ZUPT");
}

static void prueba_puerta(void)
{
    printf("puerta de innovación\n");
    eskf_params_t prm;
    eskf_t f;
    snapshot_t s;
    test_params(&prm);
    prm.innov_gate = 5;
    ready_filter(&f, &prm);
    snap(&f, &s);
    CHECK(eskf_update_baro(&f, 500, (eskf_real)0.5) == ESKF_E_GATED && unchanged(&f, &s), "pico de 500 m");
    CHECK(eskf_update_baro(&f, (eskf_real)0.3, (eskf_real)0.5) == 1, "medida normal");
}

static void prueba_baro_desfase_e_inflado(void)
{
    printf("baro con desfase temporal e inflado de covarianza\n");
    eskf_params_t prm;
    eskf_t f, g;
    snapshot_t s;
    test_params(&prm);
    ready_filter(&f, &prm);
    f.v[2] = -100; /* subiendo a 100 m/s */
    g = f;
    CHECK(eskf_update_baro_at(&f, (eskf_real)0.3, (eskf_real)0.5, 0) == 1 &&
          eskf_update_baro(&g, (eskf_real)0.3, (eskf_real)0.5) == 1 && memcmp(f.P, g.P, sizeof f.P) == 0 &&
          memcmp(f.p, g.p, sizeof f.p) == 0, "tau = 0 debe ser idéntico a eskf_update_baro");
    ready_filter(&f, &prm);
    f.v[2] = -100;
    eskf_real z0 = f.p[2];
    /* medida 10 ms después: el cohete ha subido 1 m (z NED baja 1 m) -> innovación nula */
    CHECK(eskf_update_baro_at(&f, z0 - 1, (eskf_real)0.5, (eskf_real)0.01) == 1, "baro_at");
    CHECK(fabs((double)(f.p[2] - z0)) < 1e-6 && fabs((double)f.v[2] + 100) < 1e-6,
          "con el desfase bien modelado no debe corregir nada (Δz=%.2e)", (double)(f.p[2] - z0));
    snap(&f, &s);
    CHECK(eskf_update_baro_at(&f, 0, (eskf_real)0.5, (eskf_real)0.016) == ESKF_E_INPUT && unchanged(&f, &s),
          "|tau| > dt_max");
    CHECK(eskf_update_baro_at(&f, 0, (eskf_real)0.5, (eskf_real)NAN) == ESKF_E_INPUT && unchanged(&f, &s), "tau NaN");
    f.prm.tau_max = (eskf_real)0.03; /* latencia real mayor que dt_max */
    CHECK(eskf_update_baro_at(&f, f.p[2], (eskf_real)0.5, (eskf_real)-0.025) == 1, "|tau| 0.025 <= tau_max 0.03");
    snap(&f, &s);
    CHECK(eskf_update_baro_at(&f, 0, (eskf_real)0.5, (eskf_real)0.031) == ESKF_E_INPUT && unchanged(&f, &s),
          "|tau| > tau_max");
    eskf_params_t bp;
    test_params(&bp);
    bp.tau_max = (eskf_real)-0.01;
    CHECK(eskf_validate_params(&bp) == ESKF_E_CONFIG, "tau_max < 0");

    eskf_real add[3] = {1, 2, 3}, bad[3] = {1, (eskf_real)NAN, 0}, neg[3] = {-1, 0, 0};
    eskf_real pv0 = f.P[ESKF_IDX_V][ESKF_IDX_V];
    CHECK(eskf_inflate_block(&f, ESKF_IDX_V, add) == ESKF_OK && f.P[ESKF_IDX_V][ESKF_IDX_V] == pv0 + 1 &&
          f.P[ESKF_IDX_V + 2][ESKF_IDX_V + 2] > 9 && P_ok(&f), "inflado de velocidad");
    snap(&f, &s);
    CHECK(eskf_inflate_block(&f, ESKF_IDX_V, bad) == ESKF_E_INPUT && unchanged(&f, &s), "inflado NaN");
    CHECK(eskf_inflate_block(&f, ESKF_IDX_V, neg) == ESKF_E_INPUT && unchanged(&f, &s), "inflado negativo");
    CHECK(eskf_inflate_block(&f, 4, add) == ESKF_E_INPUT && unchanged(&f, &s), "bloque inexistente");
}

/* Coherencia estadística del puente (problema 3). Monte Carlo independiente del filtro:
   se sortea UN error por lectura, se aplica a la lectura retenida durante T y se compara
   la covarianza empírica del error con la P del filtro. Con el puente de la v2 el cociente
   empírico/P en v y θ era 4 (= nº de subpasos). */
static void mc_bridge(int nsub_expected, double T, double dt_max, double gap_max, int independent_samples,
                      double ratio[3])
{
    const int NTR = 4000;
    const double sa = 1.0, sw = 0.1;
    eskf_params_t prm;
    test_params(&prm);
    prm.sigma_a = (eskf_real)sa; prm.sigma_w = (eskf_real)sw; prm.sigma_aw = 0; prm.sigma_ww = 0;
    prm.p0_std = prm.v0_std = prm.th0_std = prm.ba0_std = prm.bg0_std = 0;
    prm.dt_max = (eskf_real)dt_max; prm.gap_max = (eskf_real)gap_max;
    unsigned st = 99u;
    double s_vv = 0, s_tt = 0, s_pp = 0, P_vv = 0, P_tt = 0, P_pp = 0;
    for (int k = 0; k < NTR; k++) {
        eskf_t f;
        eskf_init(&f, &prm);
        int nsteps = independent_samples ? nsub_expected : 1;
        double h = T / nsteps;
        for (int j = 0; j < nsteps; j++) { /* lecturas: una sola retenida, o nsteps independientes */
            eskf_real a[3], w[3];
            for (int i = 0; i < 3; i++) {
                a[i] = (eskf_real)(sa * nrand(&st));
                w[i] = (eskf_real)(sw * nrand(&st));
            }
            a[2] = (eskf_real)((double)a[2] - 9.81);
            eskf_predict(&f, a, w, (eskf_real)h);
        }
        if (!independent_samples && f.cnt.n_gaps_bridged != 1u) printf("  (no se puenteó)\n");
        double th_x = 2.0 * (double)f.q[1] / (double)f.q[0]; /* verdad: identidad */
        s_vv += (double)f.v[0] * (double)f.v[0];
        s_tt += th_x * th_x;
        s_pp += (double)f.p[0] * (double)f.p[0];
        P_vv += (double)f.P[ESKF_IDX_V][ESKF_IDX_V];
        P_tt += (double)f.P[ESKF_IDX_TH][ESKF_IDX_TH];
        P_pp += (double)f.P[ESKF_IDX_P][ESKF_IDX_P];
    }
    ratio[0] = s_vv / P_vv;
    ratio[1] = s_tt / P_tt;
    ratio[2] = s_pp / P_pp;
}

static void prueba_puente_estadistica(void)
{
    printf("3. coherencia estadística del puente (Monte Carlo, 4000 ensayos)\n");
    double r[3];
    /* una lectura retenida 0.04 s, puenteada en 4 subpasos */
    mc_bridge(4, 0.04, 0.01, 0.05, 0, r);
    printf("   lectura retenida, puente: empírica/P  v %.3f  θ %.3f  p %.3f (v2: v 4, θ 4)\n", r[0], r[1], r[2]);
    CHECK(r[0] > 0.9 && r[0] < 1.1, "v: %.3f", r[0]);
    CHECK(r[1] > 0.9 && r[1] < 1.1, "θ: %.3f", r[1]);
    CHECK(r[2] > 0.55 && r[2] < 1.05, "p conservadora (36/56 = 0.64 esperado): %.3f", r[2]);
    /* funcionamiento normal: 4 lecturas independientes: la parametrización no cambia */
    mc_bridge(4, 0.04, 0.015, 0.0, 1, r);
    printf("   4 lecturas independientes, paso normal: empírica/P  v %.3f  θ %.3f  p %.3f\n", r[0], r[1], r[2]);
    CHECK(r[0] > 0.9 && r[0] < 1.1 && r[1] > 0.9 && r[1] < 1.1, "paso normal: v %.3f θ %.3f", r[0], r[1]);
}

int main(void)
{
    regresion_1_entradas_no_finitas();
    regresion_1b_cuaternion_invalido();
    regresion_2_dt();
    regresion_4_psd();
    regresion_5_cancelacion();
    regresion_7_alineacion();
    regresion_8_reposo();
    regresion_9_diagnosticos();
    prueba_zupt_rampa();
    prueba_puerta();
    prueba_baro_desfase_e_inflado();
    prueba_puente_estadistica();
    return report("test_eskf");
}
