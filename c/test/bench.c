/*
 * bench.c — tiempo por operación EN EL ORDENADOR donde se ejecuta. No representa al
 * microcontrolador: allí hay que medir con un temporizador de ciclos (p. ej. DWT->CYCCNT
 * en Cortex-M) con el mismo compilador y opciones del firmware.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <time.h>

#include "navigation.h"

static double now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

int main(void)
{
    eskf_params_t prm;
    eskf_t f;
    eskf_default_params(&prm);
    prm.dt_max = (eskf_real)0.015;
    prm.has_mag_ref = 1;
    prm.mag_ref[0] = 27; prm.mag_ref[1] = 0; prm.mag_ref[2] = 34;
    eskf_init(&f, &prm);
    eskf_set_rest(&f, 1);
    eskf_real a[3] = {(eskf_real)0.1, 0, (eskf_real)-9.81}, w[3] = {(eskf_real)0.01, 0, 0};
    eskf_real m[3] = {27, 0, 34}, s3[3] = {(eskf_real)0.5, (eskf_real)0.5, (eskf_real)0.5};
    eskf_align_static(&f, a, w, m);
    f.prm.innov_gate = 0;
    const int n = 200000;
    volatile int sink = 0;
    double t0;

    printf("tiempo por operación en ESTE ordenador [%s], %d repeticiones:\n",
           sizeof(eskf_real) == 4 ? "float" : "double", n);
#define BENCH(label, expr)                                         \
    do {                                                           \
        eskf_init(&f, &prm); eskf_set_rest(&f, 1);                 \
        eskf_align_static(&f, a, w, m);                            \
        t0 = now_ns();                                             \
        for (int i = 0; i < n; i++) sink += (expr);                \
        printf("  %-28s %8.0f ns\n", label, (now_ns() - t0) / n);  \
    } while (0)
    BENCH("eskf_predict", eskf_predict(&f, a, w, (eskf_real)0.01));
    BENCH("eskf_update_baro_at (1)", eskf_update_baro_at(&f, 0, (eskf_real)0.5, (eskf_real)0.002));
    BENCH("eskf_update_zupt (3)", eskf_update_zupt(&f, s3));
    BENCH("eskf_update_accel (3)", eskf_update_accel(&f, a, s3));
    BENCH("eskf_update_mag (3)", eskf_update_mag(&f, m, s3));
    BENCH("eskf_update_gps (6)", eskf_update_gps(&f, f.p, f.v, s3, s3));
    printf("  sizeof(eskf_t) = %zu bytes\n", sizeof(eskf_t));

    /* navigation.c: nav_on_imu en vuelo (predicción + validaciones) */
    static nav_t nav;
    nav_config_t c;
    nav_default_config(&c);
    c.eskf.sigma_a = (eskf_real)0.05; c.eskf.sigma_w = (eskf_real)0.005;
    c.eskf.sigma_aw = (eskf_real)1e-4; c.eskf.sigma_ww = (eskf_real)1e-5;
    c.eskf.dt_max = (eskf_real)0.015; c.eskf.ba0_std = (eskf_real)0.2; c.eskf.bg0_std = (eskf_real)0.01;
    c.rest_window_s = 1.0; c.rest_acc_dev_max = 0.5; c.rest_acc_std_max = 0.2; c.rest_gyro_max = 0.2;
    c.liftoff_acc = 19.62; c.acc_sat_limit = 0; c.gyro_sat_limit = 0; c.baro_sigma = 0.5; c.baro_latency_s = 0;
    if (nav_init(&nav, &c) != NAV_OK) return 1;
    nav_imu_sample_t s;
    s.sat = 0;
    for (int i = 0; i < 3; i++) { s.acc[i] = 0; s.gyro[i] = 0; }
    s.acc[2] = -9.81f;
    nav_baro_sample_t b;
    b.alt_m = 0;
    for (int k = 0; k <= 200; k++) {
        s.t = k * 0.01;
        nav_on_imu(&nav, &s);
        b.t = s.t + 0.001;
        nav_on_baro(&nav, &b); /* referencia de altura */
    }
    if (!nav.have_alt_ref) return 1;
    nav.phase = NAV_PHASE_FLIGHT;
    eskf_set_rest(&nav.f, 0);
    t0 = now_ns();
    for (int i = 0; i < n; i++) { s.t = 2.0 + (i + 1) * 0.01; sink += nav_on_imu(&nav, &s); }
    printf("  %-28s %8.0f ns\n", "nav_on_imu (vuelo)", (now_ns() - t0) / n);
    uint32_t used0 = nav.cnt.baro_used;
    t0 = now_ns();
    for (int i = 0; i < n; i++) { b.t = s.t + 1e-9 * (i + 1); sink += nav_on_baro(&nav, &b); }
    printf("  %-28s %8.0f ns  (%u de %d aplicados)\n", "nav_on_baro", (now_ns() - t0) / n,
           (unsigned)(nav.cnt.baro_used - used0), n);
    printf("  sizeof(nav_t) = %zu bytes\n", sizeof(nav_t));
    return sink == 12345 ? 1 : 0;
}
