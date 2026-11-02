/*
 * vn_bench.c — ver vn_bench.h. Datos sintéticos SÓLO para medir tiempos y pila.
 */
#include "vn_bench.h"

#include <math.h>
#include <string.h>

static uint32_t lcg_state;
static double noise(void) /* uniforme en [-1, 1], determinista */
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (double)(lcg_state >> 8) / 8388607.5 - 1.0;
}

static void acc_add(vn_bench_acc_t *a, double us, double period_us)
{
    if (a->n == 0u || us < a->min) a->min = us;
    if (a->n == 0u || us > a->max) a->max = us;
    a->sum += us;
    a->n++;
    if (us > period_us) a->over++;
}

int vn_bench_run(nav_t *n, const nav_config_t *cfg, double fs_hz, vn_cycles_fn clk, double cycles_per_us,
                 vn_bench_result_t *r)
{
    memset(r, 0, sizeof *r);
    r->init_ret = nav_init(n, cfg);
    if (r->init_ret != NAV_OK) return -1;
    lcg_state = 12345u;

    const double g = 9.80665;
    const double dt = 1.0 / fs_hz, period_us = 1e6 / fs_hz;
    double t_pad = 2.0 * cfg->rest_window_s + 0.5; /* dos ventanas: alineación y refresco */
    if (!(t_pad > 2.0)) t_pad = 2.0;
    double a_up = 1.3 * cfg->liftoff_acc - g; /* ‖f‖ = 1.3·umbral durante el empuje */
    if (!(a_up > 30.0)) a_up = 30.0;
    const double t_burn = 3.0, t_coast = 4.0;
    const double t_end = t_pad + t_burn + t_coast;

    double alt = 0.0, vup = 0.0;
    long next_baro = 1;
    int was_aligned = 0;
    uint32_t baro_used0 = 0u;
    for (long k = 1;; k++) {
        double t = (double)k * dt;
        if (t > t_end) break;
        double f_down; /* fuerza específica en z (NED, cuerpo nivelado): reposo = −g */
        if (t < t_pad) f_down = -g;
        else if (t < t_pad + t_burn) f_down = -(g + a_up);
        else f_down = 0.0;
        /* cinemática vertical (sólo para que el baro sea coherente) */
        if (t >= t_pad) {
            double acc_up = -f_down - g;
            vup += acc_up * dt;
            alt += vup * dt;
        }
        nav_imu_sample_t s;
        s.t = t;
        s.acc[0] = (float)(0.02 * noise());
        s.acc[1] = (float)(0.02 * noise());
        s.acc[2] = (float)(f_down + 0.02 * noise());
        s.gyro[0] = (float)(0.002 * noise());
        s.gyro[1] = (float)(0.002 * noise());
        s.gyro[2] = (float)(0.002 * noise());
        s.sat = 0u;
        int in_flight = (n->phase == NAV_PHASE_FLIGHT);
        uint32_t c0 = clk();
        (void)nav_on_imu(n, &s);
        uint32_t c1 = clk();
        double us = (double)(uint32_t)(c1 - c0) / cycles_per_us;
        if (!was_aligned && n->aligned) {
            acc_add(&r->align, us, period_us);
            r->aligned = 1;
        } else if (in_flight) {
            acc_add(&r->imu_flight, us, period_us);
        } else {
            acc_add(&r->imu_pad, us, period_us);
        }
        was_aligned = n->aligned;
        if (!in_flight && n->phase == NAV_PHASE_FLIGHT) {
            r->liftoff = 1;
            baro_used0 = n->cnt.baro_used;
        }

        if (t * 50.0 >= (double)next_baro) { /* baro a 50 Hz, justo después de la IMU */
            next_baro++;
            nav_baro_sample_t b;
            b.t = t + 1e-4;
            b.alt_m = (float)(alt + 0.05 * noise());
            int fl = (n->phase == NAV_PHASE_FLIGHT);
            c0 = clk();
            (void)nav_on_baro(n, &b);
            c1 = clk();
            if (fl) acc_add(&r->baro_flight, (double)(uint32_t)(c1 - c0) / cycles_per_us, period_us);
        }
    }
    r->baro_used = r->liftoff && n->cnt.baro_used > baro_used0;
    return (r->aligned && r->liftoff && r->baro_used) ? 0 : -2;
}
