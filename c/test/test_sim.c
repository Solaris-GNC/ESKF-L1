/*
 * test_sim.c — simulación de vuelo que ejecuta el MÓDULO QUE SE INTEGRA (navigation.c),
 * no un adaptador propio de la prueba.
 *
 * Todo lo numérico (frecuencias, rangos, ruidos, empuje, latencia) es una HIPÓTESIS DE
 * PRUEBA, no la sensórica del Viradinha MKII. Las cotas son de regresión de esta
 * simulación, no requisitos de vuelo.
 *
 * Verdad a 10 kHz: rampa inclinada 5°, empuje, resistencia ∝ v², alabeo y cabeceo.
 * "Adquisición" simulada: muestras con marca de tiempo (jitter), bits de saturación del
 * driver, latencia real del barómetro, huecos, duplicados, desórdenes y valores NaN.
 * "Registro" simulado: un registro de salida por muestra recibida, sea cual sea el estado.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "navigation.h"
#include "test_util.h"

#define G 9.81
#define PI 3.14159265358979323846
#define H_TRUTH 1e-4

typedef struct {
    const char *name;
    unsigned seed;
    double t_launch;
    double imu_hz, imu_jitter;
    double acc_noise, gyro_noise, acc_bias[3], gyro_bias[3];
    double acc_range, gyro_range; /* recorte; 0 = sin recorte */
    int driver_sat_flags;         /* el driver marca los ejes recortados */
    double baro_hz, baro_noise, baro_true_latency, baro_cfg_latency;
    double baro_transonic, baro_ejection, baro_loss_t, baro_loss_len;
    double imu_gap_t, imu_gap_len; /* respecto al despegue */
    double pad_gap_t, pad_gap_len; /* absoluto, en rampa */
    double p_dup, p_swap, p_nan;   /* anomalías de la adquisición por muestra */
    double anom_tf0;               /* las anomalías empiezan en tf > anom_tf0 (respecto al despegue) */
    int pad_bumps;                 /* golpes de una muestra (3 g) en rampa tras alinear */
    double roll_rate, pitch_rate, thrust, t_burn;
    /* configuración del módulo */
    double gap_periods;            /* gap_max en periodos nominales; 0 = sin puente */
    double acc_sat_max_s, acc_sat_excess, gyro_sat_max_s, gyro_sat_excess;
    int sat_by_limit;              /* además, límites configurados en el módulo */
    double gate;
} cfg_t;

typedef struct {
    double alt_err_max, alt_err_rms, frac_3sigma, tilt_max_deg, apo_t_err, apo_alt_true;
    double frac_valid, frac_degraded, frac_invalid; /* sobre las muestras de vuelo */
    double first_invalid_tf, liftoff_delay, t_state_end, t_gap_start;
    uint32_t reasons_or;
    int n_samples_sent, n_records, final_validity, valid_at_liftoff, aligned_count, bumps;
    nav_counters_t cnt;
} result_t;

static void qmul(const double a[4], const double b[4], double o[4])
{
    double r[4] = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                   a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                   a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                   a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    memcpy(o, r, sizeof r);
}
static void qrot(const double q[4], double R[3][3])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z); R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z); R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y); R[2][1] = 2 * (y * z + w * x); R[2][2] = 1 - 2 * (x * x + y * y);
}
static double clampd(double x, double r) { return r > 0 ? (x > r ? r : (x < -r ? -r : x)) : x; }
static double thrust_acc(const cfg_t *c, double tf)
{
    if (tf < 0) return 0;
    if (tf < 0.1) return c->thrust * tf / 0.1;
    if (tf < c->t_burn) return c->thrust;
    if (tf < c->t_burn + 0.1) return c->thrust * (1 - (tf - c->t_burn) / 0.1);
    return 0;
}

static void make_nav_config(const cfg_t *c, nav_config_t *k)
{
    double dt_nom = 1.0 / c->imu_hz;
    nav_default_config(k);
    k->eskf.sigma_a = (eskf_real)c->acc_noise;
    k->eskf.sigma_w = (eskf_real)c->gyro_noise;
    k->eskf.sigma_aw = (eskf_real)1e-4;
    k->eskf.sigma_ww = (eskf_real)1e-5;
    k->eskf.dt_max = (eskf_real)(1.5 * dt_nom * (1 + c->imu_jitter));
    k->eskf.gap_max = (eskf_real)(c->gap_periods * dt_nom);
    k->eskf.ba0_std = (eskf_real)0.2;
    k->eskf.bg0_std = (eskf_real)0.01;
    k->eskf.p0_std = (eskf_real)0.5;
    k->eskf.innov_gate = (eskf_real)c->gate;
    k->rest_acc_dev_max = 0.5;
    k->rest_acc_std_max = 5 * c->acc_noise + 0.02;
    k->rest_gyro_max = 0.2;
    k->rest_tilt_change_max = 0.005;
    k->eskf.accel_threshold = (eskf_real)0.5;
    k->eskf.align_acc_tol = (eskf_real)0.5;
    k->eskf.align_gyro_max = (eskf_real)0.2;
    k->liftoff_acc = 2 * G;
    k->acc_sat_limit = c->sat_by_limit ? c->acc_range : 0.0;
    k->gyro_sat_limit = c->sat_by_limit ? c->gyro_range : 0.0;
    k->acc_sat_max_s = c->acc_sat_max_s;
    k->acc_sat_excess = c->acc_sat_excess;
    k->gyro_sat_max_s = c->gyro_sat_max_s;
    k->gyro_sat_excess = c->gyro_sat_excess;
    k->baro_sigma = c->baro_noise;
    k->baro_latency_s = c->baro_cfg_latency;
    double need = c->baro_cfg_latency + (double)k->eskf.dt_max;
    if (need > (double)k->eskf.dt_max) k->eskf.tau_max = (eskf_real)(need * 1.2);
}

typedef struct { double t_read, t_meas, z_true; } baro_pending_t;

static result_t run(const cfg_t *c)
{
    result_t res;
    memset(&res, 0, sizeof res);
    res.first_invalid_tf = res.t_gap_start = res.liftoff_delay = (double)NAN;
    unsigned rng = c->seed;
    nav_config_t kcfg;
    make_nav_config(c, &kcfg);
    nav_t nav;
    if (nav_init(&nav, &kcfg) != NAV_OK) {
        printf("  %s: configuración rechazada: %s\n", c->name, nav_config_check(&kcfg));
        res.final_validity = -1;
        return res;
    }

    double p[3] = {0, 0, 0}, v[3] = {0, 0, 0}, q[4];
    {
        double el = 85.0 * PI / 180, az = 30.0 * PI / 180;
        double qz[4] = {cos(az / 2), 0, 0, sin(az / 2)}, qy[4] = {cos(el / 2), 0, sin(el / 2), 0};
        qmul(qz, qy, q);
    }
    double dt_nom = 1.0 / c->imu_hz, t = 0, t_end = 1e9, t_next_imu = dt_nom, t_next_baro = 0.01;
    double apo_t_true = -1, apo_t_est = -1, err2 = 0;
    int n_err = 0, n_in3 = 0, n_flight = 0, n_val = 0, n_deg = 0, n_inv = 0, bumps_left = c->pad_bumps;
    baro_pending_t pend[16];
    int npend = 0;
    nav_imu_sample_t held;
    int have_held = 0; /* para simular desorden: una muestra retenida y entregada después */

    while (t < t_end) {
        double tf = t - c->t_launch, R[3][3], w_b[3] = {0, 0, 0}, f_b[3];
        qrot(q, R);
        if (tf < 0) {
            for (int i = 0; i < 3; i++) f_b[i] = -G * R[2][i];
        } else {
            double sp = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            double fx = thrust_acc(c, tf) - 2e-4 * sp * sp;
            f_b[0] = fx; f_b[1] = 0; f_b[2] = 0;
            double a_nav[3];
            for (int i = 0; i < 3; i++) a_nav[i] = R[i][0] * fx + (i == 2 ? G : 0);
            if (tf < 1.0 && a_nav[0] * R[0][0] + a_nav[1] * R[1][0] + a_nav[2] * R[2][0] < 0) {
                a_nav[0] = a_nav[1] = a_nav[2] = 0;
                for (int i = 0; i < 3; i++) f_b[i] = -G * R[2][i];
            }
            w_b[0] = c->roll_rate;
            w_b[1] = (tf > 1.0) ? c->pitch_rate : 0;
            for (int i = 0; i < 3; i++) {
                v[i] += a_nav[i] * H_TRUTH;
                p[i] += v[i] * H_TRUTH;
            }
            double th = sqrt(w_b[0] * w_b[0] + w_b[1] * w_b[1]) * H_TRUTH;
            if (th > 0) {
                double s = sin(th / 2) / th * H_TRUTH;
                double dq[4] = {cos(th / 2), w_b[0] * s, w_b[1] * s, 0};
                qmul(q, dq, q);
                double nn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                for (int i = 0; i < 4; i++) q[i] /= nn;
            }
            if (apo_t_true < 0 && tf > c->t_burn + 1 && v[2] >= 0) {
                apo_t_true = t;
                res.apo_alt_true = -p[2];
                t_end = t + 3.0;
            }
        }
        t += H_TRUTH;

        /* ---------------- IMU ---------------- */
        if (t >= t_next_imu) {
            t_next_imu += dt_nom * (1 + c->imu_jitter * urand(&rng));
            int in_gap = (c->imu_gap_len > 0 && tf > c->imu_gap_t && tf < c->imu_gap_t + c->imu_gap_len) ||
                         (c->pad_gap_len > 0 && t > c->pad_gap_t && t < c->pad_gap_t + c->pad_gap_len);
            if (in_gap) {
                if (isnan(res.t_gap_start) && tf > 0) res.t_gap_start = nav.t_state;
                goto baro;
            }
            nav_imu_sample_t s;
            s.t = t;
            s.sat = 0;
            for (int i = 0; i < 3; i++) {
                double a = f_b[i] + c->acc_bias[i] + c->acc_noise * nrand(&rng);
                double w = w_b[i] + c->gyro_bias[i] + c->gyro_noise * nrand(&rng);
                if (c->acc_range > 0 && fabs(a) >= c->acc_range && c->driver_sat_flags) s.sat |= (uint8_t)(NAV_SAT_ACC_X << i);
                if (c->gyro_range > 0 && fabs(w) >= c->gyro_range && c->driver_sat_flags) s.sat |= (uint8_t)(NAV_SAT_GYRO_X << i);
                s.acc[i] = (float)clampd(a, c->acc_range);
                s.gyro[i] = (float)clampd(w, c->gyro_range);
            }
            if (bumps_left > 0 && nav.aligned && tf < -1.0 && urand(&rng) > 0.99) { /* golpe en rampa */
                s.acc[2] = (float)(-3 * G);
                bumps_left--;
                res.bumps++;
            }
            /* NaN sólo en vuelo: en rampa una muestra inválida corta la ventana de reposo y
               con esta tasa no se alinearía nunca (lo comprueba test_nav) */
            if (c->p_nan > 0 && tf > 0 && 0.5 * (urand(&rng) + 1) < c->p_nan) s.gyro[1] = (float)NAN;

            /* entrega (con anomalías de la adquisición) */
            nav_imu_sample_t out[3];
            int nout = 0;
            int anom = tf > c->anom_tf0;
            if (anom && c->p_swap > 0 && !have_held && 0.5 * (urand(&rng) + 1) < c->p_swap) {
                held = s; /* se entregará después de la siguiente */
                have_held = 1;
            } else {
                out[nout++] = s;
                if (have_held) {
                    out[nout++] = held;
                    have_held = 0;
                }
                if (anom && c->p_dup > 0 && 0.5 * (urand(&rng) + 1) < c->p_dup) out[nout++] = s;
            }
            for (int j = 0; j < nout; j++) {
                res.n_samples_sent++;
                nav_on_imu(&nav, &out[j]); /* el retorno no condiciona el registro */
                nav_output_t o;
                nav_get_output(&nav, &o); /* registro: uno por muestra recibida */
                res.n_records++;
                if (nav.phase == NAV_PHASE_FLIGHT && isnan(res.liftoff_delay)) {
                    res.liftoff_delay = nav.t_liftoff - c->t_launch;
                    res.valid_at_liftoff = o.validity;
                }
                if (tf < 0) continue;
                n_flight++;
                res.reasons_or |= o.reasons;
                if (o.validity == NAV_VALID) n_val++;
                else if (o.validity == NAV_DEGRADED) n_deg++;
                else {
                    n_inv++;
                    if (isnan(res.first_invalid_tf)) res.first_invalid_tf = tf;
                }
                if (o.validity == NAV_INVALID || o.t_state != out[j].t || out[j].t != t) continue;
                /* métricas en el instante del estado (= verdad actual) */
                double e = (double)o.p[2] - p[2], sg = (double)o.sigma_p[2];
                if (fabs(e) > res.alt_err_max) res.alt_err_max = fabs(e);
                err2 += e * e;
                n_err++;
                n_in3 += fabs(e) <= 3 * sg;
                eskf_real qe[4] = {(eskf_real)o.q[0], (eskf_real)o.q[1], (eskf_real)o.q[2], (eskf_real)o.q[3]};
                eskf_real Rf[3][3];
                eskf_rotation_matrix(qe, Rf);
                double cs = R[2][0] * (double)Rf[2][0] + R[2][1] * (double)Rf[2][1] + R[2][2] * (double)Rf[2][2];
                double tilt = acos(fmin(1.0, fmax(-1.0, cs))) * 180 / PI;
                if (tilt > res.tilt_max_deg) res.tilt_max_deg = tilt;
                if (apo_t_est < 0 && tf > c->t_burn + 1 && o.v[2] >= 0) apo_t_est = t;
            }
        }

    baro:
        /* ---------------- barómetro: medida en t, lectura en t + latencia real ---------------- */
        if (t >= t_next_baro) {
            t_next_baro += 1.0 / c->baro_hz;
            int lost = c->baro_loss_len > 0 && tf > c->baro_loss_t && tf < c->baro_loss_t + c->baro_loss_len;
            if (!lost && npend < 16) {
                double sp = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                double z = p[2] + c->baro_noise * nrand(&rng);
                if (tf > 0 && sp > 250 && sp < 330) z += c->baro_transonic;
                if (apo_t_true > 0 && t > apo_t_true + 1 && t < apo_t_true + 1.3) z += c->baro_ejection;
                pend[npend].t_meas = t;
                pend[npend].t_read = t + c->baro_true_latency;
                pend[npend].z_true = z;
                npend++;
            }
        }
        while (npend > 0 && t >= pend[0].t_read) {
            nav_baro_sample_t b = {t, (float)(-pend[0].z_true)}; /* altitud = −z (ref. 0) */
            nav_on_baro(&nav, &b);
            nav_output_t o;
            nav_get_output(&nav, &o);
            res.n_records++;
            res.n_samples_sent++;
            memmove(&pend[0], &pend[1], (size_t)(npend - 1) * sizeof pend[0]);
            npend--;
        }
    }
    nav_output_t o;
    nav_get_output(&nav, &o);
    res.final_validity = o.validity;
    res.t_state_end = o.t_state;
    res.alt_err_rms = n_err ? sqrt(err2 / n_err) : 0;
    res.frac_3sigma = n_err ? (double)n_in3 / n_err : 0;
    res.frac_valid = n_flight ? (double)n_val / n_flight : 0;
    res.frac_degraded = n_flight ? (double)n_deg / n_flight : 0;
    res.frac_invalid = n_flight ? (double)n_inv / n_flight : 0;
    res.apo_t_err = (apo_t_est > 0 && apo_t_true > 0) ? apo_t_est - apo_t_true : 99;
    res.cnt = nav.cnt;
    res.aligned_count = (int)nav.cnt.align_ok;
    return res;
}

static cfg_t base(void)
{
    cfg_t c;
    memset(&c, 0, sizeof c);
    c.name = "nominal";
    c.seed = 12345u;
    c.t_launch = 10.0;
    c.imu_hz = 200; c.imu_jitter = 0.1;
    c.acc_noise = 0.05; c.gyro_noise = 0.005;
    c.acc_bias[0] = 0.08; c.acc_bias[1] = -0.05; c.acc_bias[2] = 0.06;
    c.gyro_bias[0] = 0.004; c.gyro_bias[1] = -0.003; c.gyro_bias[2] = 0.002;
    c.acc_range = 16 * G; c.gyro_range = 2000 * PI / 180; c.driver_sat_flags = 1;
    c.baro_hz = 50; c.baro_noise = 0.5;
    c.roll_rate = 2.0; c.pitch_rate = -0.02;
    c.thrust = 120; c.t_burn = 3.0;
    c.gate = 5;
    c.anom_tf0 = -1e9;
    return c;
}

static const char *vstr(int v) { return v == NAV_VALID ? "VÁLIDO" : v == NAV_DEGRADED ? "DEGRADADO" : "NO VÁLIDO"; }

static void show(const cfg_t *c, const result_t *r)
{
    printf("  %-40s err alt %5.2f m (rms %4.2f) 3σ %5.1f%% incl %5.2f° t_apo %+5.2f | vuelo V/D/N %3.0f/%3.0f/%3.0f%% "
           "final %-9s motivos 0x%04x\n",
           c->name, r->alt_err_max, r->alt_err_rms, 100 * r->frac_3sigma, r->tilt_max_deg, r->apo_t_err,
           100 * r->frac_valid, 100 * r->frac_degraded, 100 * r->frac_invalid, vstr(r->final_validity),
           (unsigned)r->reasons_or);
}

static void common(const cfg_t *c, const result_t *r)
{
    CHECK(r->n_records == r->n_samples_sent, "%s: registros %d de %d muestras", c->name, r->n_records,
          r->n_samples_sent);
    CHECK(r->aligned_count >= 1 && !isnan(r->liftoff_delay) && r->liftoff_delay >= 0 && r->liftoff_delay < 0.2,
          "%s: alineación/despegue (retardo %.3f s)", c->name, r->liftoff_delay);
}

int main(void)
{
    printf("simulación sobre navigation.c (hipótesis de prueba, NO la sensórica real)\n");
    cfg_t c;
    result_t r;

    c = base(); c.name = "S1 nominal";
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.frac_valid == 1.0 && r.alt_err_max < 1.0 && r.frac_3sigma > 0.95 && r.tilt_max_deg < 2.5 &&
          fabs(r.apo_t_err) < 0.1, "S1 fuera de cotas");

    c = base(); c.name = "S2 baro: transónico/eyección/pérdida 0.8 s";
    c.baro_transonic = -60; c.baro_ejection = 80; c.baro_loss_t = 6; c.baro_loss_len = 0.8;
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.frac_invalid == 0 && r.cnt.baro_gated > 0 && r.alt_err_max < 2 && fabs(r.apo_t_err) < 0.1, "S2");

    c = base(); c.name = "S3 latencia baro 20 ms configurada";
    c.baro_true_latency = 0.02; c.baro_cfg_latency = 0.02;
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.frac_invalid == 0 && r.frac_3sigma > 0.95 && r.alt_err_max < 1.0, "S3");
    cfg_t c3 = c; c3.name = "S3' latencia 20 ms NO configurada (ref.)"; c3.baro_cfg_latency = 0;
    result_t r3 = run(&c3); show(&c3, &r3); common(&c3, &r3);
    printf("     -> baro bloqueados por la puerta: %u (con latencia configurada: %u)\n", (unsigned)r3.cnt.baro_gated,
           (unsigned)r.cnt.baro_gated);
    CHECK(r3.frac_3sigma < r.frac_3sigma - 0.2 && r3.cnt.baro_gated > 10 * (r.cnt.baro_gated + 1),
          "S3': la latencia no configurada debe notarse (3σ %.2f, baro en puerta %u)", r3.frac_3sigma,
          (unsigned)r3.cnt.baro_gated);

    c = base(); c.name = "S4 hueco IMU 60 ms en vuelo, sin puente";
    c.imu_gap_t = 8; c.imu_gap_len = 0.06;
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.final_validity == NAV_INVALID && (r.reasons_or & NAV_R_TIME_GAP) && fabs(r.first_invalid_tf - 8.06) < 0.02 &&
          r.t_state_end == r.t_gap_start, "S4: NO VÁLIDO desde el hueco, estado congelado en su t_state");
    cfg_t c4 = c; c4.name = "S4' igual con puente (gap_max 20 periodos)"; c4.gap_periods = 20;
    result_t r4 = run(&c4); show(&c4, &r4); common(&c4, &r4);
    CHECK(r4.frac_invalid == 0 && (r4.reasons_or & NAV_R_GAP_BRIDGED) && r4.final_validity == NAV_DEGRADED &&
          r4.alt_err_max < 1.0, "S4' con puente: DEGRADADO y preciso");

    c = base(); c.name = "S5 hueco 0.3 s en rampa (lanz. 20 s)";
    c.t_launch = 20; c.pad_gap_t = 8; c.pad_gap_len = 0.3;
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.aligned_count == 2 && r.valid_at_liftoff == NAV_VALID && r.frac_valid == 1.0 && r.alt_err_max < 1.0,
          "S5: debe realinear en rampa (alineaciones %d)", r.aligned_count);

    c = base(); c.name = "S6 acc ±8 g, empuje 12 g, política por defecto";
    c.acc_range = 8 * G;
    r = run(&c); show(&c, &r); common(&c, &r);
    CHECK(r.cnt.imu_acc_sat > 0 && (r.reasons_or & NAV_R_ACC_SAT_LIMIT) && r.final_validity == NAV_INVALID,
          "S6: saturación detectada y NO VÁLIDO");
    cfg_t c6 = c; c6.name = "S6' admitida 5 s con exceso 4 g";
    c6.acc_sat_max_s = 5; c6.acc_sat_excess = 4 * G;
    result_t r6 = run(&c6); show(&c6, &r6); common(&c6, &r6);
    CHECK(r6.frac_invalid == 0 && (r6.reasons_or & NAV_R_ACC_SAT) && r6.final_validity == NAV_DEGRADED &&
          r6.frac_3sigma > 0.95 && fabs(r6.apo_t_err) < 0.1, "S6': DEGRADADO y coherente");

    c = base(); c.name = "S7 giro limitado 0.1 rad/s, alabeo 2 rad/s";
    c.gyro_range = 0.1; c.driver_sat_flags = 0; c.sat_by_limit = 1; c.acc_range = 160; /* sólo límite */
    r = run(&c); show(&c, &r); common(&c, &r);
    printf("     ejes de giro saturados detectados en %u muestras (v2: 0)\n", (unsigned)r.cnt.imu_gyro_sat);
    CHECK(r.cnt.imu_gyro_sat > 0 && (r.reasons_or & NAV_R_GYRO_SAT_LIMIT) && r.final_validity == NAV_INVALID &&
          r.first_invalid_tf < 0.1, "S7: saturación del giro detectada por límite y NO VÁLIDO");

    c = base(); c.name = "S8p dup 1%/desorden 0.5% en rampa";
    c.p_dup = 0.01; c.p_swap = 0.005;
    r = run(&c); show(&c, &r);
    printf("     -> alineaciones %d, ventanas cortadas por huecos %u: no alinea; registros %d/%d\n", r.aligned_count,
           (unsigned)r.cnt.rest_breaks_gap, r.n_records, r.n_samples_sent);
    CHECK(r.aligned_count == 0 && r.final_validity == NAV_INVALID && (r.reasons_or & NAV_R_NOT_ALIGNED) &&
          r.n_records == r.n_samples_sent, "S8p: con pérdidas en rampa no hay alineación, el registro sigue");

    c = base(); c.name = "S8 marcas en vuelo: dup 1%, desorden 0.5%, NaN 0.2%";
    c.p_dup = 0.01; c.p_swap = 0.005; c.p_nan = 0.002; c.anom_tf0 = 0;
    r = run(&c); show(&c, &r); common(&c, &r);
    printf("     duplicadas %u, atrasadas %u, valores NaN %u\n", (unsigned)r.cnt.imu_time_dup,
           (unsigned)r.cnt.imu_time_order, (unsigned)r.cnt.imu_value_invalid);
    CHECK(r.cnt.imu_time_dup > 0 && r.cnt.imu_time_order > 0 && r.cnt.imu_value_invalid > 0 &&
          r.final_validity == NAV_INVALID && (r.reasons_or & NAV_R_TIME_GAP), "S8 sin puente: NO VÁLIDO al perder una");
    cfg_t c8 = c; c8.name = "S8' igual con puente (3 periodos)"; c8.gap_periods = 3;
    result_t r8 = run(&c8); show(&c8, &r8); common(&c8, &r8);
    printf("     -> con puente en vuelo sólo el %.0f%% de los errores cae en 3σ: el error de retención de orden cero "
           "no está modelado; por eso gap_max = 0 en la primera integración\n", 100 * r8.frac_3sigma);
    CHECK(r8.frac_invalid == 0 && r8.alt_err_max < 1.0 && r8.frac_3sigma > 0.85 && r8.frac_3sigma < 0.95,
          "S8' con puente: no inválido, pero menos coherente (%.2f)", r8.frac_3sigma);

    c = base(); c.name = "S9 golpes de 3 g en rampa";
    c.pad_bumps = 5;
    r = run(&c); show(&c, &r); common(&c, &r);
    printf("     golpes inyectados %d, despegue detectado %.3f s tras el real\n", r.bumps, r.liftoff_delay);
    CHECK(r.bumps >= 2 && r.valid_at_liftoff == NAV_VALID && r.frac_valid == 1.0 && r.cnt.rest_windows_ok >= 1 &&
          r.liftoff_delay < 0.1, "S9: sin despegue falso");

    double worst = 0, worst3 = 1;
    int nmc = 20, all_valid = 1;
    for (int k = 0; k < nmc; k++) {
        c = base();
        c.seed = 1000u + (unsigned)k * 7919u;
        unsigned s = c.seed;
        for (int i = 0; i < 3; i++) {
            c.acc_bias[i] = 0.1 * nrand(&s);
            c.gyro_bias[i] = 0.005 * nrand(&s);
        }
        r = run(&c);
        common(&c, &r);
        if (r.alt_err_max > worst) worst = r.alt_err_max;
        if (r.frac_3sigma < worst3) worst3 = r.frac_3sigma;
        all_valid &= r.frac_valid == 1.0;
    }
    printf("  Monte Carlo %d vuelos: error de altura máx %.2f m, peor fracción en 3σ %.1f%%, todos VÁLIDOS %s\n", nmc,
           worst, 100 * worst3, all_valid ? "sí" : "no");
    CHECK(worst < 1.0 && worst3 > 0.95 && all_valid, "Monte Carlo");
    return report("test_sim");
}
