/*
 * nav_replay.c — pasa un registro de muestras por navigation.c (el mismo módulo del
 * firmware) en el ordenador. Uso: nav_replay < registro.txt > salida.txt
 *
 * Entrada (una orden por línea; '#' y líneas vacías se ignoran; nº EXACTO de argumentos):
 *   CFG <campo> <valor>   fija un campo de nav_config_t (ver tabla FIELDS; "eskf.*" = núcleo)
 *   INIT                  nav_init con la configuración actual (obligatorio antes de datos)
 *   IMU t ax ay az gx gy gz sat     (sat = bits NAV_SAT_*, entero 0..63)
 *   BARO t alt
 *   TICK t                reloj (nav_tick): caducidad y marcas futuras
 * Órdenes para reproducir EXACTAMENTE un registro de la placa (firmware/tools/log_to_replay.py):
 *   IMUC <campo> <valor>  conversión de la IMU (imu.acc_lsb_per_g, imu.axis_map_x, ...)
 *   IMURAW t_us ax ay az gx gy gz   códigos crudos int16 → imu_conv (la misma función que en
 *                         la placa) → nav_on_imu con t = t_us·1e-6 (la misma cuenta en double)
 *   IMUNAN t_us           lectura fallida: valores NaN (como hace el firmware)
 *   BAROUS t_us alt       baro con t = t_us·1e-6 (alt: el float registrado, %.9g lo conserva)
 *   TICKUS t_us           nav_tick(t_us·1e-6)
 * Salida por cada IMU/BARO/TICK (y sus variantes):
 *   OUT <ret> <t_state> <validez> <motivos> p3 v3 q4 sigma_p3 sigma_v3 sigma_th3 alt_ref b_a3 b_g3
 *       alt_ref_seq f_degraded
 * Al final, en stderr, los contadores.
 *
 * Errores: formato → código 2; configuración incompleta o inválida → código 3 (con el
 * nombre del campo). Números: strtod completo; "nan"/"inf" explícitos se aceptan (sirven
 * para reproducir registros con valores corruptos).
 */
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_conv.h"
#include "navigation.h"

static unsigned long g_line = 0;

static void fail(int code, const char *msg, const char *detail)
{
    fprintf(stderr, "nav_replay: línea %lu: %s%s%s\n", g_line, msg, detail ? ": " : "", detail ? detail : "");
    exit(code);
}

static double num(const char *tok)
{
    char *end = NULL;
    errno = 0;
    double v = strtod(tok, &end);
    if (end == tok || *end != '\0') fail(2, "número inválido", tok);
    if (errno == ERANGE) fail(2, "número fuera de rango", tok);
    return v;
}

/* número entero exacto en [lo, hi] */
static long long inum(const char *tok, double lo, double hi)
{
    double v = num(tok);
    if (!(v >= lo && v <= hi) || v != floor(v)) fail(2, "entero fuera de rango", tok);
    return (long long)v;
}

/* tipo: 0 double, 1 eskf_real, 2 int */
#define DF(name) {#name, offsetof(nav_config_t, name), 0}
#define EF(name) {"eskf." #name, offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, name), 1}
#define EFN(nm, f) {nm, offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, f), 1}
static const struct {
    const char *name;
    size_t off;
    int is_real;
} FIELDS[] = {
    EFN("eskf.mag_ref_n", mag_ref[0]), EFN("eskf.mag_ref_e", mag_ref[1]), EFN("eskf.mag_ref_d", mag_ref[2]),
    {"eskf.has_mag_ref", offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, has_mag_ref), 2},
    EF(sigma_a), EF(sigma_w), EF(sigma_aw), EF(sigma_ww), EF(accel_threshold), EF(innov_gate),
    EF(dt_max), EF(gap_max), EF(tau_max), EF(align_acc_tol), EF(align_gyro_max), EF(p0_std),
    EF(v0_std), EF(th0_std), EF(ba0_std), EF(bg0_std),
    DF(rest_window_s), DF(rest_acc_dev_max), DF(rest_acc_std_max), DF(rest_gyro_max),
    DF(rest_update_period_s), DF(zupt_sigma), DF(grav_sigma), DF(liftoff_acc), DF(liftoff_hold_s),
    DF(acc_sat_limit), DF(gyro_sat_limit), DF(acc_sat_max_s), DF(gyro_sat_max_s), DF(acc_sat_excess),
    DF(gyro_sat_excess), DF(baro_sigma), DF(baro_latency_s), DF(baro_timeout_s),
    DF(rest_tilt_change_max), DF(baro_ref_min_n), DF(baro_ref_max_step), DF(stale_max_s),
};

/* float con %.9g y double con %.17g: el texto conserva el valor exacto (comparación con la placa) */
static void out(const nav_t *n, int ret)
{
    nav_output_t o;
    nav_get_output(n, &o);
    printf("OUT %d %.17g %u 0x%04x", ret, o.t_state, (unsigned)o.validity, (unsigned)o.reasons);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.p[i]);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.v[i]);
    for (int i = 0; i < 4; i++) printf(" %.9g", (double)o.q[i]);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.sigma_p[i]);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.sigma_v[i]);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.sigma_th[i]);
    printf(" %.17g", o.alt_ref);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.b_a[i]);
    for (int i = 0; i < 3; i++) printf(" %.9g", (double)o.b_g[i]);
    printf(" %u %u\n", (unsigned)o.alt_ref_seq, (unsigned)n->f.degraded);
}

int main(void)
{
    static nav_t nav;
    nav_config_t cfg;
    nav_default_config(&cfg);
    imu_conv_cfg_t ic;
    memset(&ic, 0, sizeof ic);
    for (int i = 0; i < 3; i++) ic.map[i] = 0xFFu; /* sin fijar: imu_conv_check falla */
    int started = 0;
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        g_line++;
        size_t len = strlen(line);
        if (len == sizeof line - 1 && line[len - 1] != '\n' && !feof(stdin)) fail(2, "línea demasiado larga", NULL);
        char *tok[12];
        int nt = 0;
        for (char *t = strtok(line, " \t\r\n"); t; t = strtok(NULL, " \t\r\n")) {
            if (nt == 12) fail(2, "demasiados campos", NULL);
            tok[nt++] = t;
        }
        if (nt == 0 || tok[0][0] == '#') continue;

        if (!strcmp(tok[0], "CFG")) {
            if (nt != 3) fail(2, "CFG necesita <campo> <valor>", NULL);
            if (started) fail(2, "CFG después de INIT", NULL);
            size_t k = 0, nf = sizeof FIELDS / sizeof FIELDS[0];
            while (k < nf && strcmp(FIELDS[k].name, tok[1]) != 0) k++;
            if (k == nf) fail(2, "campo de configuración desconocido", tok[1]);
            double v = num(tok[2]);
            char *base = (char *)&cfg + FIELDS[k].off;
            if (FIELDS[k].is_real == 2) {
                int iv = (int)inum(tok[2], -2147483647.0, 2147483647.0);
                memcpy(base, &iv, sizeof iv);
            } else if (FIELDS[k].is_real) {
                eskf_real r = (eskf_real)v;
                memcpy(base, &r, sizeof r);
            } else {
                memcpy(base, &v, sizeof v);
            }
        } else if (!strcmp(tok[0], "INIT")) {
            if (nt != 1) fail(2, "INIT no lleva argumentos", NULL);
            const char *bad = nav_config_check(&cfg);
            if (bad) fail(3, "configuración incompleta o inválida", bad);
            if (nav_init(&nav, &cfg) != NAV_OK) fail(3, "nav_init", "float/double incoherente");
            started = 1;
        } else if (!strcmp(tok[0], "IMU")) {
            if (nt != 9) fail(2, "IMU necesita t ax ay az gx gy gz sat", NULL);
            if (!started) fail(2, "IMU antes de INIT", NULL);
            nav_imu_sample_t s;
            s.t = num(tok[1]);
            for (int i = 0; i < 3; i++) {
                s.acc[i] = (float)num(tok[2 + i]);
                s.gyro[i] = (float)num(tok[5 + i]);
            }
            double sat = num(tok[8]);
            if (!(sat >= 0 && sat <= 63) || sat != (double)(int)sat) fail(2, "sat debe ser un entero 0..63", tok[8]);
            s.sat = (uint8_t)(int)sat;
            out(&nav, nav_on_imu(&nav, &s));
        } else if (!strcmp(tok[0], "TICK")) {
            if (nt != 2) fail(2, "TICK necesita t", NULL);
            if (!started) fail(2, "TICK antes de INIT", NULL);
            double t = num(tok[1]);
            if (!isfinite(t)) fail(2, "TICK no finito", tok[1]);
            out(&nav, nav_tick(&nav, t));
        } else if (!strcmp(tok[0], "BARO")) {
            if (nt != 3) fail(2, "BARO necesita t alt", NULL);
            if (!started) fail(2, "BARO antes de INIT", NULL);
            nav_baro_sample_t b;
            b.t = num(tok[1]);
            b.alt_m = (float)num(tok[2]);
            out(&nav, nav_on_baro(&nav, &b));
        } else if (!strcmp(tok[0], "IMUC")) {
            if (nt != 3) fail(2, "IMUC necesita <campo> <valor>", NULL);
            static const char *const ax[3] = {"x", "y", "z"};
            const char *f = tok[1];
            int done = 0;
            if (!strcmp(f, "imu.acc_lsb_per_g")) ic.acc_lsb_per_g = (float)num(tok[2]), done = 1;
            else if (!strcmp(f, "imu.gyro_lsb_per_dps")) ic.gyro_lsb_per_dps = (float)num(tok[2]), done = 1;
            else if (!strcmp(f, "imu.g_si")) ic.g_si = (float)num(tok[2]), done = 1;
            else if (!strcmp(f, "imu.sat_margin_lsb")) ic.sat_margin_lsb = (int32_t)inum(tok[2], 0, 32767), done = 1;
            for (int i = 0; i < 3 && !done; i++) {
                char nm[32];
                snprintf(nm, sizeof nm, "imu.axis_map_%s", ax[i]);
                if (!strcmp(f, nm)) ic.map[i] = (uint8_t)inum(tok[2], 0, 2), done = 1;
                snprintf(nm, sizeof nm, "imu.axis_sign_%s", ax[i]);
                if (!done && !strcmp(f, nm)) {
                    long long sg = inum(tok[2], -1, 1);
                    if (sg == 0) fail(2, "signo 0", tok[2]);
                    ic.sign[i] = (int8_t)sg;
                    done = 1;
                }
            }
            if (!done) fail(2, "campo de IMUC desconocido", f);
        } else if (!strcmp(tok[0], "IMURAW") || !strcmp(tok[0], "IMUNAN")) {
            int raw = !strcmp(tok[0], "IMURAW");
            if (nt != (raw ? 8 : 2)) fail(2, raw ? "IMURAW necesita t_us ax ay az gx gy gz" : "IMUNAN necesita t_us", NULL);
            if (!started) fail(2, "IMURAW/IMUNAN antes de INIT", NULL);
            nav_imu_sample_t s;
            s.t = (double)inum(tok[1], -9.0e15, 9.0e15) * 1e-6;
            s.sat = 0u;
            if (raw) {
                if (imu_conv_check(&ic) != 0) fail(3, "conversión de la IMU incompleta (IMUC)", NULL);
                int16_t a[3], g[3];
                for (int i = 0; i < 3; i++) {
                    a[i] = (int16_t)inum(tok[2 + i], -32768, 32767);
                    g[i] = (int16_t)inum(tok[5 + i], -32768, 32767);
                }
                s.sat = imu_conv(&ic, a, g, s.acc, s.gyro);
            } else {
                for (int i = 0; i < 3; i++) s.acc[i] = s.gyro[i] = (float)NAN;
            }
            out(&nav, nav_on_imu(&nav, &s));
        } else if (!strcmp(tok[0], "BAROUS")) {
            if (nt != 3) fail(2, "BAROUS necesita t_us alt", NULL);
            if (!started) fail(2, "BAROUS antes de INIT", NULL);
            nav_baro_sample_t b;
            b.t = (double)inum(tok[1], -9.0e15, 9.0e15) * 1e-6;
            b.alt_m = (float)num(tok[2]);
            out(&nav, nav_on_baro(&nav, &b));
        } else if (!strcmp(tok[0], "TICKUS")) {
            if (nt != 2) fail(2, "TICKUS necesita t_us", NULL);
            if (!started) fail(2, "TICKUS antes de INIT", NULL);
            out(&nav, nav_tick(&nav, (double)inum(tok[1], -9.0e15, 9.0e15) * 1e-6));
        } else {
            fail(2, "orden desconocida", tok[0]);
        }
    }
    if (ferror(stdin)) fail(2, "error de lectura", NULL);
    const nav_counters_t *c = &nav.cnt;
    fprintf(stderr,
            "nav_replay: imu rx %u aplicadas %u (t no finito %u, dup %u, atrasadas %u, valor %u, sat acc %u giro %u) "
            "baro rx %u usados %u puerta %u rechazados %u huecos %u alineaciones %u futuras %u ref %u/%u\n",
            (unsigned)c->imu_rx, (unsigned)c->imu_applied, (unsigned)c->imu_time_nonfinite, (unsigned)c->imu_time_dup,
            (unsigned)c->imu_time_order, (unsigned)c->imu_value_invalid, (unsigned)c->imu_acc_sat,
            (unsigned)c->imu_gyro_sat, (unsigned)c->baro_rx, (unsigned)c->baro_used, (unsigned)c->baro_gated,
            (unsigned)c->baro_rejected, (unsigned)c->gaps, (unsigned)c->align_ok, (unsigned)c->time_future,
            (unsigned)c->ref_updates, (unsigned)c->ref_rejected);
    return 0;
}
