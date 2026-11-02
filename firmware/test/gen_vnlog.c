/*
 * gen_vnlog.c — genera un registro binario SINTÉTICO con el formato y la ruta de datos del
 * firmware (adquisición → cola → ordenación por tiempo → navigation.c → registro), para probar
 * las herramientas en el ordenador. NO es un registro de la placa: la cabecera dice
 * board = "PC-SIM" y las herramientas lo avisan.
 *
 * Reproduce la lógica de vn_app.cpp que afecta al registro: interrupción de dato listo con
 * relojes distintos de acelerómetro (1125/(1+div) Hz) y giróscopo (1100/(1+div) Hz), lectura
 * de toda interrupción, sondeo del baro cada >= fw.baro_poll_us tras una IMU, vuelta de vn_nav
 * con nav_tick + nav_merge_pop + registro de TICK/NAVEV/NAVOUT/NAVCNT con las mismas reglas.
 *
 * Uso: gen_vnlog [opciones] salida.bin [verdad.json]
 *   --profile acq|bench   (acq)        --seconds S (60)        --seed N (1)
 *   --up +x|-x|+y|-y|+z|-z (+z)  eje del SENSOR que apunta hacia arriba
 *   --acc-noise N (0.0025 m/s²/√Hz)   --gyro-noise N (0.00026 rad/s/√Hz)
 *   --acc-bias bx,by,bz (m/s², ejes sensor)   --acc-scale sx,sy,sz (fracción)
 *   --gyro-bias bx,by,bz (rad/s)               --gyro-rw K (rad/s/√s, paseo del sesgo)
 *   --baro-noise S (0.3 m)   --baro-lag L (0.006 s: retardo verdadero respecto al dato listo)
 *   --imu-model union|acc (union)  --clock-ppm P (0)
 *   --spin rate,t0,dur      giro alrededor de la vertical (rad/s)
 *   --updown A,P,N,t0       subir y bajar A m con periodo P s, N veces, desde t0
 *   --flight                tras S s: 3 s de empuje (30 m/s²) y 4 s de planeo
 *   --lose-irq-every N      se pierde la interrupción N-ésima (dato sobrescrito)
 *   --i2c-err-every N       la lectura N-ésima de la IMU falla
 *   --ring-drop-every N     se pierde del REGISTRO la trama IMU N-ésima (no de la navegación)
 *   --stall T,D             la IMU deja de dar datos D s desde T (hueco)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../viradinha_nav/src/nav/imu_conv.h"
#include "../viradinha_nav/src/nav/nav_events.h"
#include "../viradinha_nav/src/nav/navigation.h"
#include "../viradinha_nav/src/vn/vn_logfmt.h"
#include "../viradinha_nav/src/vn/vn_params.h"

#define G0 9.80665
#define PI 3.14159265358979323846

/* ------------------------------------------------------------------ opciones */
typedef struct {
    int profile;
    double seconds;
    unsigned long long seed;
    int up_axis, up_sign;
    double acc_noise, gyro_noise, acc_bias[3], acc_scale[3], gyro_bias[3], gyro_rw;
    double baro_noise, baro_lag;
    int imu_union;
    double clock_ppm;
    double spin_rate, spin_t0, spin_dur;
    double ud_a, ud_p, ud_t0;
    int ud_n;
    int flight;
    long lose_irq_every, i2c_err_every, ring_drop_every;
    double stall_t, stall_d;
} opt_t;

static void usage(void)
{
    fprintf(stderr, "uso: gen_vnlog [opciones] salida.bin [verdad.json] (ver cabecera de gen_vnlog.c)\n");
    exit(2);
}

static void parse3(const char *s, double v[3])
{
    if (sscanf(s, "%lf,%lf,%lf", &v[0], &v[1], &v[2]) != 3) usage();
}

/* ------------------------------------------------------------------ azar */
static unsigned long long rs;
static double urand(void)
{
    rs ^= rs >> 12;
    rs ^= rs << 25;
    rs ^= rs >> 27;
    return (double)((rs * 2685821657736338717ull) >> 11) * (1.0 / 9007199254740992.0);
}
static double grand(void)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

/* ------------------------------------------------------------------ salida */
static FILE *g_out;
static unsigned long g_frames[16];
static void put(uint8_t type, const void *p, uint16_t len)
{
    uint8_t buf[VN_FRAME_MAX];
    size_t n = vn_frame(buf, sizeof buf, type, p, len);
    if (n == 0) {
        fprintf(stderr, "gen_vnlog: trama demasiado larga\n");
        exit(1);
    }
    fwrite(buf, 1, n, g_out);
    if (type < 16) g_frames[type]++;
}
static void text(const char *s)
{
    vn_rec_text_t t;
    memset(&t, 0, sizeof t);
    vn_strfill(t.text, sizeof t.text, s);
    put(VN_REC_TEXT, &t, sizeof t);
}

/* ------------------------------------------------------------------ verdad */
static opt_t O;
static double R[3][3]; /* sensor ← NED */

static void build_R(void)
{
    const double up[3] = {0, 0, -1}, north[3] = {1, 0, 0};
    int k = O.up_axis, i = (k + 1) % 3, j = (k + 2) % 3;
    for (int c = 0; c < 3; c++) {
        R[k][c] = O.up_sign * up[c];
        R[i][c] = north[c];
    }
    /* fila j = fila k × fila i (base dextrógira) */
    R[j][0] = R[k][1] * R[i][2] - R[k][2] * R[i][1];
    R[j][1] = R[k][2] * R[i][0] - R[k][0] * R[i][2];
    R[j][2] = R[k][0] * R[i][1] - R[k][1] * R[i][0];
}

/* altura (m, hacia arriba), aceleración vertical (m/s²) y giro alrededor de la vertical */
static void truth(double t, double *alt, double *a_up, double *w_up)
{
    *alt = 0.0;
    *a_up = 0.0;
    *w_up = 0.0;
    if (O.ud_n > 0 && t >= O.ud_t0 && t < O.ud_t0 + O.ud_n * O.ud_p) {
        double w = 2.0 * PI / O.ud_p, x = t - O.ud_t0;
        *alt = 0.5 * O.ud_a * (1.0 - cos(w * x));
        *a_up = 0.5 * O.ud_a * w * w * cos(w * x);
    }
    if (O.spin_dur > 0 && t >= O.spin_t0 && t < O.spin_t0 + O.spin_dur) *w_up = O.spin_rate;
    if (O.flight && t >= O.seconds) {
        const double ab = 30.0, tb = 3.0;
        double x = t - O.seconds;
        if (x < tb) {
            *a_up = ab;
            *alt = 0.5 * ab * x * x;
        } else {
            double vb = ab * tb, hb = 0.5 * ab * tb * tb, y = x - tb;
            *a_up = -G0;
            *alt = hb + vb * y - 0.5 * G0 * y * y;
        }
    }
}

/* ------------------------------------------------------------------ estado del «firmware» */
static nav_config_t g_cfg;
static imu_conv_cfg_t g_ic, g_ic_log;
static vn_hw_t g_hw;
static int g_nav_enabled;
static nav_t g_nav;
static nav_merge_t g_merge;
static uint32_t g_nav_calls;
static int64_t next_out = 0, next_tel = 0, next_cnt = 0;

static struct {
    uint32_t imu_irq_lost, ring_drop_recs, ring_drop_bytes, i2c_err_imu, imu_events, baro_events, nav_events;
} C;

static int16_t clamp16(int v) { return (int16_t)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v)); }

static void log_tick(int64_t now, int rt)
{
    if (!g_hw.log_ticks) return;
    vn_rec_tick_t k;
    k.t_us = now;
    k.ret = clamp16(rt);
    put(VN_REC_TICK, &k, sizeof k);
}

/* vuelta de vn_nav (misma lógica que nav_task en vn_app.cpp) */
static void nav_loop(int64_t now)
{
    if (!g_nav_enabled) return;
    int rt = nav_tick(&g_nav, (double)now * 1e-6);
    int want_out = now >= next_out, want_tel = now >= next_tel;
    int tick_logged = 0;
    if (want_out || want_tel) {
        log_tick(now, rt);
        tick_logged = 1;
    }
    for (;;) {
        nav_ev_imu_t ei;
        nav_ev_baro_t eb;
        nav_ev_kind_t k = nav_merge_pop(&g_merge, now, g_hw.merge_holdoff_us, &ei, &eb);
        if (k == NAV_EV_NONE) break;
        if (!tick_logged) {
            log_tick(now, rt);
            tick_logged = 1;
        }
        int ret = (k == NAV_EV_IMU) ? nav_on_imu(&g_nav, &ei.s) : nav_on_baro(&g_nav, &eb.s);
        g_nav_calls++;
        C.nav_events++;
        if (g_hw.log_navev) {
            vn_rec_navev_t e;
            uint32_t reasons = 0;
            e.src_seq = (k == NAV_EV_IMU) ? ei.seq : eb.seq;
            e.kind = (uint8_t)k;
            e.ret = clamp16(ret);
            e.validity = (uint8_t)nav_validity(&g_nav, &reasons);
            e.reasons = reasons;
            e.exec_us = 0; /* simulado: sin medida de tiempo */
            put(VN_REC_NAVEV, &e, sizeof e);
        }
    }
    if (want_out || want_tel) {
        nav_output_t o;
        nav_get_output(&g_nav, &o);
        if (want_out) {
            vn_rec_navout_t r;
            memset(&r, 0, sizeof r);
            r.t_us = now;
            r.n_events = g_nav_calls;
            r.t_state = o.t_state;
            r.t_eval = o.t_eval;
            r.alt_ref = o.alt_ref;
            r.reasons = o.reasons;
            r.alt_ref_seq = o.alt_ref_seq;
            r.f_degraded = (uint32_t)g_nav.f.degraded;
            r.validity = o.validity;
            r.phase = o.phase;
            r.at_rest = o.at_rest;
            r.heading_known = o.heading_known;
            memcpy(r.p, o.p, sizeof r.p);
            memcpy(r.v, o.v, sizeof r.v);
            memcpy(r.q, o.q, sizeof r.q);
            memcpy(r.sigma_p, o.sigma_p, sizeof r.sigma_p);
            memcpy(r.sigma_v, o.sigma_v, sizeof r.sigma_v);
            memcpy(r.sigma_th, o.sigma_th, sizeof r.sigma_th);
            memcpy(r.b_a, o.b_a, sizeof r.b_a);
            memcpy(r.b_g, o.b_g, sizeof r.b_g);
            put(VN_REC_NAVOUT, &r, sizeof r);
            next_out = now + (int64_t)(1e6 / g_hw.navout_hz);
        }
        if (want_tel) next_tel = now + (int64_t)(1e6 / g_hw.telemetry_hz);
    }
    if (now >= next_cnt) {
        vn_rec_navcnt_t c;
        memset(&c, 0, sizeof c);
        c.t_us = now;
        memcpy(c.cnt, &g_nav.cnt, sizeof g_nav.cnt);
        const nav_last_t *l = &g_nav.last;
        c.last[0] = clamp16(l->imu);
        c.last[1] = clamp16(l->predict);
        c.last[2] = clamp16(l->baro);
        c.last[3] = clamp16(l->zupt);
        c.last[4] = clamp16(l->grav);
        c.last[5] = clamp16(l->inflate);
        c.last[6] = clamp16(l->align);
        c.last[7] = clamp16(l->ref);
        put(VN_REC_NAVCNT, &c, sizeof c);
        next_cnt = now + (int64_t)(1e6 / g_hw.stats_hz);
    }
}

static void header_block(int64_t now, uint32_t boot_id)
{
    vn_rec_header_t h;
    memset(&h, 0, sizeof h);
    memcpy(h.magic, "VNLOG\0\0", 7);
    h.magic[7] = (char)VN_LOG_FMT_VERSION;
    h.fmt_version = VN_LOG_FMT_VERSION;
    h.sizeof_eskf_real = (uint16_t)sizeof(eskf_real);
    h.profile = (uint8_t)O.profile;
    h.nav_enabled = (uint8_t)g_nav_enabled;
    h.imu_irq = 1;
    h.baro_irq = 0;
    h.cpu_mhz = 0;
    h.t_us = now;
    h.boot_id = boot_id;
    vn_strfill(h.fw_version, sizeof h.fw_version, "gen_vnlog");
    vn_strfill(h.build, sizeof h.build, __DATE__ " " __TIME__);
    vn_strfill(h.compiler, sizeof h.compiler, "gcc " __VERSION__);
    vn_strfill(h.idf, sizeof h.idf, "-");
    vn_strfill(h.core, sizeof h.core, "-");
    vn_strfill(h.board, sizeof h.board, "PC-SIM");
    put(VN_REC_HEADER, &h, sizeof h);
    if (O.profile == VN_PROFILE_BENCH) text("PERFIL DE BANCO: valores PROVISIONALES. NO VOLAR.");
    size_t np;
    const vn_param_t *T = vn_params_table(&np);
    int vprof = (O.profile == VN_PROFILE_ACQ) ? VN_PROFILE_FLIGHT : O.profile;
    for (size_t i = 0; i < np; i++) {
        vn_rec_param_t r;
        memset(&r, 0, sizeof r);
        vn_strfill(r.name, sizeof r.name, T[i].name);
        r.value = vn_param_value(&T[i], vprof);
        r.status = vn_param_status(&T[i], vprof);
        r.used = (uint8_t)((T[i].kind == VN_PK_DOC) ? 0 : (T[i].kind == VN_PK_NAV ? g_nav_enabled : 1));
        vn_strfill(r.source, sizeof r.source, T[i].source);
        put(VN_REC_PARAM, &r, sizeof r);
    }
    vn_rec_reg_t g = {1, 0, 0x00, 0xFF, 0xEA, 0xEA, 1, 0};
    put(VN_REC_REG, &g, sizeof g);
    vn_rec_barocal_t cal;
    memset(&cal, 0, sizeof cal);
    cal.addr = 0x77;
    cal.chip_id = 0x60;
    put(VN_REC_BAROCAL, &cal, sizeof cal);
}

int main(int argc, char **argv)
{
    memset(&O, 0, sizeof O);
    O.profile = VN_PROFILE_ACQ;
    O.seconds = 60;
    O.seed = 1;
    O.up_axis = 2;
    O.up_sign = 1;
    O.acc_noise = 0.0025;
    O.gyro_noise = 0.00026;
    O.baro_noise = 0.3;
    O.baro_lag = 0.006;
    O.imu_union = 1;
    const char *out = NULL, *truth_path = NULL;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
#define NEXT() (i + 1 < argc ? argv[++i] : (usage(), ""))
        if (!strcmp(a, "--profile")) {
            const char *v = NEXT();
            if (!strcmp(v, "acq")) O.profile = VN_PROFILE_ACQ;
            else if (!strcmp(v, "bench")) O.profile = VN_PROFILE_BENCH;
            else usage();
        } else if (!strcmp(a, "--seconds")) O.seconds = atof(NEXT());
        else if (!strcmp(a, "--seed")) O.seed = strtoull(NEXT(), NULL, 10);
        else if (!strcmp(a, "--up")) {
            const char *v = NEXT();
            if (strlen(v) != 2 || (v[0] != '+' && v[0] != '-') || v[1] < 'x' || v[1] > 'z') usage();
            O.up_sign = v[0] == '+' ? 1 : -1;
            O.up_axis = v[1] - 'x';
        } else if (!strcmp(a, "--acc-noise")) O.acc_noise = atof(NEXT());
        else if (!strcmp(a, "--gyro-noise")) O.gyro_noise = atof(NEXT());
        else if (!strcmp(a, "--acc-bias")) parse3(NEXT(), O.acc_bias);
        else if (!strcmp(a, "--acc-scale")) parse3(NEXT(), O.acc_scale);
        else if (!strcmp(a, "--gyro-bias")) parse3(NEXT(), O.gyro_bias);
        else if (!strcmp(a, "--gyro-rw")) O.gyro_rw = atof(NEXT());
        else if (!strcmp(a, "--baro-noise")) O.baro_noise = atof(NEXT());
        else if (!strcmp(a, "--baro-lag")) O.baro_lag = atof(NEXT());
        else if (!strcmp(a, "--imu-model")) {
            const char *v = NEXT();
            if (!strcmp(v, "union")) O.imu_union = 1;
            else if (!strcmp(v, "acc")) O.imu_union = 0;
            else usage();
        } else if (!strcmp(a, "--clock-ppm")) O.clock_ppm = atof(NEXT());
        else if (!strcmp(a, "--spin")) {
            if (sscanf(NEXT(), "%lf,%lf,%lf", &O.spin_rate, &O.spin_t0, &O.spin_dur) != 3) usage();
        } else if (!strcmp(a, "--updown")) {
            if (sscanf(NEXT(), "%lf,%lf,%d,%lf", &O.ud_a, &O.ud_p, &O.ud_n, &O.ud_t0) != 4) usage();
        } else if (!strcmp(a, "--flight")) O.flight = 1;
        else if (!strcmp(a, "--lose-irq-every")) O.lose_irq_every = atol(NEXT());
        else if (!strcmp(a, "--i2c-err-every")) O.i2c_err_every = atol(NEXT());
        else if (!strcmp(a, "--ring-drop-every")) O.ring_drop_every = atol(NEXT());
        else if (!strcmp(a, "--stall")) {
            if (sscanf(NEXT(), "%lf,%lf", &O.stall_t, &O.stall_d) != 2) usage();
        } else if (a[0] == '-') usage();
        else if (!out) out = a;
        else if (!truth_path) truth_path = a;
        else usage();
    }
    if (!out || !(O.seconds > 0)) usage();
    rs = O.seed * 0x9E3779B97F4A7C15ull + 1ull;
    build_R();

    const char *why = "";
    g_nav_enabled = vn_params_build(O.profile, &g_cfg, &g_ic, &g_hw, &why);
    g_ic_log = g_ic;
    if (imu_conv_check(&g_ic) != 0)
        for (int i = 0; i < 3; i++) g_ic_log.map[i] = (uint8_t)i, g_ic_log.sign[i] = 1;
    if (g_nav_enabled && nav_init(&g_nav, &g_cfg) != NAV_OK) g_nav_enabled = 0;
    nav_merge_init(&g_merge);

    g_out = fopen(out, "wb");
    if (!g_out) {
        perror(out);
        return 1;
    }
    const uint32_t boot_id = (uint32_t)(O.seed * 2654435761u);
    header_block(0, boot_id);
    text("IMU begin=0 dir=0x69 irq=1");
    text("BARO begin=0 dir=0x77 id=0x60 irq=0");
    if (!g_nav_enabled) {
        char b[96];
        snprintf(b, sizeof b, "navegacion desactivada: %s", why);
        text(b);
    }

    /* relojes de los sensores (µs, double) */
    const double scale = 1.0 + O.clock_ppm * 1e-6;
    const double Ta = 1e6 * (1.0 + g_hw.imu_acc_div) / 1125.0 * scale;
    const double Tg = 1e6 * (1.0 + g_hw.imu_gyro_div) / 1100.0 * scale;
    const double Tb = 1e6 / (double)g_hw.baro_odr_hz * scale;
    double ta = 1000.0 + 0.37 * Ta, tg = 1000.0 + 0.11 * Tg, tb = 3000.0;
    const double t_end_us = 1e6 * (O.seconds + (O.flight ? 7.0 : 0.0));

    /* muestras vigentes en los registros del sensor */
    double acc_s[3] = {0, 0, 0}, gyr_s[3] = {0, 0, 0}, gbias[3];
    memcpy(gbias, O.gyro_bias, sizeof gbias);
    int16_t prev_acc[3] = {0, 0, 0}, prev_gyr[3] = {0, 0, 0};
    int have_prev = 0;
    double baro_val = NAN; /* altitud del último dato listo del baro */
    int baro_ready = 0;
    double baro_ready_t = 0;
    int64_t baro_last_poll = -1000000000, baro_prev_nodata = 0;
    uint32_t imu_seq = 0, baro_seq = 0, irq_count = 0;
    long irq_lost_pending = 0;
    double lag_sum = 0;
    long lag_n = 0, injected_lost = 0, injected_i2c = 0, injected_ring = 0, stalled_events = 0;
    int64_t last_nav = 0, next_stats = 1000000, next_hdr = 10000000;

    const double fa_s = 1125.0 / (1.0 + g_hw.imu_acc_div), fg_s = 1100.0 / (1.0 + g_hw.imu_gyro_div);
    const double sa = O.acc_noise * sqrt(fa_s), sg = O.gyro_noise * sqrt(fg_s);

    for (;;) {
        /* siguiente instante de interrupción de la IMU */
        double t_ev;
        int upd_a = 0, upd_g = 0;
        if (O.imu_union) {
            t_ev = ta < tg ? ta : tg;
            upd_a = ta <= t_ev + 50.0; /* pulsos a menos de 50 µs: una sola interrupción */
            upd_g = tg <= t_ev + 50.0;
        } else {
            t_ev = ta;
            upd_a = 1;
            upd_g = tg <= t_ev;
        }
        if (t_ev > t_end_us) break;
        double t_s = t_ev * 1e-6;
        double alt, a_up, w_up;
        truth(t_s, &alt, &a_up, &w_up);
        double f_n[3] = {0, 0, -(a_up + G0)}, w_n[3] = {0, 0, -w_up};
        if (upd_a) {
            for (int i = 0; i < 3; i++) {
                double f = R[i][0] * f_n[0] + R[i][1] * f_n[1] + R[i][2] * f_n[2];
                acc_s[i] = (1.0 + O.acc_scale[i]) * f + O.acc_bias[i] + sa * grand();
            }
            ta += Ta;
        }
        if (upd_g) {
            for (int i = 0; i < 3; i++) {
                double w = R[i][0] * w_n[0] + R[i][1] * w_n[1] + R[i][2] * w_n[2];
                if (O.gyro_rw > 0) gbias[i] += O.gyro_rw * sqrt(Tg * 1e-6) * grand();
                gyr_s[i] = w + gbias[i] + sg * grand();
            }
            tg += Tg;
        }

        /* dato listo del baro */
        while (tb <= t_ev) {
            double ab, aa, aw;
            truth(tb * 1e-6 - O.baro_lag, &ab, &aa, &aw);
            baro_val = ab + O.baro_noise * grand();
            baro_ready = 1;
            baro_ready_t = tb;
            tb += Tb;
        }

        /* interrupción → tarea de adquisición */
        int stalled = O.stall_d > 0 && t_s >= O.stall_t && t_s < O.stall_t + O.stall_d;
        if (stalled) {
            stalled_events++;
            continue;
        }
        irq_count++;
        if (O.lose_irq_every > 0 && irq_count % (unsigned long)O.lose_irq_every == 0) {
            irq_lost_pending++;
            injected_lost++;
            continue;
        }
        int64_t t_drdy = (int64_t)llround(t_ev + 3.0 + 4.0 * urand());
        int64_t t_read = t_drdy + 420 + (int64_t)(60.0 * urand());
        vn_rec_imu_t rec;
        memset(&rec, 0, sizeof rec);
        uint8_t flags = 0;
        if (irq_lost_pending) {
            flags |= VN_IMU_F_IRQ_LOST;
            C.imu_irq_lost += (uint32_t)irq_lost_pending;
            irq_lost_pending = 0;
        }
        int i2c_err = O.i2c_err_every > 0 && (long)(imu_seq + 1) % O.i2c_err_every == 0;
        float a_si[3] = {NAN, NAN, NAN}, w_si[3] = {NAN, NAN, NAN};
        uint8_t sat = 0;
        rec.seq = imu_seq;
        rec.t_drdy_us = t_drdy;
        rec.t_read_us = t_read;
        if (i2c_err) {
            flags |= VN_IMU_F_I2C_ERR;
            C.i2c_err_imu++;
            injected_i2c++;
        } else {
            int16_t acc[3], gyr[3];
            for (int i = 0; i < 3; i++) {
                acc[i] = clamp16((int)lround(acc_s[i] / G0 * 2048.0));
                gyr[i] = clamp16((int)lround(gyr_s[i] * 180.0 / PI * 16.4));
            }
            if (!have_prev || memcmp(acc, prev_acc, sizeof acc)) flags |= VN_IMU_F_ACC_NEW;
            if (!have_prev || memcmp(gyr, prev_gyr, sizeof gyr)) flags |= VN_IMU_F_GYR_NEW;
            memcpy(prev_acc, acc, sizeof acc);
            memcpy(prev_gyr, gyr, sizeof gyr);
            have_prev = 1;
            memcpy(rec.acc, acc, sizeof acc);
            memcpy(rec.gyro, gyr, sizeof gyr);
            rec.temp = (int16_t)lround(4.0 * 333.87); /* ≈ 25 °C */
            sat = imu_conv(&g_ic_log, acc, gyr, a_si, w_si);
            if (g_hw.mag_div > 0 && imu_seq % (uint32_t)g_hw.mag_div == 0) {
                rec.mag[0] = 100;
                rec.mag[1] = -50;
                rec.mag[2] = 300;
                rec.mag_st1 = 1;
                flags |= VN_IMU_F_MAG;
            }
        }
        rec.sat = sat;
        rec.flags = flags;
        C.imu_events++;
        /* vueltas de vn_nav sin eventos (espera de 2 ms) ANTES de que llegue éste a la cola */
        while (last_nav + 2000 < t_read) {
            last_nav += 2000;
            nav_loop(last_nav);
        }
        if (O.ring_drop_every > 0 && (long)(imu_seq + 1) % O.ring_drop_every == 0) {
            C.ring_drop_recs++;
            C.ring_drop_bytes += (uint32_t)(sizeof rec + VN_FRAME_OVERHEAD);
            injected_ring++;
        } else {
            put(VN_REC_IMU, &rec, sizeof rec);
        }
        if (g_nav_enabled) {
            nav_ev_imu_t e;
            e.t_us = t_drdy;
            e.seq = imu_seq;
            e.s.t = (double)t_drdy * 1e-6;
            memcpy(e.s.acc, a_si, sizeof a_si);
            memcpy(e.s.gyro, w_si, sizeof w_si);
            e.s.sat = sat;
            nav_merge_push_imu(&g_merge, &e);
        }
        imu_seq++;

        /* sondeo del baro tras la IMU */
        int64_t t_poll = t_read + 20;
        int64_t t_after = t_poll;
        if (t_poll - baro_last_poll >= g_hw.baro_poll_us) {
            baro_last_poll = t_poll;
            if (baro_ready && baro_ready_t <= (double)t_poll) {
                int64_t t_r = t_poll + 330;
                double p = g_hw.baro_p0_pa * pow(1.0 - baro_val / 44330.0, 1.0 / 0.1903);
                double alt_fw = 44330.0 * (1.0 - pow(p / g_hw.baro_p0_pa, 0.1903));
                vn_rec_baro_t b;
                memset(&b, 0, sizeof b);
                b.seq = baro_seq;
                b.t_prev_poll_us = baro_prev_nodata;
                b.t_detect_us = t_poll;
                b.t_read_us = t_r;
                b.pressure_pa = (float)p;
                b.temp_c = 25.0f;
                b.alt_m = (float)alt_fw;
                b.status = 0x70;
                b.flags = VN_BARO_F_POLLED;
                put(VN_REC_BARO, &b, sizeof b);
                C.baro_events++;
                lag_sum += (double)t_r * 1e-6 - (baro_ready_t * 1e-6 - O.baro_lag);
                lag_n++;
                if (g_nav_enabled) {
                    nav_ev_baro_t e;
                    e.t_us = t_r;
                    e.seq = baro_seq;
                    e.s.t = (double)t_r * 1e-6;
                    e.s.alt_m = (float)alt_fw;
                    nav_merge_push_baro(&g_merge, &e);
                }
                baro_seq++;
                baro_ready = 0;
                t_after = t_r;
            } else {
                baro_prev_nodata = t_poll;
            }
        }

        /* la vuelta de vn_nav que recoge este evento (y el baro, si lo hubo) */
        last_nav = t_after + 40;
        nav_loop(last_nav);

        /* estadísticas y cabecera periódicas (tarea loop) */
        if (t_after >= next_stats) {
            vn_rec_stats_t s;
            memset(&s, 0, sizeof s);
            s.t_us = t_after;
            s.imu_irq_lost = C.imu_irq_lost;
            s.ring_drop_recs = C.ring_drop_recs;
            s.ring_drop_bytes = C.ring_drop_bytes;
            s.i2c_err_imu = C.i2c_err_imu;
            s.merge_imu_overflow = g_merge.imu_overflow;
            s.merge_baro_overflow = g_merge.baro_overflow;
            s.imu_events = C.imu_events;
            s.baro_events = C.baro_events;
            s.nav_events = C.nav_events;
            s.ring_free_min = g_hw.ring_bytes;
            /* valores de relleno (simulación): sin medida real de pila ni montón */
            s.hwm_acq = s.hwm_nav = s.hwm_log = 12000u;
            s.hwm_loop = 6000u;
            s.heap_free_min = 200000u;
            put(VN_REC_STATS, &s, sizeof s);
            next_stats += 1000000;
        }
        if (t_after >= next_hdr) {
            header_block(t_after, boot_id);
            next_hdr += (int64_t)(1e6 * g_hw.header_period_s);
        }
    }
    fclose(g_out);

    if (truth_path) {
        FILE *f = fopen(truth_path, "w");
        if (!f) {
            perror(truth_path);
            return 1;
        }
        fprintf(f, "{\n  \"profile\": %d, \"seconds\": %.6f, \"flight\": %d, \"nav_enabled\": %d,\n", O.profile,
                O.seconds, O.flight, g_nav_enabled);
        fprintf(f, "  \"fs_acc\": %.9f, \"fs_gyro\": %.9f, \"fs_baro\": %.9f, \"imu_union\": %d,\n", 1e6 / Ta, 1e6 / Tg,
                1e6 / Tb, O.imu_union);
        fprintf(f, "  \"acc_noise\": %.9g, \"gyro_noise\": %.9g, \"gyro_rw\": %.9g,\n", O.acc_noise, O.gyro_noise,
                O.gyro_rw);
        fprintf(f, "  \"acc_bias\": [%.9g, %.9g, %.9g], \"acc_scale\": [%.9g, %.9g, %.9g],\n", O.acc_bias[0],
                O.acc_bias[1], O.acc_bias[2], O.acc_scale[0], O.acc_scale[1], O.acc_scale[2]);
        fprintf(f, "  \"gyro_bias\": [%.9g, %.9g, %.9g],\n", O.gyro_bias[0], O.gyro_bias[1], O.gyro_bias[2]);
        fprintf(f, "  \"up_axis\": %d, \"up_sign\": %d,\n", O.up_axis, O.up_sign);
        fprintf(f, "  \"R_sensor_from_ned\": [[%g, %g, %g], [%g, %g, %g], [%g, %g, %g]],\n", R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]);
        fprintf(f, "  \"baro_noise\": %.9g, \"baro_lag\": %.9g, \"baro_latency_eff_mean\": %.9g,\n", O.baro_noise,
                O.baro_lag, lag_n ? lag_sum / (double)lag_n : 0.0);
        fprintf(f, "  \"spin\": [%.9g, %.9g, %.9g], \"updown\": [%.9g, %.9g, %d, %.9g],\n", O.spin_rate, O.spin_t0,
                O.spin_dur, O.ud_a, O.ud_p, O.ud_n, O.ud_t0);
        fprintf(f, "  \"injected\": {\"irq_lost\": %ld, \"i2c_err\": %ld, \"ring_drop\": %ld, \"stalled_events\": %ld},\n",
                injected_lost, injected_i2c, injected_ring, stalled_events);
        fprintf(f, "  \"stall\": [%.9g, %.9g], \"imu_records\": %u, \"baro_records\": %u, \"nav_calls\": %u\n}\n", O.stall_t,
                O.stall_d, imu_seq, baro_seq, g_nav_calls);
        fclose(f);
    }
    fprintf(stderr, "gen_vnlog: %u IMU, %u baro, %u llamadas de navegación, %lu NAVOUT\n", imu_seq, baro_seq,
            g_nav_calls, g_frames[VN_REC_NAVOUT]);
    return 0;
}
