/*
 * eskf_service.c — núcleo del servicio (ver eskf_service.h). C99, memoria estática, trabajo
 * acotado por llamada, sin SPP ni sistema operativo (salvo las secciones críticas de ESP-IDF).
 */
#include "eskf_service.h"

#include <math.h>
#include <string.h>

#include "eskf_svc_msg.h"
#include "eskf_svc_params.h"

/* Secciones críticas breves: buzón (deliver/consume) y copia de las consultas entre tareas.
   Dentro sólo se copian estructuras; nunca se ejecuta el filtro. */
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static portMUX_TYPE s_mb_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_out_mux = portMUX_INITIALIZER_UNLOCKED;
#define MB_LOCK() portENTER_CRITICAL(&s_mb_mux)
#define MB_UNLOCK() portEXIT_CRITICAL(&s_mb_mux)
#define OUT_LOCK() portENTER_CRITICAL(&s_out_mux)
#define OUT_UNLOCK() portEXIT_CRITICAL(&s_out_mux)
#else
static volatile char s_out_flag = 0; /* cerrojo de espera activa para el PC (copias de ~1 kB) */
#define MB_LOCK() ((void)0)
#define MB_UNLOCK() ((void)0)
#define OUT_LOCK()                                                       \
    do {                                                                 \
    } while (__atomic_test_and_set(&s_out_flag, __ATOMIC_ACQUIRE))
#define OUT_UNLOCK() __atomic_clear(&s_out_flag, __ATOMIC_RELEASE)
#endif

#define SVC_MAGIC 0x45535633u /* "ESV3" */
#define RESERVED_KPID 0x0003u /* K_SPP_KPID_LOG | K_SPP_KPID_FSM */
#define BB_REST_V_MPS 1.0     /* la referencia en rampa sólo se actualiza con |v| del respaldo <= 1 m/s */

/* ------------------------------------------------------------------ utilidades */
uint16_t eskf_svc_crc16(const uint8_t *p, size_t n, uint16_t crc)
{
    for (size_t i = 0; i < n; i++) {
        crc = (uint16_t)(crc ^ ((uint16_t)p[i] << 8));
        for (int b = 0; b < 8; b++)
            crc = (uint16_t)((crc & 0x8000u) ? (((unsigned)crc << 1) ^ 0x1021u) : ((unsigned)crc << 1));
    }
    return crc;
}

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float rd_f32(const uint8_t *p)
{
    uint32_t u = rd_u32(p);
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}
static uint32_t ld_acq(const volatile uint32_t *p) { return __atomic_load_n(p, __ATOMIC_ACQUIRE); }
static void st_rel(volatile uint32_t *p, uint32_t v) { __atomic_store_n(p, v, __ATOMIC_RELEASE); }
static uint32_t sat_add(uint32_t a, uint32_t b) { return (a > 0xFFFFFFFFu - b) ? 0xFFFFFFFFu : a + b; }
static uint16_t c16(uint32_t v) { return (uint16_t)(v > 0xFFFFu ? 0xFFFFu : v); }
static int16_t ci16(int32_t v) { return (int16_t)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v)); }
static int one_bit(uint16_t v) { return v != 0u && (v & (uint16_t)(v - 1u)) == 0u; }
static int finf(float x) { return isfinite(x) ? 1 : 0; }
static uint16_t age_ms(int64_t now, int64_t t)
{
    if (t == INT64_MIN || now < t) return 0xFFFFu;
    int64_t a = (now - t) / 1000;
    return (uint16_t)(a > 0xFFFE ? 0xFFFE : a);
}

/* ------------------------------------------------------------------ configuración */
const char *eskf_svc_check_transport(const eskf_svc_config_t *c)
{
    if (!c) return "configuración nula";
    if (c->profile != ESKF_SVC_PROFILE_ACQ && c->profile != ESKF_SVC_PROFILE_EXP && c->profile != ESKF_SVC_PROFILE_STRICT)
        return "perfil no admitido";
    if (!one_bit(c->kpid_imu) || !one_bit(c->kpid_baro)) return "kpid_imu/kpid_baro: deben ser un único bit asignado";
    if (c->kpid_imu == c->kpid_baro) return "kpid_imu y kpid_baro iguales";
    if ((c->kpid_imu | c->kpid_baro) & RESERVED_KPID) return "kpid de entrada en un bit reservado (log o FSM)";
    if (c->kpid_out != 0u) {
        if (!one_bit(c->kpid_out) || (c->kpid_out & RESERVED_KPID)) return "kpid_out: un único bit no reservado";
        if (c->kpid_out & (c->kpid_imu | c->kpid_baro)) return "kpid_out coincide con una entrada: autosuscripción";
    }
    if (c->imu_format != ESKF_SVC_IMU_FMT_SPP44 && c->imu_format != ESKF_SVC_IMU_FMT_HIST48) return "in.imu_format";
    if (c->baro_semantics != ESKF_SVC_BARO_REL_PRODUCER && c->baro_semantics != ESKF_SVC_BARO_ABS_ISA)
        return "in.baro_semantics";
    if (!(c->acc_div > 0.0f) || !(c->gyro_div > 0.0f) || !finf(c->acc_div) || !finf(c->gyro_div))
        return "in.acc_div / in.gyro_div";
    for (int i = 0; i < 3; i++)
        if (!finf(c->acc_off[i]) || !finf(c->gyro_off[i])) return "in.*_off no finito";
    if (!(c->sat_margin_codes >= 0.0f && c->sat_margin_codes < 100.0f)) return "in.sat_margin_codes";
    if (c->sat_to_nav != 0 && c->sat_to_nav != 1) return "in.sat_to_nav";
    if (c->time_source != ESKF_SVC_TIME_PACKET_MS) return "in.time_source";
    if (c->future_tol_us < 0 || c->future_tol_us > 1000000) return "in.future_tol_us";
    if (!finf(c->baro_h_abs_max_m) || !(c->baro_h_abs_max_m > 0.0f) || !finf(c->baro_p_min_pa) ||
        !finf(c->baro_p_max_pa) || !(c->baro_p_min_pa < c->baro_p_max_pa) || !finf(c->baro_t_min_c) ||
        !finf(c->baro_t_max_c) || !(c->baro_t_min_c < c->baro_t_max_c))
        return "in.baro_* (límites físicos)";
    if (!(c->bb_alpha > 0.0f && c->bb_alpha < 1.0f) || !(c->bb_beta > 0.0f && c->bb_beta <= c->bb_alpha) ||
        !(c->bb_gate_m > 0.0f) || !finf(c->bb_gate_m) || !(c->bb_dt_max_s > 0.0f) || !finf(c->bb_dt_max_s) ||
        !(c->bb_timeout_s > 0.0f) || !finf(c->bb_timeout_s) || !(c->bb_ref_tau_s > 0.0f) || !finf(c->bb_ref_tau_s) ||
        !(c->bb_liftoff_m > 0.0f) || !finf(c->bb_liftoff_m) || c->bb_min_samples < 3u ||
        c->bb_min_samples > ESKF_SVC_BB_WIN_MAX || c->bb_reseed_n == 0u || !(c->bb_coh_m > 0.0f) || !finf(c->bb_coh_m) ||
        !(c->bb_flight_vz_mps > 0.0f) || !finf(c->bb_flight_vz_mps) || !(c->bb_hold_s > 0.0f) ||
        !finf(c->bb_hold_s))
        return "bb.* (respaldo barométrico)";
    if (!(c->pad_start_s > 0.0f) || !finf(c->pad_start_s) || !(c->pad_gap_max_s > 0.0f) || !finf(c->pad_gap_max_s) ||
        !(c->pad_lat_max_s >= 0.0f) || !finf(c->pad_lat_max_s))
        return "svc.pad_start_s / svc.pad_gap_max_s / svc.pad_lat_max_s";
    if (c->max_items == 0u || c->max_items > ESKF_SVC_MAILBOX_CAP || c->max_events == 0u || c->max_events > 1000u)
        return "svc.max_items / svc.max_events";
    if (c->merge_holdoff_us < 0 || c->merge_holdoff_us > 1000000) return "svc.merge_holdoff_us";
    if (c->snapshot_period_us < 0 || c->diag_period_us < 0 || c->ops_period_us < 0) return "periodos de salida negativos";
    if (c->snapshot_period_us > 0 && c->snapshot_period_us < 10000) return "svc.snapshot_period_us < 10 ms";
    if (c->diag_period_us > 0 && c->diag_period_us < 100000) return "svc.diag_period_us < 100 ms";
    if (c->ops_period_us > 0 && c->ops_period_us < 100000) return "svc.ops_period_us < 100 ms";
    if ((c->snapshot_mask & ~(uint32_t)ESKF_MSG_SNAP_MASK_ALL) || (c->snapshot_mask & 1u) == 0u)
        return "svc.snapshot_mask (debe incluir el fragmento 1 de estado)";
    if (c->trace_enable != 0 && c->trace_enable != 1) return "svc.trace_enable";
    return NULL;
}

int eskf_svc_check_rotation(const float r[9])
{
    for (int i = 0; i < 9; i++)
        if (!isfinite(r[i])) return -1;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double d = 0.0;
            for (int k = 0; k < 3; k++) d += (double)r[3 * k + i] * (double)r[3 * k + j];
            if (fabs(d - (i == j ? 1.0 : 0.0)) > 1e-5) return -2;
        }
    double m[9];
    for (int i = 0; i < 9; i++) m[i] = (double)r[i];
    double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                 m[2] * (m[3] * m[7] - m[4] * m[6]);
    return fabs(det - 1.0) <= 1e-5 ? 0 : -3;
}

const char *eskf_svc_check_nav(const eskf_svc_config_t *c, uint16_t *why_code)
{
    uint16_t dummy;
    if (!why_code) why_code = &dummy;
    *why_code = 0;
    if (!c) return "configuración nula";
    if (c->profile == ESKF_SVC_PROFILE_ACQ) {
        *why_code = 0x8001u;
        return "perfil de adquisición: navegación desactivada";
    }
    size_t n;
    const eskf_param_t *T = eskf_svc_params_table(&n);
    for (size_t i = 0; i < n; i++)
        if (!isfinite(eskf_svc_param_read(c, &T[i]))) {
            *why_code = (uint16_t)(i + 1u);
            return T[i].name; /* primer pendiente o no finito, publicado como why_code */
        }
    *why_code = 0x8002u;
    if (!(c->acc_unit_si > 0.0f) || !(c->gyro_unit_si > 0.0f)) return "in.*_unit_si";
    *why_code = 0x8003u;
    if (eskf_svc_check_rotation(c->r_sb) != 0) return "in.r_sb (no es ortonormal con det +1)";
    *why_code = 0x8004u;
    if ((double)c->merge_holdoff_us * 1e-6 + (double)c->nav.eskf.dt_max > c->nav.stale_max_s)
        return "stale_max_s < espera de orden + dt_max";
    *why_code = 0x8005u;
    if (c->nav.eskf.gap_max != 0 || c->nav.gyro_sat_max_s != 0.0 || c->nav.eskf.has_mag_ref != 0)
        return "puente, continuación del giróscopo y magnetómetro deben seguir deshabilitados";
    *why_code = 0x8007u;
    if (c->nav.acc_sat_max_s > 0.0 && !(c->sat_motor_window_s > 0.0f))
        return "continuación en saturación sin ventana del motor (in.sat_motor_window_s)";
    *why_code = 0x8006u;
    const char *w = nav_config_check(&c->nav);
    if (w) return w;
    *why_code = 0;
    return NULL;
}

uint16_t eskf_svc_subscription(const eskf_svc_config_t *c)
{
    if (eskf_svc_check_transport(c)) return 0;
    return (uint16_t)(c->kpid_imu | c->kpid_baro);
}

int eskf_svc_default_config(int profile, eskf_svc_config_t *cfg) { return eskf_svc_build_config(profile, cfg); }

/* ------------------------------------------------------------------ traza */
uint16_t eskf_svc_trec_crc(const eskf_svc_trec_t *r, uint16_t crc)
{
    uint8_t b[16];
    uint64_t u = (uint64_t)r->t;
    b[0] = r->type;
    b[1] = (uint8_t)r->res;
    b[2] = (uint8_t)r->a, b[3] = (uint8_t)(r->a >> 8);
    b[4] = (uint8_t)r->b, b[5] = (uint8_t)(r->b >> 8);
    b[6] = (uint8_t)r->c, b[7] = (uint8_t)(r->c >> 8);
    for (int k = 0; k < 8; k++) b[8 + k] = (uint8_t)(u >> (8 * k));
    return eskf_svc_crc16(b, sizeof b, crc);
}
static void trace_add(eskf_svc_t *s, uint8_t type, int res, uint16_t a, uint16_t b, uint16_t c, int64_t t)
{
    if (!s->cfg.trace_enable || s->trace_closed) return;
    if (s->trace_n - s->trace_rd >= ESKF_SVC_TRACE_CAP) {
        s->diag.trace_overflow++; /* no se sobrescribe: una traza con huecos no sirve para reproducir */
        return;
    }
    eskf_svc_trec_t *r = &s->trace[s->trace_n % ESKF_SVC_TRACE_CAP];
    r->type = type;
    r->res = (int8_t)(res < -128 ? -128 : (res > 127 ? 127 : res));
    r->a = a, r->b = b, r->c = c;
    r->t = t;
    s->trace_n++;
    s->trace_count++;
    s->trace_crc = eskf_svc_trec_crc(r, s->trace_crc);
}
int eskf_svc_trace_close(eskf_svc_t *s)
{
    if (!s || s->magic != SVC_MAGIC) return ESKF_SVC_E_NOT_INIT;
    if (!s->cfg.trace_enable) return ESKF_SVC_E_CONFIG;
    if (s->trace_closed) return ESKF_SVC_OK;
    if (s->trace_n - s->trace_rd >= ESKF_SVC_TRACE_CAP) return ESKF_SVC_E_FULL; /* vaciar y repetir */
    eskf_svc_trec_t *r = &s->trace[s->trace_n % ESKF_SVC_TRACE_CAP];
    r->type = ESKF_SVC_TR_END;
    r->res = 0;
    r->a = s->trace_crc;
    r->b = c16(s->diag.trace_overflow);
    r->c = 0;
    r->t = (int64_t)s->trace_count;
    s->trace_n++;
    s->trace_closed = 1;
    return ESKF_SVC_OK;
}

uint32_t eskf_svc_trace_read(eskf_svc_t *s, eskf_svc_trec_t *dst, uint32_t n)
{
    if (!s || !dst || s->magic != SVC_MAGIC) return 0;
    uint32_t k = 0;
    while (k < n && s->trace_rd != s->trace_n) dst[k++] = s->trace[s->trace_rd++ % ESKF_SVC_TRACE_CAP];
    return k;
}

static void fill_output(eskf_svc_t *s, int64_t now);
static void publish_query(eskf_svc_t *s);

/* ------------------------------------------------------------------ sesión */
/* Sesión nueva. boundary_us: las muestras con marca anterior pertenecen a la sesión previa y se
   descartan (buzón y router incluidos); los eventos ya pendientes en la cola de orden se borran. */
static void session_start(eskf_svc_t *s, int reset_streams, int64_t boundary_us)
{
    s->session = (uint16_t)(s->session + 1u);
    if (s->session == 0) s->session = 1;
    nav_merge_init(&s->merge);
    if (reset_streams) {
        memset(&s->st_imu, 0, sizeof s->st_imu);
        memset(&s->st_baro, 0, sizeof s->st_baro);
    }
    s->last_popped_us = INT64_MIN;
    s->t_session0_us = boundary_us;
    s->seq_out_imu = s->seq_out_baro = 0;
    s->latched = 0;
    s->t_fail_s = -INFINITY;
    s->prev_rest_ok = 0;
    s->have_baro = 0;
    s->baro_ref_changed = 0;
    memset(&s->bb, 0, sizeof s->bb);
    s->bb.t_us = INT64_MIN;
    s->bb.fast_t0 = INT64_MIN;
    s->bb.rest_t0 = INT64_MIN;
    s->flight = 0;
    s->nav_cut = 0;
    /* contrato de inicio: la sesión empieza en rampa y no hay despegue antes de pad_start_s */
    s->pad = 1;
    s->start_us = boundary_us;
    s->pad_t_us = boundary_us == INT64_MIN ? INT64_MIN : boundary_us + (int64_t)((double)s->cfg.pad_start_s * 1e6);
    s->imu_run_t0 = s->imu_prev_t = INT64_MIN;
    s->pad_rx_us = INT64_MIN;
    if (s->nav_enabled && nav_init(&s->nav, &s->cfg.nav) != NAV_OK) {
        s->nav_enabled = 0;
        s->why = "nav_init";
        s->why_code = 0x80FFu;
    }
    s->next_snap_us = s->next_diag_us = s->next_ops_us = INT64_MIN;
    s->tx_n = s->tx_next = 0;
    s->txd_pending = s->txo_pending = 0;
    /* diagnósticos: por sesión, salvo los contadores de reinicio y de la traza */
    uint32_t rq = s->diag.restarts_requested, rf = s->diag.restarts_refused, tov = s->diag.trace_overflow;
    uint32_t bn = s->diag.baro_ref_notified;
    memset(&s->diag, 0, sizeof s->diag);
    s->diag.restarts_requested = rq;
    s->diag.restarts_refused = rf;
    s->diag.trace_overflow = tov;
    s->diag.baro_ref_notified = bn;
    s->diag.first_imu_us = s->diag.first_baro_us = INT64_MIN;
    for (int i = 0; i < 4; i++) s->diag.dmp_q_last[i] = (float)NAN;
    s->diag.dmp_accuracy_last = s->diag.baro_p_last = s->diag.baro_t_last = (float)NAN;
}

int eskf_svc_init(eskf_svc_t *s, const eskf_svc_config_t *cfg, eskf_svc_clock_fn clock, void *clock_ctx)
{
    if (!s || !cfg || !clock) return ESKF_SVC_E_NULL;
    if (eskf_svc_check_transport(cfg)) return ESKF_SVC_E_CONFIG;
    eskf_svc_hook_fn hook = (s->magic == SVC_MAGIC) ? s->hook : NULL;
    void *hctx = (s->magic == SVC_MAGIC) ? s->hook_ctx : NULL;
    memset(s, 0, sizeof *s);
    s->hook = hook;
    s->hook_ctx = hctx;
    s->cfg = *cfg;
    s->clock = clock;
    s->clock_ctx = clock_ctx;
    uint16_t code = 0;
    const char *why = eskf_svc_check_nav(cfg, &code);
    s->nav_enabled = (why == NULL);
    s->why = why ? why : "";
    s->why_code = code;
    s->readiness = eskf_svc_readiness_cfg(cfg); /* preparación de ESTA configuración exacta */
    s->config_id = eskf_svc_config_id(cfg);
    s->magic = SVC_MAGIC;
    s->now_us = INT64_MIN;
    s->trace_crc = 0xFFFFu;
    session_start(s, 1, INT64_MIN);
    int64_t t0 = clock(clock_ctx); /* inicio de la primera sesión: el contrato de rampa cuenta desde aquí */
    s->start_us = t0;
    s->pad_t_us = t0 + (int64_t)((double)cfg->pad_start_s * 1e6);
    fill_output(s, INT64_MIN); /* las consultas anteriores al primer consume ya ven sesión 1, NO VÁLIDO y NaN */
    publish_query(s);
    s->initialized = 1;
    trace_add(s, ESKF_SVC_TR_CONFIG, ESKF_SVC_TRACE_VERSION, s->config_id,
              (uint16_t)((uint16_t)cfg->profile | (uint16_t)(ESKF_MSG_VERSION << 8)), cfg->boot_id, t0);
    return ESKF_SVC_OK;
}

int eskf_svc_ensure_init(eskf_svc_t *s, const eskf_svc_config_t *cfg, eskf_svc_clock_fn clock, void *clock_ctx)
{
    if (!s) return ESKF_SVC_E_NULL;
    if (s->magic == SVC_MAGIC && s->initialized) return 0; /* segunda llamada: no toca nada */
    int r = eskf_svc_init(s, cfg, clock, clock_ctx);
    return r == ESKF_SVC_OK ? 1 : r;
}

void eskf_svc_request_restart(eskf_svc_t *s)
{
    if (s) __atomic_store_n(&s->restart_req, 1u, __ATOMIC_RELEASE);
}

void eskf_svc_notify_baro_ref_change(eskf_svc_t *s)
{
    if (s) __atomic_store_n(&s->baro_ref_req, 1u, __ATOMIC_RELEASE);
}

void eskf_svc_set_hook(eskf_svc_t *s, eskf_svc_hook_fn fn, void *ctx)
{
    if (!s) return;
    s->hook = fn;
    s->hook_ctx = ctx;
}

/* ------------------------------------------------------------------ entrada */
int eskf_svc_deliver_packet(eskf_svc_t *s, const uint8_t pkt[ESKF_SVC_PKT_SIZE])
{
    if (!s || !pkt) return ESKF_SVC_E_NULL;
    if (s->magic != SVC_MAGIC || !s->initialized) return ESKF_SVC_E_NOT_INIT;
    int64_t rx = s->clock(s->clock_ctx);
    uint16_t apid = rd_u16(pkt + ESKF_SVC_OFF_APID), seq = rd_u16(pkt + ESKF_SVC_OFF_SEQ);
    uint16_t len = rd_u16(pkt + ESKF_SVC_OFF_LEN);
    int ret = ESKF_SVC_OK;
    if (pkt[ESKF_SVC_OFF_VERSION] != ESKF_SVC_PKT_VERSION) {
        s->diag.rejected_version++;
        ret = ESKF_SVC_E_VERSION;
    } else if (eskf_svc_crc16(pkt, ESKF_SVC_OFF_CRC, 0xFFFFu) != rd_u16(pkt + ESKF_SVC_OFF_CRC)) {
        s->diag.rejected_crc++; /* la pérdida se verá como salto de secuencia del flujo */
        ret = ESKF_SVC_E_CRC;
    } else if (apid != s->cfg.kpid_imu && apid != s->cfg.kpid_baro) {
        s->diag.rejected_apid++; /* incluye la salida propia: nunca se procesa */
        ret = ESKF_SVC_E_APID;
    } else if (len > ESKF_SVC_PAYLOAD_MAX) {
        s->diag.rejected_len++;
        ret = ESKF_SVC_E_LEN;
    } else {
        MB_LOCK();
        uint32_t head = s->mb_head, tail = ld_acq(&s->mb_tail);
        if (head - tail >= ESKF_SVC_MAILBOX_CAP) {
            MB_UNLOCK();
            s->diag.mailbox_full++; /* causa; la pérdida se cuenta UNA vez, como salto de secuencia */
            if (apid == s->cfg.kpid_imu) s->diag.mailbox_full_imu++;
            else s->diag.mailbox_full_baro++;
            ret = ESKF_SVC_E_FULL;
        } else {
            eskf_svc_item_t *it = &s->mb[head % ESKF_SVC_MAILBOX_CAP];
            it->rx_us = rx;
            it->apid = apid;
            it->seq = seq;
            it->len = len;
            it->ts_ms = rd_u32(pkt + ESKF_SVC_OFF_TS);
            memcpy(it->payload, pkt + ESKF_SVC_OFF_PAYLOAD, len);
            st_rel(&s->mb_head, head + 1u);
            MB_UNLOCK();
            uint32_t occ = head + 1u - tail;
            if (occ > s->diag.mailbox_hwm) s->diag.mailbox_hwm = occ;
            s->diag.delivered++;
        }
    }
    trace_add(s, ESKF_SVC_TR_DELIVER, ret, apid, seq, len, rx);
    return ret;
}

/* ------------------------------------------------------------------ IMU */
int eskf_svc_imu_decode(const eskf_svc_config_t *c, const uint8_t *payload, uint16_t len, nav_imu_sample_t *out,
                        uint8_t *sat_sensor, int *bad)
{
    float v[6];
    int b = 0;
    uint8_t ss = 0;
    if (!c || !payload || !out || len < 24u) {
        if (bad) *bad = 2;
        return 0;
    }
    for (int i = 0; i < 6; i++) {
        v[i] = rd_f32(payload + 4 * i);
        if (!isfinite(v[i])) b = 1;
    }
    if (!b) {
        for (int j = 0; j < 3; j++) {
            /* vuelta al dominio de códigos del productor, ANTES de rotar (identidad de ejes):
               código = (v + offset) · divisor. Los offsets no se vuelven a restar en la señal. */
            double ca = ((double)v[j] + (double)c->acc_off[j]) * (double)c->acc_div;
            double cg = ((double)v[3 + j] + (double)c->gyro_off[j]) * (double)c->gyro_div;
            if (fabs(ca) > 32768.5 || fabs(cg) > 32768.5) b = 2; /* imposible para el productor */
            if (fabs(ca) >= 32768.0 - (double)c->sat_margin_codes) ss = (uint8_t)(ss | (1u << j));
            if (fabs(cg) >= 32768.0 - (double)c->sat_margin_codes) ss = (uint8_t)(ss | (1u << (3 + j)));
        }
    }
    out->sat = 0u;
    if (b) {
        for (int i = 0; i < 3; i++) out->acc[i] = out->gyro[i] = (float)NAN;
    } else {
        /* unidades una sola vez, después la rotación sensor → cuerpo, en float y en este orden */
        float sa[3], sg[3];
        for (int j = 0; j < 3; j++) sa[j] = v[j] * c->acc_unit_si, sg[j] = v[3 + j] * c->gyro_unit_si;
        for (int i = 0; i < 3; i++) {
            const float *r = c->r_sb + 3 * i;
            float a0 = r[0] * sa[0], a1 = r[1] * sa[1], a2 = r[2] * sa[2];
            float g0 = r[0] * sg[0], g1 = r[1] * sg[1], g2 = r[2] * sg[2];
            out->acc[i] = (a0 + a1) + a2;
            out->gyro[i] = (g0 + g1) + g2;
            if (c->sat_to_nav)
                for (int j = 0; j < 3; j++) {
                    if (fabsf(r[j]) <= 1e-6f) continue;
                    if (ss & (1u << j)) out->sat = (uint8_t)(out->sat | (NAV_SAT_ACC_X << i));
                    if (ss & (1u << (3 + j))) out->sat = (uint8_t)(out->sat | (NAV_SAT_GYRO_X << i));
                }
        }
    }
    if (sat_sensor) *sat_sensor = ss;
    if (bad) *bad = b;
    return b == 0;
}

/* Pérdida o descarte de IMU: degradado y enclavado; en rampa se recupera tras una ventana de
   reposo COMPLETA posterior (t_fail_s). Pérdida de baro: informativa. */
static void note_loss(eskf_svc_t *s, int imu, uint32_t reason, int64_t t_us)
{
    if (imu) {
        s->diag.imu_loss_events++;
        s->latched |= reason;
        double t = (double)t_us * 1e-6;
        if (t > s->t_fail_s) s->t_fail_s = t;
    } else {
        s->latched |= ESKF_SVC_R_BARO_LOSS;
    }
}

/* Vuelo reconocido: una sola condición, enclavada hasta el final de la sesión, para todas las
   decisiones del servicio (reinicios, avisos, referencia de lanzamiento, recuperación de pérdidas,
   alimentación de navigation.c). Detectores válidos:
     1. navigation.c ha pasado a VUELO (su detector de despegue con la IMU);
     2. el respaldo sube más de bb_liftoff_m sobre la referencia de lanzamiento, y
     3. el respaldo mantiene |v_z| >= bb_flight_vz_mps durante bb_hold_s,
   2 y 3 sólo si el ESKF no puede atestiguar el reposo (deshabilitado, sin alinear o NO VÁLIDO).
   No se inventa un instante de despegue: sólo se sabe que el vuelo ya ha empezado.
   Si se reconoce con navigation.c todavía en RAMPA, deja de recibir muestras el resto de la sesión:
   así no puede alinearse, realinearse, aplicar ZUPT ni fijar otra referencia por un falso reposo
   (p. ej. un descenso uniforme bajo paracaídas). */
static int in_flight(const eskf_svc_t *s) { return s->flight; }

/* ---- Rampa CONFIRMADA frente a fase INCIERTA. La ausencia de despegue detectado no acredita la
   rampa: consta sólo por el contrato de inicio (sesión empezada en rampa, sin despegue antes de
   svc.pad_start_s) y, después, por evidencia CONTINUA de que no ha habido despegue:
     - IMU: racha de muestras válidas entregadas a navigation.c, con intervalos <= eskf.dt_max, de al
       menos nav.liftoff_hold_s (un despegue durante el hueco anterior lo vería su detector: el
       empuje > 2,5 g dura ~1 s, más que svc.pad_gap_max_s + la espera);
     - baro: medida aceptada por el respaldo en seguimiento, sin movimiento rápido (|v_z| <
       bb.flight_vz_mps; un despegue con el baro siguiendo se reconoce como vuelo).
   Un hueco entre evidencias mayor que svc.pad_gap_max_s deja la fase INCIERTA para el resto de la
   sesión: nada que llegue después (un descenso lento, una IMU con ~1 g y poco giro, un baro casi
   quieto) vuelve a confirmar la rampa. Sólo una sesión nueva pedida por el anfitrión (que declara
   que está en rampa) la restablece. El hueco se mide con las marcas de las muestras (pad_note) y,
   si no llega ninguna, con el reloj (pad_clock): el silencio total también caduca la rampa. */
static uint16_t ms16(int64_t us) { return (uint16_t)(us <= 0 ? 0 : (us / 1000 > 0xFFFF ? 0xFFFF : us / 1000)); }
/* navigation.c deja de recibir muestras y reloj. Antes, un último nav_tick con el reloj de ESTE
   consume (que queda en la traza): su estado congelado no depende de consumes vacíos sin registrar. */
static void nav_freeze(eskf_svc_t *s)
{
    if (!s->nav_enabled || s->nav_cut) return;
    if (s->now_us != INT64_MIN) {
        double tnow = (double)s->now_us * 1e-6;
        int r = nav_tick(&s->nav, tnow);
        if (s->hook) s->hook(s->hook_ctx, 0, NULL, NULL, tnow, r, &s->nav);
    }
    s->nav_cut = 1;
    nav_merge_init(&s->merge); /* los eventos pendientes tampoco llegan */
}
static void pad_lost(eskf_svc_t *s, uint32_t cause, int64_t hole_us, int64_t t)
{
    if (!s->pad) return;
    s->pad = 0;
    s->diag.pad_lost_hole_us = hole_us;
    s->diag.pad_lost_cause = cause;
    trace_add(s, ESKF_SVC_TR_PHASE, (int)ESKF_SVC_PHASE_UNKNOWN, (uint16_t)cause, ms16(hole_us), 0, t);
    if (s->bb.have_ref) s->bb.ref_frozen = 1; /* el cero de rampa ya no sigue la deriva */
    if (s->nav_enabled && !s->nav_cut && s->nav.phase == NAV_PHASE_PAD)
        nav_freeze(s); /* ni alineación, ni realineación, ni ZUPT, ni referencia nueva */
}
static int64_t pad_wait_us(const eskf_svc_t *s) /* racha de IMU por confirmar */
{
    double hold = s->cfg.nav.liftoff_hold_s;
    return isfinite(hold) && hold > 0.0 ? (int64_t)(hold * 1e6) : 0;
}
/* Muestra con marca t, llegada en rx; ev_t0 = inicio de la evidencia que aporta (INT64_MIN si ninguna). */
static void pad_note(eskf_svc_t *s, int64_t t, int64_t rx, int64_t ev_t0)
{
    if (rx > s->pad_rx_us) s->pad_rx_us = rx;
    if (s->flight || !s->pad || s->pad_t_us == INT64_MIN) return;
    int64_t gap = (int64_t)((double)s->cfg.pad_gap_max_s * 1e6);
    if (ev_t0 != INT64_MIN && ev_t0 - s->pad_t_us <= gap) {
        if (t > s->pad_t_us) s->pad_t_us = t;
        return;
    }
    if (t - s->pad_t_us > gap + pad_wait_us(s)) pad_lost(s, 1, t - s->pad_t_us, t);
}
/* Silencio total: el reloj avanza y no se procesa ninguna muestra. Desde la llegada de la última
   muestra procesada (o desde el fin del contrato de inicio) se admiten pad_gap_max_s + la espera de
   la racha de IMU + pad_lat_max_s (latencia de entrega, cola del router incluida: tras un parón del
   router sus muestras llegan después y sus marcas siguen continuas). Se comprueba al empezar cada
   consume, ANTES de atender reinicios y avisos de referencia: éstos usan la fase actualizada. */
static void pad_clock(eskf_svc_t *s, int64_t now)
{
    if (s->flight || !s->pad || s->pad_t_us == INT64_MIN || s->start_us == INT64_MIN) return;
    int64_t ref = s->start_us + (int64_t)((double)s->cfg.pad_start_s * 1e6); /* contrato de inicio */
    if (s->pad_rx_us > ref) ref = s->pad_rx_us;
    int64_t lim = (int64_t)((double)s->cfg.pad_gap_max_s * 1e6) + pad_wait_us(s) + (int64_t)((double)s->cfg.pad_lat_max_s * 1e6);
    if (now - ref > lim) pad_lost(s, 2, now - ref, now);
}

static void flight_update(eskf_svc_t *s, int64_t now)
{
    eskf_svc_bb_t *b = &s->bb;
    const eskf_svc_config_t *c = &s->cfg;
    int nav_on = s->nav_enabled && !s->nav_cut;
    int attests = nav_on && s->nav.aligned && nav_validity(&s->nav, NULL) != NAV_INVALID;
    int tracking = b->state == ESKF_SVC_BB_TRACK;
    uint32_t det = 0;
    if (nav_on && s->nav.phase == NAV_PHASE_FLIGHT) det = 1;
    else if (!attests && tracking && b->have_ref && b->h - b->ref_m > (double)c->bb_liftoff_m) det = 2;
    else if (!attests && tracking && b->fast_t0 != INT64_MIN &&
             (double)(b->t_us - b->fast_t0) * 1e-6 >= (double)c->bb_hold_s)
        det = 3;
    if (!s->flight && det) {
        s->flight = 1;
        s->diag.flight_detector = det;
        trace_add(s, ESKF_SVC_TR_PHASE, (int)ESKF_SVC_PHASE_FLIGHT, (uint16_t)det, 0, 0, now);
    }
    if (!s->flight) return;
    if (b->have_ref) b->ref_frozen = 1; /* referencia de lanzamiento fija para todo el vuelo */
    if (nav_on && s->nav.phase == NAV_PHASE_PAD) nav_freeze(s);
}

static void handle_imu(eskf_svc_t *s, const eskf_svc_item_t *it, int64_t t_us)
{
    eskf_svc_diag_t *d = &s->diag;
    d->imu_rx++;
    if (it->len != (uint16_t)s->cfg.imu_format) {
        d->imu_bad_len++; /* 44 definitivo o 48 histórico, según configuración: no se mezclan */
        note_loss(s, 1, ESKF_SVC_R_IMU_LOSS, t_us);
        return;
    }
    /* DMP y accuracy: sólo diagnóstico; inválidos no afectan a la entrada inercial */
    float q[4], qn = 0.0f, acc = rd_f32(it->payload + 40);
    for (int i = 0; i < 4; i++) q[i] = rd_f32(it->payload + 24 + 4 * i), qn += q[i] * q[i];
    memcpy(d->dmp_q_last, q, sizeof q);
    d->dmp_accuracy_last = acc;
    if (!isfinite(qn) || fabsf(sqrtf(qn) - 1.0f) > 1e-3f || !isfinite(acc)) {
        d->dmp_invalid++;
        s->latched |= ESKF_SVC_R_DMP_INVALID;
    }
    if (s->cfg.imu_format == ESKF_SVC_IMU_FMT_HIST48 && rd_u32(it->payload + 44) != 0u) d->imu_reserved_nonzero++;
    nav_ev_imu_t e;
    uint8_t ss = 0;
    int bad = 0;
    eskf_svc_imu_decode(&s->cfg, it->payload, it->len, &e.s, &ss, &bad);
    if (ss & 0x07u) {
        d->sat_cand_acc++;
        /* La continuación en saturación modela la combustión: después de la ventana del motor
           (p. ej. despliegue) la saturación queda fuera del modelo → estimación NO VÁLIDA. */
        if (s->nav_enabled && s->nav.phase == NAV_PHASE_FLIGHT &&
            (double)t_us * 1e-6 > s->nav.t_liftoff + (double)s->cfg.sat_motor_window_s) {
            if (!(s->latched & ESKF_SVC_R_SAT_OUT_OF_MODEL)) d->sat_out_of_model++;
            s->latched |= ESKF_SVC_R_SAT_OUT_OF_MODEL;
        }
    }
    if (ss & 0x38u) d->sat_cand_gyro++;
    if (ss) s->latched |= ESKF_SVC_R_SAT_CANDIDATE;
    if (bad) {
        if (bad == 1) d->imu_nonfinite++;
        else d->imu_out_of_range++;
        note_loss(s, 1, ESKF_SVC_R_IMU_LOSS, t_us); /* se entrega como NaN: navigation.c conserva el tiempo */
    }
    if (d->first_imu_us == INT64_MIN) d->first_imu_us = t_us;
    int to_nav = s->nav_enabled && !s->nav_cut && t_us >= s->last_popped_us;
    int64_t ev = INT64_MIN;
    if (to_nav && !bad) { /* racha continua como la del detector de despegue de navigation.c */
        if (s->imu_prev_t == INT64_MIN || t_us - s->imu_prev_t > (int64_t)llround((double)s->cfg.nav.eskf.dt_max * 1e6))
            s->imu_run_t0 = t_us;
        s->imu_prev_t = t_us;
        double hold = s->cfg.nav.liftoff_hold_s;
        if (t_us - s->imu_run_t0 >= (isfinite(hold) && hold > 0.0 ? (int64_t)(hold * 1e6) : 0)) ev = s->imu_run_t0;
    } else if (bad) {
        s->imu_prev_t = s->imu_run_t0 = INT64_MIN; /* una muestra inválida rompe la continuidad */
    }
    pad_note(s, t_us, it->rx_us, ev); /* puede dejar la fase incierta y cortar navigation.c ANTES de esta muestra */
    if (!s->nav_enabled) return;
    if (s->nav_cut) {
        d->nav_cut_samples++;
        return;
    }
    if (t_us < s->last_popped_us) { /* ya se entregó un evento posterior: no se puede ordenar */
        d->late_imu++;
        note_loss(s, 1, ESKF_SVC_R_LATE_EVENT, t_us);
        return;
    }
    e.t_us = t_us;
    e.seq = s->seq_out_imu++;
    e.s.t = (double)t_us * 1e-6;
    if (nav_merge_push_imu(&s->merge, &e) != 0) note_loss(s, 1, ESKF_SVC_R_IMU_LOSS, t_us); /* no ocurre: contrapresión */
}

/* ------------------------------------------------------------------ respaldo barométrico */
/* Estados: NONE (sin ventana), RECOVER (reuniendo una ventana coherente) y TRACK (alfa-beta).
   El estado (h, v, t_us) es SIEMPRE el de la última medida ACEPTADA: un rechazo no lo propaga ni
   renueva su frescura; la predicción para la puerta se calcula al vuelo. Pasa a recuperación tras
   un corte (> bb_dt_max_s entre medidas recibidas) o tras bb_reseed_n rechazos seguidos; es
   válido sólo siguiendo y con la última aceptada a <= bb_timeout_s. Inicio y recuperación
   exigen bb_min_samples medidas consecutivas (intervalos <= bb_dt_max_s) cuyo ajuste lineal deja
   todos los residuos <= bb_coh_m: una lectura anómala nunca fija la altura ni la referencia. */
static void bb_enter_recover(eskf_svc_bb_t *b)
{
    b->state = ESKF_SVC_BB_RECOVER;
    b->win_n = 0;
    b->consec_rej = 0;
    b->fast_t0 = INT64_MIN;
    b->rest_t0 = INT64_MIN;
}

/* Ajuste lineal por mínimos cuadrados de la ventana; 1 si todos los residuos <= tol. */
static int bb_window_fit(const eskf_svc_bb_t *b, double tol, double *h_last, double *v)
{
    uint32_t n = b->win_n;
    int64_t t0 = b->win_t[n - 1u];
    double st = 0.0, sz = 0.0;
    for (uint32_t i = 0; i < n; i++) st += (double)(b->win_t[i] - t0) * 1e-6, sz += (double)b->win_z[i];
    double tm = st / (double)n, zm = sz / (double)n, sxx = 0.0, sxz = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        double dt = (double)(b->win_t[i] - t0) * 1e-6 - tm;
        sxx += dt * dt;
        sxz += dt * ((double)b->win_z[i] - zm);
    }
    if (!(sxx > 0.0)) return 0;
    double slope = sxz / sxx;
    for (uint32_t i = 0; i < n; i++) {
        double dt = (double)(b->win_t[i] - t0) * 1e-6 - tm;
        if (fabs((double)b->win_z[i] - (zm + slope * dt)) > tol) return 0;
    }
    *h_last = zm + slope * (0.0 - tm);
    *v = slope;
    return 1;
}

/* Devuelve 1 si la medida queda ACEPTADA en seguimiento (TRACK). */
static int bb_update(eskf_svc_t *s, int64_t t_us, double z)
{
    eskf_svc_bb_t *b = &s->bb;
    const eskf_svc_config_t *c = &s->cfg;
    eskf_svc_diag_t *d = &s->diag;
    if (b->state == ESKF_SVC_BB_TRACK) {
        double dt = (double)(t_us - b->t_us) * 1e-6;      /* desde la última ACEPTADA */
        double dr = (double)(t_us - b->t_rx_us) * 1e-6;   /* desde la última RECIBIDA */
        if (!(dt > 0.0) || !(dr > 0.0)) return 0; /* el seguimiento del flujo ya descarta repetidos y antiguos */
        b->t_rx_us = t_us;
        if (dr > (double)c->bb_dt_max_s) { /* corte: no se inventa continuidad; ventana nueva */
            d->bb_gaps++;
            bb_enter_recover(b);
        } else if (t_us - b->t_us > (int64_t)((double)c->bb_timeout_s * 1e6)) {
            /* caducado (rechazos sin ninguna aceptada durante más de bb.timeout_s): una sola medida
               compatible no lo devuelve a válido; hace falta una ventana coherente nueva */
            d->bb_expired++;
            bb_enter_recover(b);
        } else {
            double hp = b->h + b->v * dt, r = z - hp;
            if (fabs(r) > (double)c->bb_gate_m) { /* rechazada: ni propaga el estado ni renueva su frescura */
                d->bb_rejected++;
                if (++b->consec_rej >= c->bb_reseed_n) {
                    d->bb_reseeds++;
                    bb_enter_recover(b);
                }
                return 0;
            }
            b->h = hp + (double)c->bb_alpha * r;
            b->v += (double)c->bb_beta / dt * r;
            b->t_us = t_us;
            b->consec_rej = 0;
            d->bb_used++;
            if (fabs(b->v) < (double)c->bb_flight_vz_mps) b->fast_t0 = INT64_MIN;
            else if (b->fast_t0 == INT64_MIN) b->fast_t0 = t_us;
            if (fabs(b->v) > BB_REST_V_MPS) b->rest_t0 = INT64_MIN;
            else if (b->rest_t0 == INT64_MIN) b->rest_t0 = t_us;
            if (!b->have_ref) {
                /* referencia de lanzamiento SÓLO en rampa: vuelo no reconocido y reposo continuo de bb_hold_s
                   (descarta un ascenso, un descenso o el paso por el apogeo); nunca la altura del primer
                   dato recibido en vuelo */
                if (s->pad && !s->flight && b->rest_t0 != INT64_MIN &&
                    (double)(t_us - b->rest_t0) * 1e-6 >= (double)c->bb_hold_s) {
                    b->have_ref = 1;
                    b->ref_m = b->h;
                    b->n_ref = 1;
                }
            } else if (!b->ref_frozen && fabs(b->v) <= BB_REST_V_MPS) {
                /* referencia en rampa: sigue la deriva lenta del productor mientras está quieto */
                double k = dt / (double)c->bb_ref_tau_s;
                if (k > 1.0) k = 1.0;
                b->ref_m += k * (z - b->ref_m);
                b->n_ref++;
            }
            return 1;
        }
    }
    /* NONE o RECOVER: ventana de medidas consecutivas */
    if (b->win_n > 0u) {
        int64_t tl = b->win_t[b->win_n - 1u];
        if (t_us <= tl) return 0;
        if ((double)(t_us - tl) * 1e-6 > (double)c->bb_dt_max_s) b->win_n = 0; /* hueco: ventana nueva */
    }
    if (b->win_n == c->bb_min_samples) { /* llena e incoherente: se desliza */
        memmove(b->win_t, b->win_t + 1, (b->win_n - 1u) * sizeof b->win_t[0]);
        memmove(b->win_z, b->win_z + 1, (b->win_n - 1u) * sizeof b->win_z[0]);
        b->win_n--;
    }
    b->win_t[b->win_n] = t_us;
    b->win_z[b->win_n] = (float)z;
    b->win_n++;
    double h0, v0;
    if (b->win_n == c->bb_min_samples && bb_window_fit(b, (double)c->bb_coh_m, &h0, &v0)) {
        b->state = ESKF_SVC_BB_TRACK;
        b->h = h0;
        b->v = v0;
        b->t_us = b->t_rx_us = t_us;
        b->consec_rej = 0;
        b->win_n = 0;
        d->bb_recoveries++;
        b->fast_t0 = fabs(v0) >= (double)c->bb_flight_vz_mps ? t_us : INT64_MIN;
        b->rest_t0 = fabs(v0) <= BB_REST_V_MPS ? t_us : INT64_MIN;
        /* la ventana no fija la referencia: sólo el reposo continuo en rampa (arriba). Una referencia ya
           existente no cambia al recuperar: en rampa sigue la deriva; congelada, no cambia nunca */
        return 1;
    }
    return 0;
}

/* Salida del respaldo. La congelación de la referencia la decide flight_update (vuelo enclavado). */
static void bb_eval(eskf_svc_t *s, int64_t now, eskf_svc_baro_out_t *o)
{
    eskf_svc_bb_t *b = &s->bb;
    const eskf_svc_config_t *c = &s->cfg;
    int tracking = b->state == ESKF_SVC_BB_TRACK;
    int have_state = b->state != ESKF_SVC_BB_NONE || b->t_us != INT64_MIN;
    memset(o, 0, sizeof *o);
    o->state = b->state;
    o->t_us = have_state ? b->t_us : INT64_MIN;
    o->age_us = (have_state && b->t_us != INT64_MIN) ? now - b->t_us : INT64_MAX;
    o->ref_m = b->have_ref ? (float)b->ref_m : (float)NAN;
    o->h_m = (have_state && b->have_ref && b->t_us != INT64_MIN) ? (float)(b->h - b->ref_m) : (float)NAN;
    o->vz_mps = (have_state && b->t_us != INT64_MIN) ? (float)b->v : (float)NAN;
    o->ref_frozen = (uint8_t)b->ref_frozen;
    o->valid = (uint8_t)(tracking && b->have_ref && o->age_us <= (int64_t)((double)c->bb_timeout_s * 1e6) &&
                         !s->baro_ref_changed);
}

static void handle_baro(eskf_svc_t *s, const eskf_svc_item_t *it, int64_t t_us)
{
    eskf_svc_diag_t *d = &s->diag;
    const eskf_svc_config_t *c = &s->cfg;
    d->baro_rx++;
    if (it->len != ESKF_SVC_BARO_LEN) {
        d->baro_bad_len++;
        note_loss(s, 0, 0, t_us);
        return;
    }
    float alt = rd_f32(it->payload), p = rd_f32(it->payload + 4), tc = rd_f32(it->payload + 8);
    if (!isfinite(alt) || !(fabsf(alt) <= c->baro_h_abs_max_m) || !(p >= c->baro_p_min_pa && p <= c->baro_p_max_pa) ||
        !(tc >= c->baro_t_min_c && tc <= c->baro_t_max_c)) {
        d->baro_bad_value++; /* medida aislada: se descarta; el baro sigue en uso */
        note_loss(s, 0, 0, t_us);
        pad_note(s, t_us, it->rx_us, INT64_MIN);
        return;
    }
    d->baro_p_last = p;
    d->baro_t_last = tc;
    s->have_baro = 1;
    if (d->first_baro_us == INT64_MIN) d->first_baro_us = t_us;
    if (s->baro_ref_changed) { /* referencia cambiada en vuelo: ni ESKF ni respaldo el resto de la sesión */
        d->baro_not_fed++;
        pad_note(s, t_us, it->rx_us, INT64_MIN);
        return;
    }
    int acc = bb_update(s, t_us, (double)alt); /* la altura publicada por el productor, tal cual */
    pad_note(s, t_us, it->rx_us, (acc && fabs(s->bb.v) < (double)c->bb_flight_vz_mps) ? t_us : INT64_MIN);
    if (!s->nav_enabled) return;
    if (s->nav_cut) {
        d->nav_cut_samples++;
        return;
    }
    if (t_us < s->last_popped_us) {
        d->late_baro++;
        note_loss(s, 0, 0, t_us);
        return;
    }
    nav_ev_baro_t e;
    e.t_us = t_us;
    e.seq = s->seq_out_baro++;
    e.s.t = (double)t_us * 1e-6;
    e.s.alt_m = alt;
    if (nav_merge_push_baro(&s->merge, &e) != 0) note_loss(s, 0, 0, t_us);
}

/* Seguimiento de un flujo. Devuelve 0 si el paquete se procesa, −1 si se descarta. Nunca reinicia
   la sesión: el orden lo da el tiempo; la secuencia sólo sirve para contar pérdidas.
   - repetido (misma secuencia y marca no posterior): se descarta sin contarlo como pérdida;
   - antiguo (marca anterior): se descarta y se cuenta; no toca el estado del flujo;
   - secuencia hacia atrás con marca POSTERIOR: el productor reinició su contador con el reloj
     continuo → resincronización (no se sabe cuántos faltan: motivo SEQ_RESYNC);
   - salto hacia delante: pérdida real de (ds − 1) paquetes. */
static int track(eskf_svc_t *s, eskf_svc_stream_t *st, const eskf_svc_item_t *it, int imu, int64_t t_us)
{
    eskf_svc_diag_t *d = &s->diag;
    if (st->have) {
        uint16_t ds = (uint16_t)(it->seq - st->last_seq);
        if (t_us <= st->last_t_us) {
            if (ds == 0u) {
                d->seq_dup++;
                return -1;
            }
            if (t_us < st->last_t_us || ds >= 0x8000u) {
                /* antiguo: ya se procesó información posterior; si faltaba, el salto de
                   secuencia ya lo contó como pérdida. Sólo se cuenta. */
                d->seq_old++;
                if (imu) d->ts_back_imu++;
                else d->ts_back_baro++;
                return -1;
            }
            /* misma marca de ms y secuencia siguiente: se procesa (navigation.c decide) */
        } else if (ds == 0u || ds >= 0x8000u) {
            d->seq_resync++;
            s->latched |= ESKF_SVC_R_SEQ_RESYNC;
            if (imu) note_loss(s, 1, ESKF_SVC_R_SEQ_RESYNC, t_us);
        } else if (ds > 1u) {
            if (imu) d->seq_gaps_imu++, d->seq_lost_imu = sat_add(d->seq_lost_imu, ds - 1u),
                     note_loss(s, 1, ESKF_SVC_R_IMU_LOSS, t_us);
            else d->seq_gaps_baro++, d->seq_lost_baro = sat_add(d->seq_lost_baro, ds - 1u), note_loss(s, 0, 0, t_us);
        }
    }
    st->have = 1;
    st->last_seq = it->seq;
    st->last_t_us = t_us;
    return 0;
}

/* ------------------------------------------------------------------ salida */
static int64_t to_us(double t) { return isfinite(t) ? (int64_t)llround(t * 1e6) : INT64_MIN; }

static void fill_output(eskf_svc_t *s, int64_t now)
{
    eskf_svc_output_t *o = &s->scratch;
    memset(o, 0, sizeof *o);
    o->session = s->session;
    o->config_id = s->config_id;
    o->readiness = s->readiness;
    o->why_code = !s->nav_cut ? s->why_code : s->flight ? ESKF_SVC_WHY_FLIGHT_NAV_PAD : ESKF_SVC_WHY_PHASE_UNKNOWN;
    o->boot_id = s->cfg.boot_id;
    o->why = !s->nav_cut ? s->why
             : s->flight ? "vuelo reconocido con navigation.c en rampa: ESKF fuera de servicio el resto de la sesión"
                         : "fase incierta (no consta que siga en rampa): ESKF fuera de servicio el resto de la sesión";
    o->snap_id = s->snap_id;
    o->profile = (uint8_t)s->cfg.profile;
    o->nav_enabled = (uint8_t)(s->nav_enabled && !s->nav_cut);
    o->time_source = (uint8_t)s->cfg.time_source;
    if (s->nav_enabled) {
        nav_get_output(&s->nav, &o->nav);
    } else {
        static const nav_t no_nav = {0}; /* sin iniciar: nav_get_output publica NaN y NO VÁLIDO */
        nav_get_output(&no_nav, &o->nav);
    }
    o->t_state_us = to_us(o->nav.t_state);
    o->t_eval_us = to_us(o->nav.t_eval);
    o->t_proc_us = now;
    o->phase = (uint8_t)ESKF_SVC_PHASE_PAD;
    if (s->flight) { /* fase publicada coherente con el vuelo reconocido, aunque navigation.c no lo viera */
        o->nav.phase = (uint8_t)NAV_PHASE_FLIGHT;
        o->nav.at_rest = 0;
        o->phase = (uint8_t)ESKF_SVC_PHASE_FLIGHT;
    } else if (!s->pad) { /* ni rampa ni vuelo constan: se publica la incertidumbre */
        o->nav.at_rest = 0;
        o->phase = (uint8_t)ESKF_SVC_PHASE_UNKNOWN;
    } else if (o->nav.phase == (uint8_t)NAV_PHASE_FLIGHT) {
        o->phase = (uint8_t)ESKF_SVC_PHASE_FLIGHT;
    }
    bb_eval(s, now, &o->bb);
    int nav_on = s->nav_enabled && !s->nav_cut;
    int aligned = nav_on && s->nav.aligned;
    int have_ref = nav_on && s->nav.have_alt_ref;
    uint32_t r = s->latched | ESKF_SVC_R_SAT_UNKNOWN | ESKF_SVC_R_TIME_APPROX;
    if (!nav_on) r |= ESKF_SVC_R_NAV_DISABLED;
    if (!s->have_baro) r |= ESKF_SVC_R_NO_BARO;
    if (aligned && !have_ref) r |= ESKF_SVC_R_NO_ALT_REF; /* un primer BMP no es una referencia */
    if (s->readiness) r |= ESKF_SVC_R_NOT_VALIDATED;
    if (s->cfg.profile == ESKF_SVC_PROFILE_EXP) r |= ESKF_SVC_R_EXPERIMENTAL;
    o->svc_reasons = r;
    o->nav_validity = o->nav.validity;
    uint8_t v = o->nav.validity;
    if (r & ESKF_SVC_R_INVALID_MASK) v = (uint8_t)NAV_INVALID;
    else if (v == (uint8_t)NAV_VALID && (r & ESKF_SVC_R_DEGRADED_MASK)) v = (uint8_t)NAV_DEGRADED;
    o->svc_validity = v;
    uint8_t caps = 0;
    if (aligned && v != (uint8_t)NAV_INVALID) caps |= ESKF_SVC_CAP_ATTITUDE;
    if (have_ref) caps |= ESKF_SVC_CAP_BARO_REF;
    if ((caps & ESKF_SVC_CAP_ATTITUDE) && have_ref) caps |= ESKF_SVC_CAP_ESKF_NAV;
    if (o->bb.valid) caps |= ESKF_SVC_CAP_BARO_BACKUP;
    if (o->bb.ref_frozen) caps |= ESKF_SVC_CAP_LAUNCH_REF;
    o->caps = caps;
    o->source = (caps & ESKF_SVC_CAP_ESKF_NAV) ? (uint8_t)ESKF_SVC_SRC_ESKF
                : (caps & ESKF_SVC_CAP_BARO_BACKUP) ? (uint8_t)ESKF_SVC_SRC_BARO : (uint8_t)ESKF_SVC_SRC_NONE;
    /* aceptación para vuelo: sólo ESTRICTO con todo confirmado y validado en placa (hoy nunca) */
    o->flight_accept = (uint8_t)(s->cfg.profile == ESKF_SVC_PROFILE_STRICT && s->readiness == 0 && v != (uint8_t)NAV_INVALID);
}

static uint8_t msg_flags(const eskf_svc_t *s, const eskf_svc_output_t *o)
{
    uint8_t f = ESKF_MSG_F_TIME_MS;
    if (s->nav_enabled && !s->nav_cut) f |= ESKF_MSG_F_NAV_ON;
    if (s->cfg.profile == ESKF_SVC_PROFILE_EXP) f |= ESKF_MSG_F_EXP;
    if (s->cfg.profile == ESKF_SVC_PROFILE_STRICT) f |= ESKF_MSG_F_STRICT;
    if (o->flight_accept) f |= ESKF_MSG_F_ACCEPT;
    return f;
}

static void build_snapshot(eskf_svc_t *s, const eskf_svc_output_t *o)
{
    eskf_msg_snapshot_t m;
    memset(&m, 0, sizeof m);
    m.session = o->session;
    m.snap_id = o->snap_id;
    m.flags = msg_flags(s, o);
    m.mask = (uint8_t)(s->nav_enabled ? s->cfg.snapshot_mask : 1u); /* sin navegación: sólo estado y motivo */
    m.t_state_us = o->t_state_us;
    m.t_eval_us = o->t_eval_us;
    m.t_proc_us = o->t_proc_us;
    m.nav_reasons = (uint16_t)o->nav.reasons;
    m.svc_reasons = (uint16_t)o->svc_reasons;
    m.readiness = o->readiness;
    m.why_code = o->why_code;
    m.config_id = o->config_id;
    m.nav_validity = o->nav_validity;
    m.svc_validity = o->svc_validity;
    m.phase = o->phase;
    m.at_rest = o->nav.at_rest;
    m.heading_known = o->nav.heading_known;
    m.profile = o->profile;
    m.time_source = o->time_source;
    m.caps = o->caps;
    m.source = o->source;
    for (int i = 0; i < 3; i++) {
        m.p[i] = o->nav.p[i], m.v[i] = o->nav.v[i];
        m.sigma_th[i] = o->nav.sigma_th[i], m.sigma_p[i] = o->nav.sigma_p[i], m.sigma_v[i] = o->nav.sigma_v[i];
        m.b_a[i] = o->nav.b_a[i], m.b_g[i] = o->nav.b_g[i];
    }
    for (int i = 0; i < 4; i++) m.q[i] = o->nav.q[i];
    m.alt_ref = (float)o->nav.alt_ref;
    m.alt_ref_seq = o->nav.alt_ref_seq;
    s->tx_n = eskf_msg_encode_snapshot(&m, s->tx, s->tx_len);
    s->tx_next = 0;
}

static void build_diag(eskf_svc_t *s, const eskf_svc_output_t *o)
{
    const eskf_svc_diag_t *d = &s->diag;
    eskf_msg_diag_t m;
    memset(&m, 0, sizeof m);
    m.session = s->session;
    m.diag_id = s->diag_id;
    m.flags = msg_flags(s, o);
    m.imu_rx = d->imu_rx;
    m.baro_rx = d->baro_rx;
    /* pérdidas contadas UNA vez: los descartes del buzón y los CRC rechazados ya aparecen como
       saltos de secuencia; los repetidos no son pérdidas */
    m.imu_lost = sat_add(sat_add(d->seq_lost_imu, sat_add(d->imu_bad_len, d->imu_nonfinite)),
                         sat_add(d->imu_out_of_range, d->late_imu));
    m.baro_lost = sat_add(sat_add(d->seq_lost_baro, sat_add(d->baro_bad_len, d->baro_bad_value)), d->late_baro);
    m.rejected = c16(sat_add(sat_add(d->rejected_apid, d->rejected_version), sat_add(d->rejected_crc, d->rejected_len)));
    m.sat_cand = c16(sat_add(d->sat_cand_acc, d->sat_cand_gyro));
    m.last_imu_ret = ci16(d->last_imu_ret);
    m.last_baro_ret = ci16(d->last_baro_ret);
    m.last_tick_ret = ci16(d->last_tick_ret);
    m.nav_gaps = c16(s->nav_enabled ? s->nav.cnt.gaps : 0u);
    m.out_fail = c16(sat_add(d->publish_fail, sat_add(d->pool_empty, d->kpid_mismatch)));
    m.skipped = c16(sat_add(sat_add(d->snapshots_skipped, d->diag_skipped), d->ops_skipped));
    m.mailbox_hwm = (uint8_t)(d->mailbox_hwm > 255u ? 255u : d->mailbox_hwm);
    m.seq_resyncs = (uint8_t)(d->seq_resync > 255u ? 255u : d->seq_resync);
    m.lat_rx_max_ms = c16((uint32_t)(d->lat_rx_max_us > 0 ? d->lat_rx_max_us / 1000 : 0));
    m.lat_proc_max_ms = c16((uint32_t)(d->lat_proc_max_us > 0 ? d->lat_proc_max_us / 1000 : 0));
    s->txd_len = eskf_msg_encode_diag(&m, s->txd);
    s->txd_pending = 1;
}

static void build_ops(eskf_svc_t *s, const eskf_svc_output_t *o, int64_t now)
{
    eskf_msg_ops_t m;
    memset(&m, 0, sizeof m);
    m.session = s->session;
    m.ops_id = s->ops_id;
    m.flags = msg_flags(s, o);
    m.t_ms = (uint32_t)(now / 1000);
    m.eskf_age_ms = age_ms(now, o->t_state_us);
    m.bb_age_ms = age_ms(now, o->bb.t_us);
    m.nav_reasons = (uint16_t)o->nav.reasons;
    m.svc_reasons = (uint16_t)o->svc_reasons;
    m.readiness = o->readiness;
    m.config_id = o->config_id;
    m.boot_id = o->boot_id;
    m.nav_validity = o->nav_validity;
    m.svc_validity = o->svc_validity;
    m.caps = o->caps;
    m.source = o->source;
    m.phase = o->phase;
    m.profile = o->profile;
    m.bb_valid = o->bb.valid;
    m.bb_ref_frozen = o->bb.ref_frozen;
    m.eskf_h_m = -o->nav.p[2]; /* NED: altura sobre la referencia = −p_z; NaN antes de alinear */
    m.eskf_vz_mps = -o->nav.v[2];
    m.bb_h_m = o->bb.h_m;
    m.bb_vz_mps = o->bb.vz_mps;
    s->txo_len = eskf_msg_encode_ops(&m, s->txo);
    s->txo_pending = 1;
}

static void publish_query(eskf_svc_t *s)
{
    OUT_LOCK(); /* copia breve; el filtro nunca se ejecuta aquí */
    s->out = s->scratch;
    s->out_diag = s->diag;
    OUT_UNLOCK();
}

/* ------------------------------------------------------------------ consumo */
int eskf_svc_consume(eskf_svc_t *s)
{
    if (!s || s->magic != SVC_MAGIC || !s->initialized) return ESKF_SVC_E_NOT_INIT;
    const eskf_svc_config_t *c = &s->cfg;
    eskf_svc_diag_t *d = &s->diag;
    int64_t now = s->clock(s->clock_ctx);
    s->now_us = now;
    int control = 0;
    int flight0 = s->flight, pad0 = s->pad;
    pad_clock(s, now); /* silencio total: antes de atender reinicios y avisos, que usan la fase actual */
    if (__atomic_exchange_n(&s->restart_req, 0u, __ATOMIC_ACQ_REL)) {
        control = 1;
        s->diag.restarts_requested++;
        if (in_flight(s)) { /* en vuelo no se vuelve a rampa */
            s->diag.restarts_refused++;
            s->latched |= ESKF_SVC_R_RESTART_REFUSED;
            trace_add(s, ESKF_SVC_TR_RESTART, 1, s->session, 0, 0, now);
        } else {
            session_start(s, 0, now); /* los flujos siguen: sin pérdidas ficticias */
            trace_add(s, ESKF_SVC_TR_RESTART, 0, s->session, 0, 0, now);
        }
    }
    if (__atomic_exchange_n(&s->baro_ref_req, 0u, __ATOMIC_ACQ_REL)) {
        control = 1;
        s->diag.baro_ref_notified++;
        trace_add(s, ESKF_SVC_TR_BAROREF, 0, s->session, 0, 0, now);
        if (in_flight(s) || !s->pad) { /* en vuelo o sin rampa confirmada: sin baro el resto de la sesión */
            s->baro_ref_changed = 1;
            s->latched |= ESKF_SVC_R_BARO_REF_CHANGED;
        } else { /* en rampa: sesión nueva con datos POSTERIORES al aviso; nueva referencia en reposo */
            session_start(s, 0, now);
            trace_add(s, ESKF_SVC_TR_RESTART, 0, s->session, 1, 0, now);
        }
    }
    d->consume_calls++;
    uint32_t items = 0;
    for (uint32_t k = 0; k < c->max_items; k++) {
        uint32_t tail = s->mb_tail;
        if (tail == ld_acq(&s->mb_head)) break;
        const eskf_svc_item_t *peek = &s->mb[tail % ESKF_SVC_MAILBOX_CAP];
        int is_imu = peek->apid == c->kpid_imu;
        if (s->nav_enabled && ((is_imu && s->merge.imu_n >= NAV_EV_IMU_CAP) || (!is_imu && s->merge.baro_n >= NAV_EV_BARO_CAP))) {
            d->merge_full_waits++; /* contrapresión: se queda en el buzón */
            break;
        }
        eskf_svc_item_t it = *peek;
        st_rel(&s->mb_tail, tail + 1u);
        items++;
        /* marca de construcción (ms) extendida a 64 bits contra la recepción: misma época */
        int64_t rx_ms = it.rx_us / 1000;
        int64_t t_ms = rx_ms + (int64_t)(int32_t)(it.ts_ms - (uint32_t)rx_ms);
        int64_t t_us = t_ms * 1000;
        if (now - it.rx_us > d->lat_proc_max_us) d->lat_proc_max_us = now - it.rx_us;
        if (t_us > it.rx_us + c->future_tol_us) { /* futura: se rechaza ANTES de tocar el flujo */
            d->clock_mismatch++;
            d->ts_future++;
            if (is_imu) note_loss(s, 1, ESKF_SVC_R_LATE_EVENT, it.rx_us);
            else note_loss(s, 0, 0, it.rx_us);
            continue;
        }
        if (it.rx_us - t_us > d->lat_rx_max_us) d->lat_rx_max_us = it.rx_us - t_us;
        if (t_us < s->t_session0_us) { /* de la sesión anterior: se descarta sin contarlo como pérdida */
            eskf_svc_stream_t *st = is_imu ? &s->st_imu : &s->st_baro;
            if (!st->have || t_us > st->last_t_us) st->have = 1, st->last_seq = it.seq, st->last_t_us = t_us;
            d->session_discarded++;
            continue;
        }
        if (track(s, is_imu ? &s->st_imu : &s->st_baro, &it, is_imu, t_us) != 0) continue;
        if (is_imu) handle_imu(s, &it, t_us);
        else handle_baro(s, &it, t_us);
    }
    if (items > d->consume_items_max) d->consume_items_max = items;
    flight_update(s, now); /* antes de entregar eventos: un vuelo ya reconocido no llega a navigation.c en rampa */
    uint32_t pops = 0;
    if (s->nav_enabled && !s->nav_cut) { /* cortado: navigation.c queda congelado (ni reloj ni muestras) */
        double tnow = (double)now * 1e-6;
        d->last_tick_ret = nav_tick(&s->nav, tnow); /* reloj real: ni adelantado ni atrasado */
        if (s->hook) s->hook(s->hook_ctx, 0, NULL, NULL, tnow, d->last_tick_ret, &s->nav);
        for (; pops < c->max_events; pops++) {
            nav_ev_imu_t ei;
            nav_ev_baro_t eb;
            nav_ev_kind_t kind = nav_merge_pop(&s->merge, now, c->merge_holdoff_us, &ei, &eb);
            if (kind == NAV_EV_NONE) break;
            int ret;
            if (kind == NAV_EV_IMU) {
                ret = d->last_imu_ret = nav_on_imu(&s->nav, &ei.s);
                s->last_popped_us = ei.t_us;
                if (s->hook) s->hook(s->hook_ctx, 1, &ei.s, NULL, 0.0, ret, &s->nav);
                /* recuperación en rampa: una ventana de reposo COMPLETA que empezó después del fallo */
                uint32_t rok = s->nav.cnt.rest_windows_ok;
                if (rok != s->prev_rest_ok) {
                    s->prev_rest_ok = rok;
                    uint32_t rec = ESKF_SVC_R_IMU_LOSS | ESKF_SVC_R_LATE_EVENT | ESKF_SVC_R_SEQ_RESYNC;
                    if ((s->latched & rec) && s->nav.phase == NAV_PHASE_PAD && !s->flight && s->pad &&
                        ei.s.t - c->nav.rest_window_s - (double)c->nav.eskf.dt_max > s->t_fail_s) {
                        s->latched &= ~rec;
                        d->loss_recovered++;
                    }
                }
            } else {
                ret = d->last_baro_ret = nav_on_baro(&s->nav, &eb.s);
                if (eb.t_us > s->last_popped_us) s->last_popped_us = eb.t_us;
                if (s->hook) s->hook(s->hook_ctx, 2, NULL, &eb.s, 0.0, ret, &s->nav);
            }
            d->nav_calls++;
        }
        uint32_t backlog = s->merge.imu_n + s->merge.baro_n;
        if (backlog > d->nav_backlog_max) d->nav_backlog_max = backlog;
    }
    flight_update(s, now);
    if (s->flight != flight0 || s->pad != pad0) control = 1; /* la traza reproduce el instante de la transición */
    fill_output(s, now);
    uint16_t built = 0;
    if (c->kpid_out != 0u && c->snapshot_period_us > 0 && (s->next_snap_us == INT64_MIN || now >= s->next_snap_us)) {
        s->next_snap_us = now + c->snapshot_period_us;
        if (s->tx_next < s->tx_n) {
            d->snapshots_skipped++; /* el anterior aún no ha salido entero: no se mezclan */
        } else {
            s->snap_id++;
            s->scratch.snap_id = s->snap_id;
            d->snapshots++;
            build_snapshot(s, &s->scratch);
            built |= 1u;
        }
    }
    if (c->kpid_out != 0u && c->diag_period_us > 0 && (s->next_diag_us == INT64_MIN || now >= s->next_diag_us)) {
        s->next_diag_us = now + c->diag_period_us;
        if (s->txd_pending) d->diag_skipped++;
        s->diag_id++;
        d->diag_msgs++;
        build_diag(s, &s->scratch);
        built |= 2u;
    }
    if (c->kpid_out != 0u && c->ops_period_us > 0 && (s->next_ops_us == INT64_MIN || now >= s->next_ops_us)) {
        s->next_ops_us = now + c->ops_period_us;
        if (s->txo_pending) d->ops_skipped++; /* se sustituye por la más reciente */
        s->ops_id++;
        d->ops_msgs++;
        build_ops(s, &s->scratch, now);
        built |= 4u;
    }
    publish_query(s);
    if (items || pops || built || control)
        trace_add(s, ESKF_SVC_TR_CONSUME, 0, (uint16_t)items, (uint16_t)pops, built, now);
    return (int)items;
}

/* prioridad: compacta (telemetría), después fragmentos del snapshot, después diagnóstico */
int eskf_svc_produce(eskf_svc_t *s, uint8_t out[ESKF_SVC_PAYLOAD_MAX], uint16_t *len)
{
    if (!s || !out || !len || s->magic != SVC_MAGIC) return 0;
    if (s->txo_pending) {
        *len = s->txo_len;
        memcpy(out, s->txo, *len);
        s->txo_pending = 0;
    } else if (s->tx_next < s->tx_n) {
        *len = s->tx_len[s->tx_next];
        memcpy(out, s->tx[s->tx_next], *len);
        s->tx_next++;
    } else if (s->txd_pending) {
        *len = s->txd_len;
        memcpy(out, s->txd, *len);
        s->txd_pending = 0;
    } else {
        return 0;
    }
    s->diag.msgs_out++;
    return 1;
}

int eskf_svc_pending(const eskf_svc_t *s)
{
    if (!s || s->magic != SVC_MAGIC) return 0;
    return s->txo_pending || s->tx_next < s->tx_n || s->txd_pending;
}

void eskf_svc_note_output(eskf_svc_t *s, int outcome, uint8_t msg_type, uint16_t len, uint16_t pkt_seq,
                          uint32_t pkt_ts_ms)
{
    if (!s || s->magic != SVC_MAGIC) return;
    switch (outcome) {
    case ESKF_SVC_OUT_PUBLISHED: s->diag.publish_ok++; break;
    case ESKF_SVC_OUT_PUB_FAIL:
    case ESKF_SVC_OUT_PACKET_FAIL: s->diag.publish_fail++; break;
    case ESKF_SVC_OUT_DEFER_QUEUE: s->diag.queue_busy++; break;
    case ESKF_SVC_OUT_DEFER_POOL: s->diag.pool_empty++; break;
    default: break;
    }
    trace_add(s, ESKF_SVC_TR_OUTPUT, outcome, msg_type, len, pkt_seq, (int64_t)pkt_ts_ms);
}

int eskf_svc_get_output(const eskf_svc_t *s, eskf_svc_output_t *out)
{
    if (!s || !out || s->magic != SVC_MAGIC) return ESKF_SVC_E_NOT_INIT;
    OUT_LOCK();
    *out = s->out;
    OUT_UNLOCK();
    return ESKF_SVC_OK;
}

int eskf_svc_get_diag(const eskf_svc_t *s, eskf_svc_diag_t *dd)
{
    if (!s || !dd || s->magic != SVC_MAGIC) return ESKF_SVC_E_NOT_INIT;
    OUT_LOCK();
    *dd = s->out_diag;
    OUT_UNLOCK();
    return ESKF_SVC_OK;
}
