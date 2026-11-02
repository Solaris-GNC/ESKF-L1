/* eskf_svc_msg.c — codificador y decodificador del esquema de salida (eskf_svc_msg.h) */
#include "eskf_svc_msg.h"

#include <math.h>
#include <string.h>

const uint8_t ESKF_MSG_LEN[8] = {0, 38, 32, 28, 24, 24, 38, 38};

typedef char msg_len_fits[(38 <= ESKF_SVC_PAYLOAD_MAX - ESKF_MSG_HDR) ? 1 : -1];

static void w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v, p[1] = (uint8_t)(v >> 8); }
static void w32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}
static void w64(uint8_t *p, int64_t v)
{
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(u >> (8 * i));
}
static void wf(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    w32(p, u);
}
static uint16_t r16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t r32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int64_t r64(const uint8_t *p)
{
    uint64_t u = 0;
    for (int i = 7; i >= 0; i--) u = (u << 8) | p[i];
    return (int64_t)u;
}
static float rf(const uint8_t *p)
{
    uint32_t u = r32(p);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void hdr(uint8_t *m, uint8_t type, uint8_t mask, uint8_t flags, uint16_t session, uint32_t id, uint16_t crc)
{
    m[0] = (uint8_t)ESKF_MSG_VERSION;
    m[1] = type;
    m[2] = mask;
    m[3] = flags;
    w16(m + 4, session);
    w32(m + 6, id);
    w16(m + 10, crc);
}

/* fase (v4): un bit «vuelo» y otro «incierta»; ninguno = rampa CONFIRMADA */
static uint8_t is_flight(uint8_t ph) { return (uint8_t)(ph == ESKF_SVC_PHASE_FLIGHT); }
static uint8_t is_unknown(uint8_t ph) { return (uint8_t)(ph == ESKF_SVC_PHASE_UNKNOWN); }
static uint8_t phase_of(unsigned flight, unsigned unknown)
{
    return (uint8_t)(unknown ? ESKF_SVC_PHASE_UNKNOWN : (flight ? ESKF_SVC_PHASE_FLIGHT : ESKF_SVC_PHASE_PAD));
}

static void enc_data(const eskf_msg_snapshot_t *s, uint8_t t, uint8_t *b)
{
    switch (t) {
    case ESKF_MSG_T_STATE:
        w64(b, s->t_state_us);
        w64(b + 8, s->t_eval_us);
        w64(b + 16, s->t_proc_us);
        w16(b + 24, s->nav_reasons);
        w16(b + 26, s->svc_reasons);
        w16(b + 28, s->readiness);
        w16(b + 30, s->why_code);
        w16(b + 32, s->config_id);
        b[34] = (uint8_t)((s->nav_validity & 0x0Fu) | ((s->svc_validity & 0x0Fu) << 4));
        b[35] = (uint8_t)(is_flight(s->phase) | ((s->at_rest & 1u) << 1) | ((s->heading_known & 1u) << 2) |
                          (is_unknown(s->phase) << 3) | ((s->profile & 3u) << 4) | ((s->time_source & 3u) << 6));
        b[36] = s->caps;
        b[37] = s->source;
        break;
    case ESKF_MSG_T_PV:
        for (int i = 0; i < 3; i++) wf(b + 4 * i, s->p[i]), wf(b + 12 + 4 * i, s->v[i]);
        wf(b + 24, s->alt_ref);
        w32(b + 28, s->alt_ref_seq);
        break;
    case ESKF_MSG_T_ATT:
        for (int i = 0; i < 4; i++) wf(b + 4 * i, s->q[i]);
        for (int i = 0; i < 3; i++) wf(b + 16 + 4 * i, s->sigma_th[i]);
        break;
    case ESKF_MSG_T_SIG:
        for (int i = 0; i < 3; i++) wf(b + 4 * i, s->sigma_p[i]), wf(b + 12 + 4 * i, s->sigma_v[i]);
        break;
    default: /* BIAS */
        for (int i = 0; i < 3; i++) wf(b + 4 * i, s->b_a[i]), wf(b + 12 + 4 * i, s->b_g[i]);
        break;
    }
}

uint8_t eskf_msg_encode_snapshot(const eskf_msg_snapshot_t *s, uint8_t frags[ESKF_SVC_SNAP_FRAGS][ESKF_SVC_PAYLOAD_MAX],
                                 uint8_t lens[ESKF_SVC_SNAP_FRAGS])
{
    uint8_t mask = (uint8_t)(s->mask & ESKF_MSG_SNAP_MASK_ALL);
    uint16_t crc = 0xFFFFu;
    for (uint8_t t = 1; t <= ESKF_SVC_SNAP_FRAGS; t++) {
        if (!(mask & (1u << (t - 1u)))) continue;
        uint8_t b[ESKF_SVC_PAYLOAD_MAX - ESKF_MSG_HDR];
        memset(b, 0, sizeof b);
        enc_data(s, t, b);
        crc = eskf_svc_crc16(b, ESKF_MSG_LEN[t], crc);
    }
    uint8_t n = 0;
    for (uint8_t t = 1; t <= ESKF_SVC_SNAP_FRAGS; t++) {
        if (!(mask & (1u << (t - 1u)))) continue;
        memset(frags[n], 0, ESKF_SVC_PAYLOAD_MAX);
        hdr(frags[n], t, mask, s->flags, s->session, s->snap_id, crc);
        enc_data(s, t, frags[n] + ESKF_MSG_HDR);
        lens[n] = (uint8_t)(ESKF_MSG_HDR + ESKF_MSG_LEN[t]);
        n++;
    }
    return n;
}

uint8_t eskf_msg_encode_diag(const eskf_msg_diag_t *d, uint8_t out[ESKF_SVC_PAYLOAD_MAX])
{
    uint8_t *b = out + ESKF_MSG_HDR;
    memset(out, 0, ESKF_SVC_PAYLOAD_MAX);
    w32(b, d->imu_rx);
    w32(b + 4, d->baro_rx);
    w32(b + 8, d->imu_lost);
    w32(b + 12, d->baro_lost);
    w16(b + 16, d->rejected);
    w16(b + 18, d->sat_cand);
    w16(b + 20, (uint16_t)d->last_imu_ret);
    w16(b + 22, (uint16_t)d->last_baro_ret);
    w16(b + 24, (uint16_t)d->last_tick_ret);
    w16(b + 26, d->nav_gaps);
    w16(b + 28, d->out_fail);
    w16(b + 30, d->skipped);
    b[32] = d->mailbox_hwm;
    b[33] = d->seq_resyncs;
    w16(b + 34, d->lat_rx_max_ms);
    w16(b + 36, d->lat_proc_max_ms);
    hdr(out, ESKF_MSG_T_DIAG, 1u << (ESKF_MSG_T_DIAG - 1u), d->flags, d->session, d->diag_id,
        eskf_svc_crc16(b, ESKF_MSG_LEN[ESKF_MSG_T_DIAG], 0xFFFFu));
    return (uint8_t)(ESKF_MSG_HDR + ESKF_MSG_LEN[ESKF_MSG_T_DIAG]);
}

uint8_t eskf_msg_encode_ops(const eskf_msg_ops_t *o, uint8_t out[ESKF_SVC_PAYLOAD_MAX])
{
    uint8_t *b = out + ESKF_MSG_HDR;
    memset(out, 0, ESKF_SVC_PAYLOAD_MAX);
    w32(b, o->t_ms);
    w16(b + 4, o->eskf_age_ms);
    w16(b + 6, o->bb_age_ms);
    w16(b + 8, o->nav_reasons);
    w16(b + 10, o->svc_reasons);
    w16(b + 12, o->readiness);
    w16(b + 14, o->config_id);
    w16(b + 16, o->boot_id);
    b[18] = (uint8_t)((o->nav_validity & 0x0Fu) | ((o->svc_validity & 0x0Fu) << 4));
    b[19] = o->caps;
    b[20] = (uint8_t)((o->source & 3u) | (is_flight(o->phase) << 2) | (is_unknown(o->phase) << 3) | ((o->profile & 3u) << 4) |
                      ((o->bb_valid & 1u) << 6) | ((o->bb_ref_frozen & 1u) << 7));
    b[21] = 0;
    wf(b + 22, o->eskf_h_m);
    wf(b + 26, o->eskf_vz_mps);
    wf(b + 30, o->bb_h_m);
    wf(b + 34, o->bb_vz_mps);
    hdr(out, ESKF_MSG_T_OPS, 1u << (ESKF_MSG_T_OPS - 1u), o->flags, o->session, o->ops_id,
        eskf_svc_crc16(b, ESKF_MSG_LEN[ESKF_MSG_T_OPS], 0xFFFFu));
    return (uint8_t)(ESKF_MSG_HDR + ESKF_MSG_LEN[ESKF_MSG_T_OPS]);
}

static void dec_ops(const uint8_t *m, eskf_msg_ops_t *o)
{
    const uint8_t *b = m + ESKF_MSG_HDR;
    memset(o, 0, sizeof *o);
    o->flags = m[3];
    o->session = r16(m + 4);
    o->ops_id = r32(m + 6);
    o->t_ms = r32(b);
    o->eskf_age_ms = r16(b + 4);
    o->bb_age_ms = r16(b + 6);
    o->nav_reasons = r16(b + 8);
    o->svc_reasons = r16(b + 10);
    o->readiness = r16(b + 12);
    o->config_id = r16(b + 14);
    o->boot_id = r16(b + 16);
    o->nav_validity = (uint8_t)(b[18] & 0x0Fu);
    o->svc_validity = (uint8_t)(b[18] >> 4);
    o->caps = b[19];
    o->source = (uint8_t)(b[20] & 3u);
    o->phase = phase_of((b[20] >> 2) & 1u, (b[20] >> 3) & 1u);
    o->profile = (uint8_t)((b[20] >> 4) & 3u);
    o->bb_valid = (uint8_t)((b[20] >> 6) & 1u);
    o->bb_ref_frozen = (uint8_t)((b[20] >> 7) & 1u);
    o->eskf_h_m = rf(b + 22);
    o->eskf_vz_mps = rf(b + 26);
    o->bb_h_m = rf(b + 30);
    o->bb_vz_mps = rf(b + 34);
}

/* ---------------------------------------------------------------- decodificación */
void eskf_msg_reasm_init(eskf_msg_reasm_t *r) { memset(r, 0, sizeof *r); }

static void dec_snapshot(const eskf_msg_reasm_t *r, eskf_msg_snapshot_t *o)
{
    memset(o, 0, sizeof *o);
    o->session = r->session;
    o->snap_id = r->snap_id;
    o->flags = r->flags;
    o->mask = r->mask;
    o->t_state_us = o->t_eval_us = o->t_proc_us = INT64_MIN;
    float nanf_ = (float)NAN;
    for (int i = 0; i < 3; i++)
        o->p[i] = o->v[i] = o->sigma_th[i] = o->sigma_p[i] = o->sigma_v[i] = o->b_a[i] = o->b_g[i] = nanf_;
    for (int i = 0; i < 4; i++) o->q[i] = nanf_;
    o->alt_ref = nanf_;
    const uint8_t *b;
    if (r->mask & 1u) {
        b = r->data[0];
        o->t_state_us = r64(b);
        o->t_eval_us = r64(b + 8);
        o->t_proc_us = r64(b + 16);
        o->nav_reasons = r16(b + 24);
        o->svc_reasons = r16(b + 26);
        o->readiness = r16(b + 28);
        o->why_code = r16(b + 30);
        o->config_id = r16(b + 32);
        o->nav_validity = (uint8_t)(b[34] & 0x0Fu);
        o->svc_validity = (uint8_t)(b[34] >> 4);
        o->phase = phase_of(b[35] & 1u, (b[35] >> 3) & 1u);
        o->at_rest = (uint8_t)((b[35] >> 1) & 1u);
        o->heading_known = (uint8_t)((b[35] >> 2) & 1u);
        o->profile = (uint8_t)((b[35] >> 4) & 3u);
        o->time_source = (uint8_t)((b[35] >> 6) & 3u);
        o->caps = b[36];
        o->source = b[37];
    }
    if (r->mask & 2u) {
        b = r->data[1];
        for (int i = 0; i < 3; i++) o->p[i] = rf(b + 4 * i), o->v[i] = rf(b + 12 + 4 * i);
        o->alt_ref = rf(b + 24);
        o->alt_ref_seq = r32(b + 28);
    }
    if (r->mask & 4u) {
        b = r->data[2];
        for (int i = 0; i < 4; i++) o->q[i] = rf(b + 4 * i);
        for (int i = 0; i < 3; i++) o->sigma_th[i] = rf(b + 16 + 4 * i);
    }
    if (r->mask & 8u) {
        b = r->data[3];
        for (int i = 0; i < 3; i++) o->sigma_p[i] = rf(b + 4 * i), o->sigma_v[i] = rf(b + 12 + 4 * i);
    }
    if (r->mask & 16u) {
        b = r->data[4];
        for (int i = 0; i < 3; i++) o->b_a[i] = rf(b + 4 * i), o->b_g[i] = rf(b + 12 + 4 * i);
    }
}

static void dec_diag(const uint8_t *m, eskf_msg_diag_t *d)
{
    const uint8_t *b = m + ESKF_MSG_HDR;
    memset(d, 0, sizeof *d);
    d->flags = m[3];
    d->session = r16(m + 4);
    d->diag_id = r32(m + 6);
    d->imu_rx = r32(b);
    d->baro_rx = r32(b + 4);
    d->imu_lost = r32(b + 8);
    d->baro_lost = r32(b + 12);
    d->rejected = r16(b + 16);
    d->sat_cand = r16(b + 18);
    d->last_imu_ret = (int16_t)r16(b + 20);
    d->last_baro_ret = (int16_t)r16(b + 22);
    d->last_tick_ret = (int16_t)r16(b + 24);
    d->nav_gaps = r16(b + 26);
    d->out_fail = r16(b + 28);
    d->skipped = r16(b + 30);
    d->mailbox_hwm = b[32];
    d->seq_resyncs = b[33];
    d->lat_rx_max_ms = r16(b + 34);
    d->lat_proc_max_ms = r16(b + 36);
}

int eskf_msg_reasm_push(eskf_msg_reasm_t *r, const uint8_t *m, uint16_t len, eskf_msg_snapshot_t *snap,
                        eskf_msg_diag_t *diag)
{
    return eskf_msg_reasm_push_ops(r, m, len, snap, diag, NULL);
}

int eskf_msg_reasm_push_ops(eskf_msg_reasm_t *r, const uint8_t *m, uint16_t len, eskf_msg_snapshot_t *snap,
                            eskf_msg_diag_t *diag, eskf_msg_ops_t *ops)
{
    if (!r || !m || len < ESKF_MSG_HDR || m[0] != ESKF_MSG_VERSION) {
        if (r) r->invalid++;
        return -1;
    }
    uint8_t type = m[1], mask = m[2];
    if (type < 1u || type > ESKF_MSG_T_OPS || len != ESKF_MSG_HDR + ESKF_MSG_LEN[type]) {
        r->invalid++;
        return -1;
    }
    if (type == ESKF_MSG_T_OPS) { /* autónoma: no toca el snapshot en curso */
        if (mask != (1u << (ESKF_MSG_T_OPS - 1u)) ||
            eskf_svc_crc16(m + ESKF_MSG_HDR, ESKF_MSG_LEN[type], 0xFFFFu) != r16(m + 10)) {
            r->crc_bad++;
            return -2;
        }
        r->ops_ok++;
        if (ops) dec_ops(m, ops);
        return 3;
    }
    if (type == ESKF_MSG_T_DIAG) {
        if (mask != (1u << (ESKF_MSG_T_DIAG - 1u)) ||
            eskf_svc_crc16(m + ESKF_MSG_HDR, ESKF_MSG_LEN[type], 0xFFFFu) != r16(m + 10)) {
            r->crc_bad++;
            return -2;
        }
        r->diag_ok++;
        if (diag) dec_diag(m, diag);
        return 2;
    }
    if (!(mask & (1u << (type - 1u))) || (mask & ~ESKF_MSG_SNAP_MASK_ALL)) {
        r->invalid++;
        return -1;
    }
    uint16_t session = r16(m + 4), crc = r16(m + 10);
    uint32_t id = r32(m + 6);
    if (r->have && (session != r->session || id != r->snap_id)) {
        r->incomplete++;
        r->have = 0;
    }
    if (!r->have) {
        r->session = session;
        r->snap_id = id;
        r->crc = crc;
        r->flags = m[3];
        r->mask = mask;
    } else if (crc != r->crc || m[3] != r->flags || mask != r->mask || (r->have & (1u << (type - 1u)))) {
        r->mixed++;
        r->have = 0;
        return -2;
    }
    memcpy(r->data[type - 1u], m + ESKF_MSG_HDR, ESKF_MSG_LEN[type]);
    r->have = (uint8_t)(r->have | (1u << (type - 1u)));
    if (r->have != r->mask) return 0;
    uint16_t c = 0xFFFFu;
    for (uint8_t t = 1; t <= ESKF_SVC_SNAP_FRAGS; t++)
        if (r->mask & (1u << (t - 1u))) c = eskf_svc_crc16(r->data[t - 1u], ESKF_MSG_LEN[t], c);
    r->have = 0;
    if (c != r->crc) {
        r->crc_bad++;
        return -2;
    }
    r->complete++;
    if (snap) dec_snapshot(r, snap);
    return 1;
}
