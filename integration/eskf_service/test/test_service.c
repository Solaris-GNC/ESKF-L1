/*
 * test_service.c — pruebas del núcleo del servicio en el PC, sin SPP (los paquetes se construyen
 * aquí byte a byte como SPP_SERVICES_DATABANK_packetData). Las pruebas con el router REAL de SPP
 * están en test_spp_router.c; las de la captura y las herramientas en test_spp_tools.py.
 * 1–13 siguen la lista del pedido del 30-oct; 14–22 son las regresiones del 31-oct (mañana), 23–26 las del cierre (respaldo, sesiones, IMU en vuelo)
 * y 27–28 las de la noche del 31-oct (vuelo enclavado; cero de lanzamiento sólo en rampa); 29–30 las del 1-nov
 * (fase incierta frente a rampa confirmada; caducidad del respaldo); 31 la del silencio total (1-nov, noche)
 * (un caso por cada defecto señalado sobre 02fe8e4).
 */
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <pthread.h>
#include <stdlib.h>

#include "eskf_svc_msg.h"
#include "eskf_svc_params.h"
#include "test_common.h"

static eskf_svc_t S; /* estático, como en la placa */

#define KI 0x0004u
#define KB 0x0008u
#define KO 0x0010u

static eskf_svc_config_t cfg_of(int profile)
{
    eskf_svc_config_t c;
    eskf_svc_default_config(profile, &c);
    c.kpid_imu = KI, c.kpid_baro = KB, c.kpid_out = KO;
    return c;
}

static int start(const eskf_svc_config_t *c)
{
    memset(&S, 0, sizeof S);
    g_now_us = 1000000;
    return eskf_svc_init(&S, c, test_clock, NULL);
}

/* ---------------------------------------------------------------- simulador de productores */
typedef void (*truth_fn)(double t, double a[3], double w[3], double *h);
typedef struct {
    double imu_next, baro_next, imu_period, baro_period, baro_start;
    double gap_every, gap_len, next_gap;
    uint32_t lose_every, n_imu;
    uint16_t seq_imu, seq_baro;
    int hist48, consume_every, k;
    double lat_s;
    double baro_off; /* desplazamiento de la ALTURA publicada (cambio de referencia del productor) */
    truth_fn truth;
} sim_t;

static void static_truth(double t, double a[3], double w[3], double *h)
{
    (void)t;
    a[0] = 0.1, a[1] = -0.05, a[2] = -sqrt(1.0 - 0.0125);
    w[0] = w[1] = w[2] = 0.0;
    *h = 0.0;
}

static void sim_init(sim_t *m, double t0, truth_fn f)
{
    memset(m, 0, sizeof *m);
    m->imu_next = t0, m->baro_next = t0 + 0.003;
    m->imu_period = 1.0 / 225.0, m->baro_period = 1.0 / 100.0;
    m->baro_start = t0;
    m->consume_every = 1;
    m->lat_s = 0.0002;
    m->truth = f;
    m->next_gap = 1e30;
}

static uint32_t lcg = 12345u;
static double nz(double s)
{
    lcg = lcg * 1664525u + 1013904223u;
    return s * (((double)(lcg >> 8) / 8388608.0) - 1.0);
}

static void deliver_consume(sim_t *m, const uint8_t p[64], double t)
{
    g_now_us = (int64_t)llround((t + m->lat_s) * 1e6);
    eskf_svc_deliver_packet(&S, p);
    if (++m->k % m->consume_every == 0) eskf_svc_consume(&S);
}

static void sim_run(sim_t *m, double t_end)
{
    uint8_t p[64], pl[48];
    while (m->imu_next < t_end || m->baro_next < t_end) {
        if (m->imu_next <= m->baro_next) {
            double t = m->imu_next;
            m->imu_next += m->imu_period;
            if (t >= m->next_gap) { /* parón del productor: muestras descartadas antes de publicar */
                if (t < m->next_gap + m->gap_len) continue;
                m->next_gap += m->gap_every;
            }
            double a[3], w[3], h;
            m->truth(t, a, w, &h);
            for (int i = 0; i < 3; i++) a[i] += nz(0.002), w[i] += nz(0.2);
            m->n_imu++;
            if (m->lose_every && m->n_imu % m->lose_every == 0) { m->seq_imu++; continue; } /* publicado y perdido */
            if (m->hist48) imu48(pl, a, w, 0.0f), mk_packet(p, KI, m->seq_imu++, (uint32_t)floor(t * 1000.0), pl, 48);
            else imu44(pl, a, w), mk_packet(p, KI, m->seq_imu++, (uint32_t)floor(t * 1000.0), pl, 44);
            deliver_consume(m, p, t);
        } else {
            double t = m->baro_next;
            m->baro_next += m->baro_period;
            if (t < m->baro_start) continue; /* calibración del productor: no publica */
            double a[3], w[3], h;
            m->truth(t, a, w, &h);
            baro12(pl, h + nz(0.05));
            if (m->baro_off != 0.0) wrf(pl, rdf(pl) + (float)m->baro_off); /* misma presión física */
            mk_packet(p, KB, m->seq_baro++, (uint32_t)floor(t * 1000.0), pl, 12);
            deliver_consume(m, p, t);
        }
    }
}

/* ---------------------------------------------------------------- 1/2. IMU, unidades, ejes, límites */
static void test_imu_decode(void)
{
    printf("1/2. IMU de 44 B, histórico de 48, unidades, offsets ya aplicados, ejes y límites\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    uint8_t pl[48];
    const double a[3] = {0.5, -0.25, -1.0}, w[3] = {10.0, -20.0, 30.0};
    imu44(pl, a, w);
    nav_imu_sample_t s;
    uint8_t ss;
    int bad;
    int ok = eskf_svc_imu_decode(&c, pl, 44, &s, &ss, &bad);
    float v0 = rdf(pl), g0 = rdf(pl + 12);
    CHECK(ok && bad == 0 && ss == 0, "decodifica");
    CHECK(s.acc[0] == v0 * 9.80665f, "acel: valor publicado × 9.80665 (offset NO restado otra vez): %.9g", (double)s.acc[0]);
    CHECK(fabs((double)v0 - (0.5 - 0.004621)) < 1e-6, "el publicado ya lleva el offset del productor");
    CHECK(s.gyro[0] == g0 * 0.017453292f, "giro: publicado × π/180 una sola vez: %.9g", (double)s.gyro[0]);
    /* ejes: cuerpo x = −sensor y, cuerpo y = sensor x, cuerpo z = sensor z (rotación de 90°) */
    float r[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    memcpy(c.r_sb, r, sizeof r);
    CHECK(eskf_svc_check_rotation(c.r_sb) == 0, "rotación propia aceptada");
    eskf_svc_imu_decode(&c, pl, 44, &s, &ss, &bad);
    CHECK(s.acc[0] == -(rdf(pl + 4) * 9.80665f) && s.acc[1] == rdf(pl) * 9.80665f, "rotación aplicada a la acel.");
    CHECK(s.gyro[0] == -(rdf(pl + 16) * 0.017453292f) && s.gyro[2] == rdf(pl + 20) * 0.017453292f,
          "la MISMA rotación al giro");
    float refl[9] = {1, 0, 0, 0, 1, 0, 0, 0, -1}, skew[9] = {1, 0.1f, 0, 0, 1, 0, 0, 0, 1};
    CHECK(eskf_svc_check_rotation(refl) == -3, "reflexión (det −1) rechazada");
    CHECK(eskf_svc_check_rotation(skew) == -2, "no ortonormal rechazada");
    memcpy(c.r_sb, refl, sizeof refl);
    uint16_t code;
    CHECK(eskf_svc_check_nav(&c, &code) != NULL && code == 0x8003u, "navegación no arranca con det −1");
    /* límites: saturación en el dominio de códigos, antes de rotar */
    memcpy(c.r_sb, r, sizeof r);
    const double asat[3] = {5.0, 0.0, -1.0}, wz[3] = {0, 0, 0};
    imu44(pl, asat, wz);
    float vsat = rdf(pl);
    eskf_svc_imu_decode(&c, pl, 44, &s, &ss, &bad);
    CHECK(fabs((double)vsat - (32767.0 / 8192.0 - 0.004621)) < 1e-6, "extremo publicado desplazado: %.7f g", (double)vsat);
    CHECK(ss == 0x01u && bad == 0, "candidato en el eje x del SENSOR: código (v + offset)·8192 = 32767");
    CHECK(s.sat == NAV_SAT_ACC_Y, "y llega al eje y del CUERPO por la rotación (0x%02x)", s.sat);
    /* el mismo número sin offset (dominio antiguo ±3.99988) es imposible para este productor */
    wrf(pl, 3.9998779f);
    eskf_svc_imu_decode(&c, pl, 44, &s, &ss, &bad);
    CHECK(bad == 2 && isnan(s.acc[0]), "valor fuera del dominio de códigos: rechazado (fuera de rango)");
    const double wsat[3] = {0, -1100.0, 0};
    imu44(pl, a, wsat);
    eskf_svc_imu_decode(&c, pl, 44, &s, &ss, &bad);
    CHECK(ss == 0x10u && s.sat == NAV_SAT_GYRO_X, "giro: candidato en sensor y → cuerpo x (0x%02x)", s.sat);
    /* histórico de 48 B */
    eskf_svc_config_t h = cfg_of(ESKF_SVC_PROFILE_EXP);
    eskf_svc_config_historical(&h);
    imu48(pl, a, w, 0.0f);
    eskf_svc_imu_decode(&h, pl, 48, &s, &ss, &bad);
    CHECK(s.acc[0] == rdf(pl) * 9.80665f && bad == 0, "48 B histórico sin offsets");
    const double ah[3] = {3.99988, 0, 0};
    imu48(pl, ah, w, 0.0f);
    eskf_svc_imu_decode(&h, pl, 48, &s, &ss, &bad);
    CHECK((ss & 1u) && bad == 0, "histórico: ±3.9998779 g es candidato (sin offsets)");
}

/* ---------------------------------------------------------------- 3. paquetes inválidos */
static void test_invalid(void)
{
    printf("3. CRC, relleno 0xFF, versión, identificador, longitud, payload incompleto y valores\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_ACQ);
    start(&c);
    g_now_us = 1015000; /* recepción posterior a todas las marcas (1000–1012 ms) */
    uint8_t pl[48], p[64];
    const double a[3] = {0, 0, -1}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    mk_packet(p, KI, 0, 1000, pl, 44);
    CHECK(p[12 + 44] == 0xFF && p[61] == 0xFF, "relleno 0xFF tras el payload, como packetData");
    CHECK(eskf_svc_deliver_packet(&S, p) == ESKF_SVC_OK, "paquete válido");
    uint8_t q[64];
    memcpy(q, p, 64);
    q[60] = 0x00; /* relleno distinto: el CRC lo cubre */
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_CRC, "el relleno participa en el CRC");
    memcpy(q, p, 64);
    q[20] ^= 0x01;
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_CRC, "payload corrompido");
    memcpy(q, p, 64);
    q[0] = 2;
    wr16(q + 62, eskf_svc_crc16(q, 62, 0xFFFFu));
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_VERSION, "versión 2");
    mk_packet(q, KO, 0, 1000, pl, 44);
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_APID, "salida propia: rechazada");
    mk_packet(q, 0x0020u, 0, 1000, pl, 44);
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_APID, "APID no suscrito");
    mk_packet(q, KI, 1, 1000, pl, 44);
    wr16(q + 5, 51);
    wr16(q + 62, eskf_svc_crc16(q, 62, 0xFFFFu));
    CHECK(eskf_svc_deliver_packet(&S, q) == ESKF_SVC_E_LEN, "longitud 51");
    CHECK(S.diag.rejected_crc == 2 && S.diag.rejected_version == 1 && S.diag.rejected_apid == 2 && S.diag.rejected_len == 1,
          "contadores de rechazo");
    mk_packet(q, KI, 1, 1004, pl, 40); /* payload incompleto */
    eskf_svc_deliver_packet(&S, q);
    float nan_ = (float)NAN;
    uint8_t pn[44];
    memcpy(pn, pl, 44);
    wrf(pn + 8, nan_);
    mk_packet(q, KI, 2, 1008, pn, 44);
    eskf_svc_deliver_packet(&S, q);
    uint8_t pb[12];
    baro12(pb, 0.0);
    wrf(pb + 4, 200000.0f);
    mk_packet(q, KB, 0, 1009, pb, 12);
    eskf_svc_deliver_packet(&S, q);
    mk_packet(q, KB, 1, 1010, pb, 8);
    eskf_svc_deliver_packet(&S, q);
    uint8_t pd[44];
    memcpy(pd, pl, 44);
    wrf(pd + 24, 0.5f); /* cuaternión DMP no unitario: sólo diagnóstico */
    mk_packet(q, KI, 3, 1012, pd, 44);
    eskf_svc_deliver_packet(&S, q);
    g_now_us = 1020000;
    eskf_svc_consume(&S);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    eskf_svc_diag_t d;
    eskf_svc_get_diag(&S, &d);
    CHECK(d.imu_bad_len == 1 && d.imu_nonfinite == 1, "IMU de 40 B y NaN contadas");
    CHECK(d.baro_bad_value == 1 && d.baro_bad_len == 1, "baro fuera de rango y de 8 B contados");
    CHECK(d.dmp_invalid == 1 && (o.svc_reasons & ESKF_SVC_R_DMP_INVALID), "DMP inválido: diagnóstico");
    CHECK(o.svc_reasons & ESKF_SVC_R_IMU_LOSS, "IMU inutilizable: pérdida visible");
    CHECK(o.svc_validity == NAV_INVALID && (o.svc_reasons & ESKF_SVC_R_NAV_DISABLED), "ACQ: NO VÁLIDA (sin navegación)");
    /* DMP inválido con a y ω utilizables: la muestra sí sirve (no es pérdida) */
    eskf_svc_config_t cb = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&cb);
    mk_packet(q, KI, 0, 1000, pd, 44);
    g_now_us = 1000300;
    eskf_svc_deliver_packet(&S, q);
    eskf_svc_consume(&S);
    CHECK(S.diag.dmp_invalid == 1 && !(S.latched & ESKF_SVC_R_IMU_LOSS) && S.merge.imu_n + S.nav.cnt.imu_rx == 1,
          "DMP inválido no impide usar a y ω");
}

/* ---------------------------------------------------------------- 4. KPID */
static void test_kpid(void)
{
    printf("4. KPID asignados en distinto orden y ausencia de autosuscripción\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_ACQ);
    const uint16_t orders[3][3] = {{0x4, 0x8, 0x10}, {0x10, 0x8, 0x4}, {0x8, 0x40, 0x20}};
    for (int k = 0; k < 3; k++) {
        c.kpid_imu = orders[k][0], c.kpid_baro = orders[k][1], c.kpid_out = orders[k][2];
        uint16_t sub = eskf_svc_subscription(&c);
        CHECK(sub == (orders[k][0] | orders[k][1]) && !(sub & c.kpid_out), "orden %d: suscripción 0x%04x", k, sub);
    }
    c.kpid_imu = 0x4, c.kpid_baro = 0x4, c.kpid_out = 0x10;
    CHECK(eskf_svc_check_transport(&c) && eskf_svc_subscription(&c) == 0, "entradas iguales");
    c.kpid_imu = 0x4, c.kpid_baro = 0x8, c.kpid_out = 0x8;
    CHECK(eskf_svc_check_transport(&c) != NULL, "salida = entrada (autosuscripción)");
    c.kpid_imu = 0xC, c.kpid_baro = 0x10, c.kpid_out = 0x20;
    CHECK(eskf_svc_check_transport(&c) != NULL, "KPID de varios bits");
    c.kpid_imu = 0x1, c.kpid_baro = 0x10, c.kpid_out = 0x20;
    CHECK(eskf_svc_check_transport(&c) != NULL, "bit de log");
    c.kpid_imu = 0x4, c.kpid_baro = 0x2, c.kpid_out = 0x20;
    CHECK(eskf_svc_check_transport(&c) != NULL, "bit de FSM");
    c.kpid_imu = 0x4, c.kpid_baro = 0x8, c.kpid_out = 0x1;
    CHECK(eskf_svc_check_transport(&c) != NULL, "salida en el bit de log");
    c.kpid_imu = 0x4, c.kpid_baro = 0x8, c.kpid_out = 0;
    CHECK(eskf_svc_check_transport(&c) == NULL && eskf_svc_subscription(&c) == 0xC, "sin salida (registro parcial) admitido");
    c.kpid_imu = 0x4, c.kpid_baro = 0x8, c.kpid_out = 0x10;
    CHECK(eskf_svc_init(&S, &c, test_clock, NULL) == ESKF_SVC_OK, "válida");
    c.kpid_out = 0x4;
    eskf_svc_t T2;
    memset(&T2, 0, sizeof T2);
    CHECK(eskf_svc_init(&T2, &c, test_clock, NULL) == ESKF_SVC_E_CONFIG, "init rechaza la autosuscripción");
}

/* ---------------------------------------------------------------- 5. inicialización dual, pendiente, reinicio */
static void test_init(void)
{
    printf("5. Inicialización dual, configuración pendiente y reinicio\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    memset(&S, 0, sizeof S);
    g_now_us = 1000000;
    CHECK(eskf_svc_ensure_init(&S, &c, test_clock, NULL) == 1, "primera: arranca");
    {
        eskf_svc_output_t o0;
        eskf_svc_get_output(&S, &o0);
        CHECK(o0.session == 1 && o0.svc_validity == NAV_INVALID && isnan(o0.nav.p[2]) && o0.t_state_us == INT64_MIN &&
                  o0.source == ESKF_SVC_SRC_NONE && o0.caps == 0,
              "consulta antes del primer consume: sesión 1, NO VÁLIDO, NaN (no una estructura a cero)");
    }
    uint8_t pl[48], p[64];
    const double a[3] = {0, 0, -1}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    mk_packet(p, KI, 0, 1000, pl, 44);
    eskf_svc_deliver_packet(&S, p);
    eskf_svc_config_t other = cfg_of(ESKF_SVC_PROFILE_ACQ);
    uint16_t ses = S.session;
    CHECK(eskf_svc_ensure_init(&S, &other, test_clock, NULL) == 0, "segunda: no reinicia");
    CHECK(S.session == ses && S.cfg.profile == ESKF_SVC_PROFILE_EXP && S.mb_head - S.mb_tail == 1,
          "ni configuración, ni cola, ni sesión cambian");
    /* ESTRICTO: transporte arranca, navegación no, motivo publicado */
    eskf_svc_config_t f = cfg_of(ESKF_SVC_PROFILE_STRICT);
    CHECK(start(&f) == ESKF_SVC_OK && !S.nav_enabled, "ESTRICTO arranca el transporte sin navegación");
    CHECK(!strcmp(S.why, "in.gyro_unit_si") && S.why_code > 0 && S.why_code < 0x8000u, "primer pendiente: %s (%u)", S.why,
          S.why_code);
    eskf_svc_consume(&S);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_validity == NAV_INVALID && (o.svc_reasons & ESKF_SVC_R_NAV_DISABLED) && o.readiness != 0,
          "salida: NO VÁLIDA, motivo y readiness 0x%04x", o.readiness);
    uint8_t m[50];
    uint16_t l;
    CHECK(eskf_svc_produce(&S, m, &l) == 1 && m[1] == ESKF_MSG_T_OPS && l <= 50, "primero la compacta (%u B)", l);
    CHECK(eskf_svc_produce(&S, m, &l) == 1 && m[1] == 1 && m[2] == 1, "sin navegación sólo se publica el fragmento de estado");
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_snapshot_t sn;
    CHECK(eskf_msg_reasm_push(&r, m, l, &sn, NULL) == 1 && sn.why_code == S.why_code, "el motivo llega a tierra");
    CHECK(eskf_svc_produce(&S, m, &l) == 1 && m[1] == 6, "y el diagnóstico");
    /* ningún pendiente se rellena con cero */
    size_t n;
    const eskf_param_t *T = eskf_svc_params_table(&n);
    int pend_ok = 1, pend_n = 0;
    for (size_t i = 0; i < n; i++)
        if (T[i].st_strict == ESKF_ST_PENDING) {
            pend_n++;
            if (!isnan(T[i].strict) || T[i].type > ESKF_T_F32 || isnan(T[i].exp)) pend_ok = 0;
            if (!isnan(eskf_svc_param_read(&f, &T[i]))) pend_ok = 0;
        }
    CHECK(pend_ok && pend_n > 20, "%d pendientes en ESTRICTO: NaN y de tipo real; en EXPERIMENTAL todos numéricos", pend_n);
    /* reinicio explícito (en rampa) */
    c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t sm;
    sim_init(&sm, 1.0, static_truth);
    sm.lose_every = 50;
    sim_run(&sm, 1.5);
    CHECK(S.latched & ESKF_SVC_R_IMU_LOSS, "pérdida enclavada antes del reinicio");
    ses = S.session;
    eskf_svc_request_restart(&S);
    sm.lose_every = 0;
    sim_run(&sm, 2.0);
    CHECK(S.session == ses + 1 && S.diag.restarts_requested == 1, "sesión nueva");
    CHECK(!(S.latched & ESKF_SVC_R_IMU_LOSS) && S.diag.seq_lost_imu == 0, "sin pérdidas ficticias tras el reinicio");
}

/* ---------------------------------------------------------------- 6. secuencia, wraps y tardíos */
static void test_sequence(void)
{
    printf("6. Duplicados, pérdidas, wraps de secuencia y de reloj, eventos atrasados\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    uint8_t pl[48], p[64];
    const double a[3] = {0, 0, -1}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    uint16_t sq[] = {65533, 65534, 65534, 65535, 0, 3, 2, 4};
    uint32_t ts[] = {1000, 1004, 1004, 1009, 1013, 1026, 1022, 1031};
    for (int i = 0; i < 8; i++) {
        mk_packet(p, KI, sq[i], ts[i], pl, 44);
        g_now_us = (int64_t)ts[i] * 1000 + 300;
        if (i == 6) g_now_us = 1026300; /* el atrasado llega después del 3 */
        eskf_svc_deliver_packet(&S, p);
        eskf_svc_consume(&S);
    }
    CHECK(S.diag.seq_dup == 1, "repetido descartado (no es pérdida)");
    CHECK(S.diag.seq_gaps_imu == 1 && S.diag.seq_lost_imu == 2, "65535 → 0 sin hueco; 0 → 3 pierde 2");
    CHECK(S.diag.seq_old == 1 && S.diag.ts_back_imu == 1, "2 tras 3: antiguo, descartado y contado");
    CHECK((S.latched & ESKF_SVC_R_IMU_LOSS) && !(S.latched & ESKF_SVC_R_LATE_EVENT), "pérdida enclavada; el antiguo no añade otra");
    CHECK(S.session == 1 && S.diag.seq_resync == 0, "ningún reinicio inferido");
    /* wrap del contador de 32 bits de ms: reloj justo antes y después de 2^32 ms */
    start(&c);
    int64_t base = 4294967296LL - 20; /* ms */
    for (int i = 0; i < 10; i++) {
        int64_t tms = base + 4 * i;
        mk_packet(p, KI, (uint16_t)i, (uint32_t)(tms & 0xFFFFFFFF), pl, 44);
        g_now_us = tms * 1000 + 300;
        eskf_svc_deliver_packet(&S, p);
        eskf_svc_consume(&S);
    }
    CHECK(S.diag.ts_back_imu == 0 && S.diag.ts_future == 0 && S.st_imu.last_t_us == (base + 36) * 1000,
          "wrap de timestampMs extendido contra el reloj de 64 bits");
    /* tardío entre flujos: baro más antiguo que la última IMU ya entregada */
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 1.2);
    m.baro_next = 1e30; /* el baro se retrasa 100 ms en el anfitrión */
    sim_run(&m, 1.3);
    uint8_t pb[12];
    baro12(pb, 0.0);
    mk_packet(p, KB, m.seq_baro, 1200, pb, 12); /* posterior al último baro, anterior a la IMU ya entregada */
    g_now_us = (int64_t)(1.3 * 1e6);
    eskf_svc_deliver_packet(&S, p);
    eskf_svc_consume(&S);
    CHECK(S.diag.late_baro == 1 && (S.latched & ESKF_SVC_R_BARO_LOSS), "baro tardío: rechazado y contado");
}

/* ---------------------------------------------------------------- 7. colas llenas y propiedad */
static void test_queues(void)
{
    printf("7. Colas llenas, propiedad de los paquetes y pérdidas contadas una sola vez\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    g_now_us = 1300000;
    uint8_t pl[48], p[64];
    const double a[3] = {0, 0, -1}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    int acc = 0, full = 0;
    for (int i = 0; i < 70; i++) {
        mk_packet(p, KI, (uint16_t)i, (uint32_t)(1000 + 4 * i), pl, 44);
        int r = eskf_svc_deliver_packet(&S, p);
        acc += r == ESKF_SVC_OK, full += r == ESKF_SVC_E_FULL;
        memset(p, 0xAA, 64); /* el emisor reutiliza su buffer: la copia es nuestra */
    }
    CHECK(acc == 64 && full == 6 && S.diag.mailbox_full_imu == 6, "64 aceptados, 6 descartados y contados");
    g_now_us = 1400000;
    int n = eskf_svc_consume(&S);
    CHECK(n == (int)c.max_items, "consume procesa como mucho max_items (%d)", n);
    CHECK(S.diag.nav_calls <= c.max_events, "y como mucho max_events llamadas a navigation.c");
    while (eskf_svc_consume(&S) > 0) {
    }
    CHECK(S.diag.imu_rx == 64 && S.diag.ts_back_imu == 0, "las copias no se alteraron al reutilizar el buffer");
    mk_packet(p, KI, 70, 1000 + 4 * 70, pl, 44);
    eskf_svc_deliver_packet(&S, p);
    eskf_svc_deliver_packet(&S, p); /* repetido: no es pérdida */
    uint8_t f[50];
    uint16_t l;
    while (eskf_svc_produce(&S, f, &l)) {
    }
    g_now_us = 2500000; /* toca diagnóstico */
    eskf_svc_consume(&S);
    CHECK(S.diag.seq_lost_imu == 6 && S.diag.seq_dup == 1 && (S.latched & ESKF_SVC_R_IMU_LOSS),
          "los 6 descartados del buzón se ven como 6 pérdidas de secuencia");
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_diag_t dg;
    memset(&dg, 0, sizeof dg);
    int got = 0;
    while (eskf_svc_produce(&S, f, &l))
        if (eskf_msg_reasm_push(&r, f, l, NULL, &dg) == 2) got = 1;
    CHECK(got && dg.imu_lost == 6, "REGRESIÓN D: el diagnóstico publica 6 pérdidas, no 12 (%u)", dg.imu_lost);
}

/* ---------------------------------------------------------------- 8. caducidad */
static void test_stale(void)
{
    printf("8. Caducidad sin sensores (sin marca temporal nueva para un estado congelado)\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0); /* alineación (5 s) y referencia barométrica (siguiente ventana) */
    eskf_svc_output_t o0, o1;
    g_now_us = (int64_t)13.0e6 + 16000; /* se entregan los últimos eventos retenidos por la espera de orden */
    eskf_svc_consume(&S);
    eskf_svc_get_output(&S, &o0);
    CHECK(o0.nav_validity == NAV_VALID, "alineado y válido antes de cortar (validez %u, motivos 0x%x)", o0.nav_validity,
          o0.nav.reasons);
    for (int k = 2; k <= 20; k++) {
        g_now_us = (int64_t)(13.0e6) + k * 10000;
        eskf_svc_consume(&S);
    }
    eskf_svc_get_output(&S, &o1);
    CHECK(o1.nav_validity == NAV_INVALID && (o1.nav.reasons & NAV_R_STALE), "sin sensores: STALE");
    CHECK(o1.t_state_us == o0.t_state_us && o1.t_proc_us > o0.t_proc_us, "t_state no avanza; t_proc sí");
}

/* ---------------------------------------------------------------- 9. barómetro inicial y referencia */
static void test_baro(void)
{
    printf("9. Barómetro inicial, referencia de navegación y capacidades\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    m.baro_start = 7.0; /* calibración del productor (500 muestras) + arranque: sin publicar */
    sim_run(&m, 6.9);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK((o.svc_reasons & ESKF_SVC_R_NO_BARO) && S.diag.baro_rx == 0, "sin baro: motivo publicado");
    CHECK(S.nav.cnt.align_ok >= 1 && isnan(o.nav.alt_ref), "alinea sin baro; sin referencia de altura (align %u)",
          S.nav.cnt.align_ok);
    CHECK(o.svc_validity == NAV_DEGRADED && (o.svc_reasons & ESKF_SVC_R_NO_ALT_REF),
          "REGRESIÓN: con alt_ref NaN no se publica VÁLIDO (validez %u)", o.svc_validity);
    CHECK((o.caps & ESKF_SVC_CAP_ATTITUDE) && !(o.caps & (ESKF_SVC_CAP_BARO_REF | ESKF_SVC_CAP_ESKF_NAV)) &&
              o.source == ESKF_SVC_SRC_NONE,
          "capacidades: actitud sí, navegación ESKF no (caps 0x%02x)", o.caps);
    sim_run(&m, 7.3);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.baro_rx > 0 && isnan(o.nav.alt_ref) && (o.svc_reasons & ESKF_SVC_R_NO_ALT_REF) &&
              o.svc_validity != NAV_VALID,
          "el primer BMP no es una referencia de navegación");
    CHECK(o.bb.state == ESKF_SVC_BB_TRACK && !o.bb.valid && isnan(o.bb.ref_m),
          "el respaldo sigue las medidas; el cero de lanzamiento espera %.1f s de reposo continuo", (double)c.bb_hold_s);
    sim_run(&m, 8.5);
    eskf_svc_get_output(&S, &o);
    CHECK((o.caps & ESKF_SVC_CAP_BARO_BACKUP) && o.source == ESKF_SVC_SRC_BARO && fabsf(o.bb.h_m) < 0.3f,
          "pero sí alimenta el respaldo barométrico (h %.3f m)", (double)o.bb.h_m);
    sim_run(&m, 13.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!(o.svc_reasons & (ESKF_SVC_R_NO_BARO | ESKF_SVC_R_NO_ALT_REF)) && isfinite(o.nav.alt_ref) &&
              fabs(o.nav.alt_ref) < 0.2 && o.svc_validity == NAV_VALID,
          "referencia en la siguiente ventana de reposo: %.3f m (altura RELATIVA del productor)", o.nav.alt_ref);
    CHECK((o.caps & ESKF_SVC_CAP_ESKF_NAV) && o.source == ESKF_SVC_SRC_ESKF, "navegación ESKF utilizable (caps 0x%02x)", o.caps);
    /* salto de la altura publicada SIN aviso: no se deduce un cambio de referencia */
    uint16_t ses = S.session;
    uint32_t gated0 = S.nav.cnt.baro_gated + S.nav.cnt.baro_rejected;
    for (int k = 0; k < 50; k++) { /* 0.5 s con +15 m */
        double t = 13.0 + 0.01 * k;
        uint8_t pb[12], p[64];
        baro12(pb, 15.0);
        mk_packet(p, KB, m.seq_baro++, (uint32_t)floor(t * 1000.0), pb, 12);
        m.baro_next = t + 0.01;
        sim_run(&m, t);
        g_now_us = (int64_t)llround((t + 0.0002) * 1e6);
        eskf_svc_deliver_packet(&S, p);
        eskf_svc_consume(&S);
    }
    sim_run(&m, 13.6);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses && !o.bb.ref_frozen && S.nav.cnt.baro_gated + S.nav.cnt.baro_rejected > gated0 + 40,
          "salto sin aviso: sin sesión nueva, sin despegue del respaldo; la puerta del ESKF lo rechaza");
    /* cambio de referencia CONFIRMADO por el anfitrión, en rampa: sesión nueva */
    eskf_svc_notify_baro_ref_change(&S);
    sim_run(&m, 13.7);
    CHECK(S.session == ses + 1 && S.diag.baro_ref_notified == 1 && !(S.latched & ESKF_SVC_R_BARO_REF_CHANGED),
          "aviso en rampa: sesión nueva, sin motivo enclavado");
    sim_run(&m, 25.0);
    eskf_svc_get_output(&S, &o);
    CHECK(isfinite(o.nav.alt_ref) && o.svc_validity == NAV_VALID, "nueva referencia tras realinear (%.3f m)", o.nav.alt_ref);
}

/* ---------------------------------------------------------------- 10. reposo, movimiento, despegue, huecos, saturación */
/* Vuelo vertical de prueba: empuje g_lift_g (g) durante g_burn s a lo largo de −z del cuerpo,
   después caída libre (fuerza específica nula) y, opcionalmente, un golpe de g_shock_g en g_shock_t.
   La altura es la integral exacta del perfil (coherente con la fuerza específica). */
static double g_lift_t = 1e30, g_lift_g = 3.5, g_burn = 1e30, g_shock_t = 1e30, g_shock_g = 0.0, g_shock_len = 0.05;
static double g_chute_v = 0.0; /* > 0: tras el apogeo, descenso uniforme a esa velocidad (fuerza específica = reposo) */
static void flight_reset(void)
{
    g_lift_t = 1e30, g_lift_g = 3.5, g_burn = 1e30, g_shock_t = 1e30, g_shock_g = 0.0, g_chute_v = 0.0;
}
static void flight_truth(double t, double a[3], double w[3], double *h)
{
    static_truth(t, a, w, h);
    if (t >= g_lift_t) {
        double dt = t - g_lift_t, up = (g_lift_g - 1.0) * 9.80665;
        if (dt < g_burn) {
            a[0] = 0, a[1] = 0, a[2] = -g_lift_g; /* empuje a lo largo del eje z del cuerpo (de prueba) */
            *h = 0.5 * up * dt * dt;
        } else {
            double tau = dt - g_burn, vb = up * g_burn, t_ap = vb / 9.80665;
            a[0] = a[1] = a[2] = 0.0;
            if (g_chute_v > 0.0 && tau > t_ap) { /* bajo paracaídas: velocidad constante, como en reposo */
                double hap = 0.5 * up * g_burn * g_burn + vb * t_ap - 0.5 * 9.80665 * t_ap * t_ap;
                double ah[3], wh[3], hh;
                static_truth(t, ah, wh, &hh);
                memcpy(a, ah, sizeof ah);
                *h = hap - g_chute_v * (tau - t_ap);
            } else {
                *h = 0.5 * up * g_burn * g_burn + vb * tau - 0.5 * 9.80665 * tau * tau;
            }
        }
        if (t >= g_shock_t && t < g_shock_t + g_shock_len) a[0] = 0, a[1] = 0, a[2] = -g_shock_g;
    }
}
static void wiggle_truth(double t, double a[3], double w[3], double *h)
{
    static_truth(t, a, w, h);
    if (t > 9.0 && t < 10.0) w[0] = 40.0 * sin(6.283 * (t - 9.0)), w[1] = 20.0;
}

static void test_nav(void)
{
    printf("10. Alineación en reposo, movimiento, despegue, huecos y recuperación en rampa\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    eskf_svc_output_t o;
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, wiggle_truth);
    sim_run(&m, 8.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.nav_validity == NAV_VALID && o.flight_accept == 0 && (o.svc_reasons & ESKF_SVC_R_EXPERIMENTAL) &&
              (o.svc_reasons & ESKF_SVC_R_NOT_VALIDATED),
          "reposo: estimador válido; experimental, sin validar y NO aceptable para vuelo (readiness 0x%04x)", o.readiness);
    CHECK(o.nav.heading_known == 0, "heading_known = 0: acimut arbitrario");
    sim_run(&m, 11.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.nav_validity != NAV_INVALID, "movimiento manual: sigue utilizable (validez %u)", o.nav_validity);
    /* despegue (3.5 g < 4 g: sin saturación) */
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 13.6);
    eskf_svc_get_output(&S, &o);
    CHECK(o.nav.phase == NAV_PHASE_FLIGHT && o.svc_validity == NAV_VALID && o.bb.ref_frozen,
          "despegue detectado (fase %u, validez %u); referencia de lanzamiento congelada", o.nav.phase, o.svc_validity);
    /* hueco del productor (secuencia consecutiva): lo detecta navigation.c */
    start(&c);
    flight_reset();
    sim_init(&m, 1.0, static_truth);
    m.gap_every = 3.389, m.gap_len = 0.034, m.next_gap = 1.0 + 3.389;
    sim_run(&m, 12.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.seq_lost_imu == 0 && S.nav.cnt.rest_breaks_gap > 0 && S.nav.cnt.align_ok == 0,
          "34 ms cada 3.39 s: sin pérdida de secuencia, pero no alinea (ventana de 5 s partida)");
    /* REGRESIÓN C: pérdida en rampa con la navegación sana; se borra tras una ventana completa
       posterior, sin depender de una realineación (align_ok no cambia) */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0);
    uint32_t al0 = S.nav.cnt.align_ok;
    m.lose_every = 3;
    sim_run(&m, 13.05);
    m.lose_every = 0;
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_validity == NAV_DEGRADED && (o.svc_reasons & ESKF_SVC_R_IMU_LOSS) && o.nav_validity == NAV_VALID,
          "pérdida en rampa: DEGRADADA (el estimador sigue válido)");
    sim_run(&m, 17.9);
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_reasons & ESKF_SVC_R_IMU_LOSS, "no se borra antes de una ventana COMPLETA posterior al fallo");
    sim_run(&m, 24.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!(o.svc_reasons & ESKF_SVC_R_IMU_LOSS) && o.svc_validity == NAV_VALID && S.diag.loss_recovered == 1 &&
              S.nav.cnt.align_ok == al0,
          "recuperada tras la ventana de reposo (align_ok %u → %u: sin realinear)", al0, S.nav.cnt.align_ok);
    /* en vuelo no se borra */
    start(&c);
    g_lift_t = 13.0, g_lift_g = 3.5;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 13.5);
    m.lose_every = 3;
    sim_run(&m, 13.55);
    m.lose_every = 0;
    sim_run(&m, 16.0);
    eskf_svc_get_output(&S, &o);
    CHECK((o.svc_reasons & ESKF_SVC_R_IMU_LOSS) && o.svc_validity == NAV_DEGRADED && S.diag.loss_recovered == 0,
          "pérdida en vuelo: enclavada y DEGRADADA");
    flight_reset();
}

/* ---------------------------------------------------------------- 11. fragmentación */
static void test_fragments(void)
{
    printf("11. Fragmentación y reconstrucción de la salida\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_snapshot_t sn;
    eskf_msg_diag_t dg;
    eskf_msg_ops_t op;
    int got = 0, diags = 0, opsn = 0, maxlen = 0;
    double t = 1.0;
    while (t < 8.0) {
        t += 0.05;
        sim_run(&m, t);
        uint8_t f[50];
        uint16_t l;
        while (eskf_svc_produce(&S, f, &l)) {
            if (l > maxlen) maxlen = l;
            int k = eskf_msg_reasm_push_ops(&r, f, l, &sn, &dg, &op);
            got += k == 1, diags += k == 2, opsn += k == 3;
        }
    }
    g_now_us += 200000; /* un snapshot más con el estado actual, sin datos nuevos */
    eskf_svc_consume(&S);
    {
        uint8_t f[50];
        uint16_t l;
        while (eskf_svc_produce(&S, f, &l)) {
            int k2 = eskf_msg_reasm_push_ops(&r, f, l, &sn, &dg, &op);
            got += k2 == 1, diags += k2 == 2, opsn += k2 == 3;
        }
    }
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(maxlen <= 50 && got >= 34 && r.incomplete == 0 && r.crc_bad == 0 && diags >= 6 && opsn >= 6,
          "%d snapshots, %d diag, %d compactas, máx %d B", got, diags, opsn, maxlen);
    CHECK(sn.session == o.session && sn.config_id == o.config_id && sn.snap_id == S.snap_id, "sesión, id, configuración");
    CHECK(sn.t_state_us == o.t_state_us && memcmp(sn.q, o.nav.q, sizeof sn.q) == 0 && memcmp(sn.p, o.nav.p, sizeof sn.p) == 0 &&
              sn.nav_validity == o.nav_validity && sn.nav_reasons == (uint16_t)o.nav.reasons && sn.readiness == o.readiness,
          "campos reconstruidos idénticos");
    CHECK((sn.flags & ESKF_MSG_F_EXP) && !(sn.flags & ESKF_MSG_F_STRICT) && (sn.flags & ESKF_MSG_F_TIME_MS) &&
              !(sn.flags & ESKF_MSG_F_ACCEPT),
          "banderas: experimental, marcas en ms, no aceptado");
    CHECK(sn.caps == o.caps && sn.source == o.source, "capacidades y fuente en el fragmento de estado");
    CHECK(dg.imu_rx == S.diag.imu_rx || dg.imu_rx + 300 > S.diag.imu_rx, "diagnóstico decodificado");
    /* pérdidas y mezclas */
    uint8_t A[5][50], B[5][50], la[5], lb[5];
    eskf_msg_snapshot_t s1 = sn, s2 = sn;
    s1.mask = s2.mask = 0x1F;
    s2.snap_id = sn.snap_id + 1;
    s2.p[0] = 99.0f;
    eskf_msg_encode_snapshot(&s1, A, la);
    eskf_msg_encode_snapshot(&s2, B, lb);
    eskf_msg_reasm_init(&r);
    for (int i = 0; i < 5; i++)
        if (i != 2) eskf_msg_reasm_push(&r, A[i], la[i], &sn, NULL); /* falta el fragmento 3 */
    int k = 0;
    for (int i = 0; i < 5; i++) k = eskf_msg_reasm_push(&r, B[i], lb[i], &sn, NULL);
    CHECK(r.incomplete == 1 && k == 1 && sn.p[0] == 99.0f, "fragmento perdido: incompleto, el siguiente se reconstruye");
    eskf_msg_reasm_init(&r);
    eskf_msg_reasm_push(&r, A[0], la[0], &sn, NULL);
    eskf_msg_reasm_push(&r, B[1], lb[1], &sn, NULL);
    CHECK(r.incomplete == 1 && r.complete == 0, "fragmentos de snapshots distintos no se mezclan");
    eskf_msg_reasm_init(&r);
    B[1][20] ^= 1;
    for (int i = 0; i < 5; i++) k = eskf_msg_reasm_push(&r, B[i], lb[i], &sn, NULL);
    CHECK(k == -2 && r.crc_bad == 1, "dato alterado: CRC del snapshot");
    /* snapshot pendiente sin vaciar: se omite el siguiente, no se mezclan */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 1.5);
    uint8_t f[50];
    uint16_t l;
    eskf_svc_produce(&S, f, &l);
    eskf_svc_produce(&S, f, &l); /* la compacta y el primer fragmento */
    uint32_t id0 = S.snap_id;
    sim_run(&m, 1.8);
    CHECK(S.diag.snapshots_skipped > 0 && S.snap_id == id0, "sin vaciar: omitidos %u", S.diag.snapshots_skipped);
}

/* ---------------------------------------------------------------- 12. replay frente al mismo servicio */
static nav_t R;
static int g_rep_ok = 1, g_rep_n = 0;
static void replay_hook(void *ctx, int kind, const nav_imu_sample_t *imu, const nav_baro_sample_t *baro, double t,
                        int ret, const nav_t *nav)
{
    (void)ctx;
    int r2 = kind == 0 ? nav_tick(&R, t) : kind == 1 ? nav_on_imu(&R, imu) : nav_on_baro(&R, baro);
    nav_output_t a, b;
    memset(&a, 0, sizeof a); /* nav_get_output no escribe el relleno de la estructura */
    memset(&b, 0, sizeof b);
    nav_get_output(nav, &a);
    nav_get_output(&R, &b);
    g_rep_n++;
    if (r2 != ret || memcmp(&a, &b, sizeof a) != 0) g_rep_ok = 0;
}

static void test_replay(void)
{
    printf("12. Las mismas entradas por un nav_t independiente dan la misma salida\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    nav_init(&R, &c.nav);
    eskf_svc_set_hook(&S, replay_hook, NULL);
    sim_t m;
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 4.5, g_burn = 0.8; /* con saturación y su continuación */
    sim_init(&m, 1.0, flight_truth);
    m.consume_every = 3;
    sim_run(&m, 15.0);
    flight_reset();
    eskf_svc_set_hook(&S, NULL, NULL);
    CHECK(g_rep_ok && g_rep_n > 4000, "%d llamadas idénticas (ticks incluidos), ok=%d", g_rep_n, g_rep_ok);
}

/* ---------------------------------------------------------------- 13. trabajo, memoria y traza */
static void test_bounds(void)
{
    printf("13. Trabajo y memoria acotados; traza de validación\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    c.trace_enable = 1;
    c.boot_id = 77;
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    m.consume_every = 20; /* ráfagas */
    sim_run(&m, 3.0);
    CHECK(S.diag.consume_items_max <= c.max_items && S.diag.mailbox_hwm <= ESKF_SVC_MAILBOX_CAP, "máx %u paquetes/llamada",
          S.diag.consume_items_max);
    printf("  sizeof(eskf_svc_t) = %u bytes (estático; sin memoria dinámica)\n", (unsigned)sizeof(eskf_svc_t));
    CHECK(sizeof(eskf_svc_t) < 40000u, "memoria del servicio");
    eskf_svc_trec_t tr[ESKF_SVC_TRACE_CAP];
    uint32_t n = eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP);
    CHECK(n == ESKF_SVC_TRACE_CAP && S.diag.trace_overflow > 0, "traza llena sin sobrescribir: %u registros, %u perdidos", n,
          S.diag.trace_overflow);
    CHECK(tr[0].type == ESKF_SVC_TR_CONFIG && tr[0].res == ESKF_SVC_TRACE_VERSION && tr[0].a == S.config_id &&
              tr[0].b == (ESKF_SVC_PROFILE_EXP | (ESKF_MSG_VERSION << 8)) && tr[0].c == 77 && tr[0].t == 1000000,
          "primer registro: configuración exacta (id, perfil, esquema, arranque y reloj del inicio)");
    CHECK(tr[1].type == ESKF_SVC_TR_DELIVER && tr[1].a == KI, "después, la entrega de la IMU");
    /* traza vaciada a tiempo: completa */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    uint32_t total = 0, deliv = 0, cons = 0;
    for (double t = 1.0; t < 3.0; t += 0.1) {
        sim_run(&m, t);
        uint32_t k;
        while ((k = eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP)) > 0)
            for (uint32_t i = 0; i < k; i++) total++, deliv += tr[i].type == 1, cons += tr[i].type == 2;
    }
    CHECK(S.diag.trace_overflow == 0 && deliv == S.diag.delivered && cons > 0, "traza completa: %u registros (%u entregas)", total,
          deliv);
    eskf_svc_note_output(&S, ESKF_SVC_OUT_PUBLISHED, ESKF_MSG_T_OPS, 50, 41, 3007);
    eskf_svc_note_output(&S, ESKF_SVC_OUT_DEFER_QUEUE, 0, 0, 0, 0);
    eskf_svc_note_output(&S, ESKF_SVC_OUT_PUB_FAIL, 1, 48, 42, 3008);
    uint32_t k = eskf_svc_trace_read(&S, tr, 3);
    CHECK(k == 3 && tr[0].type == ESKF_SVC_TR_OUTPUT && tr[0].res == 0 && tr[0].a == 7 && tr[0].b == 50 && tr[0].c == 41 &&
              tr[0].t == 3007 && tr[1].res == ESKF_SVC_OUT_DEFER_QUEUE && tr[2].res == ESKF_SVC_OUT_PUB_FAIL && tr[2].c == 42,
          "la traza registra publicado / aplazado / perdido con la cabecera real del paquete");
    CHECK(S.diag.publish_ok == 1 && S.diag.queue_busy == 1 && S.diag.publish_fail == 1, "y los contadores");
    /* cierre verificable (v3): END con el nº de registros y su CRC; después no se registra nada */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 1.5); /* traza llena sin vaciar */
    CHECK(eskf_svc_trace_close(&S) == ESKF_SVC_E_FULL, "cierre con la traza llena: E_FULL (vaciar y repetir)");
    uint16_t crc = 0xFFFFu;
    uint32_t cnt = 0;
    while ((k = eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP)) > 0)
        for (uint32_t i = 0; i < k; i++) crc = eskf_svc_trec_crc(&tr[i], crc), cnt++;
    uint32_t lost = S.diag.trace_overflow;
    CHECK(eskf_svc_trace_close(&S) == ESKF_SVC_OK && eskf_svc_trace_read(&S, tr, 4) == 1 && tr[0].type == ESKF_SVC_TR_END &&
              tr[0].t == (int64_t)cnt && tr[0].a == crc && tr[0].b == lost && lost > 0,
          "END: %u registros, CRC 0x%04x, %u perdidos por desbordamiento (declarados)", cnt, crc, lost);
    sim_run(&m, 1.7);
    CHECK(eskf_svc_trace_read(&S, tr, 4) == 0 && eskf_svc_trace_close(&S) == ESKF_SVC_OK, "tras el cierre no se registra nada");
    printf("  traza: %.0f registros/s × 16 B = %.1f kB/s con IMU 225 Hz + baro 100 Hz\n", total / 2.0, total * 16.0 / 2.0 / 1000.0);
}


/* ================================================================ 31-oct: regresiones y escenarios */

static void deliver_at(uint16_t apid, uint16_t seq, uint32_t ts_ms, const void *pl, uint16_t len, int64_t now_us)
{
    uint8_t p[64];
    mk_packet(p, apid, seq, ts_ms, pl, len);
    g_now_us = now_us;
    eskf_svc_deliver_packet(&S, p);
    eskf_svc_consume(&S);
}

/* ---------------------------------------------------------------- 14. marca futura (A) */
static void test_future_mark(void)
{
    printf("14. REGRESIÓN A: una marca futura no contamina el seguimiento del flujo\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0); /* reloj a 13 s */
    uint8_t pl[48];
    const double a[3] = {0.1, -0.05, -0.99}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    uint32_t rx0 = S.diag.imu_rx;
    int64_t last0 = S.st_imu.last_t_us;
    deliver_at(KI, m.seq_imu++, 20000, pl, 44, (int64_t)13.0e6 + 200); /* IMU marcada a 20 s */
    CHECK(S.diag.ts_future == 1 && S.st_imu.last_t_us == last0 && S.diag.imu_rx == rx0,
          "rechazada ANTES de tocar el flujo (last_t %.3f s)", (double)S.st_imu.last_t_us * 1e-6);
    sim_run(&m, 14.0); /* las 225 siguientes */
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.imu_rx - rx0 == 225 && S.diag.seq_old == 0 && S.diag.ts_back_imu == 0,
          "las 225 IMU siguientes se aceptan (%u)", S.diag.imu_rx - rx0);
    CHECK(S.diag.seq_lost_imu == 1, "y la muestra rechazada cuenta UNA pérdida (salto de secuencia)");
    CHECK((o.svc_reasons & ESKF_SVC_R_LATE_EVENT) && o.svc_validity == NAV_DEGRADED && o.nav_validity == NAV_VALID,
          "DEGRADADA y enclavada hasta una ventana de reposo posterior");
    sim_run(&m, 20.5);
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_reasons & ESKF_SVC_R_LATE_EVENT, "la ventana 11–16 s empezó antes del fallo: no basta");
    sim_run(&m, 22.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!(o.svc_reasons & ESKF_SVC_R_LATE_EVENT) && o.svc_validity == NAV_VALID, "recuperada con la ventana 16–21 s");
    /* baro futuro: tampoco toca el flujo */
    uint8_t pb[12];
    baro12(pb, 0.0);
    int64_t lb = S.st_baro.last_t_us;
    deliver_at(KB, m.seq_baro, 60000, pb, 12, (int64_t)22.0e6 + 300);
    CHECK(S.diag.ts_future == 2 && S.st_baro.last_t_us == lb, "baro futuro: rechazado sin tocar el flujo");
}

/* ---------------------------------------------------------------- 15. antiguos y reinicios (B) */
static void test_old_and_restart(void)
{
    printf("15. REGRESIÓN B: paquetes antiguos, reinicio parcial del productor y reinicio pedido en vuelo\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 13.5);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.nav.phase == NAV_PHASE_FLIGHT, "en vuelo");
    uint8_t pl[48], pb[12];
    const double a[3] = {0, 0, -1}, w[3] = {0, 0, 0};
    imu44(pl, a, w);
    baro12(pb, 0.0);
    uint16_t ses = S.session;
    deliver_at(KI, 0, 1000, pl, 44, (int64_t)13.5e6 + 300); /* secuencia y tiempo atrás */
    deliver_at(KB, 0, 1003, pb, 12, (int64_t)13.5e6 + 400);
    sim_run(&m, 13.8);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses && o.nav.phase == NAV_PHASE_FLIGHT && S.diag.seq_old == 2,
          "antiguos en vuelo: descartados y contados; ni sesión nueva ni vuelta a RAMPA");
    CHECK(o.svc_validity == NAV_VALID && S.nav.cnt.imu_rx > 0, "el flujo sigue (validez %u)", o.svc_validity);
    eskf_svc_request_restart(&S);
    sim_run(&m, 13.9);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses && S.diag.restarts_refused == 1 && (o.svc_reasons & ESKF_SVC_R_RESTART_REFUSED) &&
              o.nav.phase == NAV_PHASE_FLIGHT,
          "reinicio pedido en vuelo: RECHAZADO y marcado");
    /* reinicio parcial del productor en rampa: secuencia a 0, reloj continuo */
    start(&c);
    flight_reset();
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0);
    uint32_t rx0 = S.nav.cnt.imu_rx;
    m.seq_imu = 0; /* el siguiente paquete sale con secuencia 0 y marca posterior */
    sim_run(&m, 13.5);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.seq_resync == 1 && S.session == 1 && S.diag.seq_lost_imu == 0 && S.nav.cnt.imu_rx - rx0 > 100,
          "secuencia reiniciada con tiempo continuo: resincronización, sin pérdidas inventadas ni sesión nueva");
    CHECK((o.svc_reasons & ESKF_SVC_R_SEQ_RESYNC) && o.svc_validity == NAV_DEGRADED, "DEGRADADA (no se sabe si faltó algo)");
    sim_run(&m, 22.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!(o.svc_reasons & ESKF_SVC_R_SEQ_RESYNC) && o.svc_validity == NAV_VALID, "recuperada tras una ventana de reposo");
    /* arranque completo del equipo: init de nuevo (otro boot_id), sesión 1 */
    c.boot_id = 9;
    start(&c);
    eskf_svc_consume(&S);
    eskf_svc_get_output(&S, &o);
    CHECK(o.session == 1 && o.boot_id == 9, "arranque completo: lo distingue boot_id (sesión %u, arranque %u)", o.session,
          o.boot_id);
}

/* ---------------------------------------------------------------- 16. baro aislado, atípico, pérdida */
static void test_baro_faults(void)
{
    printf("16. Baro: medida aislada inválida, atípico, pérdida y caducidad del respaldo\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0);
    eskf_svc_output_t o;
    uint8_t pb[12];
    baro12(pb, 0.0);
    wrf(pb, (float)NAN);
    m.baro_next = 13.015;
    deliver_at(KB, m.seq_baro++, 13005, pb, 12, 13005300);
    uint32_t used0 = S.nav.cnt.baro_used;
    sim_run(&m, 13.5);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.baro_bad_value == 1 && S.nav.cnt.baro_used > used0 + 30 && o.bb.valid && o.svc_validity == NAV_VALID,
          "una medida inválida se descarta; el baro sigue en uso (%u aplicadas después)", S.nav.cnt.baro_used - used0);
    baro12(pb, 30.0); /* atípico finito, dentro de los límites físicos */
    m.baro_next = 13.515;
    deliver_at(KB, m.seq_baro++, 13505, pb, 12, 13505300);
    sim_run(&m, 14.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.bb_rejected == 1 && o.bb.valid && fabsf(o.bb.h_m) < 0.5f && o.svc_validity == NAV_VALID,
          "atípico: el respaldo lo descarta (h %.2f m) y sigue válido", (double)o.bb.h_m);
    m.baro_next = 1e30; /* el baro deja de llegar */
    sim_run(&m, 14.3);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && (o.caps & ESKF_SVC_CAP_BARO_BACKUP), "0.3 s sin baro: respaldo aún válido");
    sim_run(&m, 15.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && !(o.caps & ESKF_SVC_CAP_BARO_BACKUP) && o.bb.age_us > 500000 && o.source == ESKF_SVC_SRC_ESKF,
          "1 s sin baro: el respaldo caduca (edad %.2f s); la fuente sigue siendo el ESKF", (double)o.bb.age_us * 1e-6);
    CHECK(o.svc_validity == NAV_VALID, "el ESKF en rampa no depende del baro");
}

/* ---------------------------------------------------------------- 17. cambio de referencia en vuelo */
static void test_baro_ref_flight(void)
{
    printf("17. Cambio de referencia del baro avisado en vuelo\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    c.trace_enable = 1;
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 13.5);
    uint16_t ses = S.session;
    uint32_t fed = S.nav.cnt.baro_rx;
    eskf_svc_trec_t tr[ESKF_SVC_TRACE_CAP];
    uint32_t n, found = 0;
    while (eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP) > 0) {
    }
    eskf_svc_notify_baro_ref_change(&S);
    sim_run(&m, 13.505);
    while ((n = eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP)) > 0)
        for (uint32_t i = 0; i < n; i++) found += tr[i].type == ESKF_SVC_TR_BAROREF;
    sim_run(&m, 14.0);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses && S.diag.baro_not_fed > 40 && S.nav.cnt.baro_rx <= fed + 2,
          "en vuelo: sin sesión nueva y sin baro para el ESKF (%u retenidos)", S.diag.baro_not_fed);
    CHECK((o.svc_reasons & ESKF_SVC_R_BARO_REF_CHANGED) && o.svc_validity == NAV_DEGRADED && !o.bb.valid &&
              o.source == ESKF_SVC_SRC_ESKF,
          "DEGRADADA, respaldo invalidado; el ESKF sigue publicándose");
    CHECK(found == 1, "aviso registrado en la traza");
    uint32_t used = S.diag.bb_used, nb = S.nav.cnt.baro_rx;
    float ref = o.bb.ref_m;
    sim_run(&m, 20.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.bb_used == used && S.nav.cnt.baro_rx == nb && !o.bb.valid && o.bb.ref_m == ref && S.session == ses,
          "6 s después: ni ESKF ni respaldo usan el baro; sin referencia reconstruida (sesión %u)", S.session);
    CHECK((o.nav.reasons & NAV_R_BARO_TIMEOUT) && o.svc_validity == NAV_DEGRADED && (o.caps & ESKF_SVC_CAP_ESKF_NAV) &&
              o.source == ESKF_SVC_SRC_ESKF,
          "el ESKF continúa DEGRADADO y refleja la falta de correcciones (BARO_TIMEOUT)");
    flight_reset();
}

/* ---------------------------------------------------------------- 26. pérdidas de IMU en vuelo y fuentes */
static void test_imu_gap_flight(void)
{
    printf("26. Pérdida breve de IMU en vuelo (admisible / no admisible) y fuentes publicadas\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 14.5);
    m.lose_every = 2; /* una de cada dos: intervalo real 8,9 ms <= dt_max 10 ms */
    sim_run(&m, 14.6);
    m.lose_every = 0;
    sim_run(&m, 15.0);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_validity == NAV_DEGRADED && (o.svc_reasons & ESKF_SVC_R_IMU_LOSS) && !(o.nav.reasons & NAV_R_TIME_GAP) &&
              o.source == ESKF_SVC_SRC_ESKF,
          "intervalo admisible (8,9 ms): sigue DEGRADADO, fuente ESKF");
    uint32_t al = S.nav.cnt.align_ok;
    m.next_gap = 15.0, m.gap_len = 0.03, m.gap_every = 1e9; /* 30 ms sin IMU: > dt_max */
    sim_run(&m, 15.5);
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_validity == NAV_INVALID && (o.nav.reasons & NAV_R_TIME_GAP) && o.bb.valid && o.source == ESKF_SVC_SRC_BARO &&
              !(o.caps & ESKF_SVC_CAP_ESKF_NAV) && (o.caps & ESKF_SVC_CAP_BARO_BACKUP),
          "hueco de 30 ms: ESKF NO VÁLIDO, fuente BARO (respaldo válido)");
    sim_run(&m, 19.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.svc_validity == NAV_INVALID && S.nav.cnt.align_ok == al && S.session == 1 && o.nav.phase == NAV_PHASE_FLIGHT,
          "en vuelo no se realinea ni se reinicia: sigue NO VÁLIDO (align_ok %u)", S.nav.cnt.align_ok);
    m.baro_next = 1e30; /* además, sin baro */
    sim_run(&m, 20.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.source == ESKF_SVC_SRC_NONE && !(o.caps & (ESKF_SVC_CAP_ESKF_NAV | ESKF_SVC_CAP_BARO_BACKUP)) && !o.bb.valid &&
              o.svc_validity == NAV_INVALID,
          "ninguna fuente válida: fuente NINGUNA, capacidades 0x%02x", o.caps);
    uint8_t f[50];
    uint16_t l;
    while (eskf_svc_produce(&S, f, &l)) {
    }
    g_now_us += 1100000;
    m.imu_next = 1e30;
    eskf_svc_consume(&S);
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_ops_t op;
    int got = 0;
    while (eskf_svc_produce(&S, f, &l))
        if (eskf_msg_reasm_push_ops(&r, f, l, NULL, NULL, &op) == 3) got = 1;
    CHECK(got && op.source == ESKF_SVC_SRC_NONE && op.svc_validity == NAV_INVALID && !op.bb_valid && op.bb_age_ms > 1000,
          "la compacta se sigue publicando con su estado: fuente NINGUNA, respaldo caducado (%u ms)", op.bb_age_ms);
    flight_reset();
}

/* ---------------------------------------------------------------- 18. saturación y continuación */
static void run_burn(double g, double burn, double t_end, eskf_svc_output_t *o_mid, double t_mid, uint32_t *baro_used_mid,
                     eskf_svc_output_t *o_end)
{
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = g, g_burn = burn;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 13.0);
    uint32_t u0 = S.nav.cnt.baro_used;
    sim_run(&m, t_mid);
    eskf_svc_get_output(&S, o_mid);
    if (baro_used_mid) *baro_used_mid = S.nav.cnt.baro_used - u0;
    sim_run(&m, t_end);
    eskf_svc_get_output(&S, o_end);
}

static void test_saturation(void)
{
    printf("18. Saturación ±4 g: continuación experimental, presupuesto, fuera de modelo\n");
    eskf_svc_output_t ref_mid, ref_end, mid, end;
    uint32_t used_ref, used_sat;
    run_burn(3.5, 0.6, 14.0, &ref_mid, 13.5, &used_ref, &ref_end); /* sin saturar */
    run_burn(4.5, 0.6, 14.0, &mid, 13.5, &used_sat, &end);           /* 0.6 s recortado a ±4 g */
    CHECK(S.diag.sat_cand_acc > 100 && (mid.nav.reasons & NAV_R_ACC_SAT) && !(mid.nav.reasons & NAV_R_ACC_SAT_LIMIT),
          "candidatos del dominio de códigos → saturación en navigation.c (%u)", S.diag.sat_cand_acc);
    CHECK(mid.svc_validity == NAV_DEGRADED && mid.nav.phase == NAV_PHASE_FLIGHT, "durante la combustión: DEGRADADA (no válida no)");
    CHECK(mid.nav.sigma_v[2] > 3.0f * ref_mid.nav.sigma_v[2],
          "la incertidumbre de velocidad crece: σv_z %.3f frente a %.3f m/s sin saturar", (double)mid.nav.sigma_v[2],
          (double)ref_mid.nav.sigma_v[2]);
    CHECK(used_sat > 30, "el baro se sigue aplicando durante la saturación (%u; %u sin saturar)", used_sat, used_ref);
    CHECK(end.svc_validity == NAV_DEGRADED && (end.svc_reasons & ESKF_SVC_R_SAT_CANDIDATE) &&
              !(end.svc_reasons & ESKF_SVC_R_SAT_OUT_OF_MODEL),
          "tras la combustión: sigue DEGRADADA (incidente enclavado en el historial)");
    double h_true = 0.5 * 3.5 * 9.80665 * 0.36 + 3.5 * 9.80665 * 0.6 * 0.4 - 0.5 * 9.80665 * 0.16;
    CHECK(fabs(-(double)end.nav.p[2] - h_true) < 3.0, "altura ESKF %.2f m frente a %.2f m reales a 14 s",
          -(double)end.nav.p[2], h_true);
    /* presupuesto superado: 1.4 s saturado > acc_sat_max_s 1.2 s → NO VÁLIDA */
    run_burn(4.5, 1.4, 14.6, &mid, 14.0, NULL, &end);
    CHECK((end.nav.reasons & NAV_R_ACC_SAT_LIMIT) && end.svc_validity == NAV_INVALID && !(end.caps & ESKF_SVC_CAP_ESKF_NAV),
          "saturación más larga que el modelo: NO VÁLIDA");
    CHECK(end.bb.valid && end.source == ESKF_SVC_SRC_BARO && end.bb.vz_mps > 20.0f,
          "el respaldo barométrico sigue: fuente BARO, v_z %.1f m/s (hacia arriba)", (double)end.bb.vz_mps);
    /* despliegue: saturación fuera de la ventana del motor → fuera de modelo */
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 0.6, g_shock_t = 15.0, g_shock_g = 6.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 14.9);
    eskf_svc_get_output(&S, &mid);
    sim_run(&m, 15.3);
    eskf_svc_get_output(&S, &end);
    CHECK(mid.svc_validity == NAV_VALID && (end.svc_reasons & ESKF_SVC_R_SAT_OUT_OF_MODEL) &&
              end.svc_validity == NAV_INVALID && S.diag.sat_out_of_model == 1,
          "golpe de 6 g a t_despegue + 2 s (ventana 1.5 s): FUERA DE MODELO → NO VÁLIDA");
    CHECK(end.bb.valid && end.source == ESKF_SVC_SRC_BARO, "el respaldo sigue disponible");
    flight_reset();
}

/* ---------------------------------------------------------------- 19. respaldo barométrico */
static void test_backup(void)
{
    printf("19. Respaldo barométrico independiente: signo, referencia congelada, sin ESKF\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_ACQ); /* sin navegación: el respaldo funciona igual */
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 12.9);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && !o.bb.ref_frozen && fabsf(o.bb.h_m) < 0.2f && fabsf(o.bb.vz_mps) < 0.5f && o.source == ESKF_SVC_SRC_BARO,
          "rampa: h %.3f m, v %.3f m/s; fuente BARO (ACQ)", (double)o.bb.h_m, (double)o.bb.vz_mps);
    sim_run(&m, 14.5);
    eskf_svc_get_output(&S, &o);
    double tau = 0.5, vb = 2.5 * 9.80665;
    double h_true = 0.5 * vb * 1.0 + vb * tau - 0.5 * 9.80665 * tau * tau, v_true = vb - 9.80665 * tau;
    CHECK(o.bb.ref_frozen && (o.caps & ESKF_SVC_CAP_LAUNCH_REF), "sin ESKF: referencia congelada al subir > 10 m");
    CHECK(fabs((double)o.bb.h_m - h_true) < 1.0 && fabs((double)o.bb.vz_mps - v_true) < 3.0,
          "h %.2f m (real %.2f), v_z %.2f m/s (real %.2f, positiva hacia arriba)", (double)o.bb.h_m, h_true,
          (double)o.bb.vz_mps, v_true);
    CHECK(o.svc_validity == NAV_INVALID && !(o.caps & ESKF_SVC_CAP_ESKF_NAV) && isnan(o.nav.p[2]),
          "sin ESKF: su estado se publica NO VÁLIDO, sin posición inventada");
    flight_reset();
}

/* ---------------------------------------------------------------- 20. salida operativa compacta */
static void test_ops(void)
{
    printf("20. Salida operativa compacta (≤ 50 B, autónoma)\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    c.boot_id = 3;
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 14.2);
    uint8_t f[50];
    uint16_t l;
    while (eskf_svc_produce(&S, f, &l)) {
    }
    g_now_us += 1100000;
    m.imu_next = m.baro_next = 1e30;
    eskf_svc_consume(&S);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(eskf_svc_produce(&S, f, &l) == 1 && f[1] == ESKF_MSG_T_OPS && l == 50, "la compacta sale primero (%u B)", l);
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_ops_t op;
    int k = eskf_msg_reasm_push_ops(&r, f, l, NULL, NULL, &op);
    CHECK(k == 3 && op.session == o.session && op.boot_id == 3 && op.config_id == o.config_id && op.svc_validity == o.svc_validity &&
              op.caps == o.caps && op.source == o.source && op.phase == NAV_PHASE_FLIGHT && op.profile == ESKF_SVC_PROFILE_EXP,
          "decodificada sin otros mensajes: sesión, arranque, configuración, validez, capacidades, fuente");
    CHECK(op.eskf_h_m == -o.nav.p[2] && op.eskf_vz_mps == -o.nav.v[2] && op.bb_h_m == o.bb.h_m && op.bb_vz_mps == o.bb.vz_mps,
          "alturas y velocidades (positivas hacia arriba): ESKF %.2f m %.2f m/s; respaldo %.2f m %.2f m/s",
          (double)op.eskf_h_m, (double)op.eskf_vz_mps, (double)op.bb_h_m, (double)op.bb_vz_mps);
    CHECK(op.t_ms == (uint32_t)(g_now_us / 1000) && op.bb_age_ms >= 1100 && op.eskf_age_ms >= 1000 && op.bb_valid == 0,
          "tiempo y edades reales: el respaldo caduca (edad %u ms) y lo dice", op.bb_age_ms);
    CHECK((op.flags & ESKF_MSG_F_EXP) && op.svc_reasons == (uint16_t)o.svc_reasons, "banderas y motivos");
    /* preajuste de telemetría: sólo la compacta y un diagnóstico lento */
    eskf_svc_output_preset(&c, ESKF_OUT_TELEMETRY);
    start(&c);
    sim_init(&m, 1.0, static_truth);
    int types[8] = {0};
    for (int j = 1; j <= 49; j++) { /* 1.0 → 5.9 s */
        sim_run(&m, 1.0 + 0.1 * j);
        while (eskf_svc_produce(&S, f, &l)) types[f[1] & 7]++;
    }
    CHECK(types[7] == 5 && types[1] == 0 && types[6] == 1, "TELEMETRÍA: %d compactas, %d estados, %d diagnósticos en 5 s",
          types[7], types[1], types[6]);
    CHECK(!(eskf_svc_readiness_cfg(&c) & ESKF_SVC_RDY_OVERRIDE), "cambiar la salida no se confunde con una procedencia externa");
    flight_reset();
}

/* ---------------------------------------------------------------- 21. consultas desde otra tarea */
static volatile int g_stop = 0;
static long g_reads = 0, g_torn = 0;
static void *reader(void *arg)
{
    (void)arg;
    eskf_svc_output_t o;
    eskf_svc_diag_t d;
    while (!g_stop) {
        eskf_svc_get_output(&S, &o);
        int64_t ts = isfinite(o.nav.t_state) ? (int64_t)llround(o.nav.t_state * 1e6) : INT64_MIN;
        if (ts != o.t_state_us || o.session != S.session) g_torn++;
        eskf_svc_get_diag(&S, &d);
        if (d.imu_rx < d.imu_bad_len || d.seq_lost_imu > d.imu_rx + 1000u) g_torn++;
        g_reads++;
    }
    return NULL;
}

static void test_queries_threads(void)
{
    printf("21. Consultas concurrentes: copia completa bajo sección crítica (sin datos a medias)\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    pthread_t th;
    g_stop = 0;
    pthread_create(&th, NULL, reader, NULL);
    sim_run(&m, 9.0);
    g_stop = 1;
    pthread_join(th, NULL);
    CHECK(g_reads > 100 && g_torn == 0, "%ld lecturas concurrentes, %ld incoherentes", g_reads, g_torn);
}

/* ---------------------------------------------------------------- 22. validación de configuración */
static void test_config(void)
{
    printf("22. Validación de la configuración y preparación ligada a la configuración exacta\n");
    eskf_svc_config_t c0 = cfg_of(ESKF_SVC_PROFILE_EXP), c;
    CHECK(eskf_svc_check_transport(&c0) == NULL && eskf_svc_check_nav(&c0, NULL) == NULL, "EXPERIMENTAL válida y navega");
    c = c0, c.profile = 7;
    CHECK(eskf_svc_check_transport(&c) != NULL, "perfil no admitido");
    c = c0, c.acc_off[1] = (float)NAN;
    CHECK(eskf_svc_check_transport(&c) != NULL, "offset no finito");
    c = c0, c.acc_div = (float)INFINITY;
    CHECK(eskf_svc_check_transport(&c) != NULL, "divisor infinito");
    c = c0, c.baro_p_min_pa = 130000.0f;
    CHECK(eskf_svc_check_transport(&c) != NULL, "límites del baro incoherentes");
    c = c0, c.bb_beta = 0.5f;
    CHECK(eskf_svc_check_transport(&c) != NULL, "respaldo: β > α");
    c = c0, c.bb_gate_m = (float)NAN;
    CHECK(eskf_svc_check_transport(&c) != NULL, "respaldo: puerta NaN");
    c = c0, c.snapshot_mask = 0x1E;
    CHECK(eskf_svc_check_transport(&c) != NULL, "máscara sin el fragmento de estado");
    c = c0, c.ops_period_us = 50000;
    CHECK(eskf_svc_check_transport(&c) != NULL, "compacta más rápida que 10 Hz");
    c = c0, c.future_tol_us = -1;
    CHECK(eskf_svc_check_transport(&c) != NULL, "tolerancia de futuro negativa");
    uint16_t code = 0;
    c = c0, c.nav.gyro_sat_max_s = 0.3;
    CHECK(eskf_svc_check_nav(&c, &code) != NULL && code == 0x8005u, "sin continuación del giróscopo");
    c = c0, c.nav.eskf.gap_max = (eskf_real)0.05;
    CHECK(eskf_svc_check_nav(&c, &code) != NULL && code == 0x8005u, "puente de huecos deshabilitado");
    c = c0, c.sat_motor_window_s = 0.0f;
    CHECK(eskf_svc_check_nav(&c, &code) != NULL && code == 0x8007u, "continuación sin ventana del motor");
    c = c0, c.nav.eskf.sigma_a = (eskf_real)NAN;
    CHECK(eskf_svc_check_nav(&c, &code) != NULL && code > 0 && code < 0x8000u, "valor no finito en la tabla: why_code = fila");
    /* preparación ligada a la configuración */
    uint16_t r0 = eskf_svc_readiness_cfg(&c0);
    CHECK((r0 & ESKF_SVC_RDY_PROFILE) && (r0 & ESKF_SVC_RDY_BOARD) && !(r0 & ESKF_SVC_RDY_OVERRIDE),
          "EXPERIMENTAL: perfil y placa sin validar (0x%04x)", r0);
    c = c0, c.nav.eskf.sigma_a = (eskf_real)0.2;
    CHECK((eskf_svc_readiness_cfg(&c) & ESKF_SVC_RDY_OVERRIDE) && eskf_svc_config_id(&c) != eskf_svc_config_id(&c0),
          "valor de aviónica: OVERRIDE y otro config_id");
    eskf_svc_config_t st = cfg_of(ESKF_SVC_PROFILE_STRICT);
    CHECK(!(eskf_svc_readiness_cfg(&st) & ESKF_SVC_RDY_PROFILE) && (eskf_svc_readiness_cfg(&st) & ESKF_SVC_RDY_BOARD),
          "ESTRICTO: sin bit de perfil; placa sin validar");
    start(&c0);
    eskf_svc_consume(&S);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.readiness == r0 && o.config_id == eskf_svc_config_id(&c0) && !o.flight_accept, "publicadas con la salida");
    /* el ESTRICTO con TODOS los valores confirmados tampoco acepta sin validación en placa */
    st = c0;
    st.profile = ESKF_SVC_PROFILE_STRICT;
    start(&st);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.nav_enabled && o.svc_validity == NAV_VALID && !o.flight_accept && (o.readiness & ESKF_SVC_RDY_BOARD) &&
              (o.readiness & ESKF_SVC_RDY_OVERRIDE),
          "ESTRICTO con valores externos: navega, pero flight_accept = 0 (readiness 0x%04x)", o.readiness);
}

/* ================================================================ 31-oct (tarde): cierre del servicio */

/* baro con una altura publicada dada (presión y temperatura de rampa) */
static void baro_alt(uint16_t seq, double t, float alt, int consume)
{
    uint8_t pl[12], p[64];
    baro12(pl, 0.0);
    wrf(pl, alt);
    mk_packet(p, KB, seq, (uint32_t)llround(t * 1000.0), pl, 12);
    g_now_us = (int64_t)llround(t * 1e6) + 200;
    eskf_svc_deliver_packet(&S, p);
    if (consume) eskf_svc_consume(&S);
}

/* ---------------------------------------------------------------- 23. frescura del respaldo */
static void test_bb_freshness(void)
{
    printf("23. REGRESIÓN: los rechazos no renuevan la frescura del respaldo barométrico\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_ACQ);
    start(&c);
    uint16_t q = 0;
    /* 151 aceptadas, constantes, 100 Hz (ventana de 20 + 1 s de reposo para el cero de rampa) */
    for (int k = 0; k <= 150; k++) baro_alt(q++, 1.0 + 0.01 * k, 0.0f, 1);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.t_us == 2500000, "151 aceptadas: válido, última observación a 2,500 s");
    for (int k = 151; k <= 169; k++) baro_alt(q++, 1.0 + 0.01 * k, 100.0f, 1); /* 19 rechazadas (+100 m) */
    CHECK(S.diag.bb_rejected == 19 || S.diag.bb_rejected + S.diag.bb_gaps > 0, "las 19 se rechazan (%u)", S.diag.bb_rejected);
    g_now_us = 3100000; /* 600 ms después de la última ACEPTADA */
    eskf_svc_consume(&S);
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.t_us == 2500000 && o.bb.age_us == 600000,
          "a 600 ms de la última aceptada: NO válido, antigüedad %.0f ms (antes: válido con 410 ms)", (double)o.bb.age_us * 1e-3);
    CHECK(!(o.caps & ESKF_SVC_CAP_BARO_BACKUP) && o.source == ESKF_SVC_SRC_NONE, "sin capacidad de respaldo ni fuente");
    /* rechazos continuados: nunca se mantiene válida una predicción sin medidas */
    start(&c);
    q = 0;
    for (int k = 0; k <= 150; k++) baro_alt(q++, 1.0 + 0.01 * k, 0.0f, 1);
    int valid_late = 0;
    for (int k = 151; k <= 350; k++) {
        baro_alt(q++, 1.0 + 0.01 * k, 100.0f + (float)(k % 7), 1); /* incoherentes: no forman ventana */
        eskf_svc_get_output(&S, &o);
        if (o.bb.valid && (double)o.bb.age_us > (double)c.bb_timeout_s * 1e6) valid_late++;
    }
    CHECK(valid_late == 0 && !o.bb.valid, "2 s de rechazos/incoherencias: ninguna salida válida con antigüedad > caducidad");
}

/* ---------------------------------------------------------------- 24. referencia inicial robusta y cero en vuelo */
static void test_bb_initial_ref(void)
{
    printf("24. REGRESIÓN: la primera lectura anómala no fija la referencia; el cero de lanzamiento se conserva\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_ACQ);
    start(&c);
    uint16_t q = 0;
    baro_alt(q++, 1.0, 100.0f, 1); /* primera: 100 m */
    for (int k = 1; k <= 150; k++) baro_alt(q++, 1.0 + 0.01 * k, 0.0f, 1);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && fabsf(o.bb.h_m) < 0.2f && fabsf(o.bb.ref_m) < 0.2f,
          "respaldo válido con h %.2f m y referencia %.2f m (antes: −85,2 m)", (double)o.bb.h_m, (double)o.bb.ref_m);
    /* la ventana no declara válido antes de tener medidas coherentes */
    start(&c);
    q = 0;
    int early = 0;
    for (int k = 0; k < 19; k++) {
        baro_alt(q++, 1.0 + 0.01 * k, 0.0f, 1);
        eskf_svc_get_output(&S, &o);
        early += o.bb.valid;
    }
    CHECK(early == 0, "menos de %u muestras coherentes: no válido", c.bb_min_samples);
    /* en vuelo: corte y ráfaga de rechazos; la recuperación no cambia el cero */
    eskf_svc_config_t e = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&e);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 14.5);
    eskf_svc_get_output(&S, &o);
    float ref0 = o.bb.ref_m;
    CHECK(o.bb.ref_frozen && o.bb.valid, "en vuelo, referencia congelada (%.3f m)", (double)ref0);
    m.baro_next = 15.1; /* corte de 0,6 s */
    sim_run(&m, 14.9);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.age_us > 300000, "corte de 0,4 s: aún válido, con su antigüedad real (%.0f ms <= 500)",
          (double)o.bb.age_us * 1e-3);
    sim_run(&m, 15.09);
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.age_us > 500000, "corte de 0,6 s: caducado (antigüedad %.0f ms)", (double)o.bb.age_us * 1e-3);
    sim_run(&m, 15.2);
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER, "vuelven las medidas: recuperando (menos de %u coherentes), NO válido",
          e.bb_min_samples);
    sim_run(&m, 15.4);
    eskf_svc_get_output(&S, &o);
    double tb = 15.4 - 13.0, up = 2.5 * 9.80665, h_true = 0.5 * up + up * (tb - 1.0) - 0.5 * 9.80665 * (tb - 1.0) * (tb - 1.0);
    CHECK(o.bb.valid && o.bb.ref_m == ref0 && fabs((double)o.bb.h_m - h_true) < 2.0,
          "recuperado con ventana coherente: mismo cero (%.3f m), h %.1f m (real %.1f)", (double)o.bb.ref_m, (double)o.bb.h_m, h_true);
    uint32_t rec0 = S.diag.bb_recoveries;
    m.baro_off = 60.0; /* ráfaga breve (0,1 s): rechazadas; el estado no avanza */
    sim_run(&m, 15.5);
    m.baro_off = 0.0;
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK && o.bb.age_us >= 90000,
          "ráfaga de 0,1 s: sigue con su última aceptada (antigüedad %.0f ms)", (double)o.bb.age_us * 1e-3);
    sim_run(&m, 15.7); /* medidas normales: sigue */
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.age_us < 20000, "tras la ráfaga breve, medidas aceptadas sin recuperación");
    m.baro_off = 60.0; /* ráfaga de 0,3 s: 20 rechazos seguidos → recuperación; las 10 restantes no forman ventana */
    sim_run(&m, 16.0);
    m.baro_off = 0.0;
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER, "ráfaga de 0,3 s: recuperando, NO válido");
    sim_run(&m, 16.5);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.ref_m == ref0 && S.diag.bb_recoveries > rec0, "recuperado de nuevo, cero intacto (%u recuperaciones)",
          S.diag.bb_recoveries);
    flight_reset();
}

/* ---------------------------------------------------------------- 25. frontera entre sesiones */
static void test_session_boundary(void)
{
    printf("25. REGRESIÓN: las muestras anteriores a una sesión no la inicializan\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    c.trace_enable = 1;
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 13.0);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.svc_validity == NAV_VALID, "rampa: respaldo estable y ESKF válido");
    /* 21 baro antiguos (referencia vieja) pendientes en el buzón, sin consumir */
    for (int k = 0; k < 21; k++) {
        uint8_t pl[12], p[64];
        baro12(pl, 0.0);
        double t = 13.0 + 0.01 * k;
        mk_packet(p, KB, m.seq_baro++, (uint32_t)llround(t * 1000.0), pl, 12);
        g_now_us = (int64_t)llround(t * 1e6) + 200;
        eskf_svc_deliver_packet(&S, p);
    }
    CHECK(S.mb_head - S.mb_tail == 21, "21 muestras pendientes en el buzón");
    m.baro_next = 13.21;
    m.imu_next = 13.21;
    uint16_t ses = S.session;
    eskf_svc_notify_baro_ref_change(&S);
    m.baro_off = 100.0; /* el productor publica +100 m con la misma presión física */
    sim_run(&m, 13.5);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses + 1 && S.diag.session_discarded >= 21 && S.diag.seq_lost_baro == 0 && S.diag.seq_lost_imu == 0,
          "sesión nueva; %u muestras anteriores descartadas sin pérdidas ficticias", S.diag.session_discarded);
    CHECK(!o.bb.ref_frozen && o.nav.phase == NAV_PHASE_PAD, "sin falso despegue ni referencia congelada");
    sim_run(&m, 26.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && fabsf(o.bb.h_m) < 0.5f && fabsf(o.bb.ref_m - 100.0f) < 0.5f && !o.bb.ref_frozen,
          "respaldo con referencia nueva %.2f m y h %.2f m (antes: 96,3 m y despegue falso)", (double)o.bb.ref_m,
          (double)o.bb.h_m);
    CHECK(o.svc_validity == NAV_VALID && fabs(o.nav.alt_ref - 100.0) < 0.5, "ESKF realineado con alt_ref %.2f m", o.nav.alt_ref);
    eskf_svc_request_restart(&S);
    sim_run(&m, 26.1);
    CHECK(S.session == ses + 2 && S.diag.restarts_refused == 0, "un reinicio posterior en rampa se acepta");
    /* cambios repetidos con el buzón ocupado */
    for (int r = 0; r < 2; r++) {
        for (int k = 0; k < 10; k++) {
            uint8_t pl[12], p[64];
            baro12(pl, 0.0);
            wrf(pl, rdf(pl) + (float)m.baro_off);
            double t = m.baro_next;
            m.baro_next += 0.01;
            mk_packet(p, KB, m.seq_baro++, (uint32_t)llround(t * 1000.0), pl, 12);
            g_now_us = (int64_t)llround(t * 1e6) + 200;
            eskf_svc_deliver_packet(&S, p);
        }
        m.imu_next = m.baro_next;
        eskf_svc_notify_baro_ref_change(&S);
        m.baro_off += 20.0;
        sim_run(&m, m.baro_next + 0.3);
    }
    sim_run(&m, 40.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses + 4 && S.diag.seq_lost_baro == 0 && S.diag.seq_lost_imu == 0 && o.bb.valid &&
              fabsf(o.bb.ref_m - 140.0f) < 0.5f && fabsf(o.bb.h_m) < 0.5f && !o.bb.ref_frozen,
          "dos cambios seguidos con buzón ocupado: sesión %u, referencia %.2f m, h %.2f m, sin pérdidas", S.session,
          (double)o.bb.ref_m, (double)o.bb.h_m);
    eskf_svc_trec_t tr[ESKF_SVC_TRACE_CAP];
    while (eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP) > 0) {
    }
}

/* ---------------------------------------------------------------- 27. vuelo enclavado */
static double true_h(double t)
{
    double a[3], w[3], h;
    flight_truth(t, a, w, &h);
    return h;
}

static float g_ref16;
static uint32_t g_zupt0; /* referencia del respaldo a 16 s (ya en vuelo) */
static void run_imu_back_in_descent(int aligned_before, int baro_start_late, eskf_svc_output_t *o, uint32_t *al_before)
{
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.5, g_chute_v = 5.0; /* apogeo ≈ 96 m a ≈ 18,3 s */
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    if (baro_start_late) m.baro_start = 15.0; /* primer baro ya en vuelo */
    if (!aligned_before) m.imu_next = 18.95;  /* sin IMU en rampa: nunca alineado */
    m.next_gap = 12.95, m.gap_len = 6.0, m.gap_every = 1e9; /* sin IMU de 12,95 a 18,95 s (despegue y ascenso) */
    sim_run(&m, 12.9);
    *al_before = S.nav.cnt.align_ok;
    sim_run(&m, 16.0); /* vuelo ya reconocido (sin IMU desde 12,95 s) */
    eskf_svc_get_output(&S, o);
    g_ref16 = o->bb.ref_m;
    g_zupt0 = S.nav.cnt.zupt_used;
    sim_run(&m, 30.0); /* la IMU vuelve a 18,95 s, en descenso uniforme a 5 m/s */
    eskf_svc_get_output(&S, o);
}

static void test_flight_latch(void)
{
    printf("27. REGRESIÓN: tras reconocer el vuelo no hay alineación, ZUPT, referencia nueva ni reinicio\n");
    eskf_svc_output_t o;
    uint32_t al0;
    run_imu_back_in_descent(1, 0, &o, &al0);
    double ht = true_h(30.0);
    CHECK(al0 == 1 && S.nav.cnt.align_ok == al0, "alineado en rampa; sin IMU en el ascenso y vuelta en el descenso: align_ok %u → %u",
          al0, S.nav.cnt.align_ok);
    CHECK(S.flight && o.nav.phase == NAV_PHASE_FLIGHT && o.svc_validity == NAV_INVALID && !(o.caps & ESKF_SVC_CAP_ESKF_NAV) &&
              (o.svc_reasons & ESKF_SVC_R_NAV_DISABLED),
          "vuelo enclavado; fase publicada VUELO; ESKF NO VÁLIDO y fuera de servicio (motivos 0x%04x)", o.svc_reasons);
    CHECK(o.nav_validity == NAV_INVALID && !o.nav_enabled && o.why_code == ESKF_SVC_WHY_FLIGHT_NAV_PAD,
          "núcleo publicado NO VÁLIDO y congelado; why 0x%04x", o.why_code);
    CHECK(o.source == ESKF_SVC_SRC_BARO && o.bb.valid && o.bb.ref_frozen && fabs((double)o.bb.h_m - ht) < 2.0,
          "fuente BARO: %.1f m (real %.1f; antes el ESKF publicaba ~0 m y VÁLIDO)", (double)o.bb.h_m, ht);
    CHECK(o.bb.ref_m == g_ref16 && S.diag.flight_detector == 2 && S.nav.cnt.zupt_used == g_zupt0,
          "cero de rampa intacto bit a bit (%.3f m); vuelo reconocido por altura del respaldo (detector %u); ZUPT %u → %u",
          (double)o.bb.ref_m, S.diag.flight_detector, g_zupt0, S.nav.cnt.zupt_used);
    CHECK(S.diag.nav_cut_samples > 1000, "las muestras del descenso no llegan a navigation.c (%u retenidas)", S.diag.nav_cut_samples);
    uint16_t ses = S.session;
    eskf_svc_request_restart(&S);
    eskf_svc_consume(&S);
    CHECK(S.session == ses && S.diag.restarts_refused == 1, "reinicio rechazado");
    run_imu_back_in_descent(0, 0, &o, &al0);
    CHECK(al0 == 0 && S.nav.cnt.align_ok == 0 && o.svc_validity == NAV_INVALID && o.source == ESKF_SVC_SRC_BARO &&
              o.nav.phase == NAV_PHASE_FLIGHT,
          "nunca alineado en rampa: tampoco hay alineación inicial en el descenso (align_ok %u)", S.nav.cnt.align_ok);
    CHECK(o.bb.valid && fabs((double)o.bb.h_m - ht) < 2.0, "el respaldo no se sustituye (%.1f m)", (double)o.bb.h_m);
    /* IMU perdida en el despegue Y primer baro ya en vuelo: lo reconoce la velocidad vertical del respaldo */
    run_imu_back_in_descent(1, 1, &o, &al0);
    CHECK(al0 == 1 && S.nav.cnt.align_ok == 1 && S.flight && o.nav.phase == NAV_PHASE_FLIGHT && o.svc_validity == NAV_INVALID,
          "IMU perdida en el despegue y baro tardío: vuelo reconocido por |v_z| del respaldo; sin realinear (align_ok %u)",
          S.nav.cnt.align_ok);
    CHECK(!o.bb.valid && isnan(o.bb.ref_m) && o.source == ESKF_SVC_SRC_NONE && isfinite(o.bb.vz_mps) &&
              S.diag.flight_detector == 3,
          "sin referencia de rampa: respaldo NO válido, sin cero inventado; fuente NINGUNA; v_z %.1f m/s sigue publicándose",
          (double)o.bb.vz_mps);
    /* nada en rampa: ni alineación ni baro; todo empieza en vuelo */
    run_imu_back_in_descent(0, 1, &o, &al0);
    CHECK(S.nav.cnt.align_ok == 0 && S.flight && o.nav.phase == NAV_PHASE_FLIGHT && o.svc_validity == NAV_INVALID &&
              !o.bb.valid && isnan(o.bb.ref_m) && o.source == ESKF_SVC_SRC_NONE && S.diag.bb_used > 1000,
          "sin IMU ni baro en rampa: vuelo reconocido, ni alineación ni cero; medidas registradas (%u aceptadas)",
          S.diag.bb_used);
    flight_reset();
}

/* ---------------------------------------------------------------- 28. primer baro en vuelo */
static void test_late_first_baro(void)
{
    printf("28. REGRESIÓN: el cero de lanzamiento no se fija con el primer baro recibido en vuelo\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.5;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    m.baro_start = 16.0; /* IMU desde la rampa; ningún baro hasta 16 s */
    sim_run(&m, 15.9);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(o.nav.phase == NAV_PHASE_FLIGHT && S.flight, "despegue detectado por navigation.c");
    sim_run(&m, 17.0);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.state == ESKF_SVC_BB_TRACK && S.diag.bb_used > 50 && isfinite(o.bb.vz_mps),
          "el respaldo sigue las medidas (%u aceptadas, v_z %.1f m/s)", S.diag.bb_used, (double)o.bb.vz_mps);
    CHECK(!o.bb.valid && isnan(o.bb.ref_m) && isnan(o.bb.h_m) && !o.bb.ref_frozen && o.source != ESKF_SVC_SRC_BARO,
          "sin referencia de rampa: altura sobre el lanzamiento desconocida, NO válido (antes: 0,44 m válido con cero en 42,5 m)");
    uint32_t rec0 = S.diag.bb_recoveries;
    sim_run(&m, 17.5);
    m.baro_next = 18.1; /* corte de 0,6 s y recuperación con ventana coherente, todavía en vuelo */
    sim_run(&m, 19.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.diag.bb_recoveries > rec0 && o.bb.state == ESKF_SVC_BB_TRACK && !o.bb.valid && isnan(o.bb.ref_m),
          "una recuperación en vuelo tampoco fija un cero (%u recuperaciones)", S.diag.bb_recoveries);
    /* primer baro justo en el apogeo (|v| pequeña durante la ventana) con la IMU perdida en el despegue */
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.5, g_chute_v = 5.0; /* apogeo ≈ 96,6 m a ≈ 18,25 s */
    sim_init(&m, 1.0, flight_truth);
    m.baro_start = 18.15;
    m.next_gap = 12.95, m.gap_len = 100.0, m.gap_every = 1e9;
    sim_run(&m, 18.6);
    eskf_svc_get_output(&S, &o);
    CHECK(S.nav.cnt.align_ok == 1 && o.bb.state == ESKF_SVC_BB_TRACK && isnan(o.bb.ref_m) && !o.bb.valid,
          "primer baro en el apogeo: sigue las medidas sin fijar el cero (el reposo no dura %.1f s)", (double)c.bb_hold_s);
    sim_run(&m, 21.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.flight && S.diag.flight_detector == 3 && o.nav.phase == NAV_PHASE_FLIGHT && isnan(o.bb.ref_m) && !o.bb.valid &&
              o.source == ESKF_SVC_SRC_NONE && S.nav.cnt.align_ok == 1,
          "vuelo reconocido en el descenso por |v_z| = %.1f m/s; sin cero ni fuente; sin realinear", (double)o.bb.vz_mps);
    flight_reset();
}

/* ---------------------------------------------------------------- 29. fase incierta */
/* Última compacta y último snapshot publicados (para comprobar la fase codificada en el mensaje) */
static void last_published(eskf_msg_snapshot_t *sn, eskf_msg_ops_t *op, int *got_sn, int *got_op)
{
    uint8_t f[50];
    uint16_t l;
    eskf_msg_reasm_t r;
    eskf_msg_reasm_init(&r);
    eskf_msg_diag_t dg;
    *got_sn = *got_op = 0;
    while (eskf_svc_produce(&S, f, &l)) { /* lo pendiente de antes */
    }
    g_now_us += 1100000; /* toca snapshot, diagnóstico y compacta */
    eskf_svc_consume(&S);
    while (eskf_svc_produce(&S, f, &l)) {
        int k = eskf_msg_reasm_push_ops(&r, f, l, sn, &dg, op);
        if (k == 1) *got_sn = 1;
        if (k == 3) *got_op = 1;
    }
}

/* Despegue a 13 s (apogeo ≈ 150 m a ≈ 19,5 s) y descenso uniforme a v_chute; IMU perdida desde el
   despegue y de vuelta a 20,5 s; primer baro a 20 s, ya en el descenso. */
static void run_slow_descent(double v_chute, int aligned_before, eskf_svc_output_t *o, uint32_t *al0, uint32_t *z0)
{
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.87, g_chute_v = v_chute;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    m.baro_start = 20.0;
    if (!aligned_before) m.imu_next = 20.5; /* tampoco hubo IMU en rampa: nunca alineado */
    m.next_gap = 12.95, m.gap_len = 20.5 - 12.95, m.gap_every = 1e9;
    sim_run(&m, 13.0);
    *al0 = S.nav.cnt.align_ok;
    *z0 = S.nav.cnt.zupt_used;
    sim_run(&m, 31.5); /* 12 s de descenso */
    eskf_svc_get_output(&S, o);
}

static void test_phase_unknown(void)
{
    printf("29. REGRESIÓN: fase incierta; un descenso lento no se convierte en rampa (alineación, ZUPT, referencias)\n");
    const double vs[2] = {2.0, 0.8};
    for (int iv = 0; iv < 2; iv++)
        for (int al = 1; al >= 0; al--) {
            eskf_svc_output_t o;
            uint32_t al0, z0;
            run_slow_descent(vs[iv], al, &o, &al0, &z0);
            double ht = true_h(31.5);
            CHECK(al0 == (uint32_t)al && S.nav.cnt.align_ok == al0 && S.nav.cnt.zupt_used == z0 && S.nav.cnt.ref_updates == 0 &&
                      isnan(o.nav.alt_ref),
                  "descenso a %.1f m/s, %s: align_ok %u → %u, ZUPT %u → %u, sin referencia de navigation.c", vs[iv],
                  al ? "alineado en rampa" : "nunca alineado", al0, S.nav.cnt.align_ok, z0, S.nav.cnt.zupt_used);
            CHECK(!S.pad && !S.flight && o.phase == ESKF_SVC_PHASE_UNKNOWN && o.nav.at_rest == 0 && o.svc_validity == NAV_INVALID &&
                      (o.svc_reasons & ESKF_SVC_R_NAV_DISABLED) && o.why_code == ESKF_SVC_WHY_PHASE_UNKNOWN &&
                      S.diag.nav_cut_samples > 1000,
                  "  fase INCIERTA publicada (ni rampa ni vuelo); ESKF fuera de servicio (why 0x%04x, %u muestras retenidas)",
                  o.why_code, S.diag.nav_cut_samples);
            CHECK(o.bb.state == ESKF_SVC_BB_TRACK && isnan(o.bb.ref_m) && isnan(o.bb.h_m) && !o.bb.valid &&
                      fabs((double)o.bb.vz_mps + vs[iv]) < 0.5 && o.source == ESKF_SVC_SRC_NONE &&
                      !(o.caps & (ESKF_SVC_CAP_ATTITUDE | ESKF_SVC_CAP_BARO_REF | ESKF_SVC_CAP_ESKF_NAV | ESKF_SVC_CAP_BARO_BACKUP)),
                  "  respaldo siguiendo (v_z %.2f m/s) sin cero nuevo; fuente NINGUNA (real %.1f m; antes: ~%s m VÁLIDO)",
                  (double)o.bb.vz_mps, ht, iv == 0 ? "−7,5 ESKF" : "−3,5 respaldo");
            eskf_msg_snapshot_t sn;
            eskf_msg_ops_t op;
            int gs, go;
            last_published(&sn, &op, &gs, &go);
            CHECK(gs && go && sn.phase == ESKF_SVC_PHASE_UNKNOWN && op.phase == ESKF_SVC_PHASE_UNKNOWN && op.source == 0 &&
                      !op.bb_valid,
                  "  snapshot y compacta publican la fase incierta (esquema v%u)", (unsigned)ESKF_MSG_VERSION);
        }
    flight_reset();

    /* arranque normal: rampa confirmada por el contrato de inicio y la evidencia continua */
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 20.0);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(S.pad && o.phase == ESKF_SVC_PHASE_PAD && o.svc_validity == NAV_VALID && o.source == ESKF_SVC_SRC_ESKF && o.bb.valid,
          "arranque normal: rampa confirmada, VÁLIDO, fuente ESKF, respaldo válido");
    /* referencia barométrica tardía en rampa: la IMU continua mantiene la rampa confirmada */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    m.baro_start = 40.0;
    sim_run(&m, 39.9);
    eskf_svc_get_output(&S, &o);
    CHECK(S.pad && o.phase == ESKF_SVC_PHASE_PAD && o.svc_validity == NAV_DEGRADED && (o.svc_reasons & ESKF_SVC_R_NO_ALT_REF) &&
              (o.caps & ESKF_SVC_CAP_ATTITUDE) && !(o.caps & ESKF_SVC_CAP_BARO_REF),
          "sin BMP durante 39 s: rampa confirmada por la IMU; DEGRADADO por NO_ALT_REF");
    sim_run(&m, 52.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.pad && o.svc_validity == NAV_VALID && isfinite(o.nav.alt_ref) && (o.caps & ESKF_SVC_CAP_BARO_REF) && o.bb.valid &&
              fabsf(o.bb.h_m) < 0.5f && o.source == ESKF_SVC_SRC_ESKF,
          "BMP a 40 s: referencias de navegación y del respaldo en rampa; VÁLIDO (alt_ref %.2f m)", o.nav.alt_ref);
    /* parones de 34 ms en rampa con baro: la evidencia barométrica los cubre */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    m.gap_every = 3.389, m.gap_len = 0.034, m.next_gap = 1.0 + 3.389;
    sim_run(&m, 30.0);
    CHECK(S.pad && S.nav.cnt.rest_breaks_gap > 0, "parones de 34 ms en rampa: la rampa sigue confirmada");
    /* hueco de IMU en rampa sin baro, pasado el plazo de inicio: 0,3 s (cubierto) frente a 0,6 s (incierta) */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    m.baro_start = 1e30;
    m.next_gap = 15.0, m.gap_len = 0.3, m.gap_every = 1e9;
    sim_run(&m, 30.0);
    eskf_svc_get_output(&S, &o);
    uint32_t al1 = S.nav.cnt.align_ok;
    CHECK(S.pad && (o.nav.reasons & NAV_R_TIME_GAP) == 0 && o.nav_validity != NAV_INVALID && al1 >= 2,
          "sin baro, hueco de IMU de 0,3 s en rampa (≤ svc.pad_gap_max_s): rampa confirmada y realinea (align_ok %u)", al1);
    start(&c);
    sim_init(&m, 1.0, static_truth);
    m.baro_start = 1e30;
    m.next_gap = 15.0, m.gap_len = 0.6, m.gap_every = 1e9;
    sim_run(&m, 30.0);
    eskf_svc_get_output(&S, &o);
    al1 = S.nav.cnt.align_ok;
    CHECK(!S.pad && o.phase == ESKF_SVC_PHASE_UNKNOWN && o.svc_validity == NAV_INVALID && al1 == 1 &&
              S.diag.pad_lost_hole_us > 600000,
          "sin baro, hueco de 0,6 s: fase INCIERTA (política conservadora, hueco %.0f ms), no realinea",
          (double)S.diag.pad_lost_hole_us * 1e-3);
    uint16_t ses = S.session;
    eskf_svc_request_restart(&S);
    sim_run(&m, 40.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses + 1 && S.pad && o.phase == ESKF_SVC_PHASE_PAD && S.nav.cnt.align_ok >= 1 &&
              o.nav_validity != NAV_INVALID,
          "reinicio pedido por el anfitrión (declara rampa): sesión nueva, rampa confirmada, alinea");
    /* aviso de referencia con la fase incierta: como en vuelo, no abre sesión */
    start(&c);
    sim_init(&m, 1.0, static_truth);
    m.baro_start = 1e30;
    m.next_gap = 15.0, m.gap_len = 0.6, m.gap_every = 1e9;
    sim_run(&m, 20.0);
    ses = S.session;
    eskf_svc_notify_baro_ref_change(&S);
    sim_run(&m, 20.1);
    CHECK(!S.pad && S.session == ses && S.baro_ref_changed, "aviso de referencia con fase incierta: sin sesión nueva, baro fuera");
}

/* ---------------------------------------------------------------- 30. caducidad del respaldo */
static void bb_base(eskf_svc_config_t *c, uint16_t *q)
{
    *c = cfg_of(ESKF_SVC_PROFILE_ACQ);
    start(c);
    *q = 0;
    for (int k = 0; k <= 150; k++) baro_alt((*q)++, 1.0 + 0.01 * k, 0.0f, 1); /* válido; última aceptada a 2,500 s */
}

static void test_bb_expiry(void)
{
    printf("30. REGRESIÓN: un respaldo caducado sólo vuelve a ser válido con una ventana coherente nueva\n");
    eskf_svc_config_t c;
    uint16_t q;
    eskf_svc_output_t o;
    bb_base(&c, &q);
    eskf_svc_get_output(&S, &o);
    float ref0 = o.bb.ref_m;
    for (int k = 1; k <= 19; k++) baro_alt(q++, 2.5 + 0.06 * k, 100.0f, 1); /* 19 rechazadas cada 60 ms */
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.t_us == 2500000 && o.bb.age_us >= 1140000 && o.bb.age_us < 1141000 &&
              o.bb.state == ESKF_SVC_BB_RECOVER && S.diag.bb_expired == 1 && S.diag.bb_reseeds == 0 && S.diag.bb_gaps == 0,
          "19 rechazadas cada 60 ms: NO válido, antigüedad %.0f ms; caducado en la 10.ª (600 ms > 500): ventana pendiente",
          (double)o.bb.age_us * 1e-3);
    baro_alt(q++, 3.70, 0.0f, 1); /* una sola medida compatible con la predicción */
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER && S.diag.bb_expired == 1 && !(o.caps & ESKF_SVC_CAP_BARO_BACKUP) &&
              o.source == ESKF_SVC_SRC_NONE,
          "una medida compatible tras la caducidad: RECUPERANDO, NO válido, sin capacidad ni fuente (antes: TRACK y válido)");
    for (int k = 1; k <= 18; k++) baro_alt(q++, 3.70 + 0.01 * k, 0.0f, 1);
    eskf_svc_get_output(&S, &o);
    CHECK(!o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER, "19 coherentes: todavía recuperando");
    baro_alt(q++, 3.89, 0.0f, 1);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK && o.bb.ref_m == ref0 && (o.caps & ESKF_SVC_CAP_BARO_BACKUP),
          "20 coherentes: válido otra vez con la MISMA referencia (%.3f m)", (double)o.bb.ref_m);
    /* límite exacto: antigüedad = bb.timeout_s todavía vale; 1 ms más, ventana */
    for (int lim = 0; lim < 2; lim++) {
        bb_base(&c, &q);
        for (int k = 1; k <= 8; k++) baro_alt(q++, 2.5 + 0.06 * k, 100.0f, 1);
        double tc = lim ? 3.001 : 3.000;
        baro_alt(q++, tc, 0.0f, 1);
        eskf_svc_get_output(&S, &o);
        if (!lim)
            CHECK(o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK && S.diag.bb_expired == 0 && o.bb.t_us == 3000000,
                  "compatible con antigüedad exactamente %.0f ms: aceptada y válida", (double)c.bb_timeout_s * 1e3);
        else
            CHECK(!o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER && S.diag.bb_expired == 1,
                  "con %.0f ms + 1 ms: caducado, hace falta la ventana", (double)c.bb_timeout_s * 1e3);
    }
    /* intervalos irregulares admisibles y rechazos breves: sin recuperación */
    bb_base(&c, &q);
    const double dts[7] = {0.010, 0.035, 0.090, 0.020, 0.099, 0.060, 0.095};
    double t = 2.5;
    int ok = 1;
    for (int k = 0; k < 7; k++) {
        t += dts[k];
        baro_alt(q++, t, 0.0f, 1);
        eskf_svc_get_output(&S, &o);
        ok &= o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK;
    }
    for (int k = 1; k <= 19; k++) baro_alt(q++, t + 0.02 * k, 100.0f, 1); /* 380 ms de rechazos */
    baro_alt(q++, t + 0.40, 0.0f, 1);
    eskf_svc_get_output(&S, &o);
    CHECK(ok && o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK && S.diag.bb_recoveries == 1 && S.diag.bb_expired == 0 &&
              S.diag.bb_gaps == 0,
          "intervalos de 10–99 ms y 19 rechazos en 380 ms: sigue en seguimiento y válido, sin ventanas nuevas");
    /* en vuelo: la recuperación tras la caducidad conserva exactamente el cero de lanzamiento */
    eskf_svc_config_t e = cfg_of(ESKF_SVC_PROFILE_EXP);
    start(&e);
    flight_reset();
    g_lift_t = 13.0, g_lift_g = 3.5, g_burn = 1.0;
    sim_t m;
    sim_init(&m, 1.0, flight_truth);
    sim_run(&m, 14.5);
    eskf_svc_get_output(&S, &o);
    ref0 = o.bb.ref_m;
    m.baro_period = 0.06, m.baro_off = 60.0; /* 19 rechazadas en 1,14 s */
    sim_run(&m, m.baro_next + 0.06 * 18.5);
    m.baro_period = 0.01, m.baro_off = 0.0;
    double t1 = m.baro_next + 0.005;
    sim_run(&m, t1);
    eskf_svc_get_output(&S, &o);
    CHECK(S.flight && !o.bb.valid && o.bb.state == ESKF_SVC_BB_RECOVER && S.diag.bb_expired == 1 && o.bb.ref_m == ref0,
          "en vuelo: caducado, recuperando, cero intacto");
    sim_run(&m, t1 + 0.3);
    eskf_svc_get_output(&S, &o);
    CHECK(o.bb.valid && o.bb.state == ESKF_SVC_BB_TRACK && o.bb.ref_m == ref0 && o.bb.ref_frozen,
          "en vuelo: válido tras la ventana con el MISMO cero (%.3f m)", (double)o.bb.ref_m);
    flight_reset();
}

/* ---------------------------------------------------------------- 31. silencio total */
static void consume_until(int64_t t_end_us, int64_t step_us)
{
    while (g_now_us + step_us <= t_end_us) {
        g_now_us += step_us;
        eskf_svc_consume(&S);
    }
    if (g_now_us < t_end_us) {
        g_now_us = t_end_us;
        eskf_svc_consume(&S);
    }
}
static int find_phase_record(int64_t *t_out, uint16_t *hole_ms)
{
    eskf_svc_trec_t tr[ESKF_SVC_TRACE_CAP];
    uint32_t k;
    int found = 0;
    while ((k = eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP)) > 0)
        for (uint32_t i = 0; i < k; i++)
            if (tr[i].type == ESKF_SVC_TR_PHASE && tr[i].res == (int8_t)ESKF_SVC_PHASE_UNKNOWN && tr[i].a == 2) {
                found++;
                *t_out = tr[i].t;
                *hole_ms = tr[i].b;
            }
    return found;
}

static void test_total_silence(void)
{
    printf("31. REGRESIÓN: la rampa confirmada caduca también por el reloj en silencio total\n");
    eskf_svc_config_t c = cfg_of(ESKF_SVC_PROFILE_EXP);
    c.trace_enable = 1;
    start(&c);
    sim_t m;
    sim_init(&m, 1.0, static_truth);
    sim_run(&m, 15.0);
    eskf_svc_output_t o;
    eskf_svc_get_output(&S, &o);
    CHECK(S.pad && o.phase == ESKF_SVC_PHASE_PAD && o.svc_validity == NAV_VALID, "navegación VÁLIDA en rampa hasta 15 s");
    int64_t ref = S.pad_rx_us;
    int64_t lim = (int64_t)((double)c.pad_gap_max_s * 1e6) + (int64_t)(c.nav.liftoff_hold_s * 1e6) +
                  (int64_t)((double)c.pad_lat_max_s * 1e6);
    { /* la traza se vacía: sólo interesan los registros del silencio */
        eskf_svc_trec_t tr[ESKF_SVC_TRACE_CAP];
        while (eskf_svc_trace_read(&S, tr, ESKF_SVC_TRACE_CAP) > 0) {
        }
    }
    /* IMU y BMP detenidos; eskf_svc_consume sigue llamándose cada 10 ms */
    consume_until(ref + lim - 10000, 10000);
    eskf_svc_get_output(&S, &o);
    CHECK(S.pad && o.svc_validity == NAV_INVALID && (o.nav.reasons & NAV_R_STALE),
          "silencio de %.2f s (< latencia admitida): ESKF NO VÁLIDO por antigüedad, rampa aún confirmada",
          (double)(g_now_us - ref) * 1e-6);
    consume_until(ref + lim, 10000);
    CHECK(S.pad, "límite exacto (%.3f s sin muestras procesadas): todavía rampa", (double)lim * 1e-6);
    g_now_us += 1;
    eskf_svc_consume(&S);
    eskf_svc_get_output(&S, &o);
    int64_t tp = 0;
    uint16_t hole = 0;
    int nrec = find_phase_record(&tp, &hole);
    CHECK(!S.pad && o.phase == ESKF_SVC_PHASE_UNKNOWN && o.why_code == ESKF_SVC_WHY_PHASE_UNKNOWN &&
              (o.svc_reasons & ESKF_SVC_R_NAV_DISABLED) && S.diag.pad_lost_cause == 2 && S.nav_cut,
          "1 µs más: vencimiento por el reloj → fase INCIERTA (causa 2, why 0x%04x), navigation.c cortado", o.why_code);
    CHECK(nrec == 1 && tp == g_now_us && hole == (uint16_t)((lim + 1) / 1000),
          "la transición queda en la traza (PHASE, causa silencio, %u ms, en el reloj del consume)", hole);
    /* aviso de referencia durante el silencio: sin sesión nueva ni diez segundos más de rampa */
    consume_until(17000000, 10000);
    uint16_t ses = S.session;
    eskf_svc_notify_baro_ref_change(&S);
    g_now_us += 10000;
    eskf_svc_consume(&S);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses && !S.pad && S.baro_ref_changed && (o.svc_reasons & ESKF_SVC_R_BARO_REF_CHANGED) &&
              o.phase == ESKF_SVC_PHASE_UNKNOWN,
          "aviso de referencia a 17 s: conserva la sesión, baro fuera el resto de la sesión (antes: sesión nueva y 10 s de rampa)");
    /* vuelven las muestras: la fase sigue INCIERTA, sin alineación ni referencia */
    uint32_t al = S.nav.cnt.align_ok, cut0 = S.diag.nav_cut_samples;
    m.imu_next = 18.0, m.baro_next = 18.003;
    sim_run(&m, 30.0);
    eskf_svc_get_output(&S, &o);
    CHECK(!S.pad && o.phase == ESKF_SVC_PHASE_UNKNOWN && S.nav.cnt.align_ok == al && S.diag.nav_cut_samples > cut0 + 2000 &&
              S.diag.baro_not_fed > 1000 && !o.bb.valid && o.svc_validity == NAV_INVALID,
          "las muestras vuelven a 18 s: sigue INCIERTA, sin realinear (align_ok %u), baro sin usar", S.nav.cnt.align_ok);
    /* el reinicio explícito del anfitrión en rampa se conserva */
    eskf_svc_request_restart(&S);
    sim_run(&m, 45.0);
    eskf_svc_get_output(&S, &o);
    CHECK(S.session == ses + 1 && S.pad && o.phase == ESKF_SVC_PHASE_PAD && o.svc_validity == NAV_VALID && o.bb.valid,
          "reinicio pedido por el anfitrión en rampa: sesión nueva, rampa confirmada, VÁLIDO");
    /* sin falsas transiciones: latencia de entrega constante y parón del router con su cola continua */
    c.trace_enable = 0;
    for (int k = 0; k < 2; k++) {
        start(&c);
        sim_init(&m, 1.0, static_truth);
        sim_run(&m, 15.0);
        m.lat_s = k ? 1.2 : 0.9; /* las muestras de 15 s en adelante llegan tarde, en orden y sin huecos */
        sim_run(&m, 25.0);
        if (!k)
            CHECK(S.pad && S.diag.pad_lost_cause == 0 && S.nav.cnt.gaps == 0,
                  "parón de 0,9 s del router y latencia de 0,9 s después (cola continua): la rampa sigue confirmada");
        else
            CHECK(!S.pad && S.diag.pad_lost_cause == 2,
                  "parón de 1,2 s (> latencia admitida %.2f s): caduca por el reloj (con la cola real de 128 paquetes "
                  "habría además un hueco de muestras)",
                  (double)lim * 1e-6);
    }
    /* contrato de inicio: diez segundos sin datos no caducan la rampa */
    start(&c);
    consume_until(1000000 + (int64_t)((double)c.pad_start_s * 1e6) + lim, 10000);
    int pad_end_contract = S.pad;
    g_now_us += 1;
    eskf_svc_consume(&S);
    CHECK(pad_end_contract && !S.pad && S.diag.pad_lost_cause == 2,
          "sin datos tras el inicio: rampa por contrato hasta %.0f s + %.2f s; después, INCIERTA", (double)c.pad_start_s,
          (double)lim * 1e-6);
}

int main(void)
{
#ifdef ESKF_USE_FLOAT
    printf("== test_service [float]\n");
#else
    printf("== test_service [double]\n");
#endif
    test_imu_decode();
    test_invalid();
    test_kpid();
    test_init();
    test_sequence();
    test_queues();
    test_stale();
    test_baro();
    test_nav();
    test_fragments();
    test_replay();
    test_bounds();
    test_future_mark();
    test_old_and_restart();
    test_baro_faults();
    test_baro_ref_flight();
    test_saturation();
    test_backup();
    test_ops();
    test_queries_threads();
    test_config();
    test_bb_freshness();
    test_bb_initial_ref();
    test_session_boundary();
    test_imu_gap_flight();
    test_flight_latch();
    test_late_first_baro();
    test_phase_unknown();
    test_bb_expiry();
    test_total_silence();
    printf("%d/%d comprobaciones correctas\n", g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}
