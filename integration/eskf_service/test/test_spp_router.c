/*
 * test_spp_router.c — integración con el ROUTER REAL de SPP (commit 9247d9e): core.c, pubsub.c,
 * databank.c, crc.c, log.c, hal.c, time.c, spi.c compilados sin cambios.
 *
 * Dobles de prueba SÓLO para la plataforma: un puerto HAL con reloj inyectable y productores que
 * sustituyen al hardware del ICM-20948 y del BMP390 (publican sus payloads con
 * SPP_SERVICES_DATABANK_getPacket + packetData + PUBSUB_publish, como los drivers reales). El
 * contrato que se verifica (registro, KPID, init, entrega, consumo, publicación) es el real.
 *
 * El router tiene estado estático sin reinicio: cada escenario se ejecuta en su propio proceso.
 *   test_spp_router <escenario> [directorio]   (sin argumento: lista)
 * Con directorio, «congestion» escribe la traza del pegamento, las entradas que recibió el
 * servicio y los mensajes que recibió el registro, para compararlos con svc_replay --reproducir.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spp/core/core.h"
#include "spp/core/pubsub/pubsub.h"
#include "spp/hal/hal.h"
#include "spp/services/databank/databank.h"
#include "spp/util/crc.h"

#include "eskf_svc_msg.h"
#include "eskf_svc_params.h"
#include "spp_services_eskf.h"
#include "test_common.h"

/* ---------------------------------------------------------------- puerto HAL de prueba (plataforma) */
static int64_t g_tick_us = 0; /* > 0: cada lectura del reloj lo hace avanzar (cruza milisegundos) */
static spp_uint32_t port_ms(void)
{
    g_now_us += g_tick_us;
    return (spp_uint32_t)(g_now_us / 1000);
}
static spp_uint32_t port_us(void) { return (spp_uint32_t)g_now_us; }
static SPP_RetVal_t port_ok(void) { return K_SPP_OK; }
static void port_delay(spp_uint32_t ms) { (void)ms; }
static SPP_HalPort_t g_port; /* resto de punteros a NULL: el servicio no los usa */

/* ---------------------------------------------------------------- productores de prueba (hardware) */
typedef struct {
    SPP_Kpid_t kpid;
    spp_uint16_t seq;
    double next, period, start;
    int imu, held, held_pause, signal_ready, calls;
    double hold_until; /* retiene un paquete ya construido (publicación tardía) */
    SPP_Packet_t *p_held;
} prod_t;
static prod_t g_icm, g_bmp;
static double g_lift_t = 1e30;
static int g_silent; /* 1: los sensores no publican nada (el anfitrión sigue moviendo el router) */

static void truth(double t, double a[3], double w[3], double *h)
{
    a[0] = 0.1, a[1] = -0.05, a[2] = -sqrt(1.0 - 0.0125);
    w[0] = w[1] = w[2] = 0.0;
    *h = 0.0;
    if (t > g_lift_t) a[0] = a[1] = 0.0, a[2] = -3.5, *h = 0.5 * 2.5 * 9.80665 * (t - g_lift_t) * (t - g_lift_t);
}

static SPP_RetVal_t prod_acquire(prod_t *p, SPP_Kpid_t kpid)
{
    p->calls++;
    p->kpid = kpid;
    double t = (double)g_now_us * 1e-6;
    if (p->p_held && t >= p->hold_until) {
        (void)SPP_SERVICES_PUBSUB_publish(p->p_held);
        p->p_held = NULL;
    }
    if (t < p->start) { /* calibración del productor: lee pero no publica */
        p->next = p->start;
        return K_SPP_OK;
    }
    if (g_silent) { /* silencio total de los sensores */
        p->next = t + p->period;
        return K_SPP_OK;
    }
    if (t < p->next) return K_SPP_OK; /* sin DRDY */
    p->next += p->period;
    if (p->signal_ready) SPP_SERVICES_PUBSUB_signalProducerReady();
    double a[3], w[3], h;
    truth(t, a, w, &h);
    uint8_t pl[44];
    spp_uint16_t len;
    if (p->imu) imu44(pl, a, w), len = 44;
    else baro12(pl, h), len = 12;
    SPP_Packet_t *pk = SPP_SERVICES_DATABANK_getPacket();
    if (pk == NULL) return K_SPP_ERROR;
    if (SPP_SERVICES_DATABANK_packetData(pk, kpid.value, p->seq++, pl, len) != K_SPP_OK) {
        (void)SPP_SERVICES_DATABANK_returnPacket(pk);
        return K_SPP_ERROR;
    }
    if (p->held && p->p_held == NULL) { /* este paquete se publicará tarde */
        p->held = 0;
        p->p_held = pk;
        p->hold_until = t + 0.060;
        if (p->held_pause) p->next = p->hold_until + 0.001; /* sin más baro mientras tanto */
        return K_SPP_OK;
    }
    (void)SPP_SERVICES_PUBSUB_publish(pk); /* como los drivers reales: el retorno se ignora */
    return K_SPP_OK;
}
static SPP_RetVal_t icm_init(void) { return K_SPP_OK; }
static SPP_RetVal_t icm_acquire(SPP_Kpid_t k) { return prod_acquire(&g_icm, k); }
static SPP_RetVal_t bmp_acquire(SPP_Kpid_t k) { return prod_acquire(&g_bmp, k); }
static const SPP_SERVICE_ProducerContract_t ICM = {"icm_doble", 5000U, icm_init, icm_acquire};
static const SPP_SERVICE_ProducerContract_t BMP = {"bmp_doble", 5000U, icm_init, bmp_acquire};

/* ---------------------------------------------------------------- consumidor «registro» de prueba (prioridad 0, como el datalogger) */
static eskf_msg_reasm_t g_ra;
static int g_snaps, g_diags, g_log_pkts, g_log_eskf, g_log_crc_bad, g_cons_calls_hi;
static eskf_msg_snapshot_t g_last;
static eskf_msg_ops_t g_last_ops;
static int g_ops;
static spp_uint16_t g_eskf_kpid;
static FILE *g_f_in, *g_f_out, *g_f_tr; /* registro de la congestión (opcional) */
static SPP_RetVal_t log_deliver(const SPP_Packet_t pk)
{
    g_log_pkts++;
    if (SPP_UTIL_crc16((const spp_uint8_t *)&pk, 62U) != pk.crc) g_log_crc_bad++;
    if (pk.primaryHeader.apid == g_eskf_kpid) {
        g_log_eskf++;
        eskf_msg_diag_t d;
        int k = eskf_msg_reasm_push_ops(&g_ra, pk.payload, pk.primaryHeader.payloadLen, &g_last, &d, &g_last_ops);
        g_snaps += k == 1, g_diags += k == 2, g_ops += k == 3;
        if (g_f_out) fwrite(&pk, 1, 64, g_f_out);
    } else if (g_f_in) {
        fwrite(&pk, 1, 64, g_f_in); /* las mismas entradas, en el mismo orden, que recibió el ESKF */
    }
    return K_SPP_OK;
}
static SPP_RetVal_t log_consume(void *p) { (void)p; return K_SPP_OK; }
static const SPP_SERVICE_ConsumerContract_t LOG = {0, "log_doble", 1000U, icm_init, log_deliver, log_consume};
/* consumidor de prioridad 3 (para comprobar la interrupción por signalProducerReady) */
static SPP_RetVal_t hi_consume(void *p) { (void)p; g_cons_calls_hi++; return K_SPP_OK; }
static SPP_RetVal_t hi_deliver(const SPP_Packet_t pk) { (void)pk; return K_SPP_OK; }
static const SPP_SERVICE_ConsumerContract_t HI = {3, "prio3_doble", 1000U, icm_init, hi_deliver, hi_consume};

/* ---------------------------------------------------------------- bucle del anfitrión */
static int g_loop_iter, g_drain;
static void drain_trace(void)
{
    eskf_svc_trec_t r[64];
    spp_uint32_t n;
    while ((n = SPP_SERVICES_ESKF_readTrace(r, 64U)) > 0U) trace_put(g_f_tr, r, n);
}
/* fin de la captura: vaciar, cerrar con el registro END (nº de registros y CRC) y vaciar otra vez */
static int close_trace(void)
{
    drain_trace();
    SPP_RetVal_t r = SPP_SERVICES_ESKF_closeTrace();
    drain_trace();
    return r == K_SPP_OK;
}
static void host_loop(double t_end, double step_s, int consumers_every)
{
    while ((double)g_now_us * 1e-6 < t_end) {
        g_now_us += (int64_t)llround(step_s * 1e6);
        SPP_SERVICES_PUBSUB_callProducers();
        if (++g_loop_iter % consumers_every == 0) {
            /* callConsumers reparte UN paquete por llamada: el anfitrión la llama varias veces */
            for (int k = 0; k < 3; k++) SPP_SERVICES_PUBSUB_callConsumers();
        }
        if (g_f_tr || g_drain) drain_trace(); /* sin archivo: se vacía igual para no desbordar la traza */
    }
}

static void setup(double t0)
{
    g_now_us = (int64_t)(t0 * 1e6);
    g_port.time.getTimeMs = port_ms;
    g_port.time.getTimeUs = port_us;
    g_port.time.delayMs = port_delay;
    g_port.spi.spiBusInit = port_ok;
    if (SPP_HAL_init(&g_port) != K_SPP_OK) printf("SPP_HAL_init falló\n");
    memset(&g_icm, 0, sizeof g_icm);
    memset(&g_bmp, 0, sizeof g_bmp);
    g_icm.imu = 1, g_icm.period = 1.0 / 225.0, g_icm.next = t0;
    g_bmp.period = 1.0 / 100.0, g_bmp.next = t0, g_bmp.start = t0 + 5.0; /* 500 muestras de calibración */
    eskf_msg_reasm_init(&g_ra);
}

static eskf_svc_config_t bench(void)
{
    eskf_svc_config_t c;
    eskf_svc_default_config(ESKF_SVC_PROFILE_EXP, &c);
    return c;
}
static const char *g_dir = NULL;

static int finish(const char *name)
{
    printf("%s: %d/%d comprobaciones correctas\n", name, g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}

/* ---------------------------------------------------------------- escenarios */
/* A: sensores registrados primero (0x0004, 0x0008), ESKF 0x0010; flujo completo en EXPERIMENTAL */
static int sc_order_a(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    CHECK(SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki) == K_SPP_OK && ki.value == 0x0004, "ICM → 0x%04x", ki.value);
    CHECK(SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb) == K_SPP_OK && kb.value == 0x0008, "BMP → 0x%04x", kb.value);
    eskf_svc_config_t c = bench();
    CHECK(SPP_SERVICES_ESKF_configure(&c) == K_SPP_OK, "configure antes de init");
    CHECK(SPP_SERVICES_ESKF_register(ki, kb, &ko) == K_SPP_OK && ko.value == 0x0010, "ESKF → 0x%04x", ko.value);
    CHECK(SPP_SERVICES_ESKF_getRegistration() == SPP_SERVICES_ESKF_REG_BOTH, "consumidor y productor registrados");
    SPP_Kpid_t ko2 = {0};
    CHECK(SPP_SERVICES_ESKF_register(ki, kb, &ko2) == K_SPP_ERROR_ALREADY_INITIALIZED && ko2.value == 0,
          "registro repetido: rechazado sin tocar el router");
    g_eskf_kpid = ko.value;
    SPP_Kpid_t sub = SPP_SERVICES_ESKF_getSubscription();
    CHECK(sub.value == 0x000C && !(sub.value & ko.value), "suscripción 0x%04x sin la salida propia", sub.value);
    SPP_Kpid_t all = {.value = 0x001C};
    CHECK(SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all) == K_SPP_OK, "registro de prueba");
    CHECK(SPP_CORE_init() == K_SPP_OK, "SPP_CORE_init");
    SPP_RetVal_t ci = K_SPP_ERROR, pi = K_SPP_ERROR;
    SPP_SERVICES_PUBSUB_getConsumerInitResult(SPP_SERVICES_ESKF_getConsumerContract(), &ci);
    SPP_SERVICES_PUBSUB_getProducerInitResult(SPP_SERVICES_ESKF_getProducerContract(), &pi);
    CHECK(ci == K_SPP_OK && pi == K_SPP_OK && SPP_SERVICES_ESKF_getInitCalls() == 3U, "init dual: consumidor %d, productor %d",
          ci, pi);
    CHECK(SPP_SERVICES_ESKF_configure(&c) == K_SPP_ERROR_ALREADY_INITIALIZED, "configure después de init: rechazado");
    eskf_svc_output_t o;
    CHECK(SPP_SERVICES_ESKF_getConsumerContract()->init() == K_SPP_OK, "tercera llamada a init: no-op");
    host_loop(14.0, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(o.session == 1, "una sola sesión (las llamadas repetidas a init no la reiniciaron)");
    CHECK(d.rejected_apid == 0 && d.rejected_crc == 0 && d.seq_lost_imu == 0 && d.mailbox_full == 0,
          "entradas limpias: apid %u crc %u perdidas %u", d.rejected_apid, d.rejected_crc, d.seq_lost_imu);
    CHECK(d.imu_rx > 2900 && d.baro_rx > 790 && d.first_baro_us - d.first_imu_us >= 4900000,
          "IMU %u, baro %u; primer baro tras la calibración del productor", d.imu_rx, d.baro_rx);
    CHECK(o.nav_validity == NAV_VALID && o.svc_validity == NAV_VALID && o.flight_accept == 0 &&
              (o.caps & ESKF_SVC_CAP_ESKF_NAV) && (o.caps & ESKF_SVC_CAP_BARO_BACKUP),
          "EXPERIMENTAL alineado y válido (motivos 0x%x / 0x%x, caps 0x%02x), no aceptable para vuelo", o.nav.reasons,
          o.svc_reasons, o.caps);
    CHECK(g_snaps > 55 && g_ra.incomplete == 0 && g_ra.crc_bad == 0 && g_diags >= 12 && g_ops >= 12,
          "salida reconstruida: %d snapshots, %d diag, %d compactas", g_snaps, g_diags, g_ops);
    CHECK(g_last_ops.source == ESKF_SVC_SRC_ESKF && g_last_ops.bb_valid && fabsf(g_last_ops.bb_h_m) < 0.5f,
          "compacta en tierra: fuente ESKF, respaldo válido (h %.2f m)", (double)g_last_ops.bb_h_m);
    CHECK(g_last.svc_validity == NAV_VALID && fabs(g_last.p[2]) < 1.0, "último snapshot válido en tierra (p_z %.3f)",
          (double)g_last.p[2]);
    CHECK(g_log_crc_bad == 0 && d.publish_fail == 0 && d.kpid_mismatch == 0 && (int)d.publish_ok == g_log_eskf,
          "CRC de la salida correcto; %u publicados = %d recibidos", d.publish_ok, g_log_eskf);
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro: %u/%u libres (sin fugas ni dobles devoluciones)",
          (unsigned)SPP_SERVICES_DATABANK_freeCount(), (unsigned)K_SPP_DATABANK_SIZE);
    double pkt_s = (double)(g_log_pkts) / 13.0;
    printf("  carga en el router: %.0f paquetes/s (%.1f kB/s); del ESKF %.0f/s\n", pkt_s, pkt_s * 64.0 / 1000.0,
           g_log_eskf / 13.0);
    /* reinicio explícito */
    CHECK(SPP_SERVICES_ESKF_requestSessionRestart() == K_SPP_OK, "petición de reinicio");
    host_loop(14.1, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.session == 2, "reinicio explícito: sesión 2");
    return finish("orden_a");
}

/* B: el ESKF se registra PRIMERO como productor (0x0004); sensores después */
static int sc_order_b(void)
{
    setup(1.0);
    SPP_Kpid_t ko, ki, kb;
    CHECK(SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ESKF_getProducerContract(), &ko) == K_SPP_OK && ko.value == 0x0004,
          "ESKF → 0x%04x", ko.value);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    CHECK(ki.value == 0x0010 && kb.value == 0x0008, "BMP 0x%04x, ICM 0x%04x", kb.value, ki.value);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    CHECK(SPP_SERVICES_ESKF_setKpids(ki, kb, ko) == K_SPP_OK, "KPID en otro orden");
    SPP_Kpid_t bad = {.value = 0x0004};
    CHECK(SPP_SERVICES_ESKF_setKpids(bad, kb, ko) == K_SPP_ERROR_INVALID_PARAMETER, "entrada = salida: rechazado");
    g_eskf_kpid = ko.value;
    SPP_SERVICES_PUBSUB_registerConsumer(SPP_SERVICES_ESKF_getConsumerContract(), SPP_SERVICES_ESKF_getSubscription());
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    SPP_CORE_init();
    host_loop(13.0, 0.001, 1);
    eskf_svc_output_t o;
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(d.imu_rx > 2600 && d.rejected_apid == 0 && o.nav_validity == NAV_VALID, "flujo correcto con KPID 0x10/0x08 (imu %u)",
          d.imu_rx);
    CHECK(g_snaps > 50 && g_ra.incomplete == 0, "salida 0x0004 reconstruida (%d)", g_snaps);
    return finish("orden_b");
}

/* C: configuración de transporte inválida → init falla y el router no entrega nada */
static int sc_bad_config(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ESKF_getProducerContract(), &ko);
    eskf_svc_config_t c = bench();
    c.kpid_imu = ki.value, c.kpid_baro = ki.value, c.kpid_out = ko.value; /* entradas iguales */
    SPP_SERVICES_ESKF_configure(&c);
    SPP_Kpid_t sub = {.value = ki.value};
    SPP_SERVICES_PUBSUB_registerConsumer(SPP_SERVICES_ESKF_getConsumerContract(), sub);
    SPP_CORE_init();
    SPP_RetVal_t ci = K_SPP_OK;
    SPP_SERVICES_PUBSUB_getConsumerInitResult(SPP_SERVICES_ESKF_getConsumerContract(), &ci);
    CHECK(ci == K_SPP_ERROR_INVALID_PARAMETER && SPP_SERVICES_ESKF_getInitResult() == ci, "init rechazado (%d)", ci);
    host_loop(2.0, 0.001, 1);
    eskf_svc_output_t o;
    CHECK(SPP_SERVICES_ESKF_getOutput(&o) == K_SPP_ERROR_NOT_INITIALIZED, "sin servicio: consultas lo indican");
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro");
    return finish("config_invalida");
}

/* D: ESTRICTO con pendientes: el transporte funciona y se publica el motivo */
static int sc_flight_pending(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c;
    eskf_svc_default_config(ESKF_SVC_PROFILE_STRICT, &c);
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    g_eskf_kpid = ko.value;
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    CHECK(SPP_CORE_init() == K_SPP_OK && SPP_SERVICES_ESKF_getInitResult() == K_SPP_OK, "init OK con navegación pendiente");
    host_loop(3.0, 0.001, 1);
    CHECK(g_snaps >= 9 && g_last.mask == 1 && g_last.svc_validity == NAV_INVALID &&
              (g_last.svc_reasons & ESKF_SVC_R_NAV_DISABLED) && g_last.why_code > 0,
          "tierra recibe NO VÁLIDO + motivo (why_code %u)", g_last.why_code);
    size_t n;
    const eskf_param_t *T = eskf_svc_params_table(&n);
    CHECK(g_last.why_code <= n && !strcmp(T[g_last.why_code - 1].name, "in.gyro_unit_si"), "why_code → %s",
          T[g_last.why_code - 1].name);
    return finish("estricto_pendiente");
}

/* E: el anfitrión vacía la cola global demasiado despacio → publish falla en los sensores; el ESKF
   aplaza su salida. Con directorio, se registra todo para reproducirlo con svc_replay. */
static int sc_queue_full(void)
{
    setup(1.0);
    char path[512];
    if (g_dir) {
        snprintf(path, sizeof path, "%s/congestion_entradas.bin", g_dir);
        g_f_in = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/congestion_salida_real.bin", g_dir);
        g_f_out = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/congestion_traza.bin", g_dir);
        g_f_tr = fopen(path, "wb");
        if (!g_f_in || !g_f_out || !g_f_tr) return 2;
    }
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    c.trace_enable = g_dir ? 1 : 0;
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    g_eskf_kpid = ko.value;
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    SPP_CORE_init();
    host_loop(12.0, 0.001, 1);
    eskf_svc_output_t o;
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.svc_validity == NAV_VALID, "antes del atasco: VÁLIDA");
    host_loop(13.0, 0.001, 20); /* 3 paquetes cada 20 ms << 352 paquetes/s */
    printf("  cola del router tras el atasco: %u/128\n", (unsigned)SPP_SERVICES_PUBSUB_queueDepth());
    host_loop(13.5, 0.001, 1); /* la cola se vacía: el servicio ve los saltos de secuencia */
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(d.seq_lost_imu > 0 && (o.svc_reasons & ESKF_SVC_R_IMU_LOSS) && o.svc_validity != NAV_VALID,
          "publicaciones fallidas en el router → saltos de secuencia → no VÁLIDA (%u perdidas, validez %u)", d.seq_lost_imu,
          o.svc_validity);
    CHECK(d.queue_busy > 0, "el ESKF aplazó su salida con la cola alta (%u)", d.queue_busy);
    host_loop(25.0, 0.001, 1); /* rampa en reposo: recuperación */
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(o.svc_validity == NAV_VALID && d.loss_recovered >= 1 && o.session == 1,
          "recuperada en rampa tras una ventana de reposo completa, sin sesión nueva (recuperaciones %u)", d.loss_recovered);
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro tras el atasco");
    CHECK((int)d.publish_ok == g_log_eskf, "publicados %u = recibidos por el registro %d", d.publish_ok, g_log_eskf);
    if (g_dir) {
        int closed = close_trace();
        printf("  registro: %d entradas, %d mensajes del ESKF, %u aplazamientos; traza sin desbordar: %s\n",
               g_log_pkts - g_log_eskf, g_log_eskf, d.queue_busy + d.pool_empty, d.trace_overflow == 0 ? "sí" : "NO");
        CHECK(d.trace_overflow == 0 && closed, "traza completa y cerrada (END)");
        fclose(g_f_in), fclose(g_f_out), fclose(g_f_tr);
        g_f_in = g_f_out = g_f_tr = NULL;
    }
    return finish("congestion");
}

/* E1: reloj que AVANZA en cada lectura: la cabecera que construye packetData y el instante en que
   se anota la publicación caen en milisegundos distintos. Con directorio, registra lo necesario
   para reproducir y comparar paquetes COMPLETOS (no sólo cargas útiles). */
static int sc_clock_advance(void)
{
    setup(1.0);
    char path[512];
    if (g_dir) {
        snprintf(path, sizeof path, "%s/reloj_entradas.bin", g_dir);
        g_f_in = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/reloj_salida_real.bin", g_dir);
        g_f_out = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/reloj_traza.bin", g_dir);
        g_f_tr = fopen(path, "wb");
        if (!g_f_in || !g_f_out || !g_f_tr) return 2;
    }
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    c.trace_enable = 1;
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    g_eskf_kpid = ko.value;
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    SPP_CORE_init();
    g_drain = 1;
    g_tick_us = 333; /* 3 lecturas ≈ 1 ms */
    host_loop(9.0, 0.001, 1);
    g_tick_us = 0;
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(g_log_eskf > 50 && (int)d.publish_ok == g_log_eskf && d.trace_overflow == 0,
          "salida publicada (%d paquetes) con el reloj avanzando entre lecturas; traza completa", g_log_eskf);
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro");
    if (g_dir) {
        CHECK(close_trace(), "traza cerrada (END)");
        fclose(g_f_in), fclose(g_f_out), fclose(g_f_tr);
        g_f_in = g_f_out = g_f_tr = NULL;
    }
    return finish("reloj_avanza");
}

/* E2: fallos de transporte que no bloquean el servicio: pool vacío 300 ms */
static int sc_pool_empty(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    g_eskf_kpid = ko.value;
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    SPP_CORE_init();
    host_loop(12.0, 0.001, 1);
    SPP_Packet_t *held[K_SPP_DATABANK_SIZE];
    unsigned nh = 0;
    while (nh < K_SPP_DATABANK_SIZE && (held[nh] = SPP_SERVICES_DATABANK_getPacket()) != NULL) nh++;
    eskf_svc_diag_t d0, d;
    SPP_SERVICES_ESKF_getDiagnostics(&d0);
    host_loop(12.3, 0.001, 1);
    eskf_svc_output_t o;
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(nh == K_SPP_DATABANK_SIZE && d.pool_empty > d0.pool_empty && d.consume_calls > d0.consume_calls + 800,
          "pool vacío: el ESKF aplaza (%u) y sigue consumiendo (%u llamadas)", d.pool_empty - d0.pool_empty,
          d.consume_calls - d0.consume_calls);
    CHECK(o.svc_validity == NAV_INVALID && (o.nav.reasons & NAV_R_STALE), "sin IMU: el estado se marca caducado (NO VÁLIDO)");
    for (unsigned i = 0; i < nh; i++) SPP_SERVICES_DATABANK_returnPacket(held[i]);
    int eskf0 = g_log_eskf;
    host_loop(13.0, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(g_log_eskf > eskf0 + 10 && (o.nav.reasons & NAV_R_TIME_GAP) && o.svc_validity == NAV_INVALID,
          "pool devuelto: la salida se reanuda (%d mensajes); el hueco de 300 ms invalida la estimación", g_log_eskf - eskf0);
    host_loop(25.0, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.svc_validity == NAV_VALID && o.session == 1, "recuperada en rampa por navigation.c (realineación tras el hueco)");
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro");
    return finish("pool_vacio");
}

/* E3: registro parcial (registro de productores lleno) y registro de consumidores lleno */
static SPP_RetVal_t dummy_acq(SPP_Kpid_t k) { (void)k; return K_SPP_OK; }
static const SPP_SERVICE_ProducerContract_t DUMMY = {"relleno", 1000U, icm_init, dummy_acq};
static int sc_partial(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko = {0}, kx;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    for (int i = 0; i < 3; i++) SPP_SERVICES_PUBSUB_registerProducer(&DUMMY, &kx); /* 5 productores: lleno */
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_RetVal_t r = SPP_SERVICES_ESKF_register(ki, kb, &ko);
    CHECK(r == K_SPP_ERROR_REGISTRY_FULL && SPP_SERVICES_ESKF_getRegistration() == SPP_SERVICES_ESKF_REG_CONSUMER && ko.value == 0,
          "productor sin sitio: queda sólo el consumidor (%d)", r);
    CHECK(SPP_SERVICES_ESKF_register(ki, kb, &ko) == K_SPP_ERROR_ALREADY_INITIALIZED, "no se reintenta a medias");
    SPP_Kpid_t all = {.value = 0x000C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    CHECK(SPP_CORE_init() == K_SPP_OK && SPP_SERVICES_ESKF_getInitResult() == K_SPP_OK && SPP_SERVICES_ESKF_getInitCalls() == 1U,
          "init sólo del consumidor");
    host_loop(13.0, 0.001, 1);
    eskf_svc_output_t o;
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.svc_validity == NAV_VALID && g_log_eskf == 0, "navega y responde a consultas, sin salida SPP");
    return finish("registro_parcial");
}

static SPP_RetVal_t c_del(const SPP_Packet_t pk) { (void)pk; return K_SPP_OK; }
static SPP_RetVal_t c_con(void *p) { (void)p; return K_SPP_OK; }
static const SPP_SERVICE_ConsumerContract_t CFILL = {0, "relleno", 1000U, icm_init, c_del, c_con};
static int sc_consumers_full(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko = {0}, kx;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    SPP_Kpid_t all = {.value = 0x000C};
    for (int i = 0; i < 5; i++) SPP_SERVICES_PUBSUB_registerConsumer(&CFILL, all);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_RetVal_t r = SPP_SERVICES_ESKF_register(ki, kb, &ko);
    CHECK(r == K_SPP_ERROR_REGISTRY_FULL && SPP_SERVICES_ESKF_getRegistration() == SPP_SERVICES_ESKF_REG_NONE,
          "consumidor sin sitio: no se registra nada (%d)", r);
    CHECK(SPP_SERVICES_PUBSUB_registerProducer(&DUMMY, &kx) == K_SPP_OK && kx.value == 0x0010,
          "el ESKF no ocupó ningún productor (siguiente KPID 0x%04x)", kx.value);
    return finish("consumidores_llenos");
}

/* F: un productor señala DRDY en cada adquisición: callConsumers interrumpe prioridad > 1 */
static int sc_priority(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    SPP_Kpid_t all = {.value = 0x000C};
    SPP_SERVICES_PUBSUB_registerConsumer(&HI, all);
    SPP_CORE_init();
    g_icm.signal_ready = 1;
    host_loop(3.0, 0.001, 1);
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(d.consume_calls >= 3u * 1999u && g_cons_calls_hi < (int)d.consume_calls,
          "ESKF (prioridad %d) no se interrumpe: %u llamadas; prioridad 3: %d", K_SPP_ESKF_CONSUMER_PRIO, d.consume_calls,
          g_cons_calls_hi);
    return finish("prioridad");
}

/* G: el baro publica un paquete 60 ms tarde → evento tardío rechazado y contado */
static int sc_late(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    SPP_CORE_init();
    host_loop(7.0, 0.001, 1);
    g_bmp.held = 1; /* retenido 60 ms mientras siguen saliendo otros baro: llega con secuencia atrasada */
    host_loop(7.2, 0.001, 1);
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(d.seq_old == 1 && d.late_baro == 0 && d.seq_resync == 0, "baro con secuencia y marca atrasadas: antiguo, descartado (%u)",
          d.seq_old);
    g_bmp.held = 1, g_bmp.held_pause = 1; /* retenido sin otros baro: secuencia y tiempo en orden, pero tardío */
    host_loop(7.4, 0.001, 1);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(d.late_baro == 1, "baro posterior a IMU ya entregadas: tardío, rechazado (%u)", d.late_baro);
    CHECK(SPP_SERVICES_DATABANK_freeCount() == K_SPP_DATABANK_SIZE, "pool íntegro");
    return finish("tardio");
}

/* H: despegue con 3.5 g a través del router */
static int sc_liftoff(void)
{
    setup(1.0);
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    SPP_CORE_init();
    g_lift_t = 13.0;
    host_loop(14.0, 0.001, 1);
    eskf_svc_output_t o;
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.nav.phase == NAV_PHASE_FLIGHT && o.svc_validity != NAV_INVALID && o.nav.v[2] < -10.0f,
          "despegue: fase %u, validez %u, v_z %.1f m/s", o.nav.phase, o.svc_validity, (double)o.nav.v[2]);
    CHECK(o.bb.valid && o.bb.ref_frozen && o.bb.vz_mps > 10.0f, "respaldo: referencia congelada, v_z %.1f m/s hacia arriba",
          (double)o.bb.vz_mps);
    CHECK(SPP_SERVICES_ESKF_requestSessionRestart() == K_SPP_OK, "petición de reinicio en vuelo");
    host_loop(14.1, 0.001, 1);
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    CHECK(o.session == 1 && d.restarts_refused == 1 && o.nav.phase == NAV_PHASE_FLIGHT, "rechazada: sin vuelta a RAMPA");
    return finish("despegue");
}

/* F: silencio total de IMU y BMP con el router en marcha (consumeData sigue llamándose): la rampa
   caduca por el reloj; un aviso de referencia durante el silencio no abre sesión; la transición
   queda en la traza (con directorio: entradas, salida y traza para reproducirla en el PC). */
static int sc_silence(void)
{
    setup(1.0);
    char path[512];
    if (g_dir) {
        snprintf(path, sizeof path, "%s/silencio_entradas.bin", g_dir);
        g_f_in = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/silencio_salida_real.bin", g_dir);
        g_f_out = fopen(path, "wb");
        snprintf(path, sizeof path, "%s/silencio_traza.bin", g_dir);
        g_f_tr = fopen(path, "wb");
        if (!g_f_in || !g_f_out || !g_f_tr) return 2;
    }
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&ICM, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BMP, &kb);
    eskf_svc_config_t c = bench();
    c.trace_enable = 1;
    SPP_SERVICES_ESKF_configure(&c);
    SPP_SERVICES_ESKF_register(ki, kb, &ko);
    g_eskf_kpid = ko.value;
    SPP_Kpid_t all = {.value = 0x001C};
    SPP_SERVICES_PUBSUB_registerConsumer(&LOG, all);
    SPP_CORE_init();
    g_drain = 1;
    g_silent = 0;
    host_loop(15.0, 0.001, 1);
    eskf_svc_output_t o;
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.svc_validity == NAV_VALID && o.phase == ESKF_SVC_PHASE_PAD, "rampa: VÁLIDA y rampa confirmada");
    g_silent = 1;
    host_loop(15.9, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.svc_validity == NAV_INVALID && o.phase == ESKF_SVC_PHASE_PAD,
          "0,9 s de silencio: ESKF NO VÁLIDO por antigüedad; rampa aún dentro de la latencia admitida");
    host_loop(16.2, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.phase == ESKF_SVC_PHASE_UNKNOWN && o.why_code == ESKF_SVC_WHY_PHASE_UNKNOWN,
          "1,2 s de silencio: fase INCIERTA por el reloj (why 0x%04x)", o.why_code);
    host_loop(17.0, 0.001, 1);
    CHECK(SPP_SERVICES_ESKF_notifyBaroRefChange() == K_SPP_OK, "aviso de cambio de referencia durante el silencio");
    host_loop(17.1, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.session == 1 && (o.svc_reasons & ESKF_SVC_R_BARO_REF_CHANGED) && o.phase == ESKF_SVC_PHASE_UNKNOWN,
          "el aviso conserva la sesión y deja el baro fuera (sesión %u)", o.session);
    g_silent = 0;
    host_loop(20.0, 0.001, 1);
    SPP_SERVICES_ESKF_getOutput(&o);
    CHECK(o.phase == ESKF_SVC_PHASE_UNKNOWN && o.svc_validity == NAV_INVALID && o.source == ESKF_SVC_SRC_NONE,
          "vuelven las muestras: sigue INCIERTA, sin fuente");
    if (g_dir) {
        CHECK(close_trace(), "traza cerrada (END)");
        fclose(g_f_in), fclose(g_f_out), fclose(g_f_tr);
        g_f_in = g_f_out = g_f_tr = NULL;
    }
    return finish("silencio");
}

int main(int argc, char **argv)
{
    static const struct {
        const char *name;
        int (*fn)(void);
    } SC[] = {{"orden_a", sc_order_a},
              {"orden_b", sc_order_b},
              {"config_invalida", sc_bad_config},
              {"estricto_pendiente", sc_flight_pending},
              {"congestion", sc_queue_full},
              {"reloj_avanza", sc_clock_advance},
              {"pool_vacio", sc_pool_empty},
              {"registro_parcial", sc_partial},
              {"consumidores_llenos", sc_consumers_full},
              {"prioridad", sc_priority},
              {"tardio", sc_late},
              {"despegue", sc_liftoff},
              {"silencio", sc_silence}};
    if (argc < 2) {
        for (size_t i = 0; i < sizeof SC / sizeof SC[0]; i++) printf("%s\n", SC[i].name);
        return 0;
    }
    if (argc > 2) g_dir = argv[2];
    for (size_t i = 0; i < sizeof SC / sizeof SC[0]; i++)
        if (!strcmp(argv[1], SC[i].name)) return SC[i].fn();
    printf("escenario desconocido: %s\n", argv[1]);
    return 2;
}
