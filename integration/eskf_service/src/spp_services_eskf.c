/*
 * spp_services_eskf.c — callbacks del servicio dual para el router SPP REAL (commit 9247d9e).
 *
 * Hechos del router respetados (core/pubsub/pubsub.c):
 *   - KPID por orden de registro (1 << (n + 2)); suscripción = máscara aparte de registerConsumer.
 *   - init de consumidores y luego de productores: este servicio recibe init DOS veces (una por
 *     contrato, con funciones distintas para saber cuál se llamó).
 *   - sendToMailbox reparte UN paquete por callConsumers, por valor; después consumeData(NULL).
 *   - publish copia el paquete y lo devuelve SIEMPRE al pool, también si falla.
 *   - acquireData(kpid) con el KPID asignado; los callbacks se llaman desde la tarea del router.
 */
#include "spp_services_eskf.h"

#include <stddef.h>
#include <string.h>

#include "spp/core/packet.h"
#include "spp/core/pubsub/pubsub.h"
#include "spp/hal/time/time.h"
#include "spp/services/databank/databank.h"

/* ---------------------------------------------------------------- comprobaciones de compilación */
_Static_assert(sizeof(SPP_Packet_t) == ESKF_SVC_PKT_SIZE, "SPP_Packet_t debe ocupar 64 bytes");
_Static_assert(offsetof(SPP_Packet_t, primaryHeader.version) == ESKF_SVC_OFF_VERSION, "version");
_Static_assert(offsetof(SPP_Packet_t, primaryHeader.apid) == ESKF_SVC_OFF_APID, "apid");
_Static_assert(offsetof(SPP_Packet_t, primaryHeader.seq) == ESKF_SVC_OFF_SEQ, "seq");
_Static_assert(offsetof(SPP_Packet_t, primaryHeader.payloadLen) == ESKF_SVC_OFF_LEN, "payloadLen");
_Static_assert(offsetof(SPP_Packet_t, secondaryHeader.timestampMs) == ESKF_SVC_OFF_TS, "timestampMs");
_Static_assert(offsetof(SPP_Packet_t, secondaryHeader.dropCounter) == ESKF_SVC_OFF_DROP, "dropCounter");
_Static_assert(offsetof(SPP_Packet_t, payload) == ESKF_SVC_OFF_PAYLOAD, "payload");
_Static_assert(offsetof(SPP_Packet_t, crc) == ESKF_SVC_OFF_CRC, "crc");
_Static_assert(K_SPP_PKT_PAYLOAD_MAX == ESKF_SVC_PAYLOAD_MAX, "payload máximo");
_Static_assert(K_SPP_PKT_VERSION == ESKF_SVC_PKT_VERSION, "versión de paquete");
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "el formato en memoria es el de la línea: little-endian");
_Static_assert(K_SPP_ESKF_CONSUMER_PRIO >= 0 && K_SPP_ESKF_CONSUMER_PRIO <= 1,
               "prioridad 0..1: negativa rompe la comparación unsigned; > 1 la interrumpe callConsumers");
_Static_assert(K_SPP_ESKF_OUT_QUEUE_LIMIT > 0 && K_SPP_ESKF_OUT_QUEUE_LIMIT < K_SPP_SERVICES_PUBSUB_BUFFER_SIZE,
               "límite de cola");

static SPP_RetVal_t sppEskf_initConsumer(void);
static SPP_RetVal_t sppEskf_initProducer(void);
static SPP_RetVal_t sppEskf_deliverToMailbox(const SPP_Packet_t packet);
static SPP_RetVal_t sppEskf_consumeData(void *p_data);
static SPP_RetVal_t sppEskf_acquireData(SPP_Kpid_t kpid);

/* Firmas compatibles con los punteros reales del contrato (init y acquireData usan «()»). */
_Static_assert(__builtin_types_compatible_p(__typeof__(&sppEskf_deliverToMailbox),
                                            __typeof__(((SPP_SERVICE_ConsumerContract_t *)0)->deliverToMailbox)),
               "deliverToMailbox: paquete por valor");
_Static_assert(__builtin_types_compatible_p(__typeof__(&sppEskf_consumeData),
                                            __typeof__(((SPP_SERVICE_ConsumerContract_t *)0)->consumeData)),
               "consumeData(void *)");
_Static_assert(__builtin_types_compatible_p(__typeof__(&sppEskf_initConsumer),
                                            __typeof__(((SPP_SERVICE_ConsumerContract_t *)0)->init)),
               "init del consumidor");
_Static_assert(__builtin_types_compatible_p(__typeof__(&sppEskf_initProducer),
                                            __typeof__(((SPP_SERVICE_ProducerContract_t *)0)->init)),
               "init del productor");
_Static_assert(__builtin_types_compatible_p(__typeof__(&sppEskf_acquireData),
                                            __typeof__(((SPP_SERVICE_ProducerContract_t *)0)->acquireData)),
               "acquireData(SPP_Kpid_t)");

/* ---------------------------------------------------------------- estado (estático) */
static eskf_svc_t s_svc;
static eskf_svc_config_t s_cfg;
static spp_bool_t s_configured = false;
static spp_bool_t s_kpidsSet = false;
static spp_uint16_t s_kpidImu, s_kpidBaro, s_kpidOut;
static spp_bool_t s_initDone = false;
static SPP_RetVal_t s_initResult = K_SPP_ERROR_NOT_INITIALIZED;
static eskf_svc_clock_fn s_clockFn = NULL;
static void *s_clockCtx = NULL;
static spp_uint16_t s_outSeq = 0U;
static eskf_svc_config_t s_probe; /* comprobaciones previas a init: estático, no en la pila */
static spp_uint8_t s_reg = 0U;    /* SPP_SERVICES_ESKF_REG_* */
static spp_uint8_t s_initCalls = 0U; /* bit 0: init del consumidor; bit 1: init del productor */

/* Reloj por defecto: la MISMA fuente que packetData usa para timestampMs, extendida a 64 bits */
static spp_uint32_t s_clkLast = 0U;
static spp_uint64_t s_clkHigh = 0U;
static int64_t sppEskf_clockUs(void *p_ctx)
{
    (void)p_ctx;
    spp_uint32_t ms = SPP_HAL_TIME_getTimeMs();
    if (ms < s_clkLast) s_clkHigh += (spp_uint64_t)1U << 32;
    s_clkLast = ms;
    return (int64_t)((s_clkHigh + ms) * 1000U);
}

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
/* Proveedor ESP-IDF (µs). Misma época que timestampMs SÓLO si el puerto SPP es ports/hal/esp32
   (getTimeMs = esp_timer_get_time()/1000); el servicio lo vigila con diag.clock_mismatch. */
int64_t SPP_SERVICES_ESKF_clockEspTimer(void *p_ctx)
{
    (void)p_ctx;
    return esp_timer_get_time();
}
#endif

static const SPP_SERVICE_ConsumerContract_t s_consumer = {
    .priority = (spp_int8_t)K_SPP_ESKF_CONSUMER_PRIO,
    .p_nameConsumer = "eskf",
    .tiemoutMs = K_SPP_ESKF_TIMEOUT_MS,
    .init = sppEskf_initConsumer,
    .deliverToMailbox = sppEskf_deliverToMailbox,
    .consumeData = sppEskf_consumeData,
};

static const SPP_SERVICE_ProducerContract_t s_producer = {
    .p_nameProducer = "eskf",
    .tiemoutMs = K_SPP_ESKF_TIMEOUT_MS,
    .init = sppEskf_initProducer,
    .acquireData = sppEskf_acquireData,
};

/* ---------------------------------------------------------------- API pública */
const SPP_SERVICE_ConsumerContract_t *SPP_SERVICES_ESKF_getConsumerContract(void) { return &s_consumer; }
const SPP_SERVICE_ProducerContract_t *SPP_SERVICES_ESKF_getProducerContract(void) { return &s_producer; }

static void applyKpids(void)
{
    if (!s_kpidsSet) return;
    s_cfg.kpid_imu = s_kpidImu;
    s_cfg.kpid_baro = s_kpidBaro;
    s_cfg.kpid_out = s_kpidOut;
}

SPP_RetVal_t SPP_SERVICES_ESKF_configure(const eskf_svc_config_t *p_cfg)
{
    if (p_cfg == NULL) return K_SPP_ERROR_NULL_POINTER;
    if (s_initDone) return K_SPP_ERROR_ALREADY_INITIALIZED;
    s_cfg = *p_cfg;
    s_configured = true;
    applyKpids();
    return K_SPP_OK;
}

SPP_RetVal_t SPP_SERVICES_ESKF_setKpids(SPP_Kpid_t imu, SPP_Kpid_t baro, SPP_Kpid_t out)
{
    if (s_initDone) return K_SPP_ERROR_ALREADY_INITIALIZED;
    if (!s_configured) (void)eskf_svc_default_config(ESKF_SVC_PROFILE_ACQ, &s_probe);
    else s_probe = s_cfg;
    s_probe.kpid_imu = imu.value;
    s_probe.kpid_baro = baro.value;
    s_probe.kpid_out = out.value;
    if (eskf_svc_check_transport(&s_probe) != NULL) return K_SPP_ERROR_INVALID_PARAMETER;
    s_kpidImu = imu.value;
    s_kpidBaro = baro.value;
    s_kpidOut = out.value;
    s_kpidsSet = true;
    if (s_configured) applyKpids();
    return K_SPP_OK;
}

SPP_RetVal_t SPP_SERVICES_ESKF_setClock(eskf_svc_clock_fn fn, void *p_ctx)
{
    if (fn == NULL) return K_SPP_ERROR_NULL_POINTER;
    if (s_initDone) return K_SPP_ERROR_ALREADY_INITIALIZED;
    s_clockFn = fn;
    s_clockCtx = p_ctx;
    return K_SPP_OK;
}

SPP_Kpid_t SPP_SERVICES_ESKF_getSubscription(void)
{
    SPP_Kpid_t k;
    if (s_configured) s_probe = s_cfg;
    else (void)eskf_svc_default_config(ESKF_SVC_PROFILE_ACQ, &s_probe);
    if (s_kpidsSet) s_probe.kpid_imu = s_kpidImu, s_probe.kpid_baro = s_kpidBaro, s_probe.kpid_out = s_kpidOut;
    k.value = eskf_svc_subscription(&s_probe); /* nunca incluye kpid_out */
    return k;
}

/* Registro: PRIMERO el consumidor (con imu | baro; la salida aún no tiene bit, así que no puede
   coincidir) y DESPUÉS el productor. Si el productor no cabe, el servicio queda registrado sólo
   como consumidor: navega y responde a las consultas, sin salida SPP (kpid_out = 0), y se
   devuelve el error. Un segundo intento no vuelve a registrar nada. */
SPP_RetVal_t SPP_SERVICES_ESKF_register(SPP_Kpid_t imu, SPP_Kpid_t baro, SPP_Kpid_t *p_out)
{
    if (s_initDone) return K_SPP_ERROR_ALREADY_INITIALIZED;
    if (s_reg != SPP_SERVICES_ESKF_REG_NONE) return K_SPP_ERROR_ALREADY_INITIALIZED;
    SPP_Kpid_t none = {0};
    SPP_RetVal_t ret = SPP_SERVICES_ESKF_setKpids(imu, baro, none);
    if (ret != K_SPP_OK) return ret;
    SPP_Kpid_t sub = SPP_SERVICES_ESKF_getSubscription();
    if (sub.value == 0U) return K_SPP_ERROR_INVALID_PARAMETER;
    ret = SPP_SERVICES_PUBSUB_registerConsumer(&s_consumer, sub);
    if (ret != K_SPP_OK) return ret; /* nada registrado */
    s_reg = SPP_SERVICES_ESKF_REG_CONSUMER;
    SPP_Kpid_t out = {0};
    ret = SPP_SERVICES_PUBSUB_registerProducer(&s_producer, &out);
    if (ret != K_SPP_OK) return ret; /* parcial: consumidor sin salida (documentado) */
    ret = SPP_SERVICES_ESKF_setKpids(imu, baro, out);
    if (ret != K_SPP_OK) return ret; /* bit de salida incoherente: el productor no publicará */
    s_reg = SPP_SERVICES_ESKF_REG_BOTH;
    if (p_out != NULL) *p_out = out;
    return K_SPP_OK;
}

spp_uint8_t SPP_SERVICES_ESKF_getRegistration(void) { return s_reg; }
spp_uint8_t SPP_SERVICES_ESKF_getInitCalls(void) { return s_initCalls; }

SPP_RetVal_t SPP_SERVICES_ESKF_getOutput(eskf_svc_output_t *p_out)
{
    if (p_out == NULL) return K_SPP_ERROR_NULL_POINTER;
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    int r = eskf_svc_get_output(&s_svc, p_out);
    return (r == ESKF_SVC_OK) ? K_SPP_OK : K_SPP_ERROR_TIMEOUT;
}

SPP_RetVal_t SPP_SERVICES_ESKF_getDiagnostics(eskf_svc_diag_t *p_diag)
{
    if (p_diag == NULL) return K_SPP_ERROR_NULL_POINTER;
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    int r = eskf_svc_get_diag(&s_svc, p_diag);
    return (r == ESKF_SVC_OK) ? K_SPP_OK : K_SPP_ERROR_TIMEOUT;
}

SPP_RetVal_t SPP_SERVICES_ESKF_requestSessionRestart(void)
{
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    eskf_svc_request_restart(&s_svc); /* en vuelo se rechaza (diag.restarts_refused) */
    return K_SPP_OK;
}

SPP_RetVal_t SPP_SERVICES_ESKF_notifyBaroRefChange(void)
{
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    eskf_svc_notify_baro_ref_change(&s_svc);
    return K_SPP_OK;
}

spp_uint32_t SPP_SERVICES_ESKF_readTrace(eskf_svc_trec_t *p_dst, spp_uint32_t n)
{
    if ((p_dst == NULL) || (s_initResult != K_SPP_OK)) return 0U;
    return eskf_svc_trace_read(&s_svc, p_dst, n);
}

SPP_RetVal_t SPP_SERVICES_ESKF_closeTrace(void)
{
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    int r = eskf_svc_trace_close(&s_svc);
    if (r == ESKF_SVC_OK) return K_SPP_OK;
    return (r == ESKF_SVC_E_FULL) ? K_SPP_ERROR : K_SPP_ERROR_INVALID_PARAMETER;
}

SPP_RetVal_t SPP_SERVICES_ESKF_getInitResult(void) { return s_initResult; }

/* ---------------------------------------------------------------- callbacks del router */
static SPP_RetVal_t sppEskf_initCommon(void)
{
    if (s_initDone) return s_initResult; /* segunda llamada (contrato dual): no toca nada */
    s_initDone = true;
    if (!s_configured) {
        if (eskf_svc_default_config(ESKF_SVC_PROFILE_ACQ, &s_cfg) != 0) {
            s_initResult = K_SPP_ERROR;
            return s_initResult;
        }
        s_configured = true;
    }
    applyKpids();
    if (s_reg == SPP_SERVICES_ESKF_REG_CONSUMER) s_cfg.kpid_out = 0U; /* registro parcial: sin salida */
    if (s_clockFn == NULL) s_clockFn = sppEskf_clockUs;
    /* Transporte válido → OK aunque la navegación esté pendiente: se publica el motivo. */
    int r = eskf_svc_ensure_init(&s_svc, &s_cfg, s_clockFn, s_clockCtx);
    s_initResult = (r >= 0) ? K_SPP_OK : K_SPP_ERROR_INVALID_PARAMETER;
    return s_initResult;
}

static SPP_RetVal_t sppEskf_initConsumer(void)
{
    s_initCalls |= 1U;
    return sppEskf_initCommon();
}

static SPP_RetVal_t sppEskf_initProducer(void)
{
    s_initCalls |= 2U;
    return sppEskf_initCommon();
}

static SPP_RetVal_t sppEskf_deliverToMailbox(const SPP_Packet_t packet)
{
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    spp_uint8_t raw[ESKF_SVC_PKT_SIZE];
    memcpy(raw, &packet, sizeof raw); /* la copia del router es nuestra; no se guarda ningún puntero */
    int r = eskf_svc_deliver_packet(&s_svc, raw);
    if (r == ESKF_SVC_OK) return K_SPP_OK;
    return (r == ESKF_SVC_E_FULL) ? K_SPP_ERROR : K_SPP_ERROR_INVALID_PARAMETER;
}

static SPP_RetVal_t sppEskf_consumeData(void *p_data)
{
    (void)p_data; /* el router pasa NULL */
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    (void)eskf_svc_consume(&s_svc);
    return K_SPP_OK;
}

static SPP_RetVal_t sppEskf_acquireData(SPP_Kpid_t kpid)
{
    if (s_initResult != K_SPP_OK) return K_SPP_ERROR_NOT_INITIALIZED;
    if (kpid.value != s_svc.cfg.kpid_out || kpid.value == 0U) {
        s_svc.diag.kpid_mismatch++; /* nunca se publica con un identificador distinto del asignado */
        return K_SPP_ERROR_INVALID_PARAMETER;
    }
    for (spp_uint32_t k = 0U; k < (spp_uint32_t)K_SPP_ESKF_OUT_PER_CALL; k++) {
        if (!eskf_svc_pending(&s_svc)) break;
        if (SPP_SERVICES_PUBSUB_queueDepth() >= (spp_uint8_t)K_SPP_ESKF_OUT_QUEUE_LIMIT) {
            /* deja sitio a los sensores; el mensaje sigue pendiente */
            eskf_svc_note_output(&s_svc, ESKF_SVC_OUT_DEFER_QUEUE, 0U, 0U, 0U, 0U);
            break;
        }
        SPP_Packet_t *p_pkt = SPP_SERVICES_DATABANK_getPacket();
        if (p_pkt == NULL) {
            eskf_svc_note_output(&s_svc, ESKF_SVC_OUT_DEFER_POOL, 0U, 0U, 0U, 0U);
            break;
        }
        spp_uint8_t buf[ESKF_SVC_PAYLOAD_MAX];
        spp_uint16_t len = 0U;
        (void)eskf_svc_produce(&s_svc, buf, &len);
        spp_uint8_t type = (len > 1U) ? buf[1] : 0U;
        spp_uint16_t seq = s_outSeq++;
        SPP_RetVal_t r = SPP_SERVICES_DATABANK_packetData(p_pkt, kpid.value, seq, buf, len);
        if (r != K_SPP_OK) {
            (void)SPP_SERVICES_DATABANK_returnPacket(p_pkt);
            eskf_svc_note_output(&s_svc, ESKF_SVC_OUT_PACKET_FAIL, type, len, seq, 0U);
            break;
        }
        /* cabecera REAL que construyó packetData (su timestampMs), leída antes de publicar: después
           el paquete vuelve al pool */
        spp_uint32_t tsMs = p_pkt->secondaryHeader.timestampMs;
        /* publish devuelve el paquete al pool aunque falle: no se vuelve a devolver aquí */
        r = SPP_SERVICES_PUBSUB_publish(p_pkt);
        eskf_svc_note_output(&s_svc, (r == K_SPP_OK) ? ESKF_SVC_OUT_PUBLISHED : ESKF_SVC_OUT_PUB_FAIL, type, len, seq,
                             tsMs);
    }
    return K_SPP_OK;
}
