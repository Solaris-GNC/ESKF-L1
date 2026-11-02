/*
 * spp_services_eskf.h — Solaris ESKF como servicio DUAL (consumidor + productor) de Solaris
 * Packet Protocol, contra las cabeceras REALES de SPP (commit 9247d9e). Requiere que la raíz de
 * inclusión contenga «spp/» (como en el CMakeLists.txt de SPP).
 *
 * Uso en el anfitrión (antes de SPP_CORE_init):
 *
 *   SPP_Kpid_t icm, bmp;
 *   SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_ICM20948_getProducerContract(), &icm);
 *   SPP_SERVICES_PUBSUB_registerProducer(SPP_SERVICES_BMP390_getProducerContract(), &bmp);
 *   eskf_svc_config_t cfg; eskf_svc_default_config(ESKF_SVC_PROFILE_ACQ, &cfg);
 *   SPP_SERVICES_ESKF_configure(&cfg);
 *   eskf_svc_output_preset(&cfg, ESKF_OUT_LOG);   // o ESKF_OUT_TELEMETRY (sólo compacta)
 *   SPP_SERVICES_ESKF_register(icm, bmp, &out); // consumidor y después productor (KPID asignado)
 *   ...                                      // otros servicios (datalogger, telemetría)
 *   SPP_CORE_init();                         // llama a init de consumidores y productores
 *
 * El servicio no inicia drivers, buses, interrupciones, tareas ni escritores de SD: sólo
 * responde a los callbacks que el router llama desde su tarea.
 */
#ifndef SPP_SERVICES_ESKF_H
#define SPP_SERVICES_ESKF_H

#include "spp/core/returnTypes.h"
#include "spp/core/types.h"
#include "spp/services/kpid.h"
#include "spp/services/service.h"

#include "eskf_service.h"
#include "eskf_svc_params.h" /* perfiles, tabla y preajustes de salida (eskf_svc_output_preset) */

#ifdef __cplusplus
extern "C" {
#endif

/* Prioridad del consumidor. callConsumers interrumpe los de prioridad > 1 cuando un productor
   ha señalado datos; con 1 el ESKF no se salta. No usar negativas: la comparación con la
   constante unsigned del router las trataría como enormes. */
#ifndef K_SPP_ESKF_CONSUMER_PRIO
#define K_SPP_ESKF_CONSUMER_PRIO 1
#endif
/* Producción acotada: como mucho K_SPP_ESKF_OUT_PER_CALL paquetes por acquireData y sólo si la
   cola global del router tiene menos de K_SPP_ESKF_OUT_QUEUE_LIMIT paquetes (de 128). */
#ifndef K_SPP_ESKF_OUT_PER_CALL
#define K_SPP_ESKF_OUT_PER_CALL 1
#endif
#ifndef K_SPP_ESKF_OUT_QUEUE_LIMIT
#define K_SPP_ESKF_OUT_QUEUE_LIMIT 32
#endif
#define K_SPP_ESKF_TIMEOUT_MS 1000U

const SPP_SERVICE_ConsumerContract_t *SPP_SERVICES_ESKF_getConsumerContract(void);
const SPP_SERVICE_ProducerContract_t *SPP_SERVICES_ESKF_getProducerContract(void);

/* Configuración previa a la inicialización (K_SPP_ERROR_ALREADY_INITIALIZED después). Los KPID
   de cfg se ignoran si ya se fijaron con setKpids/register. */
SPP_RetVal_t SPP_SERVICES_ESKF_configure(const eskf_svc_config_t *p_cfg);
SPP_RetVal_t SPP_SERVICES_ESKF_setKpids(SPP_Kpid_t imu, SPP_Kpid_t baro, SPP_Kpid_t out);
/* Reloj (antes de init). Por defecto: SPP_HAL_TIME_getTimeMs extendido a 64 bits (misma fuente
   que timestampMs, resolución 1 ms). */
SPP_RetVal_t SPP_SERVICES_ESKF_setClock(eskf_svc_clock_fn fn, void *p_ctx);
#if defined(ESP_PLATFORM)
/* Alternativa ESP-IDF (esp_timer, µs): misma época sólo con el puerto ESP32 de SPP. */
int64_t SPP_SERVICES_ESKF_clockEspTimer(void *p_ctx);
#endif
/* Suscripción para registerConsumer: imu | baro, nunca la salida propia. value = 0 si incoherente. */
SPP_Kpid_t SPP_SERVICES_ESKF_getSubscription(void);
/* Registra PRIMERO el consumidor (suscripción imu | baro) y DESPUÉS el productor (recibe su KPID).
   Si el consumidor no cabe no se registra nada. Si el productor no cabe, queda sólo el consumidor
   (navegación y consultas sin salida SPP) y se devuelve el error del router. Una segunda llamada
   devuelve K_SPP_ERROR_ALREADY_INITIALIZED sin tocar el router. */
#define SPP_SERVICES_ESKF_REG_NONE 0U
#define SPP_SERVICES_ESKF_REG_CONSUMER 1U /* parcial: sin salida */
#define SPP_SERVICES_ESKF_REG_BOTH 2U
SPP_RetVal_t SPP_SERVICES_ESKF_register(SPP_Kpid_t imu, SPP_Kpid_t baro, SPP_Kpid_t *p_out);
spp_uint8_t SPP_SERVICES_ESKF_getRegistration(void);
/* Bits de los init recibidos del router: 1 consumidor, 2 productor. */
spp_uint8_t SPP_SERVICES_ESKF_getInitCalls(void);

/* Consultas (cualquier tarea) y control */
SPP_RetVal_t SPP_SERVICES_ESKF_getOutput(eskf_svc_output_t *p_out);
SPP_RetVal_t SPP_SERVICES_ESKF_getDiagnostics(eskf_svc_diag_t *p_diag);
/* Reinicio de sesión explícito: se ejecuta en el siguiente consumeData; en vuelo se RECHAZA. */
SPP_RetVal_t SPP_SERVICES_ESKF_requestSessionRestart(void);
/* Cambio de la referencia del BMP390 confirmado por el anfitrión (p. ej. si avionica repite su
   calibración de p0): en rampa abre sesión nueva; en vuelo deja de usar el baro en el ESKF e
   invalida el respaldo. No se deduce nunca de las muestras. */
SPP_RetVal_t SPP_SERVICES_ESKF_notifyBaroRefChange(void);
/* Traza de validación (sólo desde la tarea del router). Devuelve cuántos registros copió. */
spp_uint32_t SPP_SERVICES_ESKF_readTrace(eskf_svc_trec_t *p_dst, spp_uint32_t n);
/* Cierre verificable de la traza (registro END con nº de registros y CRC), desde la tarea del router
   al terminar la captura: K_SPP_OK; K_SPP_ERROR si no cabe (vaciar con readTrace y repetir);
   K_SPP_ERROR_INVALID_PARAMETER si la traza no está activada. Sin cierre, la traza no prueba que
   esté completa y svc_replay sólo la reproduce con --parcial. */
SPP_RetVal_t SPP_SERVICES_ESKF_closeTrace(void);
/* Resultado de la (única) inicialización real; K_SPP_ERROR_NOT_INITIALIZED si aún no. */
SPP_RetVal_t SPP_SERVICES_ESKF_getInitResult(void);

#ifdef __cplusplus
}
#endif

#endif
