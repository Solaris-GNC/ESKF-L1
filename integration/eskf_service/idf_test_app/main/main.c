/* Comprobación de COMPILACIÓN y ENLACE en ESP-IDF con el router real de SPP. No es el anfitrión:
   registra dos productores vacíos en lugar del ICM y del BMP (no se tocan buses, GPIO ni tareas),
   registra el ESKF, inicia SPP y da unas vueltas al router. No se ha ejecutado en placa. */
#include <stdio.h>

#include "spp/core/core.h"
#include "spp/core/pubsub/pubsub.h"
#include "spp_services_eskf.h"

static SPP_RetVal_t stub_init(void) { return K_SPP_OK; }
static SPP_RetVal_t stub_acquire(SPP_Kpid_t kpid)
{
    (void)kpid;
    return K_SPP_OK;
}
static const SPP_SERVICE_ProducerContract_t IMU = {"imu_vacio", 1000U, stub_init, stub_acquire};
static const SPP_SERVICE_ProducerContract_t BARO = {"baro_vacio", 1000U, stub_init, stub_acquire};

void app_main(void)
{
    SPP_Kpid_t ki, kb, ko;
    SPP_SERVICES_PUBSUB_registerProducer(&IMU, &ki);
    SPP_SERVICES_PUBSUB_registerProducer(&BARO, &kb);
    eskf_svc_config_t cfg;
    eskf_svc_default_config(ESKF_SVC_PROFILE_EXP, &cfg); /* perfil operativo experimental */
    eskf_svc_output_preset(&cfg, ESKF_OUT_LOG);
    cfg.boot_id = 1U;
    SPP_SERVICES_ESKF_configure(&cfg);
    SPP_RetVal_t r = SPP_SERVICES_ESKF_register(ki, kb, &ko);
    SPP_RetVal_t c = SPP_CORE_init();
    for (int i = 0; i < 3; i++) {
        SPP_SERVICES_PUBSUB_callProducers();
        SPP_SERVICES_PUBSUB_callConsumers();
    }
    (void)SPP_SERVICES_ESKF_notifyBaroRefChange(); /* en rampa: sesión nueva */
    (void)SPP_SERVICES_ESKF_requestSessionRestart();
    SPP_SERVICES_PUBSUB_callConsumers();
    eskf_svc_output_t o;
    eskf_svc_diag_t d;
    SPP_SERVICES_ESKF_getOutput(&o);
    SPP_SERVICES_ESKF_getDiagnostics(&d);
    printf("eskf_service: register %d core %d init %d (llamadas 0x%x) kpid out 0x%04x sesion %u caps 0x%02x consumos %u "
           "sizeof %u\n",
           (int)r, (int)c, (int)SPP_SERVICES_ESKF_getInitResult(), (unsigned)SPP_SERVICES_ESKF_getInitCalls(),
           (unsigned)ko.value, (unsigned)o.session, (unsigned)o.caps, (unsigned)d.consume_calls, (unsigned)sizeof(eskf_svc_t));
}
