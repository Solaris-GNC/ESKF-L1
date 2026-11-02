// El anfitrión puede ser C++ (Arduino-ESP32): las cabeceras públicas deben compilarse desde C++
// con enlace C, también la del pegamento con los tipos reales de SPP. Sólo se compila.
#include "eskf_service.h"
#include "eskf_svc_msg.h"
#include "eskf_svc_params.h"
#include "spp_services_eskf.h"

extern "C" int cpp_header_check(void);
int cpp_header_check(void)
{
    eskf_svc_config_t c;
    if (eskf_svc_default_config(ESKF_SVC_PROFILE_ACQ, &c) != 0) return -1;
    if (SPP_SERVICES_ESKF_configure(&c) != K_SPP_OK) return -2;
    eskf_svc_output_t o;
    return SPP_SERVICES_ESKF_getOutput(&o) == K_SPP_OK ? 0 : 1;
}
