# Componente «spp» SÓLO para la aplicación de comprobación: compila el núcleo de Solaris Packet
# Protocol (commit 9247d9e) tal cual desde SPP_ROOT, sin copiarlo ni modificarlo. En el anfitrión
# real este componente lo aporta Solaris Software.
#   SPP_ROOT      checkout de solaris-packet-protocol
#   SPP_INC_ROOT  carpeta que contiene «spp» → SPP_ROOT (raíz de inclusión de SPP)
set(SPP_ROOT "$ENV{SPP_ROOT}")
set(SPP_INC_ROOT "$ENV{SPP_INC_ROOT}")
if(NOT SPP_ROOT OR NOT SPP_INC_ROOT)
    message(FATAL_ERROR "Definir SPP_ROOT y SPP_INC_ROOT (ver integration/eskf_service/Makefile, objetivo idf)")
endif()
idf_component_register(
    SRCS "${SPP_ROOT}/core/core.c"
         "${SPP_ROOT}/core/commonbit.c"
         "${SPP_ROOT}/core/pubsub/pubsub.c"
         "${SPP_ROOT}/services/databank/databank.c"
         "${SPP_ROOT}/services/log/log.c"
         "${SPP_ROOT}/util/crc.c"
         "${SPP_ROOT}/hal/hal.c"
         "${SPP_ROOT}/hal/time/time.c"
         "${SPP_ROOT}/hal/spi/spi.c"
         "${SPP_ROOT}/ports/hal/esp32/time/timePort.c"
    INCLUDE_DIRS "${SPP_INC_ROOT}" "${SPP_ROOT}/ports/hal/esp32/time"
    REQUIRES esp_timer)
# SPP se compila como lo publica su autor: sus avisos no se convierten en errores aquí.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
