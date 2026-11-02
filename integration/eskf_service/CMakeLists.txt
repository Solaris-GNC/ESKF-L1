# Componente ESP-IDF «eskf_service»: Solaris ESKF como servicio dual de Solaris Packet Protocol.
# Reutiliza el núcleo de c/ SIN copiarlo (una sola instancia por ejecutable). Necesita un
# componente que exporte las cabeceras REALES de SPP con la raíz «spp/…» (por defecto «spp»;
# el nombre del componente SPP del anfitrión se fija con ESKF_SPP_COMPONENT).
set(ESKF_C_DIR "${CMAKE_CURRENT_LIST_DIR}/../../c")
if(NOT DEFINED ESKF_SPP_COMPONENT)
    set(ESKF_SPP_COMPONENT "spp")
endif()

idf_component_register(
    SRCS "src/eskf_service.c"
         "src/eskf_svc_params.c"
         "src/eskf_svc_msg.c"
         "src/spp_services_eskf.c"
         "${ESKF_C_DIR}/eskf.c"
         "${ESKF_C_DIR}/navigation.c"
         "${ESKF_C_DIR}/nav_events.c"
    INCLUDE_DIRS "include" "${ESKF_C_DIR}"
    REQUIRES freertos esp_timer ${ESKF_SPP_COMPONENT})

# float en TODAS las unidades que incluyan las cabeceras (PUBLIC; eskf_service.h da #error si falta).
target_compile_definitions(${COMPONENT_LIB} PUBLIC ESKF_USE_FLOAT)
# Avisos estrictos en el código propio y el núcleo; nunca -ffast-math; sin contracción FMA (misma
# aritmética que en el PC para la comparación de la traza).
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wshadow -Wdouble-promotion -Werror
                       -fno-fast-math -ffp-contract=off)
