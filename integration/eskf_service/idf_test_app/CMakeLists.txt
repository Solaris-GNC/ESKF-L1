# Aplicación MÍNIMA de comprobación de compilación y enlace (no es el firmware del anfitrión):
# compila SPP sin cambios (components/spp) y el componente eskf_service.
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_CURRENT_LIST_DIR}/components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(eskf_service_build_check)
