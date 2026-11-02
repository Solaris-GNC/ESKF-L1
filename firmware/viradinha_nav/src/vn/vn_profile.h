/*
 * vn_profile.h — perfil de compilación. Cambiar SÓLO esta línea (o definir VN_PROFILE con
 * -DVN_PROFILE=... en build_opt.h).
 *
 *   VN_PROFILE_ACQ     adquisición y calibración: registra todo lo crudo; sin navegación.
 *   VN_PROFILE_BENCH   banco: navegación con valores PROVISIONALES. NO VOLAR con este perfil;
 *                      la cabecera y un texto periódico del registro lo indican.
 *   VN_PROFILE_FLIGHT  vuelo: sólo valores confirmados/propuestos. Mientras quede algo pendiente,
 *                      la navegación no arranca (el registro crudo sí).
 */
#ifndef VN_PROFILE_H
#define VN_PROFILE_H

#include "vn_params.h"

#ifndef VN_PROFILE
#define VN_PROFILE VN_PROFILE_ACQ
#endif

#if (VN_PROFILE != VN_PROFILE_ACQ) && (VN_PROFILE != VN_PROFILE_BENCH) && (VN_PROFILE != VN_PROFILE_FLIGHT)
#error "VN_PROFILE debe ser VN_PROFILE_ACQ, VN_PROFILE_BENCH o VN_PROFILE_FLIGHT"
#endif

#define VN_FW_VERSION "vn-0.4.0"

#endif
