/* eskf_svc_params.h — tabla única de configuración del servicio: valor, unidad, procedencia,
   estado por perfil y grupo de preparación para vuelo (eskf_svc_params.c). */
#ifndef ESKF_SVC_PARAMS_H
#define ESKF_SVC_PARAMS_H

#include <stddef.h>
#include <stdint.h>

#include "eskf_service.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Procedencia/estado de cada valor. CONFIRMADO: hecho del código SPP 9247d9e (no es calibración de
   nuestra placa). OBSERVADO: medido en un registro disponible (cuál, en la fila). SIMULADO: de la
   simulación OpenRocket/RocketPy adjunta. PROPUESTO: política de diseño. PROVISIONAL: valor razonable
   sin medida. PENDIENTE: sin valor (sólo perfil ESTRICTO; NaN). */
enum {
    ESKF_ST_CONFIRMED = 1, ESKF_ST_PROPOSED = 2, ESKF_ST_PROVISIONAL = 3, ESKF_ST_PENDING = 4,
    ESKF_ST_OBSERVED = 5, ESKF_ST_SIMULATED = 6
};
enum { ESKF_T_REAL = 0, ESKF_T_F64 = 1, ESKF_T_F32 = 2, ESKF_T_I32 = 3, ESKF_T_I64 = 4, ESKF_T_U32 = 5, ESKF_T_INT = 6 };

typedef struct {
    const char *name;
    const char *unit;
    uint8_t type;
    size_t off;           /* desplazamiento dentro de eskf_svc_config_t */
    double strict, exp; /* valor en ESTRICTO (NaN = pendiente) y en EXPERIMENTAL (siempre numérico) */
    uint8_t st_strict, st_exp;
    uint16_t group;    /* bit ESKF_SVC_RDY_* (0 = política sin efecto en la preparación) */
    uint8_t need_conf; /* 1: para dar el grupo por validado hace falta CONFIRMADO */
    const char *source;
} eskf_param_t;

const eskf_param_t *eskf_svc_params_table(size_t *n);
double eskf_svc_param_value(const eskf_param_t *p, int profile);
uint8_t eskf_svc_param_status(const eskf_param_t *p, int profile);
/* Valor actual del campo en una configuración (como double; NaN si pendiente). */
double eskf_svc_param_read(const eskf_svc_config_t *cfg, const eskf_param_t *p);
/* Rellena la configuración del perfil (ACQ toma la columna ESTRICTO) sin validar. KPID a 0. */
int eskf_svc_build_config(int profile, eskf_svc_config_t *cfg);
/* Ajustes de la compatibilidad HISTÓRICA (captura del 24-oct): ICM de 48 B sin offsets,
   altura ISA absoluta. Sólo para fixtures; no es el formato definitivo. */
void eskf_svc_config_historical(eskf_svc_config_t *cfg);
/* Bits de preparación (ESKF_SVC_RDY_*) del perfil según la tabla, más OVERRIDE si un valor con
   grupo de preparación difiere de la tabla (las políticas de salida no lo activan; sí config_id). */
uint16_t eskf_svc_readiness(int profile);
uint16_t eskf_svc_readiness_cfg(const eskf_svc_config_t *cfg);
/* Ajusta la salida: ESKF_OUT_LOG (estado completo para el registro) o ESKF_OUT_TELEMETRY (sólo
   la compacta y un diagnóstico lento, para un KPID que se envíe por radio). */
#define ESKF_OUT_LOG 0
#define ESKF_OUT_TELEMETRY 1
void eskf_svc_output_preset(eskf_svc_config_t *cfg, int preset);
/* Identificador de configuración: CRC-16 de los valores de la tabla leídos de cfg (igual en PC y placa). */
uint16_t eskf_svc_config_id(const eskf_svc_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
