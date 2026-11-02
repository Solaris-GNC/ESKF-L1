/*
 * vn_params.h — configuración CENTRALIZADA del firmware y del estimador.
 *
 * Una sola tabla (vn_params.c) con, para cada parámetro: valor de VUELO, valor del perfil
 * de BANCO, estado y origen. Perfiles:
 *   VN_PROFILE_ACQ    adquisición y registro; navegación desactivada.
 *   VN_PROFILE_BENCH  banco: usa valores PROVISIONALES para poder ejercitar la navegación
 *                     antes de medir. Nunca para volar: la cabecera del registro lo dice.
 *   VN_PROFILE_FLIGHT vuelo: sólo valores confirmados o propuestos; lo pendiente queda
 *                     NAV_PENDING y la navegación no arranca (el registro sí).
 * Estados (vn_logfmt.h): 1 confirmado, 2 propuesto, 3 provisional, 4 pendiente.
 */
#ifndef VN_PARAMS_H
#define VN_PARAMS_H

#include <stddef.h>
#include <stdint.h>

#include "../nav/imu_conv.h"
#include "../nav/navigation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* macros (no enum) para poder comprobarlas en el preprocesador (vn_profile.h) */
#define VN_PROFILE_ACQ 0
#define VN_PROFILE_BENCH 1
#define VN_PROFILE_FLIGHT 2

/* Dónde va cada parámetro: nav_config_t (incl. eskf_params_t), imu_conv_cfg_t, vn_hw_t o
   ninguno (VN_PK_DOC: dato de documentación que se registra pero no configura nada) */
enum { VN_PK_HW = 0, VN_PK_NAV = 1, VN_PK_IMUC = 2, VN_PK_DOC = 3 };
/* Tipo del campo de destino */
enum { VN_T_REAL = 0, VN_T_F64 = 1, VN_T_I32 = 2, VN_T_I64 = 3, VN_T_U32 = 4, VN_T_U8 = 5, VN_T_I8 = 6, VN_T_F32 = 7 };

typedef struct {
    const char *name;
    uint8_t kind;  /* VN_PK_* */
    uint8_t type;  /* VN_T_* */
    size_t off;    /* desplazamiento en la estructura de destino */
    double flight; /* valor de vuelo; NAN si pendiente */
    double bench;  /* valor del perfil de banco */
    uint8_t status;       /* estado del valor de VUELO */
    uint8_t bench_status; /* estado del valor de BANCO */
    const char *source;
} vn_param_t;

/* Parámetros del firmware (no del estimador) */
typedef struct {
    int32_t i2c_hz, pin_sda, pin_scl, pin_imu_int, pin_baro_int, imu_addr, baro_addr;
    int32_t imu_acc_div, imu_gyro_div, imu_dlpf_cfg, imu_acc_fs_g, imu_gyro_fs_dps, mag_div;
    int32_t baro_odr_hz, baro_osr_p, baro_osr_t;
    double baro_p0_pa;
    int64_t merge_holdoff_us;
    double navout_hz, telemetry_hz, stats_hz, header_period_s;
    int32_t log_navev, log_ticks;
    uint32_t stack_bytes, ring_bytes;
    int32_t queue_depth, i2c_timeout_ms, irq_timeout_ms, baro_poll_us, imu_reinit_errors, boot_bench;
    int32_t core_acq, core_nav, core_log, prio_acq, prio_nav, prio_log;
    int32_t axes_confirmed; /* 1 si el montaje de ejes está confirmado */
} vn_hw_t;

const vn_param_t *vn_params_table(size_t *n);
double vn_param_value(const vn_param_t *p, int profile);
uint8_t vn_param_status(const vn_param_t *p, int profile);

/* Construye la configuración del perfil. Devuelve 1 si la navegación puede arrancar; si no,
   *why apunta al motivo (perfil de adquisición, ejes sin confirmar o primer campo pendiente
   según nav_config_check). El firmware registra y continúa sin navegar. */
int vn_params_build(int profile, nav_config_t *cfg, imu_conv_cfg_t *ic, vn_hw_t *hw, const char **why);

const char *vn_profile_name(int profile);

#ifdef __cplusplus
}
#endif

#endif
