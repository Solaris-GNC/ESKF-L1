/*
 * vn_params.c — TABLA ÚNICA de parámetros con estado y origen. Para cambiar un valor:
 * editar aquí (y PARAMETROS.md), nunca en otros archivos.
 *
 * Estados: CONF confirmado (hardware o documentación), PROP propuesto sin comprobar,
 * PROV estimación provisional (sólo banco), PEND pendiente de medición/información.
 * Un valor de la hoja de datos (típico o tolerancia máxima) NO es una calibración de
 * nuestra unidad: aparece como documentación, no como medida.
 */
#include "vn_params.h"

#include "vn_logfmt.h"

#include <math.h>
#include <string.h>

#if (defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)) && !defined(ESKF_FORCE_DOUBLE)
VN_STATIC_ASSERT(sizeof(eskf_real) == 4, eskf_real_float_en_placa);
#endif
VN_STATIC_ASSERT(sizeof(int) == 4, int_32_bits); /* eskf.has_mag_ref se guarda como int32 */

#define CONF VN_ST_CONFIRMED
#define PROP VN_ST_PROPOSED
#define PROV VN_ST_PROVISIONAL
#define PEND VN_ST_PENDING
#define PENDV ((double)NAN)

#define E(f) VN_PK_NAV, VN_T_REAL, offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, f)
#define N(f) VN_PK_NAV, VN_T_F64, offsetof(nav_config_t, f)
#define H(f, t) VN_PK_HW, t, offsetof(vn_hw_t, f)
#define C(f, t) VN_PK_IMUC, t, offsetof(imu_conv_cfg_t, f)

/* nombre, destino, vuelo, banco, estado vuelo, estado banco, origen */
static const vn_param_t T[] = {
    /* ---------------- IMU ICM-20948 ---------------- */
    {"hw.imu_acc_fs_g", H(imu_acc_fs_g, VN_T_I32), 16, 16, PROP, PROP, "propuesta; comprobar con OpenRocket"},
    {"hw.imu_gyro_fs_dps", H(imu_gyro_fs_dps, VN_T_I32), 2000, 2000, PROP, PROP, "propuesta equipo"},
    {"imu.acc_lsb_per_g", C(acc_lsb_per_g, VN_T_F32), 2048, 2048, CONF, CONF, "doc ±16 g (SparkFun 1.3.2 getAccMG)"},
    {"imu.gyro_lsb_per_dps", C(gyro_lsb_per_dps, VN_T_F32), 16.4, 16.4, CONF, CONF, "doc ±2000 dps (SparkFun getGyrDPS)"},
    {"imu.g_si", C(g_si, VN_T_F32), 9.80665, 9.80665, CONF, CONF, "gravedad estándar (definición de g)"},
    {"hw.imu_acc_div", H(imu_acc_div, VN_T_I32), 4, 4, PROP, PROP, "propuesta: acel 1.125k/(1+4)=225 Hz (doc)"},
    {"hw.imu_gyro_div", H(imu_gyro_div, VN_T_I32), 4, 4, PROP, PROP, "propuesta: giro 1.1k/(1+4)=220 Hz (doc)"},
    {"hw.imu_dlpf_cfg", H(imu_dlpf_cfg, VN_T_I32), 3, 3, PROP, PROP, "propuesta: 3dB 50.4 Hz acel/51.2 Hz giro"},
    {"imu.sat_margin_lsb", C(sat_margin_lsb, VN_T_I32), 0, 0, PROP, PROP, "politica: solo en el extremo; revisar DLPF"},
    {"imu.axis_map_x", C(map[0], VN_T_U8), PENDV, 0, PEND, PROV, "montaje real del sensor en el cohete"},
    {"imu.axis_map_y", C(map[1], VN_T_U8), PENDV, 1, PEND, PROV, "montaje real"},
    {"imu.axis_map_z", C(map[2], VN_T_U8), PENDV, 2, PEND, PROV, "montaje real"},
    {"imu.axis_sign_x", C(sign[0], VN_T_I8), PENDV, 1, PEND, PROV, "montaje real"},
    {"imu.axis_sign_y", C(sign[1], VN_T_I8), PENDV, 1, PEND, PROV, "montaje real"},
    {"imu.axis_sign_z", C(sign[2], VN_T_I8), PENDV, 1, PEND, PROV, "montaje real"},
    {"hw.mag_div", H(mag_div, VN_T_I32), 8, 8, PROP, PROP, "mag crudo cada 8 lecturas (no se fusiona)"},

    /* ---------------- BMP390 ---------------- */
    {"hw.baro_odr_hz", H(baro_odr_hz, VN_T_I32), 50, 50, PROP, PROP, "propuesta: normal 50 Hz, IIR off"},
    {"hw.baro_osr_p", H(baro_osr_p, VN_T_I32), 4, 4, PROP, PROP, "propuesta: presion x4"},
    {"hw.baro_osr_t", H(baro_osr_t, VN_T_I32), 1, 1, PROP, PROP, "propuesta: temperatura x1"},
    {"doc.baro_conv_typ_ms", VN_PK_DOC, VN_T_F64, 0, 10.88, 10.88, CONF, CONF, "doc conversion tipica (NO es latencia)"},
    {"doc.baro_conv_max_ms", VN_PK_DOC, VN_T_F64, 0, 12.48, 12.48, CONF, CONF, "doc conversion maxima (NO es latencia)"},
    {"hw.baro_p0_pa", H(baro_p0_pa, VN_T_F64), 101325, 101325, PROP, PROP, "ISA: altitud = 44330(1-(p/p0)^0.1903)"},

    /* ---------------- bus, pines, direcciones ---------------- */
    {"hw.i2c_hz", H(i2c_hz, VN_T_I32), 400000, 400000, PROP, PROP, "propuesta; segun compatibilidad electrica"},
    {"hw.pin_sda", H(pin_sda, VN_T_I32), -1, -1, PEND, PROV, "-1: SDA por defecto de Wire (A4)"},
    {"hw.pin_scl", H(pin_scl, VN_T_I32), -1, -1, PEND, PROV, "-1: SCL por defecto de Wire (A5)"},
    {"hw.pin_imu_int", H(pin_imu_int, VN_T_I32), -1, -1, PEND, PROV, "-1: sondeo (marcas +-1 ms)"},
    {"hw.pin_baro_int", H(pin_baro_int, VN_T_I32), -1, -1, PEND, PROV, "-1: sondeo del STATUS"},
    {"hw.imu_addr", H(imu_addr, VN_T_I32), 0, 0, PEND, PROV, "0: autodeteccion 0x69/0x68 (registrada)"},
    {"hw.baro_addr", H(baro_addr, VN_T_I32), 0, 0, PEND, PROV, "0: autodeteccion 0x77/0x76 (registrada)"},

    /* ---------------- firmware ---------------- */
    {"fw.merge_holdoff_us", H(merge_holdoff_us, VN_T_I64), 10000, 10000, PROP, PROP, "espera para ordenar IMU/baro"},
    {"fw.navout_hz", H(navout_hz, VN_T_F64), 50, 50, PROP, PROP, "salida completa al registro"},
    {"fw.telemetry_hz", H(telemetry_hz, VN_T_F64), 10, 10, PROP, PROP, "gancho de telemetria"},
    {"fw.stats_hz", H(stats_hz, VN_T_F64), 1, 1, PROP, PROP, "tiempos, pila, colas"},
    {"fw.header_period_s", H(header_period_s, VN_T_F64), 10, 10, PROP, PROP, "repite cabecera y config"},
    {"fw.log_navev", H(log_navev, VN_T_I32), 1, 1, PROP, PROP, "retorno de cada evento"},
    {"fw.log_ticks", H(log_ticks, VN_T_I32), 1, 1, PROP, PROP, "reloj de nav_tick (reproduccion exacta)"},
    {"fw.stack_bytes", H(stack_bytes, VN_T_U32), 16384, 16384, PROP, PROP, "reserva provisional; medir HWM"},
    {"fw.ring_bytes", H(ring_bytes, VN_T_U32), 49152, 49152, PROP, PROP, "~1 s de registro; segun almacenamiento"},
    {"fw.queue_depth", H(queue_depth, VN_T_I32), 48, 48, PROP, PROP, "cola adquisicion->navegacion (eventos)"},
    {"fw.i2c_timeout_ms", H(i2c_timeout_ms, VN_T_I32), 5, 5, PROP, PROP, "Wire.setTimeOut; bus colgado = error contado"},
    {"fw.irq_timeout_ms", H(irq_timeout_ms, VN_T_I32), 5, 5, PROP, PROP, "sin irq IMU en el plazo: sondeo de respaldo"},
    {"fw.baro_poll_us", H(baro_poll_us, VN_T_I32), 2000, 2000, PROP, PROP, "intervalo minimo entre sondeos del baro"},
    {"fw.imu_reinit_errors", H(imu_reinit_errors, VN_T_I32), 100, 100, PROP, PROP, "errores I2C seguidos para reiniciar la IMU"},
    {"fw.boot_bench", H(boot_bench, VN_T_I32), 1, 1, PROP, PROP, "prueba de tiempos/pila al arrancar"},
    {"fw.core_acq", H(core_acq, VN_T_I32), 1, 1, PROP, PROP, "nucleo de la tarea de adquisicion"},
    {"fw.core_nav", H(core_nav, VN_T_I32), 0, 0, PROP, PROP, "nucleo de la tarea de navegacion"},
    {"fw.core_log", H(core_log, VN_T_I32), 0, 0, PROP, PROP, "nucleo de la tarea de registro"},
    {"fw.prio_acq", H(prio_acq, VN_T_I32), 20, 20, PROP, PROP, "prioridad FreeRTOS (loop de Arduino = 1)"},
    {"fw.prio_nav", H(prio_nav, VN_T_I32), 15, 15, PROP, PROP, "prioridad FreeRTOS"},
    {"fw.prio_log", H(prio_log, VN_T_I32), 5, 5, PROP, PROP, "prioridad FreeRTOS"},
    {"fw.axes_confirmed", H(axes_confirmed, VN_T_I32), 0, 0, PEND, PROV, "1 cuando se confirme el montaje"},

    /* ---------------- estimador: núcleo ---------------- */
    {"eskf.sigma_a", E(sigma_a), PENDV, 0.05, PEND, PROV, "por evento: analyze_static (sigma_eq)"},
    {"eskf.sigma_w", E(sigma_w), PENDV, 0.005, PEND, PROV, "por evento: analyze_static (sigma_eq)"},
    {"eskf.sigma_aw", E(sigma_aw), PENDV, 1e-4, PEND, PROV, "K de Allan: registro estatico largo"},
    {"eskf.sigma_ww", E(sigma_ww), PENDV, 1e-5, PEND, PROV, "K de Allan: registro estatico largo"},
    {"eskf.ba0_std", E(ba0_std), PENDV, 0.5, PEND, PROV, "6 posiciones + arranques; banco: provisional"},
    {"eskf.bg0_std", E(bg0_std), PENDV, 0.01, PEND, PROV, "residual tras alinear; arranques repetidos"},
    {"eskf.dt_max", E(dt_max), 0.007, 0.007, PROP, PROP, "propuesta 225 Hz; revisar periodo y jitter"},
    {"eskf.gap_max", E(gap_max), 0, 0, PROP, PROP, "puente deshabilitado"},
    {"eskf.innov_gate", E(innov_gate), 5, 5, PROP, PROP, "propuesta"},
    {"eskf.tau_max", E(tau_max), PENDV, 0.03, PEND, PROV, ">= baro_latency_s + dt_max"},
    {"eskf.accel_threshold", E(accel_threshold), PENDV, 0.5, PEND, PROV, "registro estatico"},
    {"eskf.align_acc_tol", E(align_acc_tol), PENDV, 0.6, PEND, PROV, "registro estatico (6 posiciones)"},
    {"eskf.align_gyro_max", E(align_gyro_max), PENDV, 0.1, PEND, PROV, "registro estatico (sesgo giro)"},
    {"eskf.has_mag_ref", VN_PK_NAV, VN_T_I32, offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, has_mag_ref), 0, 0, PROP, PROP, "magnetometro registrado, no fusionado"},
    {"eskf.mag_ref_n", E(mag_ref[0]), 0, 0, PROP, PROP, "sin uso (has_mag_ref = 0)"},
    {"eskf.mag_ref_e", E(mag_ref[1]), 0, 0, PROP, PROP, "sin uso (has_mag_ref = 0)"},
    {"eskf.mag_ref_d", E(mag_ref[2]), 0, 0, PROP, PROP, "sin uso (has_mag_ref = 0)"},
    {"eskf.p0_std", E(p0_std), 1.0, 1.0, PROP, PROP, "politica"},
    {"eskf.v0_std", E(v0_std), 0.1, 0.1, PROP, PROP, "politica"},
    {"eskf.th0_std", E(th0_std), 0.05, 0.05, PROP, PROP, "politica"},

    /* ---------------- estimador: navegación ---------------- */
    {"rest_window_s", N(rest_window_s), 5, 5, PROP, PROP, "propuesta"},
    {"rest_update_period_s", N(rest_update_period_s), 0.05, 0.05, PROP, PROP, "propuesta"},
    {"zupt_sigma", N(zupt_sigma), 0.02, 0.02, PROP, PROP, "politica provisional, comprobar"},
    {"grav_sigma", N(grav_sigma), 0.3, 0.3, PROP, PROP, "politica provisional, comprobar"},
    {"rest_acc_dev_max", N(rest_acc_dev_max), PENDV, 0.6, PEND, PROV, "registro estatico"},
    {"rest_acc_std_max", N(rest_acc_std_max), PENDV, 0.15, PEND, PROV, "registro estatico"},
    {"rest_gyro_max", N(rest_gyro_max), PENDV, 0.15, PEND, PROV, "registro estatico"},
    {"rest_tilt_change_max", N(rest_tilt_change_max), PENDV, 0.01, PEND, PROV, "registro estatico"},
    {"liftoff_acc", N(liftoff_acc), PENDV, 30, PEND, PROV, "curva de empuje (OpenRocket)"},
    {"liftoff_hold_s", N(liftoff_hold_s), 0.05, 0.05, PROP, PROP, "propuesta"},
    {"acc_sat_limit", N(acc_sat_limit), 0, 0, PROP, PROP, "0: saturacion por codigo (imu_conv)"},
    {"gyro_sat_limit", N(gyro_sat_limit), 0, 0, PROP, PROP, "0: saturacion por codigo (imu_conv)"},
    {"acc_sat_max_s", N(acc_sat_max_s), 0, 0, PROP, PROP, "modelo deshabilitado"},
    {"gyro_sat_max_s", N(gyro_sat_max_s), 0, 0, PROP, PROP, "modelo deshabilitado"},
    {"acc_sat_excess", N(acc_sat_excess), 0, 0, PROP, PROP, "solo si acc_sat_max_s > 0"},
    {"gyro_sat_excess", N(gyro_sat_excess), 0, 0, PROP, PROP, "solo si gyro_sat_max_s > 0"},
    {"baro_sigma", N(baro_sigma), PENDV, 0.5, PEND, PROV, "registro estatico"},
    {"baro_latency_s", N(baro_latency_s), PENDV, 0.0, PEND, PROV, "medir; banco 0 = sin compensar"},
    {"baro_timeout_s", N(baro_timeout_s), 1, 1, PROP, PROP, "politica provisional"},
    {"baro_ref_min_n", N(baro_ref_min_n), 50, 50, PROP, PROP, "politica"},
    {"baro_ref_max_step", N(baro_ref_max_step), 2, 2, PROP, PROP, "politica"},
    {"stale_max_s", N(stale_max_s), 0.05, 0.05, PROP, PROP, ">= holdoff + dt_max"},
};

const vn_param_t *vn_params_table(size_t *n)
{
    if (n) *n = sizeof T / sizeof T[0];
    return T;
}

double vn_param_value(const vn_param_t *p, int profile)
{
    return (profile == VN_PROFILE_BENCH) ? p->bench : p->flight;
}

uint8_t vn_param_status(const vn_param_t *p, int profile)
{
    return (profile == VN_PROFILE_BENCH) ? p->bench_status : p->status;
}

static void store(void *base, uint8_t type, size_t off, double v)
{
    char *d = (char *)base + off;
    switch (type) {
    case VN_T_REAL: { eskf_real x = (eskf_real)v; memcpy(d, &x, sizeof x); break; }
    case VN_T_F64: { memcpy(d, &v, sizeof v); break; }
    case VN_T_F32: { float x = (float)v; memcpy(d, &x, sizeof x); break; }
    case VN_T_I32: { int32_t x = isfinite(v) ? (int32_t)v : 0; memcpy(d, &x, sizeof x); break; }
    case VN_T_I64: { int64_t x = isfinite(v) ? (int64_t)v : 0; memcpy(d, &x, sizeof x); break; }
    case VN_T_U32: { uint32_t x = isfinite(v) && v >= 0 ? (uint32_t)v : 0u; memcpy(d, &x, sizeof x); break; }
    case VN_T_U8: { uint8_t x = isfinite(v) && v >= 0 ? (uint8_t)v : 0xFFu; memcpy(d, &x, sizeof x); break; }
    case VN_T_I8: { int8_t x = isfinite(v) ? (int8_t)v : 0; memcpy(d, &x, sizeof x); break; }
    default: break;
    }
}

const char *vn_profile_name(int profile)
{
    switch (profile) {
    case VN_PROFILE_ACQ: return "ADQUISICION";
    case VN_PROFILE_BENCH: return "BANCO (provisional, no volar)";
    case VN_PROFILE_FLIGHT: return "VUELO";
    default: return "desconocido";
    }
}

int vn_params_build(int profile, nav_config_t *cfg, imu_conv_cfg_t *ic, vn_hw_t *hw, const char **why)
{
    static const char *no_why = "";
    const char *w = no_why;
    if (nav_default_config(cfg) != NAV_OK) {
        if (why) *why = "nav_default_config: float/double incoherente";
        return 0;
    }
    memset(ic, 0, sizeof *ic);
    memset(hw, 0, sizeof *hw);
    int map_pending = 0;
    int vprof = (profile == VN_PROFILE_ACQ) ? VN_PROFILE_FLIGHT : profile; /* ADQ registra los de vuelo */
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
        const vn_param_t *p = &T[i];
        if (p->kind == VN_PK_DOC) continue; /* informativos */
        double v = vn_param_value(p, vprof);
        void *base = (p->kind == VN_PK_NAV) ? (void *)cfg : (p->kind == VN_PK_IMUC) ? (void *)ic : (void *)hw;
        if (p->kind == VN_PK_IMUC && !isfinite(v)) map_pending = 1;
        store(base, p->type, p->off, v);
    }
    int ok = 1;
    if (profile == VN_PROFILE_ACQ) {
        ok = 0;
        w = "perfil de adquisicion: navegacion desactivada";
    } else if (map_pending || imu_conv_check(ic) != 0) {
        ok = 0;
        w = "ejes de la IMU pendientes o no son una rotacion propia";
    } else if (!hw->axes_confirmed && profile == VN_PROFILE_FLIGHT) {
        ok = 0;
        w = "fw.axes_confirmed = 0";
    } else {
        const char *bad = nav_config_check(cfg);
        if (bad) {
            ok = 0;
            w = bad;
        }
    }
    if (why) *why = w;
    return ok;
}
