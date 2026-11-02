/*
 * eskf_svc_params.c — TABLA ÚNICA de configuración del servicio SPP: valor, unidad, procedencia y
 * estado por perfil. Separada de los parámetros de los drivers y de la aplicación Arduino v4.
 *
 * Dos columnas:
 *   ESTRICTO      sólo lo confirmado; lo demás PENDIENTE (NaN) y la navegación no arranca.
 *   EXPERIMENTAL  todo numérico y coherente para operar con las hipótesis disponibles. Cada valor
 *                 dice de dónde sale: CONFIRMADO (código SPP 9247d9e), OBSERVADO (registro: la
 *                 captura histórica del 24-oct, firmware anterior, reposo 1,768–10,768 s),
 *                 SIMULADO (OpenRocket «Simulation 6» de PUMEL_v3-7_Klima.ork y RocketPy de
 *                 Montecarlo_Viradinha.py), PROPUESTO (política) o PROVISIONAL (sin medida).
 * Ningún valor experimental es una calibración ni una validación física; no hay Allan.
 * Aviónica puede sustituir cualquier campo con SPP_SERVICES_ESKF_configure sin tocar el algoritmo:
 * la diferencia con esta tabla se publica como RDY_OVERRIDE y cambia config_id.
 */
#include "eskf_svc_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CONF ESKF_ST_CONFIRMED
#define PROP ESKF_ST_PROPOSED
#define PROV ESKF_ST_PROVISIONAL
#define PEND ESKF_ST_PENDING
#define OBS ESKF_ST_OBSERVED
#define SIM ESKF_ST_SIMULATED
#define NA ((double)NAN)
#define DEG2RAD 0.017453292519943295
#define G0 9.80665

#define U ESKF_SVC_RDY_UNITS
#define AX ESKF_SVC_RDY_AXES
#define RG ESKF_SVC_RDY_RANGE
#define TM ESKF_SVC_RDY_TIME
#define BA ESKF_SVC_RDY_BARO
#define NO ESKF_SVC_RDY_NOISE
#define TH ESKF_SVC_RDY_THRESH
#define LO ESKF_SVC_RDY_LIFTOFF
#define HO ESKF_SVC_RDY_HOST

#define S(f, t) t, offsetof(eskf_svc_config_t, f)
#define E(f) ESKF_T_REAL, offsetof(eskf_svc_config_t, nav) + offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, f)
#define N(f) ESKF_T_F64, offsetof(eskf_svc_config_t, nav) + offsetof(nav_config_t, f)

/* nombre, unidad, destino, ESTRICTO, EXPERIMENTAL, estado estricto, estado experimental, grupo,
   exige confirmación, procedencia */
static const eskf_param_t T[] = {
    /* ---- formato de entrada (hechos del código SPP 9247d9e) ---- */
    {"in.imu_format", "B", S(imu_format, ESKF_T_I32), 44, 44, CONF, CONF, 0, 0, "icm20948.c: 11 float32"},
    {"in.baro_semantics", "-", S(baro_semantics, ESKF_T_I32), 1, 1, CONF, CONF, 0, 0,
     "bmp390.c: altura relativa a p0/T0 medios de 500 muestras"},
    {"in.acc_div", "LSB/g", S(acc_div, ESKF_T_F32), 8192, 8192, CONF, CONF, 0, 0, "icm20948.c: raw/8192"},
    {"in.gyro_div", "LSB/(deg/s)", S(gyro_div, ESKF_T_F32), 32.8, 32.8, CONF, CONF, 0, 0, "icm20948.c: raw/32.8"},
    {"in.acc_off_x", "g", S(acc_off[0], ESKF_T_F32), 0.004621, 0.004621, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    {"in.acc_off_y", "g", S(acc_off[1], ESKF_T_F32), -0.001157, -0.001157, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    {"in.acc_off_z", "g", S(acc_off[2], ESKF_T_F32), 0.010014, 0.010014, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    {"in.gyro_off_x", "deg/s", S(gyro_off[0], ESKF_T_F32), 0.681567, 0.681567, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    {"in.gyro_off_y", "deg/s", S(gyro_off[1], ESKF_T_F32), 0.428753, 0.428753, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    {"in.gyro_off_z", "deg/s", S(gyro_off[2], ESKF_T_F32), -0.329620, -0.329620, CONF, CONF, 0, 0, "icm20948.h: ya restado"},
    /* ---- unidades: hipótesis del contrato (aviónica puede sustituirlas) ---- */
    {"in.acc_unit_si", "m/s2 por unidad", S(acc_unit_si, ESKF_T_F32), 9.80665, 9.80665, PROP, PROP, U, 1,
     "contrato: aceleracion publicada en g"},
    {"in.gyro_unit_si", "rad/s por unidad", S(gyro_unit_si, ESKF_T_F32), NA, DEG2RAD, PEND, PROP, U, 1,
     "contrato: deg/s; sin correccion (FS 2000 con /32.8 sin aclarar)"},
    /* ---- ejes: sin montaje confirmado, cuerpo = marco de la IMU ---- */
    {"in.r_sb_00", "-", S(r_sb[0], ESKF_T_F32), NA, 1, PEND, PROP, AX, 1, "identidad: cuerpo = IMU (no es el eje del cohete)"},
    {"in.r_sb_01", "-", S(r_sb[1], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_02", "-", S(r_sb[2], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_10", "-", S(r_sb[3], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_11", "-", S(r_sb[4], ESKF_T_F32), NA, 1, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_12", "-", S(r_sb[5], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_20", "-", S(r_sb[6], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_21", "-", S(r_sb[7], ESKF_T_F32), NA, 0, PEND, PROP, AX, 1, "identidad"},
    {"in.r_sb_22", "-", S(r_sb[8], ESKF_T_F32), NA, 1, PEND, PROP, AX, 1, "identidad"},
    /* ---- saturación ---- */
    {"in.sat_margin_codes", "LSB", S(sat_margin_codes, ESKF_T_F32), 1.5, 1.5, PROP, PROP, RG, 1,
     "candidato si abs(codigo) >= 32768-1.5; rango util a la salida del DMP sin verificar"},
    {"in.sat_to_nav", "-", S(sat_to_nav, ESKF_T_I32), 1, 1, PROP, PROP, 0, 0, "candidatos -> saturacion en navigation.c"},
    {"in.sat_motor_window_s", "s", S(sat_motor_window_s, ESKF_T_F32), NA, 1.5, PEND, SIM, RG, 1,
     "SIM: >4 g de 0.028 a 0.949 s; burnout 1.076 s; ventana desde el despegue detectado"},
    /* ---- tiempo ---- */
    {"in.time_source", "-", S(time_source, ESKF_T_I32), 1, 1, PROP, PROP, TM, 1,
     "timestampMs de packetData (construccion, 1 ms)"},
    {"in.future_tol_us", "us", S(future_tol_us, ESKF_T_I64), 1000, 1000, PROP, PROP, 0, 0,
     "marca truncada a ms: nunca posterior a la recepcion con el mismo reloj"},
    /* ---- límites físicos del baro ---- */
    {"in.baro_h_abs_max_m", "m", S(baro_h_abs_max_m, ESKF_T_F32), 5000, 5000, PROP, PROP, 0, 0,
     "altura relativa; apogeo maximo Monte Carlo 474.7 m"},
    {"in.baro_p_min_pa", "Pa", S(baro_p_min_pa, ESKF_T_F32), 30000, 30000, PROP, PROP, 0, 0, "BMP390: 300 hPa"},
    {"in.baro_p_max_pa", "Pa", S(baro_p_max_pa, ESKF_T_F32), 125000, 125000, PROP, PROP, 0, 0, "BMP390: 1250 hPa"},
    {"in.baro_t_min_c", "degC", S(baro_t_min_c, ESKF_T_F32), -40, -40, PROP, PROP, 0, 0, "BMP390: -40 C"},
    {"in.baro_t_max_c", "degC", S(baro_t_max_c, ESKF_T_F32), 85, 85, PROP, PROP, 0, 0, "BMP390: +85 C"},
    /* ---- respaldo barométrico ---- */
    {"bb.alpha", "-", S(bb_alpha, ESKF_T_F32), 0.25, 0.25, PROP, PROP, 0, 0, "alfa-beta fijo a ~100 Hz"},
    {"bb.beta", "-", S(bb_beta, ESKF_T_F32), 0.02, 0.02, PROP, PROP, 0, 0, "alfa-beta fijo a ~100 Hz"},
    {"bb.gate_m", "m", S(bb_gate_m, ESKF_T_F32), 10, 10, PROP, PROP, 0, 0, "residuo maximo admitido"},
    {"bb.reseed_n", "muestras", S(bb_reseed_n, ESKF_T_U32), 20, 20, PROP, PROP, 0, 0, "rechazos seguidos que pasan a recuperacion"},
    {"bb.dt_max_s", "s", S(bb_dt_max_s, ESKF_T_F32), 0.1, 0.1, PROP, PROP, 0, 0,
     "10 periodos del BMP a 100 Hz sin medida aceptada: recuperacion"},
    {"bb.timeout_s", "s", S(bb_timeout_s, ESKF_T_F32), 0.5, 0.5, PROP, PROP, 0, 0,
     "antiguedad maxima de la ultima medida ACEPTADA"},
    {"bb.ref_tau_s", "s", S(bb_ref_tau_s, ESKF_T_F32), 5, 5, PROP, PROP, 0, 0, "referencia en rampa (media exponencial)"},
    {"bb.liftoff_m", "m", S(bb_liftoff_m, ESKF_T_F32), 10, 10, PROP, PROP, 0, 0,
     "congela la referencia si el ESKF no detecta el despegue"},
    {"bb.min_samples", "muestras", S(bb_min_samples, ESKF_T_U32), 20, 20, PROP, PROP, 0, 0,
     "ventana coherente de inicio y de recuperacion (0.2 s a 100 Hz)"},
    {"bb.coh_m", "m", S(bb_coh_m, ESKF_T_F32), 1.0, 1.0, PROV, PROV, 0, 0,
     "residuo max. del ajuste lineal de la ventana: ~7 sigma del ruido observado (0.14 m) con margen de curvatura"},
    {"bb.flight_vz_mps", "m/s", S(bb_flight_vz_mps, ESKF_T_F32), 3.0, 3.0, PROV, PROV, 0, 0,
     "vuelo reconocido por el respaldo sin atestacion inercial: descenso simulado 4.3-5.9 m/s; ruido de v ~0.4 m/s"},
    {"bb.hold_s", "s", S(bb_hold_s, ESKF_T_F32), 1.0, 1.0, PROV, PROV, 0, 0,
     "duracion continua de esa |v_z| (>= 3 m de desplazamiento vertical) o del reposo (|v_z| <= 1 m/s) que fija la referencia de lanzamiento"},
    /* ---- fase: rampa confirmada o incierta ---- */
    {"svc.pad_start_s", "s", S(pad_start_s, ESKF_T_F32), 10.0, 10.0, PROP, PROP, 0, 0,
     "contrato de inicio: ningun despegue en este plazo tras encender o reiniciar la sesion en rampa (arranque de "
     "productores: calibracion del BMP 500 muestras ~5 s)"},
    {"svc.pad_gap_max_s", "s", S(pad_gap_max_s, ESKF_T_F32), 0.5, 0.5, PROV, PROV, 0, 0,
     "hueco max. entre evidencias de rampa: < empuje > 2.5 g (~1 s, simulado) - nav.liftoff_hold_s; > paron de SD de 34 ms"},
    {"svc.pad_lat_max_s", "s", S(pad_lat_max_s, ESKF_T_F32), 0.5, 0.5, PROP, PROP, 0, 0,
     "latencia de entrega admitida antes de dar por caducada la rampa en silencio total: cola del router 128 "
     "paquetes / 325 paquetes/s de ICM+BMP = 0.39 s (codigo SPP) + margen"},
    /* ---- trabajo y salida ---- */
    {"svc.merge_holdoff_us", "us", S(merge_holdoff_us, ESKF_T_I64), 15000, 15000, PROP, PROP, HO, 1,
     "espera de orden: baro cada ~10 ms"},
    {"svc.max_items", "paquetes", S(max_items, ESKF_T_U32), 16, 16, PROP, PROP, HO, 1, "por consumeData"},
    {"svc.max_events", "llamadas", S(max_events, ESKF_T_U32), 24, 24, PROP, PROP, HO, 1, "nav_on_* por consumeData"},
    {"svc.snapshot_period_us", "us", S(snapshot_period_us, ESKF_T_I64), 200000, 200000, PROP, PROP, 0, 0,
     "estado completo 5 Hz x 5 fragmentos (registro)"},
    {"svc.diag_period_us", "us", S(diag_period_us, ESKF_T_I64), 1000000, 1000000, PROP, PROP, 0, 0, "1 Hz"},
    {"svc.ops_period_us", "us", S(ops_period_us, ESKF_T_I64), 1000000, 1000000, PROP, PROP, 0, 0,
     "compacta 1 Hz (LoRa SF7/125 kHz: ~160 ms por paquete)"},
    {"svc.snapshot_mask", "-", S(snapshot_mask, ESKF_T_U32), 0x1F, 0x1F, PROP, PROP, 0, 0, "5 fragmentos"},
    {"svc.trace_enable", "-", S(trace_enable, ESKF_T_I32), 0, 0, PROP, PROP, 0, 0, "traza de validacion: activar en banco"},
    /* ---- núcleo ---- */
    {"eskf.sigma_a", "m/s2", E(sigma_a), NA, 0.12, PEND, OBS, NO, 1,
     "OBS: 0.06 por eje en reposo (registro historico) x2; vibracion de vuelo no medida"},
    {"eskf.sigma_w", "rad/s", E(sigma_w), NA, 0.01, PEND, OBS, NO, 1, "OBS: <=0.0046 por eje en reposo x2"},
    {"eskf.sigma_aw", "m/s2/sqrt(s)", E(sigma_aw), NA, 1e-4, PEND, PROV, NO, 1, "sin Allan"},
    {"eskf.sigma_ww", "rad/s/sqrt(s)", E(sigma_ww), NA, 1e-5, PEND, PROV, NO, 1, "sin Allan"},
    {"eskf.ba0_std", "m/s2", E(ba0_std), NA, 0.3, PEND, PROV, NO, 1, "residual tras los offsets del productor: sin medir"},
    {"eskf.bg0_std", "rad/s", E(bg0_std), NA, 0.01, PEND, OBS, NO, 1, "OBS: media en reposo <=0.0095 (sin offsets)"},
    {"eskf.dt_max", "s", E(dt_max), NA, 0.010, PEND, OBS, TM, 1,
     "OBS: intervalos normales 1-7 ms (marcas de 1 ms); 1 muestra perdida = 8.9 ms"},
    {"eskf.gap_max", "s", E(gap_max), 0, 0, PROP, PROP, 0, 0, "puente deshabilitado"},
    {"eskf.innov_gate", "sigma", E(innov_gate), 5, 5, PROP, PROP, 0, 0, "puerta de innovacion"},
    {"eskf.tau_max", "s", E(tau_max), NA, 0.03, PEND, PROV, BA, 1, ">= baro_latency_s + dt_max + margen"},
    {"eskf.accel_threshold", "m/s2", E(accel_threshold), NA, 0.5, PEND, OBS, TH, 1, "OBS: abs(norma(a)-g) max 0.29 en reposo"},
    {"eskf.align_acc_tol", "m/s2", E(align_acc_tol), NA, 0.6, PEND, OBS, TH, 1, "OBS: abs(norma(a media)-g) 0.06; margen x10"},
    {"eskf.align_gyro_max", "rad/s", E(align_gyro_max), NA, 0.1, PEND, OBS, TH, 1, "OBS: norma(w media) 0.012"},
    {"eskf.p0_std", "m", E(p0_std), 1.0, 1.0, PROP, PROP, 0, 0, "politica"},
    {"eskf.v0_std", "m/s", E(v0_std), 0.1, 0.1, PROP, PROP, 0, 0, "politica"},
    {"eskf.th0_std", "rad", E(th0_std), 0.05, 0.05, PROP, PROP, 0, 0, "politica"},
    {"eskf.has_mag_ref", "-", ESKF_T_INT,
     offsetof(eskf_svc_config_t, nav) + offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, has_mag_ref), 0, 0,
     PROP, PROP, 0, 0, "magnetometro no fusionado"},
    {"eskf.mag_ref_n", "uT", E(mag_ref[0]), 0, 0, PROP, PROP, 0, 0, "sin uso"},
    {"eskf.mag_ref_e", "uT", E(mag_ref[1]), 0, 0, PROP, PROP, 0, 0, "sin uso"},
    {"eskf.mag_ref_d", "uT", E(mag_ref[2]), 0, 0, PROP, PROP, 0, 0, "sin uso"},
    /* ---- navegación ---- */
    {"rest_window_s", "s", N(rest_window_s), 5, 5, PROP, PROP, 0, 0, "no se acorta por huecos; 5 s x 100 Hz = 500 < 512"},
    {"rest_update_period_s", "s", N(rest_update_period_s), 0.05, 0.05, PROP, PROP, 0, 0, "politica"},
    {"zupt_sigma", "m/s", N(zupt_sigma), 0.02, 0.02, PROP, PROP, 0, 0, "politica"},
    {"grav_sigma", "m/s2", N(grav_sigma), 0.3, 0.3, PROP, PROP, 0, 0, "politica"},
    {"rest_acc_dev_max", "m/s2", N(rest_acc_dev_max), NA, 0.6, PEND, OBS, TH, 1, "OBS: max 0.29 en reposo x2"},
    {"rest_acc_std_max", "m/s2", N(rest_acc_std_max), NA, 0.15, PEND, OBS, TH, 1, "OBS: 0.061 en reposo x2.5"},
    {"rest_gyro_max", "rad/s", N(rest_gyro_max), NA, 0.15, PEND, PROV, TH, 1, "OBS: max 0.027 en mesa; margen para viento en rampa"},
    {"rest_tilt_change_max", "rad", N(rest_tilt_change_max), NA, 0.01, PEND, PROV, TH, 1, "provisional"},
    {"liftoff_acc", "m/s2", N(liftoff_acc), NA, 2.5 * G0, PEND, SIM, LO, 1,
     "SIM: f > 4 g casi toda la combustion; 2.5 g < recorte de 4 g"},
    {"liftoff_hold_s", "s", N(liftoff_hold_s), NA, 0.05, PEND, SIM, LO, 1, "SIM: f > 2.5 g continuo ~0.9 s"},
    {"acc_sat_limit", "m/s2", N(acc_sat_limit), 0, 0, PROP, PROP, 0, 0, "0: solo candidatos del dominio de codigos"},
    {"gyro_sat_limit", "rad/s", N(gyro_sat_limit), 0, 0, PROP, PROP, 0, 0, "0: solo candidatos del dominio de codigos"},
    {"acc_sat_max_s", "s", N(acc_sat_max_s), 0, 1.2, PROP, SIM, RG, 1,
     "SIM: 0.92 s por encima de 4 g en la combustion, +30 %"},
    {"gyro_sat_max_s", "s", N(gyro_sat_max_s), 0, 0, PROP, PROP, 0, 0, "giroscopo: sin continuacion"},
    {"acc_sat_excess", "m/s2", N(acc_sat_excess), 0, 110, PROP, SIM, RG, 1,
     "SIM: pico 130.95 - 39.2 recortado = 91.8, +20 %"},
    {"gyro_sat_excess", "rad/s", N(gyro_sat_excess), 0, 0, PROP, PROP, 0, 0, "giroscopo: sin continuacion"},
    {"baro_sigma", "m", N(baro_sigma), NA, 0.5, PEND, OBS, NO, 1,
     "OBS: 0.14 entre muestras en reposo; margen por efectos dinamicos no medidos"},
    {"baro_latency_s", "s", N(baro_latency_s), NA, 0.010, PEND, PROV, BA, 1,
     "IIR coeficiente 1 (registro 0x02) a 100 Hz: ~1 muestra; no medido"},
    {"baro_timeout_s", "s", N(baro_timeout_s), 1, 1, PROP, PROP, 0, 0, "politica"},
    {"baro_ref_min_n", "muestras", N(baro_ref_min_n), 50, 50, PROP, PROP, 0, 0, "politica"},
    {"baro_ref_max_step", "m", N(baro_ref_max_step), 2, 2, PROP, PROP, 0, 0, "politica"},
    {"stale_max_s", "s", N(stale_max_s), 0.05, 0.05, PROP, PROP, HO, 1, ">= espera de orden + dt_max"},
};

const eskf_param_t *eskf_svc_params_table(size_t *n)
{
    if (n) *n = sizeof T / sizeof T[0];
    return T;
}

double eskf_svc_param_value(const eskf_param_t *p, int profile)
{
    return profile == ESKF_SVC_PROFILE_EXP ? p->exp : p->strict;
}

uint8_t eskf_svc_param_status(const eskf_param_t *p, int profile)
{
    return profile == ESKF_SVC_PROFILE_EXP ? p->st_exp : p->st_strict;
}

static void store(void *base, const eskf_param_t *p, double v)
{
    char *d = (char *)base + p->off;
    switch (p->type) {
    case ESKF_T_REAL: { eskf_real x = (eskf_real)v; memcpy(d, &x, sizeof x); break; }
    case ESKF_T_F64: memcpy(d, &v, sizeof v); break;
    case ESKF_T_F32: { float x = (float)v; memcpy(d, &x, sizeof x); break; }
    case ESKF_T_I32: { int32_t x = (int32_t)v; memcpy(d, &x, sizeof x); break; }
    case ESKF_T_INT: { int x = (int)v; memcpy(d, &x, sizeof x); break; }
    case ESKF_T_I64: { int64_t x = (int64_t)v; memcpy(d, &x, sizeof x); break; }
    default: { uint32_t x = (uint32_t)v; memcpy(d, &x, sizeof x); break; }
    }
}

double eskf_svc_param_read(const eskf_svc_config_t *cfg, const eskf_param_t *p)
{
    const char *d = (const char *)cfg + p->off;
    switch (p->type) {
    case ESKF_T_REAL: { eskf_real x; memcpy(&x, d, sizeof x); return (double)x; }
    case ESKF_T_F64: { double x; memcpy(&x, d, sizeof x); return x; }
    case ESKF_T_F32: { float x; memcpy(&x, d, sizeof x); return (double)x; }
    case ESKF_T_I32: { int32_t x; memcpy(&x, d, sizeof x); return (double)x; }
    case ESKF_T_INT: { int x; memcpy(&x, d, sizeof x); return (double)x; }
    case ESKF_T_I64: { int64_t x; memcpy(&x, d, sizeof x); return (double)x; }
    default: { uint32_t x; memcpy(&x, d, sizeof x); return (double)x; }
    }
}

int eskf_svc_build_config(int profile, eskf_svc_config_t *cfg)
{
    if (!cfg || profile < ESKF_SVC_PROFILE_ACQ || profile > ESKF_SVC_PROFILE_STRICT) return -1;
    memset(cfg, 0, sizeof *cfg);
    if (nav_default_config(&cfg->nav) != NAV_OK) return -1;
    cfg->profile = profile;
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) store(cfg, &T[i], eskf_svc_param_value(&T[i], profile));
    return 0;
}

void eskf_svc_config_historical(eskf_svc_config_t *cfg)
{
    cfg->imu_format = ESKF_SVC_IMU_FMT_HIST48;
    cfg->baro_semantics = ESKF_SVC_BARO_ABS_ISA;
    for (int i = 0; i < 3; i++) cfg->acc_off[i] = cfg->gyro_off[i] = 0.0f; /* la captura antigua no los tenía */
}

uint16_t eskf_svc_readiness(int profile)
{
    uint16_t r = 0;
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
        uint8_t st = eskf_svc_param_status(&T[i], profile);
        if (st != CONF && (st == PEND || st == PROV || T[i].need_conf)) r = (uint16_t)(r | T[i].group);
    }
    if (profile != ESKF_SVC_PROFILE_STRICT) r = (uint16_t)(r | ESKF_SVC_RDY_PROFILE);
    if (!ESKF_SVC_BOARD_VALIDATED) r = (uint16_t)(r | ESKF_SVC_RDY_BOARD);
    return r;
}

static eskf_svc_config_t s_ref; /* referencia de la tabla (sólo al iniciar; estática, no en la pila) */

uint16_t eskf_svc_readiness_cfg(const eskf_svc_config_t *cfg)
{
    uint16_t r = eskf_svc_readiness(cfg->profile);
    if (eskf_svc_build_config(cfg->profile, &s_ref) != 0) return (uint16_t)(r | ESKF_SVC_RDY_OVERRIDE);
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
        if (T[i].group == 0u) continue; /* políticas (salida, respaldo, límites): sólo cambian config_id */
        double a = eskf_svc_param_read(cfg, &T[i]), b = eskf_svc_param_read(&s_ref, &T[i]);
        if (!(a == b || (isnan(a) && isnan(b)))) {
            r = (uint16_t)(r | ESKF_SVC_RDY_OVERRIDE); /* valor ajeno a la tabla: procedencia externa */
            break;
        }
    }
    return r;
}

void eskf_svc_output_preset(eskf_svc_config_t *cfg, int preset)
{
    if (!cfg) return;
    if (preset == ESKF_OUT_TELEMETRY) { /* un KPID que se envía por radio: sólo lo compacto */
        cfg->snapshot_period_us = 0;
        cfg->diag_period_us = 10000000;
        cfg->ops_period_us = 1000000;
    } else { /* registro: estado completo a 5 Hz, diagnóstico y compacta a 1 Hz */
        cfg->snapshot_period_us = 200000;
        cfg->diag_period_us = 1000000;
        cfg->ops_period_us = 1000000;
    }
}

uint16_t eskf_svc_config_id(const eskf_svc_config_t *cfg)
{
    uint16_t c = 0xFFFFu;
    uint8_t b[8];
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
        double v = eskf_svc_param_read(cfg, &T[i]);
        uint64_t u;
        if (isnan(v)) u = 0x7FF8000000000000ull; /* NaN canónico: «pendiente» */
        else memcpy(&u, &v, 8);
        for (int k = 0; k < 8; k++) b[k] = (uint8_t)(u >> (8 * k));
        c = eskf_svc_crc16(b, 8, c);
    }
    uint8_t p = (uint8_t)cfg->profile;
    return eskf_svc_crc16(&p, 1, c);
}
