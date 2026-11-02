/*
 * test_vn.c — pruebas en el ordenador de las partes en C puro del firmware:
 * formato de registro, tabla de parámetros (cobertura, estados, perfiles) y prueba de tiempos.
 * No sustituyen a las pruebas en placa (PROCEDIMIENTOS.md).
 */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "../viradinha_nav/src/nav/imu_conv.h"
#include "../viradinha_nav/src/nav/navigation.h"
#include "../viradinha_nav/src/vn/vn_bench.h"
#include "../viradinha_nav/src/vn/vn_logfmt.h"
#include "../viradinha_nav/src/vn/vn_params.h"

static int g_fails = 0, g_checks = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        g_checks++;                                        \
        if (!(cond)) {                                     \
            printf("  FALLO %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                           \
            printf("\n");                                  \
            g_fails++;                                     \
        }                                                  \
    } while (0)

/* ---------------------------------------------------------------- trama */
static void test_frame(void)
{
    printf("formato de registro\n");
    const uint8_t v[] = "123456789";
    CHECK(vn_crc16(v, 9, 0xFFFFu) == 0x29B1u, "CRC-16/CCITT-FALSE de '123456789' = 0x29B1 (es 0x%04X)",
          vn_crc16(v, 9, 0xFFFFu));
    uint8_t buf[VN_FRAME_MAX];
    vn_rec_text_t t;
    memset(&t, 0, sizeof t);
    strcpy(t.text, "hola");
    size_t n = vn_frame(buf, sizeof buf, VN_REC_TEXT, &t, sizeof t);
    CHECK(n == sizeof t + VN_FRAME_OVERHEAD, "longitud de trama %zu", n);
    CHECK(buf[0] == VN_SYNC0 && buf[1] == VN_SYNC1, "sincronismo");
    CHECK((size_t)(buf[2] | (buf[3] << 8)) == sizeof t, "len little-endian");
    CHECK(buf[4] == VN_REC_TEXT, "tipo");
    uint16_t crc = vn_crc16(&buf[4], 1u + sizeof t, 0xFFFFu);
    CHECK(buf[n - 2] == (uint8_t)(crc & 0xFFu) && buf[n - 1] == (uint8_t)(crc >> 8), "crc al final, LE");
    CHECK(vn_frame(buf, 10, VN_REC_TEXT, &t, sizeof t) == 0, "no cabe: 0");
    vn_rec_navout_t no;
    CHECK(vn_frame(buf, sizeof buf, VN_REC_NAVOUT, &no, sizeof no) > 0, "el registro más largo cabe en VN_FRAME_MAX");
    char s[4];
    vn_strfill(s, sizeof s, "abcdef");
    CHECK(s[3] == '\0' && strcmp(s, "abc") == 0, "vn_strfill trunca y termina");
}

/* ---------------------------------------------------------------- parámetros */
typedef struct {
    const char *name;
    int kind;
    size_t off, size;
} field_t;

#define FE(nm, f) {nm, VN_PK_NAV, offsetof(nav_config_t, eskf) + offsetof(eskf_params_t, f), sizeof(((eskf_params_t *)0)->f)}
#define FN(f) {#f, VN_PK_NAV, offsetof(nav_config_t, f), sizeof(double)}
#define FH(nm, f) {nm, VN_PK_HW, offsetof(vn_hw_t, f), sizeof(((vn_hw_t *)0)->f)}
#define FC(nm, f) {nm, VN_PK_IMUC, offsetof(imu_conv_cfg_t, f), sizeof(((imu_conv_cfg_t *)0)->f)}

/* TODOS los campos de nav_config_t (incl. eskf_params_t), vn_hw_t e imu_conv_cfg_t. Si se
   añade un campo a alguna de esas estructuras, cambia su tamaño y la prueba lo exige aquí y en
   la tabla. */
static const field_t FIELDS[] = {
    FE("eskf.sigma_a", sigma_a), FE("eskf.sigma_w", sigma_w), FE("eskf.sigma_aw", sigma_aw),
    FE("eskf.sigma_ww", sigma_ww), FE("eskf.mag_ref_n", mag_ref[0]), FE("eskf.mag_ref_e", mag_ref[1]),
    FE("eskf.mag_ref_d", mag_ref[2]), FE("eskf.has_mag_ref", has_mag_ref), FE("eskf.accel_threshold", accel_threshold),
    FE("eskf.innov_gate", innov_gate), FE("eskf.dt_max", dt_max), FE("eskf.gap_max", gap_max),
    FE("eskf.tau_max", tau_max), FE("eskf.align_acc_tol", align_acc_tol), FE("eskf.align_gyro_max", align_gyro_max),
    FE("eskf.p0_std", p0_std), FE("eskf.v0_std", v0_std), FE("eskf.th0_std", th0_std), FE("eskf.ba0_std", ba0_std),
    FE("eskf.bg0_std", bg0_std),
    FN(rest_window_s), FN(rest_acc_dev_max), FN(rest_acc_std_max), FN(rest_gyro_max), FN(rest_tilt_change_max),
    FN(rest_update_period_s), FN(zupt_sigma), FN(grav_sigma), FN(liftoff_acc), FN(liftoff_hold_s),
    FN(acc_sat_limit), FN(gyro_sat_limit), FN(acc_sat_max_s), FN(gyro_sat_max_s), FN(acc_sat_excess),
    FN(gyro_sat_excess), FN(baro_sigma), FN(baro_latency_s), FN(baro_timeout_s), FN(baro_ref_min_n),
    FN(baro_ref_max_step), FN(stale_max_s),
    FH("hw.i2c_hz", i2c_hz), FH("hw.pin_sda", pin_sda), FH("hw.pin_scl", pin_scl), FH("hw.pin_imu_int", pin_imu_int),
    FH("hw.pin_baro_int", pin_baro_int), FH("hw.imu_addr", imu_addr), FH("hw.baro_addr", baro_addr),
    FH("hw.imu_acc_div", imu_acc_div), FH("hw.imu_gyro_div", imu_gyro_div), FH("hw.imu_dlpf_cfg", imu_dlpf_cfg),
    FH("hw.imu_acc_fs_g", imu_acc_fs_g), FH("hw.imu_gyro_fs_dps", imu_gyro_fs_dps), FH("hw.mag_div", mag_div),
    FH("hw.baro_odr_hz", baro_odr_hz), FH("hw.baro_osr_p", baro_osr_p), FH("hw.baro_osr_t", baro_osr_t),
    FH("hw.baro_p0_pa", baro_p0_pa), FH("fw.merge_holdoff_us", merge_holdoff_us), FH("fw.navout_hz", navout_hz),
    FH("fw.telemetry_hz", telemetry_hz), FH("fw.stats_hz", stats_hz), FH("fw.header_period_s", header_period_s),
    FH("fw.log_navev", log_navev), FH("fw.log_ticks", log_ticks), FH("fw.stack_bytes", stack_bytes), FH("fw.ring_bytes", ring_bytes),
    FH("fw.queue_depth", queue_depth), FH("fw.i2c_timeout_ms", i2c_timeout_ms),
    FH("fw.irq_timeout_ms", irq_timeout_ms), FH("fw.baro_poll_us", baro_poll_us),
    FH("fw.imu_reinit_errors", imu_reinit_errors), FH("fw.boot_bench", boot_bench), FH("fw.core_acq", core_acq),
    FH("fw.core_nav", core_nav), FH("fw.core_log", core_log), FH("fw.prio_acq", prio_acq),
    FH("fw.prio_nav", prio_nav), FH("fw.prio_log", prio_log), FH("fw.axes_confirmed", axes_confirmed),
    FC("imu.acc_lsb_per_g", acc_lsb_per_g), FC("imu.gyro_lsb_per_dps", gyro_lsb_per_dps), FC("imu.g_si", g_si),
    FC("imu.axis_map_x", map[0]), FC("imu.axis_map_y", map[1]), FC("imu.axis_map_z", map[2]),
    FC("imu.axis_sign_x", sign[0]), FC("imu.axis_sign_y", sign[1]), FC("imu.axis_sign_z", sign[2]),
    FC("imu.sat_margin_lsb", sat_margin_lsb),
};

static size_t type_size(int t)
{
    switch (t) {
    case VN_T_REAL: return sizeof(eskf_real);
    case VN_T_F64: return 8;
    case VN_T_I32: case VN_T_U32: case VN_T_F32: return 4;
    case VN_T_I64: return 8;
    case VN_T_U8: case VN_T_I8: return 1;
    default: return 0;
    }
}

static const vn_param_t *find(const char *name)
{
    size_t n;
    const vn_param_t *T = vn_params_table(&n);
    for (size_t i = 0; i < n; i++)
        if (strcmp(T[i].name, name) == 0) return &T[i];
    return NULL;
}

static void test_params(void)
{
    printf("tabla de parámetros\n");
    /* tamaños de referencia (doble / float): si cambian, revisar FIELDS y la tabla */
#ifdef ESKF_USE_FLOAT
    CHECK(sizeof(eskf_params_t) == 19 * 4 + 4, "sizeof(eskf_params_t) = %zu: revisar campos", sizeof(eskf_params_t));
#else
    CHECK(sizeof(eskf_params_t) == 19 * 8 + 8, "sizeof(eskf_params_t) = %zu: revisar campos", sizeof(eskf_params_t));
#endif
    CHECK(sizeof(nav_config_t) == sizeof(eskf_params_t) + 22 * 8, "sizeof(nav_config_t) = %zu: revisar campos",
          sizeof(nav_config_t));
    CHECK(sizeof(imu_conv_cfg_t) == 3 * 4 + 3 + 3 + 2 + 4, "sizeof(imu_conv_cfg_t) = %zu", sizeof(imu_conv_cfg_t));
    CHECK(sizeof(vn_hw_t) == 33 * 4 + 6 * 8 + 4, /* 33 campos de 4 bytes + 6 de 8 + relleno final */
          "sizeof(vn_hw_t) = %zu: revisar campos", sizeof(vn_hw_t));

    size_t n;
    const vn_param_t *T = vn_params_table(&n);
    /* cada campo de las estructuras está en la tabla, en su sitio y con su tamaño */
    for (size_t i = 0; i < sizeof FIELDS / sizeof FIELDS[0]; i++) {
        const field_t *f = &FIELDS[i];
        const vn_param_t *p = find(f->name);
        CHECK(p != NULL, "%s no está en la tabla", f->name);
        if (!p) continue;
        CHECK(p->kind == f->kind, "%s: destino", f->name);
        CHECK(p->off == f->off, "%s: desplazamiento %zu != %zu", f->name, p->off, f->off);
        CHECK(type_size(p->type) == f->size, "%s: tamaño %zu != %zu", f->name, type_size(p->type), f->size);
    }
    /* cada entrada de la tabla (salvo documentación) es un campo conocido; nombres únicos */
    for (size_t i = 0; i < n; i++) {
        const vn_param_t *p = &T[i];
        int known = (p->kind == VN_PK_DOC);
        for (size_t k = 0; k < sizeof FIELDS / sizeof FIELDS[0] && !known; k++)
            known = strcmp(FIELDS[k].name, p->name) == 0;
        CHECK(known, "%s: entrada de tabla sin campo", p->name);
        int dup = 0;
        for (size_t j = i + 1; j < n; j++) dup |= strcmp(p->name, T[j].name) == 0;
        CHECK(!dup, "%s repetido", p->name);
        CHECK(strlen(p->name) < sizeof(((vn_rec_param_t *)0)->name), "%s: nombre largo", p->name);
        CHECK(p->source && strlen(p->source) > 0, "%s: sin origen", p->name);
        CHECK(p->source && strlen(p->source) < sizeof(((vn_rec_param_t *)0)->source),
              "%s: origen de más de 45 bytes (se truncaría en el registro)", p->name);
        /* estados: vuelo ∈ {confirmado, propuesto, pendiente}; banco ∈ {confirmado, propuesto, provisional} */
        CHECK(p->status == VN_ST_CONFIRMED || p->status == VN_ST_PROPOSED || p->status == VN_ST_PENDING,
              "%s: estado de vuelo %u", p->name, p->status);
        CHECK(p->bench_status == VN_ST_CONFIRMED || p->bench_status == VN_ST_PROPOSED ||
                  p->bench_status == VN_ST_PROVISIONAL,
              "%s: estado de banco %u", p->name, p->bench_status);
        /* lo no pendiente es igual en banco y vuelo: el banco sólo sustituye lo pendiente */
        if (p->status != VN_ST_PENDING) {
            CHECK(p->bench == p->flight && p->bench_status == p->status, "%s: banco difiere de vuelo sin ser pendiente",
                  p->name);
        } else {
            CHECK(p->bench_status == VN_ST_PROVISIONAL, "%s: pendiente en vuelo pero no provisional en banco", p->name);
        }
        CHECK(isfinite(p->bench), "%s: banco sin valor", p->name);
        CHECK(isfinite(p->flight) || p->status == VN_ST_PENDING, "%s: NaN sin estar pendiente", p->name);
    }
    /* Los tiempos de conversión del baro son documentación, NO la latencia */
    const vn_param_t *lat = find("baro_latency_s");
    CHECK(lat && lat->status == VN_ST_PENDING && isnan(lat->flight), "baro_latency_s debe seguir pendiente");
    CHECK(lat && lat->bench == 0.0, "banco: latencia 0 (sin compensar), no la conversión de la hoja de datos");
    const vn_param_t *ct = find("doc.baro_conv_typ_ms");
    CHECK(ct && ct->kind == VN_PK_DOC, "conversión típica: sólo documentación");

    nav_config_t cfg;
    imu_conv_cfg_t ic;
    vn_hw_t hw;
    const char *why = NULL;
    CHECK(vn_params_build(VN_PROFILE_ACQ, &cfg, &ic, &hw, &why) == 0 && strstr(why, "adquisicion"),
          "ADQ: sin navegación (%s)", why);
    CHECK(hw.imu_acc_div == 4 && hw.baro_odr_hz == 50 && hw.stack_bytes == 16384u, "ADQ: hardware cargado");
    CHECK(imu_conv_check(&ic) != 0, "ADQ: ejes de vuelo pendientes → conversión no válida");
    why = NULL;
    CHECK(vn_params_build(VN_PROFILE_FLIGHT, &cfg, &ic, &hw, &why) == 0, "VUELO: no arranca con pendientes");
    CHECK(why && strlen(why) > 0, "VUELO: motivo (%s)", why ? why : "(null)");
    printf("  VUELO hoy: navegación desactivada — %s\n", why ? why : "");
    why = NULL;
    CHECK(vn_params_build(VN_PROFILE_BENCH, &cfg, &ic, &hw, &why) == 1, "BANCO: arranca (%s)", why);
    CHECK(nav_config_check(&cfg) == NULL, "BANCO: configuración completa");
    CHECK(imu_conv_check(&ic) == 0 && ic.map[0] == 0 && ic.map[1] == 1 && ic.map[2] == 2, "BANCO: identidad");
    CHECK(hw.axes_confirmed == 0, "BANCO: ejes sin confirmar");
    CHECK(cfg.eskf.has_mag_ref == 0, "magnetómetro no fusionado");
    /* revisión v4, punto 12: modelos de puente y de saturación admitida, deshabilitados */
    CHECK(cfg.eskf.gap_max == 0 && cfg.acc_sat_max_s == 0.0 && cfg.gyro_sat_max_s == 0.0,
          "puente y saturación admitida deshabilitados (gap_max, *_sat_max_s = 0)");
    const vn_param_t *gm = find("eskf.gap_max");
    CHECK(gm && gm->flight == 0.0 && gm->bench == 0.0, "gap_max = 0 también en vuelo");
    CHECK(cfg.baro_latency_s == 0.0, "BANCO: latencia 0");
    CHECK(cfg.stale_max_s >= (double)hw.merge_holdoff_us * 1e-6 + (double)cfg.eskf.dt_max,
          "stale_max_s cubre la espera de ordenación + dt_max");
    nav_t nv;
    CHECK(nav_init(&nv, &cfg) == NAV_OK, "BANCO: nav_init");
}

/* ---------------------------------------------------------------- prueba de tiempos */
static uint32_t fake_cycles(void)
{
    static uint32_t c = 0;
    c += 240; /* 1 µs por llamada a 240 MHz */
    return c;
}

static void test_bench(void)
{
    printf("prueba de tiempos (recorrido)\n");
    nav_config_t cfg;
    imu_conv_cfg_t ic;
    vn_hw_t hw;
    const char *why;
    (void)vn_params_build(VN_PROFILE_BENCH, &cfg, &ic, &hw, &why);
    static nav_t nv;
    vn_bench_result_t r;
    int ret = vn_bench_run(&nv, &cfg, 225.0, fake_cycles, 240.0, &r);
    CHECK(ret == 0, "vn_bench_run = %d", ret);
    CHECK(r.aligned && r.liftoff && r.baro_used, "recorre alineación (%d), despegue (%d) y baro en vuelo (%d)",
          r.aligned, r.liftoff, r.baro_used);
    CHECK(r.align.n == 1, "una llamada que alinea (%u)", r.align.n);
    CHECK(r.imu_flight.n > 1000 && r.baro_flight.n > 200, "llamadas de vuelo: imu %u baro %u", r.imu_flight.n,
          r.baro_flight.n);
    CHECK(fabs(r.imu_pad.min - 1.0) < 1e-9 && fabs(r.imu_pad.max - 1.0) < 1e-9, "conversión ciclos→µs");
    printf("  imu_pad %u, align %u, imu_flight %u, baro_flight %u llamadas\n", r.imu_pad.n, r.align.n,
           r.imu_flight.n, r.baro_flight.n);
    nav_config_t bad = cfg;
    bad.baro_sigma = NAV_PENDING;
    CHECK(vn_bench_run(&nv, &bad, 225.0, fake_cycles, 240.0, &r) == -1 && r.init_ret == NAV_E_CONFIG,
          "configuración incompleta: no corre");
}

int main(void)
{
    printf("test_vn (%s)\n", sizeof(eskf_real) == 4 ? "float" : "double");
    test_frame();
    test_params();
    test_bench();
    printf("%d/%d comprobaciones correctas\n", g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}
