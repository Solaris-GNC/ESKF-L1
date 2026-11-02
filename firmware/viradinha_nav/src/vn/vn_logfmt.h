/*
 * vn_logfmt.h — formato binario del registro del Viradinha MKII (versión 1).
 *
 * C puro: lo compilan el firmware (Arduino/Xtensa) y las pruebas del ordenador, y
 * firmware/tools/vnlog.py lo decodifica con las mismas definiciones.
 *
 * Trama:  A5 5A | len (u16, bytes de carga) | tipo (u8) | carga | crc16 (u16)
 *   - todo en little-endian (ESP32-S3 y x86 lo son);
 *   - crc16-CCITT (polinomio 0x1021, inicial 0xFFFF) sobre tipo + carga;
 *   - el decodificador se resincroniza buscando A5 5A si una trama está corrupta.
 * Las estructuras son empaquetadas y de tamaño fijo (comprobado en compilación).
 */
#ifndef VN_LOGFMT_H
#define VN_LOGFMT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VN_LOG_FMT_VERSION 1
#define VN_SYNC0 0xA5u
#define VN_SYNC1 0x5Au
#define VN_FRAME_OVERHEAD 7u /* sync(2) + len(2) + tipo(1) + crc(2) */
#define VN_FRAME_MAX 256u

#define VN_PACKED __attribute__((packed))

enum {
    VN_REC_HEADER = 1,  /* versiones, placa, compilador, perfil */
    VN_REC_PARAM = 2,   /* un parámetro de configuración con su estado y origen */
    VN_REC_REG = 3,     /* registro de un sensor releído tras configurarlo */
    VN_REC_IMU = 4,     /* muestra cruda de la IMU */
    VN_REC_BARO = 5,    /* muestra del barómetro */
    VN_REC_NAVEV = 6,   /* retorno de navigation.c para un evento */
    VN_REC_NAVOUT = 7,  /* salida estimada */
    VN_REC_STATS = 8,   /* tiempos, pila, colas y pérdidas (periódico) */
    VN_REC_TEXT = 9,    /* mensaje */
    VN_REC_NAVCNT = 10, /* contadores y últimos códigos de navigation.c (periódico) */
    VN_REC_BAROCAL = 11, /* coeficientes NVM del BMP390 (para recalcular la compensación) */
    VN_REC_BENCH = 12,   /* medida de tiempos de navigation.c en la placa (prueba de arranque) */
    VN_REC_TICK = 13     /* reloj entregado a nav_tick() (para reproducir exactamente en el PC) */
};

/* Estado de cada parámetro (ver firmware/PARAMETROS.md) */
enum {
    VN_ST_CONFIRMED = 1,   /* confirmado por hardware o documentación */
    VN_ST_PROPOSED = 2,    /* configuración propuesta, sin comprobar */
    VN_ST_PROVISIONAL = 3, /* estimación provisional (sólo perfil de banco) */
    VN_ST_PENDING = 4      /* pendiente de medición o de información del equipo */
};

typedef struct VN_PACKED {
    char magic[8]; /* "VNLOG" + 0 0 + versión */
    uint16_t fmt_version;
    uint16_t sizeof_eskf_real;
    uint8_t profile; /* 0 adquisición, 1 banco, 2 vuelo */
    uint8_t nav_enabled;
    uint8_t imu_irq; /* 1: dato listo por interrupción; 0: sondeo */
    uint8_t baro_irq;
    uint32_t cpu_mhz;
    int64_t t_us; /* esp_timer al emitir la cabecera */
    uint32_t boot_id;
    char fw_version[16];
    char build[24];    /* __DATE__ __TIME__ */
    char compiler[40]; /* __VERSION__ */
    char idf[24];
    char core[24];
    char board[24];
} vn_rec_header_t;

typedef struct VN_PACKED {
    char name[40];
    double value; /* NaN si pendiente */
    uint8_t status;
    uint8_t used; /* 1 si este perfil lo usa */
    char source[46];
} vn_rec_param_t;

typedef struct VN_PACKED {
    uint8_t dev; /* 1 IMU, 2 baro */
    uint8_t bank;
    uint8_t addr;
    uint8_t mask;
    uint8_t expected;
    uint8_t read;
    uint8_t ok;
    uint8_t reserved;
} vn_rec_reg_t;

/* flags de vn_rec_imu_t */
#define VN_IMU_F_POLLED 0x01u   /* instante de sondeo, no de interrupción */
#define VN_IMU_F_I2C_ERR 0x02u  /* lectura fallida: valores no válidos */
#define VN_IMU_F_IRQ_LOST 0x04u /* se perdió alguna interrupción antes de ésta */
#define VN_IMU_F_ACC_NEW 0x08u  /* el acelerómetro cambió respecto a la lectura anterior */
#define VN_IMU_F_GYR_NEW 0x10u  /* el giróscopo cambió */
#define VN_IMU_F_MAG 0x20u      /* esta lectura incluye magnetómetro */
#define VN_IMU_F_RACE 0x40u     /* llegó otra interrupción durante la lectura: el dato puede ser
                                   de esa interrupción más reciente (la marca es de la anterior) */
#define VN_IMU_F_TIMEOUT 0x80u  /* sin interrupción en el plazo: lectura por sondeo de respaldo */

/* flags de vn_rec_baro_t */
#define VN_BARO_F_POLLED 0x01u
#define VN_BARO_F_I2C_ERR 0x02u
#define VN_BARO_F_IRQ 0x04u

typedef struct VN_PACKED {
    uint32_t seq;
    int64_t t_drdy_us; /* dato listo (interrupción) o instante de sondeo */
    int64_t t_read_us; /* fin de la lectura I2C */
    int16_t acc[3];    /* códigos crudos, ejes del SENSOR */
    int16_t gyro[3];
    int16_t temp;
    int16_t mag[3]; /* AK09916 crudo (sólo si VN_IMU_F_MAG) */
    uint8_t mag_st1, mag_st2;
    uint8_t sat; /* bits NAV_SAT_* en ejes del COHETE (tras imu_conv) */
    uint8_t flags;
} vn_rec_imu_t;

typedef struct VN_PACKED {
    uint32_t seq;
    int64_t t_prev_poll_us; /* último sondeo SIN dato nuevo */
    int64_t t_detect_us;    /* sondeo (o interrupción) que detectó el dato */
    int64_t t_read_us;      /* fin de la lectura */
    float pressure_pa;      /* compensada (API Bosch) */
    float temp_c;
    float alt_m;            /* altitud ISA entregada a navigation.c */
    uint8_t status;         /* registro STATUS del BMP390 */
    uint8_t flags;          /* VN_BARO_F_* */
} vn_rec_baro_t;

typedef struct VN_PACKED {
    uint32_t src_seq;
    uint8_t kind; /* 1 IMU, 2 baro (el reloj va en VN_REC_TICK) */
    int16_t ret;
    uint8_t validity;
    uint32_t reasons;
    uint16_t exec_us; /* tiempo de nav_on_imu / nav_on_baro (esp_timer) */
} vn_rec_navev_t;

typedef struct VN_PACKED {
    int64_t t_us; /* esp_timer al generar la salida */
    uint32_t n_events; /* llamadas a nav_on_imu/nav_on_baro hechas hasta esta salida */
    double t_state, t_eval, alt_ref;
    uint32_t reasons, alt_ref_seq, f_degraded;
    uint8_t validity, phase, at_rest, heading_known;
    float p[3], v[3], q[4], sigma_p[3], sigma_v[3], sigma_th[3], b_a[3], b_g[3];
} vn_rec_navout_t;

typedef struct VN_PACKED {
    int64_t t_us;
    uint32_t imu_irq_lost, imu_q_drop, baro_q_drop, ring_drop_recs, ring_drop_bytes;
    uint32_t i2c_err_imu, i2c_err_baro, merge_imu_overflow, merge_baro_overflow;
    uint32_t imu_events, baro_events, nav_events;
    uint32_t nav_exec_max_us, nav_exec_sum_us, acq_read_max_us, log_write_max_us;
    uint32_t hwm_acq, hwm_nav, hwm_log, hwm_loop; /* bytes libres mínimos (FreeRTOS, ESP-IDF: bytes) */
    uint32_t heap_free_min, ring_free_min, sink_bytes;
    uint32_t sink_discard_bytes; /* descartados por el sumidero (p. ej. USB sin conectar) */
} vn_rec_stats_t;

typedef struct VN_PACKED {
    char text[96];
} vn_rec_text_t;

/* nav_tick(t_us·1e-6) en una vuelta de vn_nav que entrega eventos o publica salida.
   Sólo importa el último reloj antes de cada evento: se registra uno por vuelta. */
typedef struct VN_PACKED {
    int64_t t_us;
    int16_t ret;
} vn_rec_tick_t;

#define VN_NAVCNT_N 32
typedef struct VN_PACKED {
    int64_t t_us;
    uint32_t cnt[VN_NAVCNT_N]; /* nav_counters_t, en orden de declaración */
    int16_t last[8];           /* nav_last_t: imu, predict, baro, zupt, grav, inflate, align, ref */
} vn_rec_navcnt_t;

typedef struct VN_PACKED {
    uint8_t addr;
    uint8_t chip_id;
    uint8_t nvm[21]; /* registros 0x31..0x45 */
} vn_rec_barocal_t;

/* Prueba de tiempos al arrancar (vn_app.cpp): una fila por tipo de llamada */
enum { VN_BENCH_IMU_PAD = 1, VN_BENCH_IMU_FLIGHT = 2, VN_BENCH_BARO_FLIGHT = 3, VN_BENCH_ALIGN = 4 };
typedef struct VN_PACKED {
    uint8_t what;       /* VN_BENCH_* */
    uint8_t profile;    /* perfil cuya configuración se usó */
    uint32_t n;
    float min_us, mean_us, max_us; /* contador de ciclos de la CPU (ESP.getCycleCount) */
    uint32_t over_dt;   /* llamadas que superaron el periodo nominal de la IMU */
    uint32_t hwm_after; /* pila libre mínima de la tarea tras la prueba (bytes) */
    uint32_t cpu_mhz;
} vn_rec_bench_t;

/* Comprobación de tamaños en compilación (C99) */
#define VN_STATIC_ASSERT(cond, name) typedef char vn_static_assert_##name[(cond) ? 1 : -1]
VN_STATIC_ASSERT(sizeof(vn_rec_header_t) == 184, header);
VN_STATIC_ASSERT(sizeof(vn_rec_stats_t) == 104, stats);
VN_STATIC_ASSERT(sizeof(vn_rec_bench_t) == 30, bench);
VN_STATIC_ASSERT(sizeof(vn_rec_barocal_t) == 23, barocal);
VN_STATIC_ASSERT(sizeof(vn_rec_param_t) == 96, param);
VN_STATIC_ASSERT(sizeof(vn_rec_reg_t) == 8, reg);
VN_STATIC_ASSERT(sizeof(vn_rec_text_t) == 96, text);
VN_STATIC_ASSERT(sizeof(vn_rec_imu_t) == 44, imu);
VN_STATIC_ASSERT(sizeof(vn_rec_baro_t) == 42, baro);
VN_STATIC_ASSERT(sizeof(vn_rec_navev_t) == 14, navev);
VN_STATIC_ASSERT(sizeof(vn_rec_navout_t) == 152, navout);
VN_STATIC_ASSERT(sizeof(vn_rec_tick_t) == 10, tick);
VN_STATIC_ASSERT(sizeof(vn_rec_navcnt_t) == 152, navcnt);
VN_STATIC_ASSERT(sizeof(vn_rec_navout_t) + VN_FRAME_OVERHEAD <= VN_FRAME_MAX, frame);

uint16_t vn_crc16(const uint8_t *p, size_t n, uint16_t crc);
/* Escribe una trama en out (cap bytes). Devuelve su longitud, o 0 si no cabe. */
size_t vn_frame(uint8_t *out, size_t cap, uint8_t type, const void *payload, uint16_t len);
/* Copia texto con terminación garantizada (rellena con ceros) */
void vn_strfill(char *dst, size_t n, const char *src);

#ifdef __cplusplus
}
#endif

#endif
