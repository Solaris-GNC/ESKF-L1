/*
 * vn_app.cpp — ver vn_app.h. Arduino-ESP32 2.0.x (ESP-IDF 4.4, FreeRTOS SMP de Espressif).
 *
 * Contrato de tiempos:
 *   - Toda marca sale de esp_timer_get_time() (µs, int64 desde el arranque). navigation.c
 *     recibe t = µs·1e-6 (double: exacto en µs durante > 100 años).
 *   - IMU: t = instante de la interrupción de dato listo (o del sondeo que lo detectó, marcado).
 *   - Baro: t = fin de la lectura; la latencia del sensor la aplica navigation.c con
 *     baro_latency_s, medido con estas mismas marcas (PROCEDIMIENTOS.md).
 *   - vn_nav llama a nav_tick() con el reloj en cada vuelta, y entrega los eventos en orden
 *     de tiempo con nav_merge (espera fw.merge_holdoff_us si falta el otro sensor).
 * Pila: en ESP-IDF StackType_t es uint8_t → profundidades y marcas de agua en BYTES.
 */
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <freertos/task.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../nav/imu_conv.h"
#include "../nav/nav_events.h"
#include "../nav/navigation.h"
#include "vn_app.h"
#include "vn_bench.h"
#include "vn_logfmt.h"
#include "vn_params.h"
#include "vn_profile.h"
#include "vn_sensors.h"

static_assert(sizeof(nav_counters_t) <= sizeof(((vn_rec_navcnt_t *)0)->cnt), "nav_counters_t no cabe en NAVCNT");
static_assert(sizeof(StackType_t) == 1, "se espera ESP-IDF: pila en bytes");
#ifndef ESKF_FORCE_DOUBLE
/* eskf.h activa ESKF_USE_FLOAT con ARDUINO_ARCH_ESP32, que el núcleo pasa por línea de órdenes a
   TODAS las unidades (.c y .cpp); vn_params.c comprueba lo mismo en C. */
static_assert(sizeof(eskf_real) == 4, "en la placa eskf_real debe ser float en todas las unidades");
#endif

/* ------------------------------------------------------------------ estado */
static nav_config_t g_cfg;
static imu_conv_cfg_t g_ic;     /* conversión para la navegación (válida si g_nav_enabled) */
static imu_conv_cfg_t g_ic_log; /* conversión para las marcas de saturación del registro */
static vn_hw_t g_hw;
static volatile int g_nav_enabled;
static const char *g_nav_why = "";
static uint32_t g_boot_id;
static uint8_t g_imu_irq, g_baro_irq;

static RingbufHandle_t g_rb;
static QueueHandle_t g_q;
static TaskHandle_t g_t_acq, g_t_nav, g_t_log, g_t_loop;
static volatile int g_nav_ready; /* vn_nav ya acepta eventos */

static nav_t g_nav;         /* SÓLO vn_nav (salvo la prueba de arranque, en la misma tarea) */
static uint32_t g_nav_calls; /* SÓLO vn_nav: llamadas a nav_on_imu/nav_on_baro */
static nav_merge_t g_merge; /* SÓLO vn_nav */

/* ISR → vn_acq. 64 bits no son atómicos en Xtensa LX7: spinlock. */
static portMUX_TYPE g_isr_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t g_isr_imu_t, g_isr_baro_t;
static volatile uint32_t g_isr_imu_n, g_isr_baro_n;

/* vn_nav → cualquier tarea */
static portMUX_TYPE g_snap_mux = portMUX_INITIALIZER_UNLOCKED;
static nav_output_t g_snap;
static uint32_t g_snap_deg;
static int g_snap_ok;

/* relecturas de registros y calibración del baro (se repiten con la cabecera) */
static portMUX_TYPE g_reg_mux = portMUX_INITIALIZER_UNLOCKED;
static vn_rec_reg_t g_regs_imu[16], g_regs_baro[8];
static int g_nregs_imu, g_nregs_baro;
static vn_rec_barocal_t g_barocal;
static int g_have_barocal;
static vn_rec_bench_t g_bench[4];
static int g_nbench;

/* Contadores. Acumulados salvo los *_max/_sum (por periodo de STATS, se ponen a 0 al leer). */
typedef struct {
    uint32_t imu_irq_lost, imu_q_drop, baro_q_drop, ring_drop_recs, ring_drop_bytes;
    uint32_t i2c_err_imu, i2c_err_baro, imu_events, baro_events, nav_events;
    uint32_t nav_exec_max_us, nav_exec_sum_us, acq_read_max_us, log_write_max_us;
    uint32_t ring_free_min, sink_bytes, sink_discard_bytes;
} vn_counters_t;
static vn_counters_t g_c;

static inline void c_add(uint32_t *x, uint32_t v) { __atomic_fetch_add(x, v, __ATOMIC_RELAXED); }
static inline void c_max(uint32_t *x, uint32_t v)
{
    uint32_t cur = __atomic_load_n(x, __ATOMIC_RELAXED);
    while (v > cur && !__atomic_compare_exchange_n(x, &cur, v, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
    }
}
static inline void c_min(uint32_t *x, uint32_t v)
{
    uint32_t cur = __atomic_load_n(x, __ATOMIC_RELAXED);
    while (v < cur && !__atomic_compare_exchange_n(x, &cur, v, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
    }
}
static inline uint32_t c_take(uint32_t *x, uint32_t reset) { return __atomic_exchange_n(x, reset, __ATOMIC_RELAXED); }
static inline uint32_t c_get(const uint32_t *x) { return __atomic_load_n(x, __ATOMIC_RELAXED); }
static inline uint32_t sat_u32(int64_t v) { return v < 0 ? 0u : (v > 0xFFFFFFFFll ? 0xFFFFFFFFu : (uint32_t)v); }

typedef struct {
    uint8_t kind; /* NAV_EV_IMU / NAV_EV_BARO */
    union {
        nav_ev_imu_t imu;
        nav_ev_baro_t baro;
    } u;
} vn_qitem_t;

/* ------------------------------------------------------------------ registro */
static int log_put(uint8_t type, const void *p, uint16_t len, TickType_t wait)
{
    uint8_t buf[VN_FRAME_MAX];
    size_t n = vn_frame(buf, sizeof buf, type, p, len);
    if (n == 0 || g_rb == NULL) {
        c_add(&g_c.ring_drop_recs, 1u);
        return 0;
    }
    if (xRingbufferSend(g_rb, buf, n, wait) != pdTRUE) {
        c_add(&g_c.ring_drop_recs, 1u);
        c_add(&g_c.ring_drop_bytes, (uint32_t)n);
        return 0;
    }
    c_min(&g_c.ring_free_min, (uint32_t)xRingbufferGetCurFreeSize(g_rb));
    return 1;
}

static void log_vtext(TickType_t wait, const char *fmt, va_list ap)
{
    vn_rec_text_t t;
    memset(&t, 0, sizeof t);
    vsnprintf(t.text, sizeof t.text, fmt, ap);
    log_put(VN_REC_TEXT, &t, sizeof t, wait);
}

extern "C" void vn_log_text(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vtext(0, fmt, ap);
    va_end(ap);
}

static void log_text_wait(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vtext(pdMS_TO_TICKS(50), fmt, ap);
    va_end(ap);
}

/* ------------------------------------------------------------------ ganchos débiles */
extern "C" __attribute__((weak)) void vn_sink_begin(void) { Serial.begin(921600); /* USB CDC: baudios ignorados */ }

extern "C" __attribute__((weak)) size_t vn_sink_write(const uint8_t *data, size_t len, size_t *discarded)
{
    *discarded = 0;
    if (!Serial) { /* sin anfitrión USB: se descarta y se cuenta */
        *discarded = len;
        return len;
    }
    int av = Serial.availableForWrite();
    if (av <= 0) return 0;
    size_t n = len < (size_t)av ? len : (size_t)av;
    return Serial.write(data, n);
}

extern "C" __attribute__((weak)) void vn_telemetry_publish(const nav_output_t *out, uint32_t f_degraded)
{
    (void)out;
    (void)f_degraded;
}

extern "C" int vn_nav_snapshot(nav_output_t *out, uint32_t *f_degraded)
{
    portENTER_CRITICAL(&g_snap_mux);
    int ok = g_snap_ok;
    if (ok) {
        *out = g_snap;
        if (f_degraded) *f_degraded = g_snap_deg;
    }
    portEXIT_CRITICAL(&g_snap_mux);
    return ok;
}

/* ------------------------------------------------------------------ ISR */
static void IRAM_ATTR isr_imu(void)
{
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&g_isr_mux);
    g_isr_imu_t = t;
    g_isr_imu_n = g_isr_imu_n + 1u;
    portEXIT_CRITICAL_ISR(&g_isr_mux);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_t_acq, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}

static void IRAM_ATTR isr_baro(void)
{
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&g_isr_mux);
    g_isr_baro_t = t;
    g_isr_baro_n = g_isr_baro_n + 1u;
    portEXIT_CRITICAL_ISR(&g_isr_mux);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_t_acq, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}

static void isr_get(int baro, int64_t *t, uint32_t *n)
{
    portENTER_CRITICAL(&g_isr_mux);
    *t = baro ? g_isr_baro_t : g_isr_imu_t;
    *n = baro ? g_isr_baro_n : g_isr_imu_n;
    portEXIT_CRITICAL(&g_isr_mux);
}

/* ------------------------------------------------------------------ adquisición */
static void reg_cb(const vn_rec_reg_t *r)
{
    log_put(VN_REC_REG, r, sizeof *r, pdMS_TO_TICKS(10));
    portENTER_CRITICAL(&g_reg_mux);
    if (r->dev == 1 && g_nregs_imu < 16) g_regs_imu[g_nregs_imu++] = *r;
    if (r->dev == 2 && g_nregs_baro < 8) g_regs_baro[g_nregs_baro++] = *r;
    portEXIT_CRITICAL(&g_reg_mux);
}

static int imu_start(void)
{
    portENTER_CRITICAL(&g_reg_mux);
    g_nregs_imu = 0;
    portEXIT_CRITICAL(&g_reg_mux);
    uint8_t addr = 0;
    int r = vn_imu_begin(&g_hw, reg_cb, &addr);
    vn_log_text("IMU begin=%d dir=0x%02X irq=%d", r, addr, (int)g_imu_irq);
    return r == 0;
}

static int baro_start(void)
{
    portENTER_CRITICAL(&g_reg_mux);
    g_nregs_baro = 0;
    portEXIT_CRITICAL(&g_reg_mux);
    vn_rec_barocal_t cal;
    memset(&cal, 0, sizeof cal);
    int r = vn_baro_begin(&g_hw, g_baro_irq, reg_cb, &cal);
    vn_log_text("BARO begin=%d dir=0x%02X id=0x%02X irq=%d", r, cal.addr, cal.chip_id, (int)g_baro_irq);
    if (r == 0) {
        portENTER_CRITICAL(&g_reg_mux);
        g_barocal = cal;
        g_have_barocal = 1;
        portEXIT_CRITICAL(&g_reg_mux);
        log_put(VN_REC_BAROCAL, &cal, sizeof cal, pdMS_TO_TICKS(10));
    }
    return r == 0;
}

static void acq_task(void *arg)
{
    (void)arg;
    /* la ISR notifica a esta tarea: fijar el manejador antes de conectar ninguna interrupción
       (xTaskCreatePinnedToCore puede no haber vuelto aún si esta tarea tiene más prioridad) */
    g_t_acq = xTaskGetCurrentTaskHandle();
    const vn_hw_t *hw = &g_hw;
    int imu_ok = 0, baro_ok = 0, irq_attached = 0;
    int64_t imu_retry = 0, baro_retry = 0;
    uint32_t imu_seq = 0, baro_seq = 0, isr_seen = 0, isrb_seen = 0;
    uint32_t imu_consec_err = 0, baro_consec_err = 0;
    int16_t prev_acc[3] = {0, 0, 0}, prev_gyr[3] = {0, 0, 0};
    int have_prev = 0;
    int64_t baro_last_poll = 0, baro_prev_nodata = 0;
    const float nanf_ = NAN;

    for (;;) {
        int64_t now = esp_timer_get_time();
        if (!imu_ok && now >= imu_retry) {
            imu_ok = imu_start();
            imu_retry = now + 1000000;
            have_prev = 0;
            imu_consec_err = 0;
            if (imu_ok && g_imu_irq && !irq_attached) {
                pinMode(hw->pin_imu_int, INPUT);
                attachInterrupt(hw->pin_imu_int, isr_imu, RISING);
                irq_attached = 1;
            }
            if (imu_ok) {
                uint32_t n;
                int64_t t;
                isr_get(0, &t, &n);
                isr_seen = n; /* no contar como perdidas las de antes de configurar */
            }
        }
        if (!baro_ok && now >= baro_retry) {
            baro_ok = baro_start();
            baro_retry = now + 1000000;
            baro_consec_err = 0;
            if (baro_ok && g_baro_irq) {
                pinMode(hw->pin_baro_int, INPUT);
                attachInterrupt(hw->pin_baro_int, isr_baro, RISING);
                uint32_t n;
                int64_t t;
                isr_get(1, &t, &n);
                isrb_seen = n;
            }
        }

        /* espera: interrupción (con plazo) o un tick de sondeo (1 ms) */
        int timed_out = 0;
        if (imu_ok && g_imu_irq) timed_out = (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(hw->irq_timeout_ms)) == 0);
        else vTaskDelay(1);

        /* ---------------- IMU ---------------- */
        if (imu_ok) {
            int have = 0;
            int64_t t_drdy = 0;
            uint8_t flags = 0;
            uint32_t lost = 0;
            if (g_imu_irq) {
                int64_t t;
                uint32_t n;
                isr_get(0, &t, &n);
                if (n != isr_seen) {
                    have = 1;
                    t_drdy = t;
                    lost = n - isr_seen - 1u;
                    isr_seen = n;
                } else if (timed_out) { /* respaldo: la línea de interrupción no responde */
                    int64_t tp = esp_timer_get_time();
                    int r = vn_imu_poll_drdy();
                    if (r == 1) {
                        have = 1;
                        t_drdy = tp;
                        flags |= VN_IMU_F_POLLED | VN_IMU_F_TIMEOUT;
                    } else if (r < 0) {
                        c_add(&g_c.i2c_err_imu, 1u);
                        imu_consec_err++;
                    }
                }
            } else {
                int64_t tp = esp_timer_get_time();
                int r = vn_imu_poll_drdy();
                if (r == 1) {
                    have = 1;
                    t_drdy = tp;
                    flags |= VN_IMU_F_POLLED;
                } else if (r < 0) {
                    c_add(&g_c.i2c_err_imu, 1u);
                    imu_consec_err++;
                }
            }
            if (have) {
                vn_rec_imu_t rec;
                memset(&rec, 0, sizeof rec);
                int16_t acc[3], gyr[3], temp = 0;
                int64_t t0 = esp_timer_get_time();
                int r = vn_imu_read(acc, gyr, &temp);
                int64_t t1 = esp_timer_get_time();
                if (g_imu_irq) {
                    int64_t t;
                    uint32_t n;
                    isr_get(0, &t, &n);
                    if (n != isr_seen) flags |= VN_IMU_F_RACE;
                }
                if (lost) {
                    flags |= VN_IMU_F_IRQ_LOST;
                    c_add(&g_c.imu_irq_lost, lost);
                }
                rec.seq = imu_seq;
                rec.t_drdy_us = t_drdy;
                rec.t_read_us = t1;
                float a_si[3] = {nanf_, nanf_, nanf_}, w_si[3] = {nanf_, nanf_, nanf_};
                uint8_t sat = 0;
                if (r != 0) {
                    flags |= VN_IMU_F_I2C_ERR;
                    c_add(&g_c.i2c_err_imu, 1u);
                    imu_consec_err++;
                } else {
                    imu_consec_err = 0;
                    if (!have_prev || memcmp(acc, prev_acc, sizeof acc) != 0) flags |= VN_IMU_F_ACC_NEW;
                    if (!have_prev || memcmp(gyr, prev_gyr, sizeof gyr) != 0) flags |= VN_IMU_F_GYR_NEW;
                    memcpy(prev_acc, acc, sizeof acc);
                    memcpy(prev_gyr, gyr, sizeof gyr);
                    have_prev = 1;
                    memcpy(rec.acc, acc, sizeof acc);
                    memcpy(rec.gyro, gyr, sizeof gyr);
                    rec.temp = temp;
                    sat = imu_conv(&g_ic_log, acc, gyr, a_si, w_si);
                    if (hw->mag_div > 0 && (imu_seq % (uint32_t)hw->mag_div) == 0u) {
                        int16_t mag[3];
                        uint8_t st1 = 0, st2 = 0;
                        if (vn_imu_read_mag(mag, &st1, &st2) == 0) {
                            memcpy(rec.mag, mag, sizeof mag);
                            rec.mag_st1 = st1;
                            rec.mag_st2 = st2;
                            flags |= VN_IMU_F_MAG;
                        }
                    }
                }
                rec.sat = sat;
                rec.flags = flags;
                log_put(VN_REC_IMU, &rec, sizeof rec, 0);
                c_max(&g_c.acq_read_max_us, sat_u32(t1 - t0));
                c_add(&g_c.imu_events, 1u);

                /* Se entrega a la navegación TODA lectura (también si ningún sensor cambió:
                   un valor retenido durante su propio periodo no sesga el modelo de ruido, y
                   omitir lecturas «iguales» por azar abriría huecos falsos). Si la lectura
                   falló, valores NaN: navigation.c cuenta el tiempo y rompe la continuidad. */
                if (g_nav_enabled && g_nav_ready) {
                    vn_qitem_t it;
                    it.kind = NAV_EV_IMU;
                    it.u.imu.t_us = t_drdy;
                    it.u.imu.seq = imu_seq;
                    it.u.imu.s.t = (double)t_drdy * 1e-6;
                    memcpy(it.u.imu.s.acc, a_si, sizeof a_si);
                    memcpy(it.u.imu.s.gyro, w_si, sizeof w_si);
                    it.u.imu.s.sat = sat;
                    if (xQueueSend(g_q, &it, 0) != pdTRUE) c_add(&g_c.imu_q_drop, 1u);
                }
                imu_seq++;
            }
            if (imu_consec_err >= (uint32_t)hw->imu_reinit_errors) {
                vn_log_text("IMU: %u errores I2C seguidos: reinicio", (unsigned)imu_consec_err);
                imu_ok = 0;
                imu_retry = 0;
            }
        }

        /* ---------------- baro ---------------- */
        if (baro_ok) {
            int due = 0;
            uint8_t bflags = 0, status = 0;
            int64_t t_det = 0;
            if (g_baro_irq) {
                int64_t t;
                uint32_t n;
                isr_get(1, &t, &n);
                if (n != isrb_seen) {
                    isrb_seen = n;
                    due = 1;
                    t_det = t;
                    bflags |= VN_BARO_F_IRQ;
                }
            } else {
                int64_t tp = esp_timer_get_time();
                if (tp - baro_last_poll >= hw->baro_poll_us) {
                    baro_last_poll = tp;
                    int r = vn_baro_poll(&status);
                    if (r == 1) {
                        due = 1;
                        t_det = tp;
                        bflags |= VN_BARO_F_POLLED;
                    } else if (r == 0) {
                        baro_prev_nodata = tp;
                    } else {
                        c_add(&g_c.i2c_err_baro, 1u);
                        baro_consec_err++;
                    }
                }
            }
            if (due) {
                double p = NAN, tc = NAN;
                int r = vn_baro_read(&p, &tc);
                int64_t t_r = esp_timer_get_time();
                vn_rec_baro_t b;
                memset(&b, 0, sizeof b);
                b.seq = baro_seq;
                b.t_prev_poll_us = baro_prev_nodata;
                b.t_detect_us = t_det;
                b.t_read_us = t_r;
                b.status = status;
                double alt = NAN;
                if (r != 0) {
                    bflags |= VN_BARO_F_I2C_ERR;
                    c_add(&g_c.i2c_err_baro, 1u);
                    baro_consec_err++;
                } else {
                    baro_consec_err = 0;
                    alt = 44330.0 * (1.0 - pow(p / hw->baro_p0_pa, 0.1903));
                }
                b.pressure_pa = (float)p;
                b.temp_c = (float)tc;
                b.alt_m = (float)alt;
                b.flags = bflags;
                log_put(VN_REC_BARO, &b, sizeof b, 0);
                c_add(&g_c.baro_events, 1u);
                if (r == 0 && isfinite(alt) && g_nav_enabled && g_nav_ready) {
                    vn_qitem_t it;
                    it.kind = NAV_EV_BARO;
                    it.u.baro.t_us = t_r;
                    it.u.baro.seq = baro_seq;
                    it.u.baro.s.t = (double)t_r * 1e-6;
                    it.u.baro.s.alt_m = (float)alt;
                    if (xQueueSend(g_q, &it, 0) != pdTRUE) c_add(&g_c.baro_q_drop, 1u);
                }
                baro_seq++;
            }
            if (baro_consec_err >= (uint32_t)hw->imu_reinit_errors) {
                vn_log_text("BARO: %u errores I2C seguidos: reinicio", (unsigned)baro_consec_err);
                baro_ok = 0;
                baro_retry = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------ navegación */
static uint32_t cpu_cycles(void) { return ESP.getCycleCount(); }

static void bench_rec(uint8_t what, int profile, const vn_bench_acc_t *a, uint32_t hwm)
{
    if (g_nbench >= 4) return;
    vn_rec_bench_t b;
    memset(&b, 0, sizeof b);
    b.what = what;
    b.profile = (uint8_t)profile;
    b.n = a->n;
    b.min_us = (float)a->min;
    b.mean_us = a->n ? (float)(a->sum / a->n) : NAN;
    b.max_us = (float)a->max;
    b.over_dt = a->over;
    b.hwm_after = hwm;
    b.cpu_mhz = getCpuFrequencyMhz();
    g_bench[g_nbench++] = b;
    log_put(VN_REC_BENCH, &b, sizeof b, pdMS_TO_TICKS(50));
}

static void run_boot_bench(void)
{
    nav_config_t bc;
    imu_conv_cfg_t bic;
    vn_hw_t bhw;
    const char *why = "";
    const nav_config_t *cfg = &g_cfg;
    int prof = VN_PROFILE;
    if (!g_nav_enabled) { /* sin configuración de vuelo completa: valores provisionales de banco */
        (void)vn_params_build(VN_PROFILE_BENCH, &bc, &bic, &bhw, &why);
        cfg = &bc;
        prof = VN_PROFILE_BENCH;
    }
    vn_bench_result_t r;
    double fs = 1125.0 / (1.0 + (double)g_hw.imu_acc_div);
    int ret = vn_bench_run(&g_nav, cfg, fs, cpu_cycles, (double)getCpuFrequencyMhz(), &r);
    uint32_t hwm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    bench_rec(VN_BENCH_IMU_PAD, prof, &r.imu_pad, hwm);
    bench_rec(VN_BENCH_ALIGN, prof, &r.align, hwm);
    bench_rec(VN_BENCH_IMU_FLIGHT, prof, &r.imu_flight, hwm);
    bench_rec(VN_BENCH_BARO_FLIGHT, prof, &r.baro_flight, hwm);
    log_text_wait("BENCH ret=%d init=%d alin=%d desp=%d baro=%d hwm=%u perfil=%d", ret, r.init_ret, r.aligned,
                  r.liftoff, r.baro_used, (unsigned)hwm, prof);
}

static void emit_navout(int64_t now, const nav_output_t *o)
{
    vn_rec_navout_t r;
    memset(&r, 0, sizeof r);
    r.t_us = now;
    r.n_events = g_nav_calls;
    r.t_state = o->t_state;
    r.t_eval = o->t_eval;
    r.alt_ref = o->alt_ref;
    r.reasons = o->reasons;
    r.alt_ref_seq = o->alt_ref_seq;
    r.f_degraded = (uint32_t)g_nav.f.degraded;
    r.validity = o->validity;
    r.phase = o->phase;
    r.at_rest = o->at_rest;
    r.heading_known = o->heading_known;
    memcpy(r.p, o->p, sizeof r.p);
    memcpy(r.v, o->v, sizeof r.v);
    memcpy(r.q, o->q, sizeof r.q);
    memcpy(r.sigma_p, o->sigma_p, sizeof r.sigma_p);
    memcpy(r.sigma_v, o->sigma_v, sizeof r.sigma_v);
    memcpy(r.sigma_th, o->sigma_th, sizeof r.sigma_th);
    memcpy(r.b_a, o->b_a, sizeof r.b_a);
    memcpy(r.b_g, o->b_g, sizeof r.b_g);
    log_put(VN_REC_NAVOUT, &r, sizeof r, 0);
}

static int16_t clamp16(int v) { return (int16_t)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v)); }

static void emit_navcnt(int64_t now)
{
    vn_rec_navcnt_t c;
    memset(&c, 0, sizeof c);
    c.t_us = now;
    memcpy(c.cnt, &g_nav.cnt, sizeof g_nav.cnt);
    const nav_last_t *l = &g_nav.last;
    c.last[0] = clamp16(l->imu);
    c.last[1] = clamp16(l->predict);
    c.last[2] = clamp16(l->baro);
    c.last[3] = clamp16(l->zupt);
    c.last[4] = clamp16(l->grav);
    c.last[5] = clamp16(l->inflate);
    c.last[6] = clamp16(l->align);
    c.last[7] = clamp16(l->ref);
    log_put(VN_REC_NAVCNT, &c, sizeof c, 0);
}

static void log_navev(uint32_t seq, uint8_t kind, int ret, uint32_t exec_us)
{
    vn_rec_navev_t e;
    uint32_t reasons = 0;
    e.src_seq = seq;
    e.kind = kind;
    e.ret = clamp16(ret);
    e.validity = (uint8_t)nav_validity(&g_nav, &reasons);
    e.reasons = reasons;
    e.exec_us = (uint16_t)(exec_us > 0xFFFFu ? 0xFFFFu : exec_us);
    log_put(VN_REC_NAVEV, &e, sizeof e, 0);
}

/* Devuelve 1 (registrado o desactivado: no volver a intentarlo en esta vuelta) */
static int log_tick(int64_t now, int ret)
{
    if (!g_hw.log_ticks) return 1;
    vn_rec_tick_t k;
    k.t_us = now;
    k.ret = clamp16(ret);
    log_put(VN_REC_TICK, &k, sizeof k, 0);
    return 1;
}

static int64_t period_us(double hz) { return (hz > 0.0) ? (int64_t)(1e6 / hz) : INT64_MAX; }

static void nav_task(void *arg)
{
    (void)arg;
    if (g_hw.boot_bench) run_boot_bench();
    if (!g_nav_enabled) {
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000)); /* sigue viva para medir su pila */
    }
    nav_merge_init(&g_merge);
    int r = nav_init(&g_nav, &g_cfg);
    if (r != NAV_OK) {
        log_text_wait("nav_init=%d: navegacion desactivada", r);
        g_nav_enabled = 0;
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    xQueueReset(g_q);
    g_nav_ready = 1;
    log_text_wait("navegacion activa, perfil %s", vn_profile_name(VN_PROFILE));

    const int64_t p_out = period_us(g_hw.navout_hz), p_tel = period_us(g_hw.telemetry_hz),
                  p_cnt = period_us(g_hw.stats_hz);
    int64_t next_out = 0, next_tel = 0, next_cnt = 0;
    for (;;) {
        vn_qitem_t it;
        if (xQueueReceive(g_q, &it, pdMS_TO_TICKS(2)) == pdTRUE) {
            do {
                if (it.kind == NAV_EV_IMU) nav_merge_push_imu(&g_merge, &it.u.imu);
                else nav_merge_push_baro(&g_merge, &it.u.baro);
            } while (xQueueReceive(g_q, &it, 0) == pdTRUE);
        }
        int64_t now = esp_timer_get_time();
        int rt = nav_tick(&g_nav, (double)now * 1e-6);
        int want_out = now >= next_out, want_tel = now >= next_tel;
        int tick_logged = 0;
        if (want_out || want_tel) tick_logged = log_tick(now, rt);
        for (;;) {
            nav_ev_imu_t ei;
            nav_ev_baro_t eb;
            nav_ev_kind_t k = nav_merge_pop(&g_merge, now, g_hw.merge_holdoff_us, &ei, &eb);
            if (k == NAV_EV_NONE) break;
            if (!tick_logged) tick_logged = log_tick(now, rt);
            int64_t t0 = esp_timer_get_time();
            int ret = (k == NAV_EV_IMU) ? nav_on_imu(&g_nav, &ei.s) : nav_on_baro(&g_nav, &eb.s);
            uint32_t ex = sat_u32(esp_timer_get_time() - t0);
            c_max(&g_c.nav_exec_max_us, ex);
            c_add(&g_c.nav_exec_sum_us, ex);
            c_add(&g_c.nav_events, 1u);
            g_nav_calls++;
            if (g_hw.log_navev) log_navev(k == NAV_EV_IMU ? ei.seq : eb.seq, (uint8_t)k, ret, ex);
        }
        if (want_out || want_tel) {
            nav_output_t o;
            nav_get_output(&g_nav, &o);
            uint32_t deg = (uint32_t)g_nav.f.degraded;
            portENTER_CRITICAL(&g_snap_mux);
            g_snap = o;
            g_snap_deg = deg;
            g_snap_ok = 1;
            portEXIT_CRITICAL(&g_snap_mux);
            if (want_out) {
                emit_navout(now, &o);
                next_out = now + p_out;
            }
            if (want_tel) {
                vn_telemetry_publish(&o, deg);
                next_tel = now + p_tel;
            }
        }
        if (now >= next_cnt) {
            emit_navcnt(now);
            next_cnt = now + p_cnt;
        }
    }
}

/* ------------------------------------------------------------------ registro → sumidero */
static void log_task(void *arg)
{
    (void)arg;
    for (;;) {
        size_t n = 0;
        uint8_t *p = (uint8_t *)xRingbufferReceiveUpTo(g_rb, &n, pdMS_TO_TICKS(20), 1024);
        if (p == NULL) continue;
        int64_t t0 = esp_timer_get_time();
        size_t off = 0;
        while (off < n) {
            size_t disc = 0;
            size_t w = vn_sink_write(p + off, n - off, &disc);
            if (w > n - off) w = n - off;
            if (disc > w) disc = w;
            c_add(&g_c.sink_discard_bytes, (uint32_t)disc);
            c_add(&g_c.sink_bytes, (uint32_t)(w - disc));
            off += w;
            if (w == 0) vTaskDelay(1);
        }
        vRingbufferReturnItem(g_rb, p);
        c_max(&g_c.log_write_max_us, sat_u32(esp_timer_get_time() - t0));
    }
}

/* ------------------------------------------------------------------ cabecera y estadísticas */
static void emit_header_block(void)
{
    const TickType_t w = pdMS_TO_TICKS(50);
    vn_rec_header_t h;
    memset(&h, 0, sizeof h);
    memcpy(h.magic, "VNLOG\0\0", 7);
    h.magic[7] = (char)VN_LOG_FMT_VERSION;
    h.fmt_version = VN_LOG_FMT_VERSION;
    h.sizeof_eskf_real = (uint16_t)sizeof(eskf_real);
    h.profile = (uint8_t)VN_PROFILE;
    h.nav_enabled = (uint8_t)(g_nav_enabled ? 1 : 0);
    h.imu_irq = g_imu_irq;
    h.baro_irq = g_baro_irq;
    h.cpu_mhz = getCpuFrequencyMhz();
    h.t_us = esp_timer_get_time();
    h.boot_id = g_boot_id;
    vn_strfill(h.fw_version, sizeof h.fw_version, VN_FW_VERSION);
    vn_strfill(h.build, sizeof h.build, __DATE__ " " __TIME__);
    vn_strfill(h.compiler, sizeof h.compiler, "gcc " __VERSION__);
    vn_strfill(h.idf, sizeof h.idf, esp_get_idf_version());
    char core[24];
    snprintf(core, sizeof core, "arduino-esp32 %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
             ESP_ARDUINO_VERSION_PATCH);
    vn_strfill(h.core, sizeof h.core, core);
    vn_strfill(h.board, sizeof h.board, ARDUINO_BOARD);
    log_put(VN_REC_HEADER, &h, sizeof h, w);

    if (VN_PROFILE == VN_PROFILE_BENCH) log_text_wait("PERFIL DE BANCO: valores PROVISIONALES. NO VOLAR.");
    if (!g_nav_enabled) log_text_wait("navegacion desactivada: %s", g_nav_why);
    if (!g_hw.axes_confirmed) log_text_wait("ejes de la IMU sin confirmar: sat del registro en ejes %s",
                                            imu_conv_check(&g_ic) == 0 ? "provisionales" : "del sensor");

    size_t np = 0;
    const vn_param_t *tab = vn_params_table(&np);
    int vprof = (VN_PROFILE == VN_PROFILE_ACQ) ? VN_PROFILE_FLIGHT : VN_PROFILE;
    for (size_t i = 0; i < np; i++) {
        const vn_param_t *p = &tab[i];
        vn_rec_param_t r;
        memset(&r, 0, sizeof r);
        vn_strfill(r.name, sizeof r.name, p->name);
        r.value = vn_param_value(p, vprof);
        r.status = vn_param_status(p, vprof);
        r.used = (uint8_t)((p->kind == VN_PK_DOC) ? 0 : (p->kind == VN_PK_NAV ? (g_nav_enabled ? 1 : 0) : 1));
        vn_strfill(r.source, sizeof r.source, p->source);
        log_put(VN_REC_PARAM, &r, sizeof r, w);
    }

    vn_rec_reg_t regs[24];
    int nr = 0;
    vn_rec_barocal_t cal;
    int have_cal;
    portENTER_CRITICAL(&g_reg_mux);
    for (int i = 0; i < g_nregs_imu; i++) regs[nr++] = g_regs_imu[i];
    for (int i = 0; i < g_nregs_baro; i++) regs[nr++] = g_regs_baro[i];
    cal = g_barocal;
    have_cal = g_have_barocal;
    portEXIT_CRITICAL(&g_reg_mux);
    for (int i = 0; i < nr; i++) log_put(VN_REC_REG, &regs[i], sizeof regs[i], w);
    if (have_cal) log_put(VN_REC_BAROCAL, &cal, sizeof cal, w);
    for (int i = 0; i < g_nbench; i++) log_put(VN_REC_BENCH, &g_bench[i], sizeof g_bench[i], w);
}

static uint32_t hwm(TaskHandle_t t) { return t ? (uint32_t)uxTaskGetStackHighWaterMark(t) : 0u; }

static void emit_stats(int64_t now)
{
    vn_rec_stats_t s;
    memset(&s, 0, sizeof s);
    s.t_us = now;
    s.imu_irq_lost = c_get(&g_c.imu_irq_lost);
    s.imu_q_drop = c_get(&g_c.imu_q_drop);
    s.baro_q_drop = c_get(&g_c.baro_q_drop);
    s.ring_drop_recs = c_get(&g_c.ring_drop_recs);
    s.ring_drop_bytes = c_get(&g_c.ring_drop_bytes);
    s.i2c_err_imu = c_get(&g_c.i2c_err_imu);
    s.i2c_err_baro = c_get(&g_c.i2c_err_baro);
    s.merge_imu_overflow = __atomic_load_n(&g_merge.imu_overflow, __ATOMIC_RELAXED);
    s.merge_baro_overflow = __atomic_load_n(&g_merge.baro_overflow, __ATOMIC_RELAXED);
    s.imu_events = c_get(&g_c.imu_events);
    s.baro_events = c_get(&g_c.baro_events);
    s.nav_events = c_get(&g_c.nav_events);
    s.nav_exec_max_us = c_take(&g_c.nav_exec_max_us, 0u);
    s.nav_exec_sum_us = c_take(&g_c.nav_exec_sum_us, 0u);
    s.acq_read_max_us = c_take(&g_c.acq_read_max_us, 0u);
    s.log_write_max_us = c_take(&g_c.log_write_max_us, 0u);
    s.hwm_acq = hwm(g_t_acq);
    s.hwm_nav = hwm(g_t_nav);
    s.hwm_log = hwm(g_t_log);
    s.hwm_loop = hwm(g_t_loop);
    s.heap_free_min = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    s.ring_free_min = c_take(&g_c.ring_free_min, g_hw.ring_bytes);
    s.sink_bytes = c_get(&g_c.sink_bytes);
    s.sink_discard_bytes = c_get(&g_c.sink_discard_bytes);
    log_put(VN_REC_STATS, &s, sizeof s, pdMS_TO_TICKS(20));
}

/* ------------------------------------------------------------------ arranque */
static int start_task(TaskFunction_t fn, const char *name, int prio, int core, TaskHandle_t *h)
{
    if (prio < 1 || prio >= configMAX_PRIORITIES || core < 0 || core > 1) return 0;
    return xTaskCreatePinnedToCore(fn, name, g_hw.stack_bytes, NULL, (UBaseType_t)prio, h, (BaseType_t)core) ==
           pdPASS;
}

extern "C" void vn_app_setup(void)
{
    vn_sink_begin();
    g_t_loop = xTaskGetCurrentTaskHandle();
    g_boot_id = esp_random();
    g_nav_enabled = vn_params_build(VN_PROFILE, &g_cfg, &g_ic, &g_hw, &g_nav_why);

    /* Marcas de saturación del registro: ejes del cohete si la conversión es válida; si los
       ejes están pendientes, identidad (ejes del SENSOR) con las mismas sensibilidades. */
    g_ic_log = g_ic;
    if (imu_conv_check(&g_ic) != 0) {
        for (int i = 0; i < 3; i++) {
            g_ic_log.map[i] = (uint8_t)i;
            g_ic_log.sign[i] = 1;
        }
    }

    g_rb = xRingbufferCreate(g_hw.ring_bytes, RINGBUF_TYPE_BYTEBUF);
    g_c.ring_free_min = g_hw.ring_bytes;
    g_q = xQueueCreate((UBaseType_t)g_hw.queue_depth, sizeof(vn_qitem_t));
    if (g_q == NULL) g_nav_enabled = 0;

    g_imu_irq = (uint8_t)(g_hw.pin_imu_int >= 0);
    g_baro_irq = (uint8_t)(g_hw.pin_baro_int >= 0);
    if (g_hw.pin_sda >= 0 && g_hw.pin_scl >= 0) Wire.begin((int)g_hw.pin_sda, (int)g_hw.pin_scl, (uint32_t)g_hw.i2c_hz);
    else Wire.begin();
    Wire.setClock((uint32_t)g_hw.i2c_hz);
    Wire.setTimeOut((uint16_t)g_hw.i2c_timeout_ms);

    int ok = 1;
    ok &= start_task(log_task, "vn_log", g_hw.prio_log, g_hw.core_log, &g_t_log);
    ok &= start_task(nav_task, "vn_nav", g_hw.prio_nav, g_hw.core_nav, &g_t_nav);
    ok &= start_task(acq_task, "vn_acq", g_hw.prio_acq, g_hw.core_acq, &g_t_acq);
    if (!ok) log_text_wait("ERROR creando tareas (pila %u B, ring %u B)", (unsigned)g_hw.stack_bytes,
                           (unsigned)g_hw.ring_bytes);
}

extern "C" void vn_app_loop(void)
{
    static int64_t next_hdr = 0, next_stats = 0;
    int64_t now = esp_timer_get_time();
    if (now >= next_hdr) {
        emit_header_block();
        next_hdr = now + period_us(1.0 / g_hw.header_period_s);
    }
    if (now >= next_stats) {
        emit_stats(now);
        next_stats = now + period_us(g_hw.stats_hz);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}
