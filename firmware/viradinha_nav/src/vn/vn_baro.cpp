/*
 * vn_baro.cpp — BMP390 con la API oficial de Bosch (src/bmp3, v2.0.6, sin modificar).
 *
 * Compensación: la de Bosch en su configuración por defecto (BMP3_FLOAT_COMPENSATION):
 * presión en Pa y temperatura en °C, en double (por software en el ESP32-S3; 50 Hz).
 * Configuración propuesta: modo normal, 50 Hz, presión ×4, temperatura ×1, IIR desactivado.
 * Los tiempos de conversión de la hoja de datos (10.88 ms típ., 12.48 ms máx.) NO son la
 * latencia del sistema: baro_latency_s se mide (ver PROCEDIMIENTOS.md).
 */
#include <Arduino.h>
#include <Wire.h>

#include "../bmp3/bmp3.h"
#include "vn_sensors.h"

static struct bmp3_dev g_dev;
static uint8_t g_addr;

static BMP3_INTF_RET_TYPE i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf)
{
    uint8_t a = *(uint8_t *)intf;
    Wire.beginTransmission(a);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return BMP3_E_COMM_FAIL;
    if (Wire.requestFrom((uint16_t)a, (size_t)len, true) != len) return BMP3_E_COMM_FAIL;
    for (uint32_t i = 0; i < len; i++) data[i] = (uint8_t)Wire.read();
    return BMP3_INTF_RET_SUCCESS;
}

static BMP3_INTF_RET_TYPE i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf)
{
    uint8_t a = *(uint8_t *)intf;
    Wire.beginTransmission(a);
    Wire.write(reg);
    Wire.write(data, len); /* bmp3_set_regs ya intercala dirección/dato */
    return Wire.endTransmission() == 0 ? BMP3_INTF_RET_SUCCESS : BMP3_E_COMM_FAIL;
}

static void delay_us(uint32_t us, void *intf)
{
    (void)intf;
    delayMicroseconds(us);
}

static void reg_check(vn_reg_cb cb, uint8_t addr, uint8_t mask, uint8_t expected, int *fails)
{
    uint8_t v = 0;
    int8_t st = bmp3_get_regs(addr, &v, 1, &g_dev);
    vn_rec_reg_t r;
    r.dev = 2;
    r.bank = 0;
    r.addr = addr;
    r.mask = mask;
    r.expected = expected;
    r.read = v;
    r.ok = (st == BMP3_OK && (v & mask) == (expected & mask)) ? 1 : 0;
    r.reserved = 0;
    if (!r.ok) (*fails)++;
    if (cb) cb(&r);
}

int vn_baro_begin(const vn_hw_t *hw, int use_int_pin, vn_reg_cb cb, vn_rec_barocal_t *cal)
{
    if (hw->baro_odr_hz != 50 || hw->baro_osr_p != 4 || hw->baro_osr_t != 1) return -10; /* sólo la propuesta */
    const uint8_t cand[2] = {0x77, 0x76};
    int ok = 0;
    for (int i = 0; i < 2 && !ok; i++) {
        g_addr = (hw->baro_addr != 0) ? (uint8_t)hw->baro_addr : cand[i];
        g_dev.intf_ptr = &g_addr;
        g_dev.intf = BMP3_I2C_INTF;
        g_dev.read = i2c_read;
        g_dev.write = i2c_write;
        g_dev.delay_us = delay_us;
        ok = (bmp3_init(&g_dev) == BMP3_OK);
        if (hw->baro_addr != 0) break;
    }
    if (!ok) return -1;

    struct bmp3_settings s;
    memset(&s, 0, sizeof s);
    s.press_en = BMP3_ENABLE;
    s.temp_en = BMP3_ENABLE;
    s.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    s.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    s.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;
    s.odr_filter.odr = BMP3_ODR_50_HZ;
    s.int_settings.drdy_en = use_int_pin ? BMP3_ENABLE : BMP3_DISABLE;
    s.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
    s.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
    s.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
    uint32_t sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                   BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN | BMP3_SEL_OUTPUT_MODE |
                   BMP3_SEL_LEVEL | BMP3_SEL_LATCH;
    if (bmp3_set_sensor_settings(sel, &s, &g_dev) != BMP3_OK) return -2;
    s.op_mode = BMP3_MODE_NORMAL;
    if (bmp3_set_op_mode(&s, &g_dev) != BMP3_OK) return -3;

    int fails = 0;
    reg_check(cb, BMP3_REG_CHIP_ID, 0xFF, BMP390_CHIP_ID, &fails);
    reg_check(cb, BMP3_REG_PWR_CTRL, BMP3_OP_MODE_MSK | BMP3_PRESS_EN_MSK | BMP3_TEMP_EN_MSK,
              (uint8_t)((BMP3_MODE_NORMAL << BMP3_OP_MODE_POS) | BMP3_PRESS_EN_MSK | BMP3_TEMP_EN_MSK), &fails);
    reg_check(cb, BMP3_REG_OSR, BMP3_PRESS_OS_MSK | BMP3_TEMP_OS_MSK,
              (uint8_t)(BMP3_OVERSAMPLING_4X | (BMP3_NO_OVERSAMPLING << BMP3_TEMP_OS_POS)), &fails);
    reg_check(cb, BMP3_REG_ODR, BMP3_ODR_MSK, BMP3_ODR_50_HZ, &fails);
    reg_check(cb, BMP3_REG_CONFIG, BMP3_IIR_FILTER_MSK, (uint8_t)(BMP3_IIR_FILTER_DISABLE << BMP3_IIR_FILTER_POS),
              &fails);
    reg_check(cb, BMP3_REG_INT_CTRL, BMP3_INT_DRDY_EN_MSK | BMP3_INT_LEVEL_MSK | BMP3_INT_LATCH_MSK,
              (uint8_t)((use_int_pin ? BMP3_INT_DRDY_EN_MSK : 0) | BMP3_INT_LEVEL_MSK), &fails);

    cal->addr = g_addr;
    cal->chip_id = g_dev.chip_id;
    if (bmp3_get_regs(BMP3_REG_CALIB_DATA, cal->nvm, BMP3_LEN_CALIB_DATA, &g_dev) != BMP3_OK) fails++;
    return fails ? -4 : 0;
}

int vn_baro_poll(uint8_t *status)
{
    uint8_t v = 0;
    if (bmp3_get_regs(BMP3_REG_SENS_STATUS, &v, 1, &g_dev) != BMP3_OK) return -1;
    *status = v;
    return (v & BMP3_DRDY_PRESS) ? 1 : 0;
}

int vn_baro_read(double *pressure_pa, double *temp_c)
{
    struct bmp3_data d;
    if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &d, &g_dev) != BMP3_OK) return -1;
    *pressure_pa = d.pressure;
    *temp_c = d.temperature;
    return 0;
}
