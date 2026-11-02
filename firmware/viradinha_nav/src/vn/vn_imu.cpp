/*
 * vn_imu.cpp — ICM-20948 con la biblioteca «SparkFun 9DoF IMU Breakout - ICM 20948» 1.3.2.
 *
 * Funciones de la biblioteca usadas (comprobadas en su código fuente, v1.3.2):
 *   begin(TwoWire&, bool ad0val), setSampleMode, setFullScale, setDLPFcfg, enableDLPF,
 *   setSampleRate, cfgIntActiveLow, cfgIntOpenDrain, cfgIntLatch, intEnableRawDataReady,
 *   setBank, read. begin() hace un arranque completo (reset, despertar, modo bajo ruido y
 *   puesta en marcha del magnetómetro por el maestro I2C interno); después se reconfigura.
 * Registros (enums de la biblioteca, que siguen la hoja de datos):
 *   banco 0: WHO_AM_I 0x00, USER_CTRL 0x03, LP_CONFIG 0x05, PWR_MGMT_1 0x06, PWR_MGMT_2 0x07,
 *            INT_PIN_CFG 0x0F, INT_ENABLE_1 0x11, INT_STATUS_1 0x1A, ACCEL_XOUT_H 0x2D,
 *            EXT_SLV_SENS_DATA_00 0x3B
 *   banco 2: GYRO_SMPLRT_DIV 0x00, GYRO_CONFIG_1 0x01, ACCEL_SMPLRT_DIV_1/2 0x10/0x11,
 *            ACCEL_CONFIG 0x14
 * ODR (comentarios de la biblioteca, citando la hoja de datos):
 *   giro = 1.1 kHz/(1+GYRO_SMPLRT_DIV), acel = 1.125 kHz/(1+ACCEL_SMPLRT_DIV)
 *   → con divisores 4: giro 220 Hz y acel 225 Hz (NO iguales). RAW_DATA_0_RDY se activa con
 *   dato nuevo de cualquier sensor: cada lectura registra qué sensor cambió.
 */
#include <Arduino.h>
#include <ICM_20948.h>
#include <Wire.h>

#include "vn_sensors.h"

static ICM_20948_I2C g_icm;

static void reg_check(vn_reg_cb cb, uint8_t bank, uint8_t addr, uint8_t mask, uint8_t expected, int *fails)
{
    uint8_t v = 0;
    g_icm.setBank(bank);
    ICM_20948_Status_e st = g_icm.read(addr, &v, 1);
    vn_rec_reg_t r;
    r.dev = 1;
    r.bank = bank;
    r.addr = addr;
    r.mask = mask;
    r.expected = expected;
    r.read = v;
    r.ok = (st == ICM_20948_Stat_Ok && (v & mask) == (expected & mask)) ? 1 : 0;
    r.reserved = 0;
    if (!r.ok) (*fails)++;
    if (cb) cb(&r);
}

int vn_imu_begin(const vn_hw_t *hw, vn_reg_cb cb, uint8_t *addr_out)
{
    /* Sólo la configuración propuesta: así las sensibilidades de imu_conv (2048, 16.4)
       no pueden quedar incoherentes con el rango programado. */
    if (hw->imu_acc_fs_g != 16 || hw->imu_gyro_fs_dps != 2000 || hw->imu_dlpf_cfg != 3) return -10;
    if (hw->imu_acc_div < 0 || hw->imu_acc_div > 4095 || hw->imu_gyro_div < 0 || hw->imu_gyro_div > 255) return -11;

    int found = 0;
    for (int attempt = 0; attempt < 2 && !found; attempt++) {
        bool ad0 = (attempt == 0);
        if (hw->imu_addr == 0x68) ad0 = false;
        if (hw->imu_addr == 0x69) ad0 = true;
        if (g_icm.begin(Wire, ad0) == ICM_20948_Stat_Ok) {
            found = 1;
            *addr_out = ad0 ? 0x69 : 0x68;
        }
        if (hw->imu_addr != 0) break; /* dirección fija: un solo intento */
    }
    if (!found) return -1;

    const uint8_t both = ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr;
    int e = 0;
    e |= g_icm.setSampleMode(both, ICM_20948_Sample_Mode_Continuous) != ICM_20948_Stat_Ok;
    ICM_20948_fss_t fss;
    fss.a = gpm16;
    fss.g = dps2000;
    e |= g_icm.setFullScale(both, fss) != ICM_20948_Stat_Ok;
    ICM_20948_dlpcfg_t d;
    d.a = acc_d50bw4_n68bw8;
    d.g = gyr_d51bw2_n73bw3;
    e |= g_icm.setDLPFcfg(both, d) != ICM_20948_Stat_Ok;
    e |= g_icm.enableDLPF(ICM_20948_Internal_Acc, true) != ICM_20948_Stat_Ok;
    e |= g_icm.enableDLPF(ICM_20948_Internal_Gyr, true) != ICM_20948_Stat_Ok;
    ICM_20948_smplrt_t sr;
    sr.a = (uint16_t)hw->imu_acc_div;
    sr.g = (uint8_t)hw->imu_gyro_div;
    e |= g_icm.setSampleRate(both, sr) != ICM_20948_Stat_Ok;
    e |= g_icm.cfgIntActiveLow(false) != ICM_20948_Stat_Ok; /* activo alto */
    e |= g_icm.cfgIntOpenDrain(false) != ICM_20948_Stat_Ok; /* push-pull */
    e |= g_icm.cfgIntLatch(false) != ICM_20948_Stat_Ok;     /* pulso de 50 µs */
    e |= g_icm.intEnableRawDataReady(true) != ICM_20948_Stat_Ok;
    if (e) return -2;

    /* Relectura de los registros configurados */
    int fails = 0;
    reg_check(cb, 0, 0x00, 0xFF, 0xEA, &fails); /* WHO_AM_I */
    reg_check(cb, 0, 0x03, 0xC0, 0x00, &fails); /* USER_CTRL: DMP_EN = 0, FIFO_EN = 0 */
    reg_check(cb, 0, 0x05, 0x30, 0x00, &fails); /* LP_CONFIG: ACCEL_CYCLE = GYRO_CYCLE = 0 (continuo) */
    reg_check(cb, 0, 0x06, 0x60, 0x00, &fails); /* PWR_MGMT_1: SLEEP = 0, LP_EN = 0 (bajo ruido) */
    reg_check(cb, 0, 0x07, 0x3F, 0x00, &fails); /* PWR_MGMT_2: acel y giro activos */
    reg_check(cb, 0, 0x0F, 0xE0, 0x00, &fails); /* INT_PIN_CFG: activo alto, push-pull, pulso */
    reg_check(cb, 0, 0x11, 0x01, 0x01, &fails); /* INT_ENABLE_1: RAW_DATA_0_RDY_EN */
    const uint8_t cfg1 = (uint8_t)(1u | (3u << 1) | (3u << 3)); /* FCHOICE=1, FS=3, DLPFCFG=3 */
    reg_check(cb, 2, 0x00, 0xFF, (uint8_t)hw->imu_gyro_div, &fails);         /* GYRO_SMPLRT_DIV */
    reg_check(cb, 2, 0x01, 0x3F, cfg1, &fails);                                /* GYRO_CONFIG_1 */
    reg_check(cb, 2, 0x10, 0x0F, (uint8_t)(hw->imu_acc_div >> 8), &fails);     /* ACCEL_SMPLRT_DIV_1 */
    reg_check(cb, 2, 0x11, 0xFF, (uint8_t)(hw->imu_acc_div & 0xFF), &fails);   /* ACCEL_SMPLRT_DIV_2 */
    reg_check(cb, 2, 0x14, 0x3F, cfg1, &fails);                                /* ACCEL_CONFIG */
    g_icm.setBank(0);
    return fails ? -3 : 0;
}

static int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

int vn_imu_read(int16_t acc[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t b[14];
    if (g_icm.read(0x2D, b, sizeof b) != ICM_20948_Stat_Ok) return -1; /* banco 0 */
    for (int i = 0; i < 3; i++) {
        acc[i] = be16(&b[2 * i]);
        gyro[i] = be16(&b[6 + 2 * i]);
    }
    *temp = be16(&b[12]);
    return 0;
}

int vn_imu_read_mag(int16_t mag[3], uint8_t *st1, uint8_t *st2)
{
    uint8_t b[9];
    if (g_icm.read(0x3B, b, sizeof b) != ICM_20948_Stat_Ok) return -1;
    /* como ICM_20948_get_agmt: ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2 (little-endian) */
    *st1 = b[0];
    mag[0] = (int16_t)(((uint16_t)b[2] << 8) | b[1]);
    mag[1] = (int16_t)(((uint16_t)b[4] << 8) | b[3]);
    mag[2] = (int16_t)(((uint16_t)b[6] << 8) | b[5]);
    *st2 = b[8];
    return 0;
}

int vn_imu_poll_drdy(void)
{
    uint8_t v = 0;
    if (g_icm.read(0x1A, &v, 1) != ICM_20948_Stat_Ok) return -1; /* INT_STATUS_1 */
    return (v & 0x01u) ? 1 : 0;
}
