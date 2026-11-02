#include "imu_conv.h"

#include <math.h>

#include "navigation.h" /* NAV_SAT_* */

#define DEG2RAD 0.017453292519943295

int imu_conv_check(const imu_conv_cfg_t *c)
{
    if (!c) return -1;
    if (!(c->acc_lsb_per_g > 0.0f) || !(c->gyro_lsb_per_dps > 0.0f) || !(c->g_si > 0.0f)) return -1;
    if (c->sat_margin_lsb < 0 || c->sat_margin_lsb > 16384) return -1;
    int used[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        if (c->map[i] > 2u || used[c->map[i]]) return -1;
        used[c->map[i]] = 1;
        if (c->sign[i] != 1 && c->sign[i] != -1) return -1;
    }
    /* determinante = paridad de la permutación × producto de signos */
    int inversions = 0;
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++)
            if (c->map[i] > c->map[j]) inversions++;
    int det = ((inversions % 2) ? -1 : 1) * c->sign[0] * c->sign[1] * c->sign[2];
    return det == 1 ? 0 : -1;
}

static int saturated(int16_t code, int32_t margin)
{
    return (int32_t)code >= 32767 - margin || (int32_t)code <= -32768 + margin;
}

uint8_t imu_conv(const imu_conv_cfg_t *c, const int16_t acc_raw[3], const int16_t gyro_raw[3], float acc_si[3],
                 float gyro_si[3])
{
    uint8_t sat = 0u;
    const float ka = c->g_si / c->acc_lsb_per_g;
    const float kg = (float)DEG2RAD / c->gyro_lsb_per_dps;
    for (int i = 0; i < 3; i++) {
        unsigned src = c->map[i];
        float sg = (float)c->sign[i];
        acc_si[i] = sg * ka * (float)acc_raw[src];
        gyro_si[i] = sg * kg * (float)gyro_raw[src];
        if (saturated(acc_raw[src], c->sat_margin_lsb)) sat = (uint8_t)(sat | (NAV_SAT_ACC_X << i));
        if (saturated(gyro_raw[src], c->sat_margin_lsb)) sat = (uint8_t)(sat | (NAV_SAT_GYRO_X << i));
    }
    return sat;
}
