/*
 * imu_conv.h — códigos crudos de la IMU → unidades SI en ejes del cohete, con saturación
 * detectada por CÓDIGO en ambos extremos.
 *
 * Por qué por código: con ±16 g y 2048 LSB/g, el código máximo +32767 equivale a
 * 15.9995 g, de modo que comparar el valor convertido con «16 g» nunca detecta el recorte
 * positivo (sí el negativo, −32768 = −16 g exactos). Además, el filtro digital interno se
 * aplica después del conversor: una entrada recortada puede salir filtrada algo por debajo
 * del extremo. Por eso el umbral es configurable como margen en LSB respecto a los
 * extremos; su valor es una POLÍTICA PROVISIONAL a comprobar.
 * El ICM-20948 no tiene un indicador específico de saturación.
 */
#ifndef IMU_CONV_H
#define IMU_CONV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float acc_lsb_per_g;    /* sensibilidad nominal (hoja de datos) para el rango configurado */
    float gyro_lsb_per_dps;
    float g_si;             /* m/s² por «g» del sensor (gravedad estándar 9.80665) */
    /* Ejes: cohete[i] = sign[i] · sensor[map[i]]. Debe ser una rotación propia
       (permutación con signos de determinante +1). PENDIENTE: montaje real. */
    uint8_t map[3];
    int8_t sign[3];
    int32_t sat_margin_lsb; /* |código| dentro de este margen respecto a +32767 / −32768 → saturado */
} imu_conv_cfg_t;

/* 0 si la configuración es válida (rotación propia, sensibilidades > 0, margen >= 0). */
int imu_conv_check(const imu_conv_cfg_t *c);

/* Convierte y devuelve los bits NAV_SAT_* EN EJES DEL COHETE. */
uint8_t imu_conv(const imu_conv_cfg_t *c, const int16_t acc_raw[3], const int16_t gyro_raw[3], float acc_si[3],
                 float gyro_si[3]);

#ifdef __cplusplus
}
#endif

#endif
