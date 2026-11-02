/*
 * vn_sensors.h — interfaz mínima de los sensores. Todas las funciones se llaman desde la
 * tarea de adquisición (único propietario del bus I2C). Si el firmware del equipo ya tiene
 * drivers adecuados, basta con reimplementar estas funciones sobre ellos.
 */
#ifndef VN_SENSORS_H
#define VN_SENSORS_H

#include <stdint.h>

#include "vn_logfmt.h"
#include "vn_params.h"

typedef void (*vn_reg_cb)(const vn_rec_reg_t *r);

/* ---- ICM-20948 (biblioteca SparkFun 1.3.2) ---- */
/* Configura y relee registros (cada comprobación se entrega a cb). Devuelve 0 si todo OK. */
int vn_imu_begin(const vn_hw_t *hw, vn_reg_cb cb, uint8_t *addr_out);
/* Acelerómetro, giróscopo y temperatura crudos (14 bytes desde ACCEL_XOUT_H, banco 0) */
int vn_imu_read(int16_t acc[3], int16_t gyro[3], int16_t *temp);
/* Magnetómetro AK09916 crudo vía EXT_SLV_SENS_DATA (9 bytes) */
int vn_imu_read_mag(int16_t mag[3], uint8_t *st1, uint8_t *st2);
/* Sondeo: 1 si RAW_DATA_0_RDY_INT (INT_STATUS_1) estaba activo (leerlo lo borra), 0 si no, <0 error */
int vn_imu_poll_drdy(void);

/* ---- BMP390 (API Bosch BMP3 v2.0.6) ---- */
int vn_baro_begin(const vn_hw_t *hw, int use_int_pin, vn_reg_cb cb, vn_rec_barocal_t *cal);
/* Sondeo del registro STATUS: 1 si hay presión nueva, 0 si no, <0 error. *status = registro */
int vn_baro_poll(uint8_t *status);
/* Presión compensada (Pa) y temperatura (°C) */
int vn_baro_read(double *pressure_pa, double *temp_c);

#endif
