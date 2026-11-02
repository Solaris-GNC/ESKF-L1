/*
 * vn_bench.h — prueba de tiempos de navigation.c con datos sintéticos (C puro).
 *
 * Recorre los caminos que el banco estático NUNCA ejercita: alineación, despegue, vuelo
 * propulsado con baro aplicado y planeo. Así la medida de tiempos (ciclos de CPU) y la marca de
 * pila libre mínima de la tarea que la ejecuta incluyen el código de vuelo.
 *
 * Los datos son sintéticos y sólo sirven para medir tiempos y pila: no validan el estimador.
 * El reloj lo da quien llama (en la placa: ESP.getCycleCount; en el PC: un contador).
 */
#ifndef VN_BENCH_H
#define VN_BENCH_H

#include <stdint.h>

#include "../nav/navigation.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*vn_cycles_fn)(void);

typedef struct {
    uint32_t n, over; /* llamadas, llamadas por encima del periodo nominal de la IMU */
    double sum, min, max; /* µs */
} vn_bench_acc_t;

typedef struct {
    vn_bench_acc_t imu_pad, align, imu_flight, baro_flight;
    int aligned;   /* se alineó durante la prueba */
    int liftoff;   /* se detectó el despegue */
    int baro_used; /* hubo baro aplicado en vuelo */
    int init_ret;  /* retorno de nav_init */
} vn_bench_result_t;

/* Ejecuta la prueba sobre *n (lo reinicia con nav_init; después hay que volver a llamar a
   nav_init para usarlo de verdad). fs_hz: frecuencia de la IMU simulada. cycles_per_us: MHz
   de la CPU. Devuelve 0 si se recorrieron alineación, despegue y baro en vuelo. */
int vn_bench_run(nav_t *n, const nav_config_t *cfg, double fs_hz, vn_cycles_fn clk, double cycles_per_us,
                 vn_bench_result_t *r);

#ifdef __cplusplus
}
#endif

#endif
