/*
 * nav_events.h — ordenación por tiempo de los eventos de IMU y barómetro antes de
 * entregarlos a navigation.c (un solo propietario de nav_t: la tarea de navegación).
 *
 * Cada sensor llega en su orden propio, pero los dos flujos se intercalan con retrasos
 * distintos (un lote de IMU puede llegar después de un barómetro posterior a parte del lote).
 * nav_merge_pop() entrega siempre el evento más antiguo de los dos, y sólo cuando es seguro:
 * si el otro flujo ya tiene un evento posterior, o si ha pasado holdoff_us desde su marca
 * (ya no puede llegar nada anterior con un retraso menor que holdoff_us).
 * A igualdad de marca, primero la IMU (el estado avanza antes de aplicar el baro).
 *
 * C99, memoria fija, sin bloqueos: el transporte entre tareas (colas de FreeRTOS) queda
 * fuera; esto vive dentro de la tarea de navegación.
 */
#ifndef NAV_EVENTS_H
#define NAV_EVENTS_H

#include <stdint.h>

#include "navigation.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_EV_IMU_CAP 64
#define NAV_EV_BARO_CAP 16

typedef struct {
    int64_t t_us;  /* marca efectiva (µs, esp_timer): la que se entregará a navigation.c */
    uint32_t seq;  /* secuencia del registro crudo de origen */
    nav_imu_sample_t s;
} nav_ev_imu_t;

typedef struct {
    int64_t t_us; /* instante de LECTURA (la latencia la aplica navigation.c) */
    uint32_t seq;
    nav_baro_sample_t s;
} nav_ev_baro_t;

typedef struct {
    nav_ev_imu_t imu[NAV_EV_IMU_CAP];
    nav_ev_baro_t baro[NAV_EV_BARO_CAP];
    uint32_t imu_head, imu_n, baro_head, baro_n;
    uint32_t imu_overflow, baro_overflow; /* eventos perdidos por cola llena */
    uint32_t imu_hwm, baro_hwm;           /* ocupación máxima */
} nav_merge_t;

typedef enum { NAV_EV_NONE = 0, NAV_EV_IMU = 1, NAV_EV_BARO = 2 } nav_ev_kind_t;

void nav_merge_init(nav_merge_t *m);
/* 0 si entra; −1 si la cola está llena (el evento se pierde y se cuenta) */
int nav_merge_push_imu(nav_merge_t *m, const nav_ev_imu_t *e);
int nav_merge_push_baro(nav_merge_t *m, const nav_ev_baro_t *e);
/* Devuelve NAV_EV_IMU o NAV_EV_BARO y copia el evento, o NAV_EV_NONE si aún no es seguro
   entregar ninguno. */
nav_ev_kind_t nav_merge_pop(nav_merge_t *m, int64_t now_us, int64_t holdoff_us, nav_ev_imu_t *imu,
                            nav_ev_baro_t *baro);

#ifdef __cplusplus
}
#endif

#endif
