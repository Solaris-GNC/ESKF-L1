/*
 * vn_app.h — integración de adquisición, navegación y registro en el ESP32-S3.
 *
 * Tareas (núcleo y prioridad en vn_params.c, fw.core_* / fw.prio_*):
 *   ISR IMU/baro  sólo captura esp_timer_get_time() y notifica a la tarea de adquisición.
 *   vn_acq        ÚNICA dueña del bus I2C: lee sensores, registra lo crudo (siempre, también
 *                 sin calibración ni navegación) y envía eventos a la navegación.
 *   vn_nav        ÚNICA dueña de nav_t: ordena eventos por tiempo (nav_events.c), llama a
 *                 navigation.c y registra retornos y salidas.
 *   vn_log        vacía el búfer acotado hacia el sumidero (USB por defecto).
 *   loop()        estadísticas periódicas, cabecera y parámetros.
 * Ninguna tarea se bloquea por el registro: si el búfer está lleno, el registro se descarta
 * y se cuenta (ring_drop_*).
 */
#ifndef VN_APP_H
#define VN_APP_H

#include <stddef.h>
#include <stdint.h>

#include "../nav/navigation.h"

#ifdef __cplusplus
extern "C" {
#endif

void vn_app_setup(void);
void vn_app_loop(void);

/* Copia de la última salida de navegación (cualquier tarea, no bloquea más que un spinlock).
   Devuelve 1 si hay salida, 0 si la navegación no está activa. f_degraded = nav.f.degraded. */
int vn_nav_snapshot(nav_output_t *out, uint32_t *f_degraded);

/* Mensaje de texto al registro (≤ 95 caracteres) */
void vn_log_text(const char *fmt, ...);

/* ---- Ganchos con implementación débil, sustituibles por el firmware del equipo ---- */
/* Sumidero del registro. Debe NO bloquear: escribir lo que quepa y devolver cuántos bytes
   aceptó (0 = reintentar luego). *discarded = bytes que se tiran sin escribir (p. ej. USB sin
   anfitrión). Por defecto: Serial (USB CDC). PENDIENTE: almacenamiento de vuelo del equipo. */
void vn_sink_begin(void);
size_t vn_sink_write(const uint8_t *data, size_t len, size_t *discarded);
/* Telemetría: llamada desde vn_nav a fw.telemetry_hz. Debe NO bloquear. Por defecto, nada. */
void vn_telemetry_publish(const nav_output_t *out, uint32_t f_degraded);

#ifdef __cplusplus
}
#endif

#endif
