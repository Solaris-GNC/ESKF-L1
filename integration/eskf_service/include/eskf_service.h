/*
 * eskf_service.h — Solaris ESKF como servicio consumidor/productor de paquetes SPP.
 *
 * Este núcleo NO incluye cabeceras de SPP: recibe paquetes de 64 bytes ya serializados y entrega
 * payloads de salida. El pegamento con los contratos reales de SPP (commit 9247d9e) está en
 * spp_services_eskf.c, que es quien llama a estas funciones desde los callbacks del router.
 * Reutiliza sin copiar c/eskf.c, c/navigation.c y c/nav_events.c.
 *
 *   deliverToMailbox → eskf_svc_deliver_packet()  valida lo mínimo y copia en un buzón estático
 *                                                  acotado (O(1)); lleno → descarta y cuenta.
 *   consumeData      → eskf_svc_consume()         ÚNICO propietario de nav_t. Trabajo acotado:
 *                                                  ≤ max_items paquetes, ≤ max_events llamadas a
 *                                                  navigation.c, un nav_tick por llamada (aunque no
 *                                                  haya sensores), ≤ 1 snapshot, ≤ 1 diagnóstico y
 *                                                  ≤ 1 salida compacta.
 *   acquireData      → eskf_svc_produce()         como mucho UN mensaje (≤ 50 B) por llamada.
 *   consultas        → eskf_svc_get_output() / eskf_svc_get_diag()  (copia completa bajo una sección
 *                                                  crítica breve: portMUX en ESP-IDF; el filtro nunca
 *                                                  se ejecuta dentro)
 *   reinicio         → eskf_svc_request_restart() (bandera atómica; lo ejecuta consume; en vuelo se rechaza)
 *   referencia baro  → eskf_svc_notify_baro_ref_change() (sólo por aviso explícito del anfitrión)
 *
 * Hilos: deliver, consume y produce desde UNA tarea (la que mueve el router SPP). Las consultas
 * y la petición de reinicio pueden venir de otra. No se crean tareas ni se toca hardware.
 *
 * Tiempo (docs/CONTRATO_ENTRADA.md): reloj inyectable de 64 bits en µs, MISMA época que timestampMs·1000.
 * La marca de un evento es timestampMs del paquete (instante de CONSTRUCCIÓN en packetData,
 * resolución 1 ms), extendida a 64 bits contra el reloj. No se inventa resolución de µs.
 */
#ifndef ESKF_SERVICE_H
#define ESKF_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "nav_events.h"
#include "navigation.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ESP_PLATFORM) && !defined(ESKF_USE_FLOAT)
#error "eskf_service en ESP-IDF: ESKF_USE_FLOAT debe estar definido en TODO el componente y sus consumidores"
#endif

/* ---------------------------------------------------------------- formato de paquete SPP */
#define ESKF_SVC_PKT_SIZE 64u
#define ESKF_SVC_PKT_VERSION 1u
#define ESKF_SVC_PAYLOAD_MAX 50u
#define ESKF_SVC_OFF_VERSION 0u
#define ESKF_SVC_OFF_APID 1u
#define ESKF_SVC_OFF_SEQ 3u
#define ESKF_SVC_OFF_LEN 5u
#define ESKF_SVC_OFF_TS 7u
#define ESKF_SVC_OFF_DROP 11u
#define ESKF_SVC_OFF_PAYLOAD 12u
#define ESKF_SVC_OFF_CRC 62u

/* ---------------------------------------------------------------- formatos de entrada */
#define ESKF_SVC_IMU_FMT_SPP44 44  /* definitivo: 11 float32 (a, ω, qw qx qy qz, accuracy) */
#define ESKF_SVC_IMU_FMT_HIST48 48 /* SÓLO compatibilidad histórica (captura del 24-oct): 11 + 1 reservado */
#define ESKF_SVC_BARO_LEN 12u      /* altura m, presión Pa, temperatura °C */

#define ESKF_SVC_BARO_REL_PRODUCER 1 /* altura relativa a p0/T0 del productor (500 muestras) */
#define ESKF_SVC_BARO_ABS_ISA 2      /* histórica: ISA con p0 = 101325 Pa */

#define ESKF_SVC_TIME_PACKET_MS 1 /* marca = timestampMs de construcción (ms), no de adquisición */

/* ---------------------------------------------------------------- perfiles */
#define ESKF_SVC_PROFILE_ACQ 0    /* transporte y diagnóstico; sin navegación */
#define ESKF_SVC_PROFILE_EXP 1    /* EXPERIMENTAL operativo: valores numéricos con procedencia
                                     (código, registro, simulación, provisional); navega y publica
                                     su validez dinámica. Nada de ello es validación física. */
#define ESKF_SVC_PROFILE_STRICT 2 /* ESTRICTO: sólo navega si no queda nada pendiente de confirmar */
#define ESKF_SVC_PROFILE_BENCH ESKF_SVC_PROFILE_EXP     /* nombre anterior (compatibilidad) */
#define ESKF_SVC_PROFILE_FLIGHT ESKF_SVC_PROFILE_STRICT /* nombre anterior (compatibilidad) */

/* ---------------------------------------------------------------- retornos */
#define ESKF_SVC_OK 0
#define ESKF_SVC_E_NULL (-1)
#define ESKF_SVC_E_CONFIG (-2)
#define ESKF_SVC_E_FULL (-3)    /* buzón lleno: descartado y contado */
#define ESKF_SVC_E_APID (-4)    /* APID no suscrito (incluida la salida propia) */
#define ESKF_SVC_E_LEN (-5)     /* longitud > 50 */
#define ESKF_SVC_E_CRC (-6)     /* CRC incorrecto */
#define ESKF_SVC_E_NOT_INIT (-7)
#define ESKF_SVC_E_VERSION (-8) /* versión de paquete distinta de 1 */
#define ESKF_SVC_E_BUSY (-9)

/* ---------------------------------------------------------------- motivos del servicio (16 bits)
   Independientes de NAV_R_* (núcleo). INVALID: la estimación ESKF no se debe usar. DEGRADED:
   utilizable con cautela. El resto informa (docs/SERVICIO.md, comportamiento ante fallos). */
#define ESKF_SVC_R_NAV_DISABLED 0x0001u     /* ESKF fuera de servicio (INVALID): perfil ACQ, configuración
                                               incompleta o vuelo reconocido con navigation.c aún en rampa */
#define ESKF_SVC_R_IMU_LOSS 0x0002u         /* IMU perdida o inutilizable (DEGRADED, enclavado) */
#define ESKF_SVC_R_LATE_EVENT 0x0004u       /* IMU atrasada/tardía/futura descartada (DEGRADED, enclavado) */
#define ESKF_SVC_R_SAT_OUT_OF_MODEL 0x0008u /* saturación del acelerómetro fuera de la ventana del motor (INVALID, enclavado) */
#define ESKF_SVC_R_BARO_LOSS 0x0010u        /* baro perdido o descartado (informativo) */
#define ESKF_SVC_R_NO_BARO 0x0020u          /* aún no ha llegado ningún baro */
#define ESKF_SVC_R_NO_ALT_REF 0x0040u       /* navegación alineada sin referencia barométrica (DEGRADED) */
#define ESKF_SVC_R_SAT_CANDIDATE 0x0080u    /* candidato a saturación (enclavado; el núcleo decide) */
#define ESKF_SVC_R_SAT_UNKNOWN 0x0100u      /* sin banderas de hardware: no se puede descartar */
#define ESKF_SVC_R_TIME_APPROX 0x0200u      /* marcas de construcción en ms */
#define ESKF_SVC_R_DMP_INVALID 0x0400u      /* cuaternión/accuracy DMP inválidos (diagnóstico) */
#define ESKF_SVC_R_SEQ_RESYNC 0x0800u       /* secuencia reiniciada con tiempo continuo (enclavado) */
#define ESKF_SVC_R_NOT_VALIDATED 0x1000u    /* readiness ≠ 0: hipótesis sin validación física */
#define ESKF_SVC_R_EXPERIMENTAL 0x2000u     /* perfil experimental */
#define ESKF_SVC_R_BARO_REF_CHANGED 0x4000u /* cambio de referencia del baro notificado (DEGRADED) */
#define ESKF_SVC_R_RESTART_REFUSED 0x8000u  /* reinicio pedido en vuelo: rechazado */
#define ESKF_SVC_R_INVALID_MASK (ESKF_SVC_R_NAV_DISABLED | ESKF_SVC_R_SAT_OUT_OF_MODEL)
#define ESKF_SVC_R_DEGRADED_MASK                                                                    \
    (ESKF_SVC_R_IMU_LOSS | ESKF_SVC_R_LATE_EVENT | ESKF_SVC_R_NO_ALT_REF | ESKF_SVC_R_BARO_REF_CHANGED | \
     ESKF_SVC_R_SEQ_RESYNC)

/* why_code publicado cuando el vuelo se reconoce con navigation.c aún en rampa */
#define ESKF_SVC_WHY_FLIGHT_NAV_PAD 0x8008u
/* why_code publicado cuando deja de constar que el vehículo siga en rampa (fase incierta) */
#define ESKF_SVC_WHY_PHASE_UNKNOWN 0x8009u
/* Fase publicada. RAMPA sólo si está CONFIRMADA (contrato de inicio + evidencia continua); la
   ausencia de despegue detectado no la acredita. INCIERTA no autoriza ninguna operación de rampa. */
#define ESKF_SVC_PHASE_PAD 0u
#define ESKF_SVC_PHASE_FLIGHT 1u
#define ESKF_SVC_PHASE_UNKNOWN 2u

/* Capacidades disponibles ahora (no son validación) */
#define ESKF_SVC_CAP_ATTITUDE 0x01u    /* inercial alineado y no inválido */
#define ESKF_SVC_CAP_BARO_REF 0x02u    /* referencia barométrica de navegación (alt_ref) */
#define ESKF_SVC_CAP_ESKF_NAV 0x04u    /* navegación ESKF utilizable: actitud + referencia + no inválida */
#define ESKF_SVC_CAP_BARO_BACKUP 0x08u /* respaldo barométrico utilizable (altura y velocidad vertical) */
#define ESKF_SVC_CAP_LAUNCH_REF 0x10u  /* referencia de lanzamiento del respaldo congelada (vuelo o fase incierta) */
/* Fuente de la estimación operativa (salida compacta) */
#define ESKF_SVC_SRC_NONE 0u
#define ESKF_SVC_SRC_ESKF 1u
#define ESKF_SVC_SRC_BARO 2u

/* Preparación (bits de grupos con hipótesis sin validación física; 0 = todo confirmado). */
#define ESKF_SVC_RDY_UNITS 0x0001u
#define ESKF_SVC_RDY_AXES 0x0002u
#define ESKF_SVC_RDY_RANGE 0x0004u
#define ESKF_SVC_RDY_TIME 0x0008u
#define ESKF_SVC_RDY_BARO 0x0010u
#define ESKF_SVC_RDY_NOISE 0x0020u
#define ESKF_SVC_RDY_THRESH 0x0040u
#define ESKF_SVC_RDY_LIFTOFF 0x0080u
#define ESKF_SVC_RDY_HOST 0x0100u
#define ESKF_SVC_RDY_PROFILE 0x0200u  /* el perfil no es ESTRICTO */
#define ESKF_SVC_RDY_BOARD 0x0400u    /* sin validación en placa (ESKF_SVC_BOARD_VALIDATED = 0) */
#define ESKF_SVC_RDY_OVERRIDE 0x0800u /* la configuración difiere de la tabla: procedencia externa */
#ifndef ESKF_SVC_BOARD_VALIDATED
#define ESKF_SVC_BOARD_VALIDATED 0
#endif

/* ---------------------------------------------------------------- configuración */
typedef int64_t (*eskf_svc_clock_fn)(void *ctx); /* µs, monótono, época de timestampMs·1000 */

typedef struct {
    int32_t profile;
    uint16_t kpid_imu, kpid_baro, kpid_out; /* un bit cada uno, asignados por el router */
    uint16_t boot_id;                       /* identificador de arranque que fije el anfitrión (0 = no hay) */
    int32_t imu_format;                     /* ESKF_SVC_IMU_FMT_* */
    int32_t baro_semantics;                 /* ESKF_SVC_BARO_* */
    /* contrato de la IMU (floats ya escalados por el productor) */
    float acc_unit_si;             /* m/s² por unidad publicada (g → 9.80665) */
    float gyro_unit_si;            /* rad/s por unidad publicada (°/s → π/180), sin corrección */
    float r_sb[9];                 /* cuerpo = R · sensor, fila mayor; identidad: cuerpo = IMU */
    float acc_div, gyro_div;       /* divisores del productor (códigos por unidad publicada) */
    float acc_off[3], gyro_off[3]; /* offsets que el PRODUCTOR ya restó: sólo para volver al
                                      dominio de códigos al detectar saturación; NO se restan */
    float sat_margin_codes;        /* candidato si |código| >= 32768 − margen */
    int32_t sat_to_nav;            /* 1: los candidatos marcan saturación para navigation.c */
    float sat_motor_window_s;      /* saturación del acel. admitida sólo hasta t_despegue + ventana */
    int32_t time_source;           /* ESKF_SVC_TIME_PACKET_MS */
    int64_t future_tol_us;         /* marca > recepción + tolerancia: futura, se descarta */
    /* límites físicos del baro (altura relativa, presión, temperatura) */
    float baro_h_abs_max_m, baro_p_min_pa, baro_p_max_pa, baro_t_min_c, baro_t_max_c;
    /* respaldo barométrico (altura relativa al lanzamiento y velocidad vertical, positiva hacia ARRIBA) */
    float bb_alpha, bb_beta;   /* filtro alfa-beta de ganancias fijas */
    float bb_gate_m;           /* residuo máximo frente a la predicción antes de rechazar una medida */
    uint32_t bb_reseed_n;      /* rechazos seguidos que pasan el respaldo a recuperación */
    float bb_dt_max_s;         /* intervalo máximo sin medida aceptada: más largo pasa a recuperación */
    float bb_timeout_s;        /* antigüedad máxima de la última medida ACEPTADA para ser válido */
    float bb_ref_tau_s;        /* constante de tiempo de la referencia en rampa */
    float bb_liftoff_m;        /* subida sobre la referencia que congela la referencia (sin ESKF) */
    uint32_t bb_min_samples;   /* tamaño de la ventana coherente de inicio y de recuperación (3..32) */
    float bb_coh_m;            /* residuo máximo del ajuste lineal de la ventana para darla por coherente */
    float bb_flight_vz_mps;    /* |v_z| del respaldo que, sostenida bb_hold_s sin atestación inercial, */
                               /* reconoce el vuelo (sirve aunque no haya referencia de lanzamiento) */
    float bb_hold_s;           /* duración continua para reconocer vuelo (|v_z| >= bb_flight_vz_mps) o
                                  reposo en rampa (|v_z| <= 1 m/s) que fija la referencia de lanzamiento */
    /* fase: rampa CONFIRMADA frente a fase incierta (docs/SERVICIO.md, «Fase») */
    float pad_start_s;         /* contrato de inicio: no hay despegue en este plazo desde el inicio de la sesión */
    float pad_gap_max_s;       /* hueco máximo entre evidencias de seguir en rampa */
    float pad_lat_max_s;       /* latencia admitida de entrega (cola del router incluida): con el reloj
                                  avanzando y SIN ninguna muestra procesada durante
                                  pad_gap_max_s + nav.liftoff_hold_s + pad_lat_max_s, la rampa caduca */
    /* trabajo y salida */
    int64_t merge_holdoff_us;
    uint32_t max_items, max_events;
    int64_t snapshot_period_us; /* estado completo (fragmentos 1–5); 0 = no */
    int64_t diag_period_us;     /* diagnóstico (tipo 6); 0 = no */
    int64_t ops_period_us;      /* salida operativa compacta (tipo 7); 0 = no */
    uint32_t snapshot_mask;     /* bits 0..4 → fragmentos 1..5 */
    int32_t trace_enable;       /* traza de validación acotada */
    nav_config_t nav;
} eskf_svc_config_t;

/* ---------------------------------------------------------------- diagnósticos */
typedef struct {
    /* entrada (deliverToMailbox) */
    uint32_t delivered, mailbox_full, mailbox_full_imu, mailbox_full_baro, mailbox_hwm;
    uint32_t rejected_apid, rejected_version, rejected_crc, rejected_len;
    /* decodificación */
    uint32_t imu_rx, imu_bad_len, imu_nonfinite, imu_out_of_range, imu_reserved_nonzero, dmp_invalid;
    uint32_t baro_rx, baro_bad_len, baro_bad_value, baro_not_fed;
    /* secuencia, orden y tiempo */
    uint32_t seq_gaps_imu, seq_lost_imu, seq_gaps_baro, seq_lost_baro, seq_dup, seq_old, seq_resync;
    uint32_t ts_back_imu, ts_back_baro, ts_future, late_imu, late_baro;
    uint32_t restarts_requested, restarts_refused, baro_ref_notified;
    uint32_t imu_loss_events, clock_mismatch;
    /* saturación */
    uint32_t sat_cand_acc, sat_cand_gyro, sat_hw_confirmed, sat_out_of_model;
    /* recuperación en rampa */
    uint32_t loss_recovered;
    /* respaldo barométrico */
    uint32_t bb_used, bb_rejected, bb_reseeds, bb_gaps, bb_recoveries;
    /* frontera de sesión: muestras anteriores al inicio de la sesión, descartadas a propósito */
    uint32_t session_discarded;
    /* vuelo enclavado: detector (1 navigation.c, 2 altura del respaldo, 3 velocidad del respaldo) y
       muestras retenidas porque navigation.c seguía en rampa */
    uint32_t flight_detector, nav_cut_samples;
    /* rampa ya no confirmada: hueco de evidencia (µs), causa (1 hueco entre muestras, 2 silencio total
       por el reloj) y respaldo caducado sin medidas aceptadas */
    int64_t pad_lost_hole_us;
    uint32_t pad_lost_cause;
    uint32_t bb_expired;
    /* navegación y trabajo */
    uint32_t consume_calls, consume_items_max, nav_calls, nav_backlog_max, merge_full_waits;
    int32_t last_imu_ret, last_baro_ret, last_tick_ret;
    /* salida (la actualiza también el pegamento) */
    uint32_t snapshots, snapshots_skipped, diag_msgs, diag_skipped, ops_msgs, ops_skipped, msgs_out;
    uint32_t publish_ok, publish_fail, pool_empty, queue_busy, kpid_mismatch, trace_overflow;
    /* tiempos (µs) */
    int64_t lat_rx_max_us, lat_proc_max_us;
    int64_t first_imu_us, first_baro_us;
    /* valores de diagnóstico (no entran en el ESKF) */
    float dmp_q_last[4], dmp_accuracy_last, baro_p_last, baro_t_last;
} eskf_svc_diag_t;

/* ---------------------------------------------------------------- respaldo barométrico */
typedef struct {
    float h_m;       /* altura sobre la referencia de lanzamiento (m, positiva hacia arriba) en t_us */
    float vz_mps;    /* velocidad vertical (m/s, positiva hacia ARRIBA) en t_us */
    float ref_m;     /* referencia de lanzamiento (altura publicada por el BMP en rampa) */
    int64_t t_us;    /* instante del estado = última medida ACEPTADA; INT64_MIN si no hay */
    int64_t age_us;  /* reloj − t_us en la última evaluación (los rechazos no la renuevan) */
    uint8_t valid;   /* 1: siguiendo, con referencia, antigüedad <= bb_timeout_s y baro utilizable
                        (pasada la caducidad, sólo vuelve con una ventana coherente nueva) */
    uint8_t ref_frozen;
    uint8_t state;   /* ESKF_SVC_BB_* */
} eskf_svc_baro_out_t;
#define ESKF_SVC_BB_NONE 0u    /* sin ventana coherente todavía */
#define ESKF_SVC_BB_RECOVER 1u /* recuperando: reuniendo una ventana coherente (no válido) */
#define ESKF_SVC_BB_TRACK 2u   /* siguiendo medidas aceptadas */
#define ESKF_SVC_BB_WIN_MAX 32u

/* ---------------------------------------------------------------- salida consultable */
typedef struct {
    uint16_t session, config_id, readiness, why_code, boot_id;
    uint32_t snap_id;
    int64_t t_state_us, t_eval_us, t_proc_us; /* INT64_MIN si no hay */
    nav_output_t nav;
    uint32_t svc_reasons;
    uint8_t nav_validity, svc_validity, flight_accept, profile, nav_enabled, time_source, caps, source;
    uint8_t phase;   /* ESKF_SVC_PHASE_*: fase publicada (rampa confirmada, vuelo, incierta) */
    eskf_svc_baro_out_t bb;
    const char *why; /* primer pendiente o motivo de navegación desactivada ("" si navega) */
} eskf_svc_output_t;

/* ---------------------------------------------------------------- traza de validación */
#define ESKF_SVC_TR_DELIVER 1 /* a=apid b=seq c=len res=retorno t=recepción (µs) */
#define ESKF_SVC_TR_CONSUME 2 /* a=paquetes b=llamadas nav c=bits (1 snapshot, 2 diag, 4 compacta) t=reloj */
#define ESKF_SVC_TR_RESTART 3 /* a=sesión nueva (o actual si rechazado) b=1 si lo abrió un aviso de referencia res=1 rechazado t=reloj */
#define ESKF_SVC_TR_CONFIG 4  /* res=versión de la traza a=config_id b=perfil | versión de esquema << 8 c=boot_id
                                 t=reloj al iniciar (inicio de la primera sesión: contrato de rampa) */
#define ESKF_SVC_TR_OUTPUT 5  /* res=resultado (ESKF_SVC_OUT_*) a=tipo b=longitud c=secuencia SPP del paquete
                                 t=timestampMs (ms) que escribió packetData; 0 si no se construyó */
#define ESKF_SVC_TR_BAROREF 6 /* notificación de cambio de referencia del baro t=reloj */
#define ESKF_SVC_TR_PHASE 8   /* transición de fase dentro de un consume: res=fase nueva (ESKF_SVC_PHASE_FLIGHT o
                                 _UNKNOWN) a=causa (incierta: 1 hueco entre muestras, 2 silencio por el reloj;
                                 vuelo: detector 1–3) b=hueco o silencio en ms (saturado; 0 en vuelo)
                                 t=marca de la muestra (causa 1) o reloj (causa 2 y vuelo) */
#define ESKF_SVC_TR_END 7     /* cierre (eskf_svc_trace_close): a=CRC-16/CCITT (0xFFFF) de TODOS los registros
                                 anteriores en su forma de 16 B little-endian; b=registros perdidos por
                                 desbordamiento (saturado a 0xFFFF); c=0; t=nº de registros anteriores */
/* v1: OUTPUT con el reloj; v2: OUTPUT con la cabecera real del paquete; v3: registro de cierre END
   (una traza sin END no prueba que esté completa); v4: CONFIG lleva el reloj del inicio; v5: registro
   PHASE de cada transición de fase (también la caducidad de la rampa por el reloj) */
#define ESKF_SVC_TRACE_VERSION 5
#define ESKF_SVC_OUT_PUBLISHED 0
#define ESKF_SVC_OUT_PUB_FAIL 1    /* producido y perdido: publish rechazado (cola del router llena) */
#define ESKF_SVC_OUT_PACKET_FAIL 2 /* producido y perdido: packetData falló */
#define ESKF_SVC_OUT_DEFER_QUEUE 3 /* aplazado: cola del router alta */
#define ESKF_SVC_OUT_DEFER_POOL 4  /* aplazado: pool sin paquetes */
typedef struct {
    uint8_t type;
    int8_t res;
    uint16_t a, b, c;
    int64_t t;
} eskf_svc_trec_t; /* 16 bytes */
#ifndef ESKF_SVC_TRACE_CAP
#define ESKF_SVC_TRACE_CAP 256u
#endif

/* Gancho de PC (pruebas): cada nav_tick (kind 0), nav_on_imu (1) y nav_on_baro (2) con sus entradas */
typedef void (*eskf_svc_hook_fn)(void *ctx, int kind, const nav_imu_sample_t *imu, const nav_baro_sample_t *baro,
                                 double t_tick, int ret, const nav_t *nav);

/* ---------------------------------------------------------------- estado (estático, opaco) */
#define ESKF_SVC_MAILBOX_CAP 64u
#define ESKF_SVC_SNAP_FRAGS 5u

typedef struct {
    int64_t rx_us;
    uint16_t apid, seq, len;
    uint32_t ts_ms;
    uint8_t payload[ESKF_SVC_PAYLOAD_MAX];
} eskf_svc_item_t;

typedef struct {
    int have;
    uint16_t last_seq;
    int64_t last_t_us;
} eskf_svc_stream_t;

typedef struct {
    uint8_t state;     /* ESKF_SVC_BB_* */
    int have_ref;
    int ref_frozen;
    double ref_m, h, v;
    int64_t t_us;      /* instante del estado = última medida ACEPTADA (no avanza con los rechazos) */
    int64_t t_rx_us;   /* última medida RECIBIDA (aceptada o no): detecta cortes */
    uint32_t n_ref, consec_rej;
    int64_t fast_t0;          /* inicio de |v_z| >= bb_flight_vz_mps continua (INT64_MIN: no) */
    int64_t rest_t0;          /* inicio del reposo continuo |v_z| <= 1 m/s (INT64_MIN: no) */
    uint32_t win_n;    /* ventana de inicio/recuperación (medidas consecutivas, sin aceptar aún) */
    int64_t win_t[ESKF_SVC_BB_WIN_MAX];
    float win_z[ESKF_SVC_BB_WIN_MAX];
} eskf_svc_bb_t;

typedef struct {
    uint32_t magic;
    int initialized;
    eskf_svc_config_t cfg;
    int nav_enabled;
    const char *why;
    uint16_t why_code, readiness, config_id, session;
    eskf_svc_clock_fn clock;
    void *clock_ctx;
    /* buzón */
    eskf_svc_item_t mb[ESKF_SVC_MAILBOX_CAP];
    volatile uint32_t mb_head, mb_tail;
    eskf_svc_stream_t st_imu, st_baro;
    int64_t last_popped_us;
    int64_t t_session0_us;    /* frontera: las muestras con marca anterior son de la sesión previa */
    int flight;               /* vuelo reconocido (enclavado hasta el final de la sesión) */
    int nav_cut;              /* navigation.c en rampa sin rampa confirmada (vuelo o fase incierta): no se le
                                 entregan más muestras ni reloj el resto de la sesión */
    int pad;                  /* rampa CONFIRMADA; 0 = fase incierta (irreversible en la sesión) */
    int64_t start_us;         /* reloj al empezar la sesión */
    int64_t pad_t_us;         /* hasta dónde consta que sigue en rampa (contrato de inicio o última evidencia) */
    int64_t pad_rx_us;        /* llegada (reloj) de la última muestra procesada: silencio total */
    int64_t now_us;           /* reloj del consume en curso (INT64_MIN antes del primero) */
    int64_t imu_run_t0, imu_prev_t; /* racha continua de IMU válida entregada a navigation.c */
    uint32_t seq_out_imu, seq_out_baro;
    nav_t nav;
    nav_merge_t merge;
    /* política de pérdidas y motivos enclavados */
    uint32_t latched;
    double t_fail_s;          /* instante (s) del último fallo que exige recuperación en rampa */
    uint32_t prev_rest_ok;    /* rest_windows_ok de navigation.c en la llamada anterior */
    int have_baro;
    int baro_ref_changed;     /* notificado: no se entregan baro a navigation.c */
    eskf_svc_bb_t bb;
    /* salida */
    int64_t next_snap_us, next_diag_us, next_ops_us;
    uint32_t snap_id, diag_id, ops_id;
    uint8_t tx[ESKF_SVC_SNAP_FRAGS][ESKF_SVC_PAYLOAD_MAX];
    uint8_t tx_len[ESKF_SVC_SNAP_FRAGS];
    uint8_t tx_n, tx_next;
    uint8_t txd[ESKF_SVC_PAYLOAD_MAX];
    uint8_t txd_len, txd_pending;
    uint8_t txo[ESKF_SVC_PAYLOAD_MAX];
    uint8_t txo_len, txo_pending;
    eskf_svc_output_t out; /* copia para consultas (sección crítica breve) */
    eskf_svc_diag_t out_diag;
    eskf_svc_output_t scratch;
    eskf_svc_diag_t diag;
    volatile uint32_t restart_req, baro_ref_req;
    /* traza de validación */
    eskf_svc_trec_t trace[ESKF_SVC_TRACE_CAP];
    uint32_t trace_n, trace_rd;
    uint32_t trace_count; /* registros escritos (para el cierre) */
    uint16_t trace_crc;   /* CRC acumulado de esos registros */
    uint8_t trace_closed; /* tras el END no se registra nada más */
    eskf_svc_hook_fn hook;
    void *hook_ctx;
} eskf_svc_t;

/* ---------------------------------------------------------------- API */
/* Configuración del perfil desde la tabla (eskf_svc_params.c). KPID a 0: los fija el anfitrión. */
int eskf_svc_default_config(int profile, eskf_svc_config_t *cfg);
/* NULL si el TRANSPORTE puede arrancar; si no, motivo (perfil, KPID, valores no finitos, límites). */
const char *eskf_svc_check_transport(const eskf_svc_config_t *cfg);
/* NULL si la NAVEGACIÓN puede arrancar; si no, primer pendiente o incoherencia. */
const char *eskf_svc_check_nav(const eskf_svc_config_t *cfg, uint16_t *why_code);
/* Suscripción del consumidor: kpid_imu | kpid_baro (nunca kpid_out). 0 si incoherente. */
uint16_t eskf_svc_subscription(const eskf_svc_config_t *cfg);

/* Arranca el servicio (sesión 1). Error sólo si el transporte no es viable. */
int eskf_svc_init(eskf_svc_t *s, const eskf_svc_config_t *cfg, eskf_svc_clock_fn clock, void *clock_ctx);
/* Idempotente: 1 si arrancó ahora, 0 si ya lo estaba (no toca configuración, colas ni sesión). */
int eskf_svc_ensure_init(eskf_svc_t *s, const eskf_svc_config_t *cfg, eskf_svc_clock_fn clock, void *clock_ctx);

int eskf_svc_deliver_packet(eskf_svc_t *s, const uint8_t pkt[ESKF_SVC_PKT_SIZE]);
int eskf_svc_consume(eskf_svc_t *s);
/* Siguiente mensaje de salida (≤ 50 B; prioridad: compacto, snapshot, diagnóstico); 1 si lo hay. */
int eskf_svc_produce(eskf_svc_t *s, uint8_t out[ESKF_SVC_PAYLOAD_MAX], uint16_t *len);
int eskf_svc_pending(const eskf_svc_t *s);
/* Registro del resultado de una publicación (pegamento): diagnóstico y traza. pkt_seq y
   pkt_ts_ms son la secuencia y el timestampMs que packetData escribió en el paquete (0 si no
   llegó a construirse): la reproducción rehace con ellos la cabecera exacta. */
void eskf_svc_note_output(eskf_svc_t *s, int outcome, uint8_t msg_type, uint16_t len, uint16_t pkt_seq,
                          uint32_t pkt_ts_ms);
int eskf_svc_get_output(const eskf_svc_t *s, eskf_svc_output_t *out);
int eskf_svc_get_diag(const eskf_svc_t *s, eskf_svc_diag_t *d);
/* Reinicio de sesión: se ejecuta en el siguiente consume; en vuelo se rechaza y se marca. */
void eskf_svc_request_restart(eskf_svc_t *s);
/* Cambio de referencia del baro CONFIRMADO por el anfitrión (no se deduce de las muestras). */
void eskf_svc_notify_baro_ref_change(eskf_svc_t *s);
void eskf_svc_set_hook(eskf_svc_t *s, eskf_svc_hook_fn fn, void *ctx);
uint32_t eskf_svc_trace_read(eskf_svc_t *s, eskf_svc_trec_t *dst, uint32_t n);
/* Cierra la traza con un registro END verificable (misma tarea que trace_read). ESKF_SVC_OK (también
   si ya estaba cerrada); ESKF_SVC_E_FULL si no cabe: vaciarla con trace_read y repetir;
   ESKF_SVC_E_CONFIG si la traza no está activada. Después no se registra nada más. */
int eskf_svc_trace_close(eskf_svc_t *s);
/* CRC-16/CCITT de un registro en su forma de 16 B (la del archivo: tipo, res, a, b, c, t LE) */
uint16_t eskf_svc_trec_crc(const eskf_svc_trec_t *r, uint16_t crc);

int eskf_svc_imu_decode(const eskf_svc_config_t *c, const uint8_t *payload, uint16_t len, nav_imu_sample_t *out,
                        uint8_t *sat_sensor, int *bad);
int eskf_svc_check_rotation(const float r[9]);
uint16_t eskf_svc_crc16(const uint8_t *p, size_t n, uint16_t crc);

#ifdef __cplusplus
}
#endif

#endif
