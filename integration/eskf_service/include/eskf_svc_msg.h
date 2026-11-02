/*
 * eskf_svc_msg.h — esquema VERSIONADO de la salida SPP del servicio (docs/SALIDA.md).
 *
 * nav_output_t no cabe en 50 bytes y su ABI no es un protocolo: se serializa campo a campo,
 * little-endian, sin depender de la alineación ni del tipo real del núcleo.
 *
 * Versión 3 (31-oct): tipo 1 añade capacidades y fuente; tipo 7 = salida operativa compacta;
 * banderas EXPERIMENTAL/ESTRICTO en lugar de BANCO/VUELO. Un decodificador v2 rechaza v3.
 * Versión 4 (1-nov): la fase tiene tres valores (0 rampa CONFIRMADA, 1 vuelo, 2 INCIERTA): bit 0 =
 * vuelo y bit 3 = incierta en el byte de fase del tipo 1 (35) y del tipo 7 (20). Un decodificador
 * v3 leería «rampa» donde ahora es «incierta»: por eso cambia la versión.
 *
 * Cabecera común de 12 bytes en cada mensaje:
 *   0 versión (ESKF_MSG_VERSION)   1 tipo   2 máscara del snapshot   3 banderas
 *   4–5 sesión u16   6–9 id u32 (snapshot o diagnóstico)   10–11 CRC-16 del mensaje completo
 *   (datos de todos los fragmentos de la máscara, en orden de tipo; inicial 0xFFFF, 0x1021)
 * Tipos 1–5: fragmentos de un snapshot (máscara = fragmentos presentes). Tipo 6: diagnóstico y
 * tipo 7: salida operativa compacta, mensajes independientes de un solo fragmento.
 */
#ifndef ESKF_SVC_MSG_H
#define ESKF_SVC_MSG_H

#include <stdint.h>

#include "eskf_service.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESKF_MSG_VERSION 4u
#define ESKF_MSG_HDR 12u
#define ESKF_MSG_T_STATE 1u
#define ESKF_MSG_T_PV 2u
#define ESKF_MSG_T_ATT 3u
#define ESKF_MSG_T_SIG 4u
#define ESKF_MSG_T_BIAS 5u
#define ESKF_MSG_T_DIAG 6u
#define ESKF_MSG_T_OPS 7u
#define ESKF_MSG_SNAP_MASK_ALL 0x1Fu

#define ESKF_MSG_F_TIME_MS 0x01u  /* marcas de construcción en ms */
#define ESKF_MSG_F_NAV_ON 0x02u   /* la navegación está habilitada */
#define ESKF_MSG_F_EXP 0x04u      /* perfil EXPERIMENTAL: hipótesis con procedencia, sin validación física */
#define ESKF_MSG_F_STRICT 0x08u   /* perfil ESTRICTO */
#define ESKF_MSG_F_ACCEPT 0x10u   /* flight_accept: ESTRICTO, readiness == 0 y validez del servicio */

/* longitud de datos por tipo (índice = tipo); todas <= 50 − 12 */
extern const uint8_t ESKF_MSG_LEN[8];

typedef struct {
    uint16_t session;
    uint32_t snap_id;
    uint8_t flags, mask;
    int64_t t_state_us, t_eval_us, t_proc_us;
    uint16_t nav_reasons, svc_reasons, readiness, why_code, config_id;
    uint8_t nav_validity, svc_validity, phase /* ESKF_SVC_PHASE_* */, at_rest, heading_known, profile, time_source, caps,
        source;
    float p[3], v[3], alt_ref;
    uint32_t alt_ref_seq;
    float q[4], sigma_th[3], sigma_p[3], sigma_v[3], b_a[3], b_g[3];
} eskf_msg_snapshot_t;

typedef struct {
    uint16_t session;
    uint32_t diag_id;
    uint8_t flags;
    uint32_t imu_rx, baro_rx, imu_lost, baro_lost;
    uint16_t rejected, sat_cand, nav_gaps, out_fail, skipped, lat_rx_max_ms, lat_proc_max_ms;
    int16_t last_imu_ret, last_baro_ret, last_tick_ret;
    uint8_t mailbox_hwm, seq_resyncs;
} eskf_msg_diag_t;

/* Salida operativa compacta (tipo 7): autónoma, para telemetría */
typedef struct {
    uint16_t session;
    uint32_t ops_id;
    uint8_t flags;
    uint32_t t_ms;            /* reloj del servicio al construirla (ms, truncado) */
    uint16_t eskf_age_ms;     /* t_ms − instante del estado ESKF; 0xFFFF si no hay */
    uint16_t bb_age_ms;       /* t_ms − último baro del respaldo; 0xFFFF si no hay */
    uint16_t nav_reasons, svc_reasons, readiness, config_id, boot_id;
    uint8_t nav_validity, svc_validity, caps, source, phase /* ESKF_SVC_PHASE_* */, profile, bb_valid, bb_ref_frozen;
    float eskf_h_m, eskf_vz_mps; /* −p_z y −v_z del ESKF: sobre la referencia, positivos hacia arriba */
    float bb_h_m, bb_vz_mps;     /* respaldo barométrico: sobre el lanzamiento, positivos hacia arriba */
} eskf_msg_ops_t;

/* Codificación (la usa el servicio). frags: hasta 5 mensajes de ≤ 50 B; devuelve cuántos. */
uint8_t eskf_msg_encode_snapshot(const eskf_msg_snapshot_t *s, uint8_t frags[ESKF_SVC_SNAP_FRAGS][ESKF_SVC_PAYLOAD_MAX],
                                 uint8_t lens[ESKF_SVC_SNAP_FRAGS]);
uint8_t eskf_msg_encode_diag(const eskf_msg_diag_t *d, uint8_t out[ESKF_SVC_PAYLOAD_MAX]);
uint8_t eskf_msg_encode_ops(const eskf_msg_ops_t *o, uint8_t out[ESKF_SVC_PAYLOAD_MAX]);

/* Reconstrucción (tierra, pruebas y herramientas). */
typedef struct {
    uint16_t session;
    uint32_t snap_id;
    uint8_t flags, mask, have;
    uint16_t crc;
    uint8_t data[ESKF_SVC_SNAP_FRAGS][ESKF_SVC_PAYLOAD_MAX - ESKF_MSG_HDR];
    uint32_t complete, incomplete, crc_bad, mixed, invalid, diag_ok, ops_ok;
} eskf_msg_reasm_t;
void eskf_msg_reasm_init(eskf_msg_reasm_t *r);
/* 1: snapshot completo en *snap; 2: diagnóstico en *diag; 3: compacta en *ops (con
   eskf_msg_reasm_push_ops); 0: falta algo; < 0: mensaje inválido.
   Un fragmento de otro (sesión, id) antes de completar cuenta «incomplete» y empieza de nuevo;
   la misma (sesión, id) con CRC o banderas distintas cuenta «mixed» y se descarta. */
int eskf_msg_reasm_push(eskf_msg_reasm_t *r, const uint8_t *msg, uint16_t len, eskf_msg_snapshot_t *snap,
                        eskf_msg_diag_t *diag);
int eskf_msg_reasm_push_ops(eskf_msg_reasm_t *r, const uint8_t *msg, uint16_t len, eskf_msg_snapshot_t *snap,
                            eskf_msg_diag_t *diag, eskf_msg_ops_t *ops);

#ifdef __cplusplus
}
#endif

#endif
