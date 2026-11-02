/* test_common.h — utilidades de prueba: CHECK, reloj inyectable, emulación de los payloads de los
   productores SPP 9247d9e (sólo la aritmética de los floats publicados; sin hardware). */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "eskf_service.h"

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

static int g_fails = 0, g_checks = 0, g_verbose = -1;
/* TEST_VERBOSE=1: imprime también las comprobaciones correctas (con sus valores medidos) */
#define CHECK(cond, ...)                                                     \
    do {                                                                     \
        g_checks++;                                                          \
        if (g_verbose < 0) g_verbose = getenv("TEST_VERBOSE") != NULL;       \
        if (!(cond)) {                                                       \
            printf("  FALLO %s:%d: ", __FILE__, __LINE__);                   \
            printf(__VA_ARGS__);                                             \
            printf("\n");                                                    \
            g_fails++;                                                       \
        } else if (g_verbose) {                                              \
            printf("  ok: ");                                                \
            printf(__VA_ARGS__);                                             \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

/* reloj inyectable (µs) */
static int64_t g_now_us = 0;
static int64_t test_clock(void *ctx)
{
    (void)ctx;
    return g_now_us;
}

static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v, p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}
static void wrf(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    wr32(p, u);
}
static float rdf(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/* Paquete como SPP_SERVICES_DATABANK_packetData: relleno 0xFF, cabeceras, payload, CRC de 62 B. */
static void mk_packet(uint8_t p[64], uint16_t apid, uint16_t seq, uint32_t ts_ms, const void *pl, uint16_t len)
{
    memset(p, 0xFF, 64);
    p[0] = 1;
    wr16(p + 1, apid);
    wr16(p + 3, seq);
    wr16(p + 5, len);
    wr32(p + 7, ts_ms);
    p[11] = 0;
    if (len) memcpy(p + 12, pl, len);
    wr16(p + 62, eskf_svc_crc16(p, 62, 0xFFFFu));
}

/* Offsets del productor (icm20948.h, SPP 9247d9e) */
static const float OFF_A[3] = {0.004621f, -0.001157f, 0.010014f};
static const float OFF_G[3] = {0.681567f, 0.428753f, -0.329620f};

static int16_t clamp_raw(double x)
{
    double r = floor(x + 0.5);
    if (r > 32767.0) r = 32767.0;
    if (r < -32768.0) r = -32768.0;
    return (int16_t)r;
}

/* Payload de 44 B como icm20948.c: v = raw/8192.0f − offset (en float). a en g, w en «°/s
   publicados» (la escala efectiva del giro está pendiente; aquí sólo importa la aritmética). */
static void imu44(uint8_t pl[44], const double a_g[3], const double w_dps[3])
{
    for (int i = 0; i < 3; i++) {
        int16_t ra = clamp_raw(a_g[i] * 8192.0), rg = clamp_raw(w_dps[i] * 32.8);
        float va = ((float)ra / 8192.0f) - OFF_A[i];
        float vg = ((float)rg / 32.8f) - OFF_G[i];
        wrf(pl + 4 * i, va);
        wrf(pl + 12 + 4 * i, vg);
    }
    wrf(pl + 24, 1.0f);
    wrf(pl + 28, 0.0f);
    wrf(pl + 32, 0.0f);
    wrf(pl + 36, 0.0f);
    wrf(pl + 40, 3.0f);
}

/* Histórico (48 B): sin offsets, reservado */
static void imu48(uint8_t pl[48], const double a_g[3], const double w_dps[3], float reserved)
{
    for (int i = 0; i < 3; i++) {
        wrf(pl + 4 * i, (float)clamp_raw(a_g[i] * 8192.0) / 8192.0f);
        wrf(pl + 12 + 4 * i, (float)clamp_raw(w_dps[i] * 32.8) / 32.8f);
    }
    wrf(pl + 24, 1.0f);
    wrf(pl + 28, 0.0f);
    wrf(pl + 32, 0.0f);
    wrf(pl + 36, 0.0f);
    wrf(pl + 40, 304.0f);
    wrf(pl + 44, reserved);
}

/* BMP390 como bmp390.c: altura relativa con p0/T0 de calibración */
static const float P0 = 94768.0f, T0C = 21.5f;
static void baro12(uint8_t pl[12], double h_m)
{
    double t0k = (double)T0C + 273.15;
    float p = (float)((double)P0 * pow(1.0 - 0.0065 * h_m / t0k, 5.255));
    float tk = T0C + 273.15f;
    float alt = (tk / 0.0065f) * (1.0f - powf(p / P0, 1.0f / 5.255f));
    wrf(pl, alt);
    wrf(pl + 4, p);
    wrf(pl + 8, T0C);
}

/* Traza de validación en disco: 16 B por registro, little-endian (tipo, res, a, b, c, t). */
static void trace_put(FILE *f, const eskf_svc_trec_t *r, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint8_t b[16];
        b[0] = r[i].type;
        b[1] = (uint8_t)r[i].res;
        wr16(b + 2, r[i].a);
        wr16(b + 4, r[i].b);
        wr16(b + 6, r[i].c);
        uint64_t u = (uint64_t)r[i].t;
        for (int k = 0; k < 8; k++) b[8 + k] = (uint8_t)(u >> (8 * k));
        if (f) fwrite(b, 1, 16, f);
    }
}
static int trace_get(FILE *f, eskf_svc_trec_t *r)
{
    uint8_t b[16];
    if (fread(b, 1, 16, f) != 16) return 0;
    r->type = b[0];
    r->res = (int8_t)b[1];
    r->a = (uint16_t)(b[2] | (b[3] << 8));
    r->b = (uint16_t)(b[4] | (b[5] << 8));
    r->c = (uint16_t)(b[6] | (b[7] << 8));
    uint64_t u = 0;
    for (int k = 7; k >= 0; k--) u = (u << 8) | b[8 + k];
    r->t = (int64_t)u;
    return 1;
}

#endif
