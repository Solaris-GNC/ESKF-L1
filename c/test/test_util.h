/* test_util.h — utilidades comunes de las pruebas en el ordenador (no van al firmware). */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "eskf.h"

static int g_fails = 0, g_checks = 0;

#define CHECK(cond, ...)                                          \
    do {                                                          \
        g_checks++;                                               \
        if (!(cond)) {                                            \
            printf("  FALLO %s:%d: ", __FILE__, __LINE__);        \
            printf(__VA_ARGS__);                                  \
            printf("\n");                                         \
            g_fails++;                                            \
        }                                                         \
    } while (0)

/* Tolerancia relativa del test de semidefinición: δ = PSD_REL·max(1, max diag P).
   En float, P lleva errores de redondeo del orden de 1e-7·‖P‖, de ahí la diferencia. */
#ifdef ESKF_USE_FLOAT
#define PSD_REL 1e-6
#else
#define PSD_REL 1e-10
#endif

/*
 * Semidefinición positiva con tolerancia, matemáticamente suficiente:
 * P es simétrica y la factorización de Cholesky de P + δI (en double) existe
 * ⇔ P + δI es definida positiva ⇔ λmin(P) > −δ.
 * Cholesky es estable hacia atrás: el error de redondeo equivale a perturbar la matriz
 * en O(n·ε·‖P‖) ≈ 1e-14·‖P‖, muy por debajo de δ. Sustituye a la comprobación antigua por
 * parejas (|P_ij|² <= P_ii·P_jj), que es necesaria pero no suficiente.
 * Devuelve 1 si se cumple. Si min_pivot != NULL, devuelve el menor pivote (diagnóstico).
 */
static inline int matrix_is_psd(eskf_real P[ESKF_N][ESKF_N], double rel_tol, double *min_pivot)
{
    double L[ESKF_N][ESKF_N];
    double maxd = 1.0;
    for (int i = 0; i < ESKF_N; i++) {
        if (!isfinite((double)P[i][i])) return 0;
        if (fabs((double)P[i][i]) > maxd) maxd = fabs((double)P[i][i]);
        for (int j = 0; j < ESKF_N; j++)
            if (P[i][j] != P[j][i] || !isfinite((double)P[i][j])) return 0;
    }
    double delta = rel_tol * maxd, mp = INFINITY;
    for (int j = 0; j < ESKF_N; j++) {
        double s = (double)P[j][j] + delta;
        for (int k = 0; k < j; k++) s -= L[j][k] * L[j][k];
        if (s < mp) mp = s;
        if (!(s > 0.0)) {
            if (min_pivot) *min_pivot = s;
            return 0;
        }
        L[j][j] = sqrt(s);
        for (int i = j + 1; i < ESKF_N; i++) {
            double t = (double)P[i][j];
            for (int k = 0; k < j; k++) t -= L[i][k] * L[j][k];
            L[i][j] = t / L[j][j];
        }
    }
    if (min_pivot) *min_pivot = mp;
    return 1;
}

static inline int P_ok(const eskf_t *f)
{
    eskf_real P[ESKF_N][ESKF_N];
    memcpy(P, f->P, sizeof P);
    return matrix_is_psd(P, PSD_REL, NULL);
}

/* Copia de la parte de la estimación que una entrada inválida no debe tocar */
typedef struct {
    eskf_real p[3], v[3], q[4], b_a[3], b_g[3];
    eskf_real P[ESKF_N][ESKF_N];
    int fault, at_rest, heading_known;
} snapshot_t;

static inline void snap(const eskf_t *f, snapshot_t *s)
{
    memset(s, 0, sizeof *s);
    memcpy(s->p, f->p, sizeof s->p);
    memcpy(s->v, f->v, sizeof s->v);
    memcpy(s->q, f->q, sizeof s->q);
    memcpy(s->b_a, f->b_a, sizeof s->b_a);
    memcpy(s->b_g, f->b_g, sizeof s->b_g);
    memcpy(s->P, f->P, sizeof s->P);
    s->fault = f->fault;
    s->at_rest = f->at_rest;
    s->heading_known = f->heading_known;
}

static inline int unchanged(const eskf_t *f, const snapshot_t *before)
{
    snapshot_t now;
    snap(f, &now);
    return memcmp(&now, before, sizeof now) == 0;
}

/* Parámetros comunes de prueba. dt_max corresponde a una IMU HIPOTÉTICA de 100 Hz:
   no es la frecuencia del vuelo, que está pendiente de confirmar. */
static inline void test_params(eskf_params_t *prm)
{
    eskf_default_params(prm);
    prm->dt_max = (eskf_real)0.015;
}

static inline double urand(unsigned *state)
{
    *state = *state * 1664525u + 1013904223u;
    return 2.0 * ((*state >> 8) / 16777216.0) - 1.0;
}

static inline double nrand(unsigned *state) /* N(0,1), Box-Muller */
{
    double u1 = 0.5 * (urand(state) + 1.0), u2 = 0.5 * (urand(state) + 1.0);
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static inline void random_quat(unsigned *st, eskf_real q[4])
{
    double v[4], n = 0;
    for (int i = 0; i < 4; i++) { v[i] = nrand(st); n += v[i] * v[i]; }
    n = sqrt(n);
    for (int i = 0; i < 4; i++) q[i] = (eskf_real)(v[i] / n);
}

/* Rᵀ v */
static inline void rotT(const eskf_real q[4], const double v[3], double out[3])
{
    eskf_real R[3][3];
    eskf_rotation_matrix(q, R);
    for (int i = 0; i < 3; i++)
        out[i] = (double)R[0][i] * v[0] + (double)R[1][i] * v[1] + (double)R[2][i] * v[2];
}

/* Ángulo entre dos orientaciones: 2·atan2(‖vec(q1*⊗q2)‖, |w|), estable cerca de 0 */
static inline double rot_angle_between(const eskf_real q1[4], const eskf_real q2[4])
{
    double a0 = q1[0], a1 = q1[1], a2 = q1[2], a3 = q1[3];
    double b0 = q2[0], b1 = q2[1], b2 = q2[2], b3 = q2[3];
    double w = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;
    double x = a0 * b1 - a1 * b0 - a2 * b3 + a3 * b2;
    double y = a0 * b2 + a1 * b3 - a2 * b0 - a3 * b1;
    double z = a0 * b3 - a1 * b2 + a2 * b1 - a3 * b0;
    return 2.0 * atan2(sqrt(x * x + y * y + z * z), fabs(w));
}

static inline int report(const char *suite)
{
    printf("%s [%s]: %d/%d comprobaciones OK\n", suite, sizeof(eskf_real) == 4 ? "float" : "double",
           g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}

#endif
