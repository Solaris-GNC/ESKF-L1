/*
 * replay.c — ejecuta el ESKF en C con comandos de texto por stdin y escribe el
 * estado completo tras cada comando. Lo usa compare_with_python.py para comprobar
 * que el port en C reproduce eskf.py paso a paso.
 *
 * Comandos (uno por línea, números separados por espacios, número EXACTO de argumentos):
 *   PARAMS sa sw saw sww thr gate has_mag mx my mz dt_max gap_max   (12)
 *   PDIAG  d0 ... d14                                               (15, varianzas)
 *   STATE  p3 v3 q4 ba3 bg3                                         (16, ver nota)
 *   REST   0|1                                                      (1)
 *   PRED   a3 w3 dt                                                 (7)
 *   GPS    p3 v3 sp3 sv3                                            (12)
 *   VEL    v3 sv3                                                   (6)
 *   ZUPT   sv3                                                      (3)
 *   MAG    m3 s3                                                    (6)
 *   ACC    a3 s3                                                    (6)
 *   BAR    z s                                                      (2)
 * Las líneas vacías y las que empiezan por '#' se ignoran.
 *
 * Números: se aceptan los que strtod() convierte completos y sin desbordamiento, incluidos
 * "nan" e "inf" escritos explícitamente (sirven para probar la validación del filtro).
 * Texto no numérico, argumentos de más o de menos, comandos desconocidos o líneas
 * demasiado largas terminan el programa con código 2 y un mensaje en stderr.
 *
 * Nota: STATE fija el estado completo, actitud incluida, como hace el comparador con
 * Python; por eso marca heading_known = 1. No existe equivalente en la API de vuelo.
 *
 * Salida tras cada comando:  OUT ret p3 v3 q4 ba3 bg3 P(225)
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "eskf.h"

#define MAXARGS 16
#define LINE_MAX_LEN 4096

static unsigned long g_line = 0;

static void fail(const char *msg, const char *detail)
{
    fprintf(stderr, "replay: línea %lu: %s%s%s\n", g_line, msg, detail ? ": " : "", detail ? detail : "");
    exit(2);
}

static double parse_number(const char *tok)
{
    char *end = NULL;
    errno = 0;
    double v = strtod(tok, &end);
    if (end == tok || *end != '\0') fail("número inválido", tok);
    if (errno == ERANGE) fail("número fuera de rango", tok);
    return v;
}

static void dump(const eskf_t *f, int ret)
{
    printf("OUT %d", ret);
    for (int i = 0; i < 3; i++) printf(" %.17g", (double)f->p[i]);
    for (int i = 0; i < 3; i++) printf(" %.17g", (double)f->v[i]);
    for (int i = 0; i < 4; i++) printf(" %.17g", (double)f->q[i]);
    for (int i = 0; i < 3; i++) printf(" %.17g", (double)f->b_a[i]);
    for (int i = 0; i < 3; i++) printf(" %.17g", (double)f->b_g[i]);
    for (int i = 0; i < ESKF_N; i++)
        for (int j = 0; j < ESKF_N; j++) printf(" %.17g", (double)f->P[i][j]);
    printf("\n");
}

static const struct {
    const char *name;
    int nargs;
} CMDS[] = {{"PARAMS", 12}, {"PDIAG", 15}, {"STATE", 16}, {"REST", 1}, {"PRED", 7}, {"GPS", 12},
            {"VEL", 6},     {"ZUPT", 3},   {"MAG", 6},    {"ACC", 6},  {"BAR", 2}};

int main(void)
{
    eskf_params_t prm;
    eskf_t f;
    if (eskf_default_params(&prm) != ESKF_OK) fail("eskf_default_params", NULL);
    prm.dt_max = 1.0; /* el escenario lo fija con PARAMS */
    if (eskf_init(&f, &prm) != ESKF_OK) fail("eskf_init", NULL);

    char line[LINE_MAX_LEN];
    while (fgets(line, sizeof line, stdin)) {
        g_line++;
        size_t len = strlen(line);
        if (len == sizeof line - 1 && line[len - 1] != '\n' && !feof(stdin)) fail("línea demasiado larga", NULL);

        char *tok = strtok(line, " \t\r\n");
        if (!tok || tok[0] == '#') continue;
        const char *cmd = tok;

        int expected = -1;
        for (size_t i = 0; i < sizeof CMDS / sizeof CMDS[0]; i++)
            if (!strcmp(cmd, CMDS[i].name)) expected = CMDS[i].nargs;
        if (expected < 0) fail("comando desconocido", cmd);

        double v[MAXARGS];
        int n = 0;
        while ((tok = strtok(NULL, " \t\r\n"))) {
            if (n >= expected) fail("sobran argumentos para", cmd);
            v[n++] = parse_number(tok);
        }
        if (n != expected) fail("faltan argumentos para", cmd);

        eskf_real r[MAXARGS];
        for (int i = 0; i < n; i++) r[i] = (eskf_real)v[i];
        int ret = 0;

        if (!strcmp(cmd, "PARAMS")) {
            if (v[6] != 0.0 && v[6] != 1.0) fail("has_mag debe ser 0 o 1", NULL);
            prm.sigma_a = r[0]; prm.sigma_w = r[1]; prm.sigma_aw = r[2]; prm.sigma_ww = r[3];
            prm.accel_threshold = r[4]; prm.innov_gate = r[5]; prm.has_mag_ref = (int)v[6];
            prm.mag_ref[0] = r[7]; prm.mag_ref[1] = r[8]; prm.mag_ref[2] = r[9];
            prm.dt_max = r[10]; prm.gap_max = r[11];
            ret = eskf_init(&f, &prm);
            if (ret != ESKF_OK) {
                fprintf(stderr, "replay: línea %lu: eskf_init: %s\n", g_line, eskf_status_str(ret));
                exit(3);
            }
        } else if (!strcmp(cmd, "PDIAG")) {
            for (int i = 0; i < ESKF_N; i++)
                for (int j = 0; j < ESKF_N; j++) f.P[i][j] = (i == j) ? r[i] : 0;
        } else if (!strcmp(cmd, "STATE")) {
            memcpy(f.p, &r[0], sizeof f.p); memcpy(f.v, &r[3], sizeof f.v);
            memcpy(f.q, &r[6], sizeof f.q); memcpy(f.b_a, &r[10], sizeof f.b_a);
            memcpy(f.b_g, &r[13], sizeof f.b_g);
            f.heading_known = 1;
        } else if (!strcmp(cmd, "REST")) {
            if (v[0] != 0.0 && v[0] != 1.0) fail("REST debe ser 0 o 1", NULL);
            ret = eskf_set_rest(&f, (int)v[0]);
        } else if (!strcmp(cmd, "PRED")) {
            ret = eskf_predict(&f, &r[0], &r[3], r[6]);
        } else if (!strcmp(cmd, "GPS")) {
            ret = eskf_update_gps(&f, &r[0], &r[3], &r[6], &r[9]);
        } else if (!strcmp(cmd, "VEL")) {
            ret = eskf_update_velocity(&f, &r[0], &r[3]);
        } else if (!strcmp(cmd, "ZUPT")) {
            ret = eskf_update_zupt(&f, &r[0]);
        } else if (!strcmp(cmd, "MAG")) {
            ret = eskf_update_mag(&f, &r[0], &r[3]);
        } else if (!strcmp(cmd, "ACC")) {
            ret = eskf_update_accel(&f, &r[0], &r[3]);
        } else if (!strcmp(cmd, "BAR")) {
            ret = eskf_update_baro(&f, r[0], r[1]);
        }
        dump(&f, ret);
    }
    if (ferror(stdin)) fail("error de lectura", NULL);
    return 0;
}
