/*
 * dump_params.c — vuelca la tabla ÚNICA de parámetros (vn_params.c) en otros formatos, para
 * que la documentación y las herramientas salgan de la misma fuente.
 *
 *   dump_params --md                 tabla Markdown (PARAMETROS.md la incluye; `make` comprueba
 *                                    que coincide)
 *   dump_params --replay bench|vuelo órdenes CFG/IMUC para nav_replay con ese perfil
 *                                    (vuelo deja «nan» lo pendiente: nav_replay lo rechaza)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../viradinha_nav/src/vn/vn_logfmt.h"
#include "../viradinha_nav/src/vn/vn_params.h"

static const char *st(uint8_t s)
{
    switch (s) {
    case VN_ST_CONFIRMED: return "confirmado";
    case VN_ST_PROPOSED: return "propuesto";
    case VN_ST_PROVISIONAL: return "provisional";
    case VN_ST_PENDING: return "**pendiente**";
    default: return "?";
    }
}

static void val(char *b, size_t n, double v)
{
    if (isnan(v)) snprintf(b, n, "—");
    else snprintf(b, n, "%.10g", v);
}

int main(int argc, char **argv)
{
    size_t n;
    const vn_param_t *T = vn_params_table(&n);
    if (argc == 2 && !strcmp(argv[1], "--md")) {
        printf("| parámetro | vuelo | estado (vuelo) | banco | estado (banco) | origen / cómo obtenerlo |\n");
        printf("|---|---|---|---|---|---|\n");
        for (size_t i = 0; i < n; i++) {
            char a[32], b[32];
            val(a, sizeof a, T[i].flight);
            val(b, sizeof b, T[i].bench);
            printf("| `%s` | %s | %s | %s | %s | %s |\n", T[i].name, a, st(T[i].status), b, st(T[i].bench_status),
                   T[i].source);
        }
        return 0;
    }
    if (argc == 3 && !strcmp(argv[1], "--replay")) {
        int prof;
        if (!strcmp(argv[2], "bench")) prof = VN_PROFILE_BENCH;
        else if (!strcmp(argv[2], "vuelo")) prof = VN_PROFILE_FLIGHT;
        else return 2;
        printf("# dump_params --replay %s (tabla de vn_params.c)\n", argv[2]);
        for (size_t i = 0; i < n; i++) {
            double v = vn_param_value(&T[i], prof);
            if (T[i].kind == VN_PK_NAV) printf("CFG %s %.17g\n", T[i].name, v);
            else if (T[i].kind == VN_PK_IMUC) printf("IMUC %s %.17g\n", T[i].name, v);
        }
        return 0;
    }
    fprintf(stderr, "uso: dump_params --md | --replay bench|vuelo\n");
    return 2;
}
