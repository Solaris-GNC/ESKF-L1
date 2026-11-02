/*
 * svc_replay.c — anfitrión de PRUEBA en el PC: pasa un registro de paquetes SPP de 64 B (binario)
 * por el servicio. Dos modos, que NO son lo mismo:
 *   - calendario propio (por defecto): reproducción DETERMINISTA en el PC; reloj = marca + retardo,
 *     un consume cada K paquetes y como mucho UN mensaje producido por consume (como acquireData).
 *     No es el router SPP (éste se prueba en test_spp_router.c).
 *   - --reproducir TRAZA: reproduce el comportamiento REAL registrado por el pegamento: entregas,
 *     consumes, reinicios, avisos de referencia y el resultado de cada publicación (publicado,
 *     perdido, aplazado). Con el mismo registro de entradas, los mensajes salen idénticos.
 *
 *   svc_replay [opciones] registro.bin
 *     --perfil adq|exp|estricto  (exp por defecto; «banco» y «vuelo» se aceptan como alias)
 *     --historico                ICM de 48 B sin offsets y altura ISA (captura del 24-oct)
 *     --kpid I,B,O               KPID de IMU, baro y salida (por defecto 0x8,0x4,0x40 en histórico,
 *                                0x4,0x8,0x10 si no)
 *     --retardo-us N             recepción = timestampMs·1000 + N
 *     --consumir-cada K          consume tras cada K paquetes
 *     --salidas F                una línea OUT por nav_tick / nav_on_imu / nav_on_baro (formato nav_replay)
 *     --entradas F               las llamadas a navigation.c como órdenes de nav_replay (CFG, INIT, IMU, BARO, TICK)
 *     --mensajes F               mensajes de salida PUBLICADOS como paquetes SPP de 64 B (KPID de salida)
 *     --telemetria               preajuste de salida ESKF_OUT_TELEMETRY
 *     --estado-us N / --compacta-us N   periodos del estado completo y de la compacta (análisis)
 *     --arranque N               boot_id
 *     --traza-val F              traza de validación (registros de 16 B) vaciada tras cada consume y
 *                                cerrada al final con su registro END
 *     --reproducir F             en lugar del calendario propio, el de una traza de validación
 *     --parcial                  con --reproducir: admite una traza SIN cierre (placa apagada o captura
 *                                cortada): reproduce lo registrado y lo declara PARCIAL (código 3)
 *     --resumen                  contadores
 *   svc_replay --dump-md         tabla de parámetros (docs/PARAMETROS_SPP.md)
 *   svc_replay --dump-cfg banco  órdenes CFG de nav_replay para el perfil
 *
 * El reloj inyectado vale timestampMs·1000 + retardo en cada entrega y en cada consume; tras el
 * archivo, 50 llamadas más cada 10 ms (sin entradas: caducidad) y se vacía la salida.
 * La traza (versión ESKF_SVC_TRACE_VERSION = 5) empieza con un registro CONFIG (versión de traza,
 * config_id, perfil, versión de esquema, arranque y reloj del inicio) y termina con un registro END (nº de registros
 * anteriores, su CRC-16 y los perdidos por desbordamiento). Con --reproducir NO hay éxito
 * silencioso: antes de reproducir se comprueba el cierre; después, el servicio reproducido vuelve a
 * generar su traza y se contrasta registro a registro con la original (entradas: KPID, secuencia,
 * longitud y resultado; consumes: paquetes, llamadas y mensajes construidos; reinicios aceptados o
 * rechazados; transiciones de fase (PHASE: vuelo o rampa caducada, por hueco o por silencio); salidas: resultado, TIPO y LONGITUD del mensaje realmente producido, secuencia y
 * timestampMs; el END final). Se detiene en el primer problema y dice el registro (índice y byte).
 *   código 0  reproducción COMPLETA: traza cerrada y contrastada entera, entradas agotadas
 *   código 3  reproducción PARCIAL (sólo con --parcial y traza sin cierre): coherente hasta donde
 *             llega; no prueba que la captura esté completa
 *   código 4  incoherencia entre traza, entradas y salida (metadatos distintos de la operación
 *             reproducida, CRC de la traza distinto, END en mitad, entradas agotadas o sobrantes,
 *             mensaje inexistente, secuencia de salida distinta, registro desconocido)
 *   código 5  captura incompleta: traza no múltiplo de 16 B, sin registro de cierre, con registros
 *             que faltan o sobran respecto al END, o desbordada en la placa; entradas no múltiplo de 64 B
 *   código 6  traza de otra versión o de otra configuración, o sin CONFIG al principio
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "eskf_service.h"
#include "eskf_svc_msg.h"
#include "eskf_svc_params.h"
#include "test_common.h"

static eskf_svc_t svc; /* estático, como en la placa */
static int64_t g_now = 0;
static int64_t clk(void *c)
{
    (void)c;
    return g_now;
}

typedef struct {
    FILE *out, *in;
    uint16_t session;
} hook_ctx_t;

static void out_line(FILE *f, int ret, const nav_t *n)
{
    nav_output_t o;
    nav_get_output(n, &o);
    fprintf(f, "OUT %d %.17g %u 0x%04x", ret, o.t_state, (unsigned)o.validity, (unsigned)o.reasons);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.p[i]);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.v[i]);
    for (int i = 0; i < 4; i++) fprintf(f, " %.9g", (double)o.q[i]);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.sigma_p[i]);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.sigma_v[i]);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.sigma_th[i]);
    fprintf(f, " %.17g", o.alt_ref);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.b_a[i]);
    for (int i = 0; i < 3; i++) fprintf(f, " %.9g", (double)o.b_g[i]);
    fprintf(f, " %u %u\n", (unsigned)o.alt_ref_seq, (unsigned)n->f.degraded);
}

static int nav_field(const char *name)
{
    return !strncmp(name, "eskf.", 5) || !strncmp(name, "rest_", 5) || !strncmp(name, "baro_", 5) ||
           !strncmp(name, "zupt", 4) || !strncmp(name, "grav", 4) || !strncmp(name, "lift", 4) ||
           !strncmp(name, "acc_", 4) || !strncmp(name, "gyro_", 5) || !strncmp(name, "stale", 5);
}

static void dump_cfg(FILE *f, const eskf_svc_config_t *c)
{
    size_t n;
    const eskf_param_t *T = eskf_svc_params_table(&n);
    for (size_t k = 0; k < n; k++) {
        if (!nav_field(T[k].name)) continue;
        double v = eskf_svc_param_read(c, &T[k]);
        if (!strcmp(T[k].name, "eskf.has_mag_ref")) fprintf(f, "CFG %s %d\n", T[k].name, (int)v);
        else fprintf(f, "CFG %s %.17g\n", T[k].name, v);
    }
}

static void hook(void *ctx, int kind, const nav_imu_sample_t *imu, const nav_baro_sample_t *baro, double t, int ret,
                 const nav_t *nav)
{
    hook_ctx_t *h = (hook_ctx_t *)ctx;
    if (h->in) {
        if (svc.session != h->session) fprintf(h->in, "INIT\n"), h->session = svc.session; /* sesión nueva: nav_init */
        if (kind == 0) fprintf(h->in, "TICK %.17g\n", t);
        else if (kind == 1)
            fprintf(h->in, "IMU %.17g %.9g %.9g %.9g %.9g %.9g %.9g %u\n", imu->t, (double)imu->acc[0], (double)imu->acc[1],
                    (double)imu->acc[2], (double)imu->gyro[0], (double)imu->gyro[1], (double)imu->gyro[2], (unsigned)imu->sat);
        else fprintf(h->in, "BARO %.17g %.9g\n", baro->t, (double)baro->alt_m);
    }
    if (h->out) out_line(h->out, ret, nav);
}

static int profile_of(const char *s)
{
    if (!strcmp(s, "adq")) return ESKF_SVC_PROFILE_ACQ;
    if (!strcmp(s, "exp") || !strcmp(s, "banco")) return ESKF_SVC_PROFILE_EXP;
    if (!strcmp(s, "estricto") || !strcmp(s, "vuelo")) return ESKF_SVC_PROFILE_STRICT;
    fprintf(stderr, "perfil desconocido: %s\n", s);
    exit(2);
}

static const char *st(uint8_t s)
{
    switch (s) {
    case ESKF_ST_CONFIRMED: return "confirmado (código SPP)";
    case ESKF_ST_PROPOSED: return "propuesto (política)";
    case ESKF_ST_PROVISIONAL: return "provisional";
    case ESKF_ST_OBSERVED: return "observado (registro)";
    case ESKF_ST_SIMULATED: return "simulado";
    default: return "**pendiente**";
    }
}

static const char *grp(uint16_t g)
{
    switch (g) {
    case ESKF_SVC_RDY_UNITS: return "unidades";
    case ESKF_SVC_RDY_AXES: return "ejes";
    case ESKF_SVC_RDY_RANGE: return "rango";
    case ESKF_SVC_RDY_TIME: return "tiempo";
    case ESKF_SVC_RDY_BARO: return "baro";
    case ESKF_SVC_RDY_NOISE: return "ruido";
    case ESKF_SVC_RDY_THRESH: return "umbrales";
    case ESKF_SVC_RDY_LIFTOFF: return "despegue";
    case ESKF_SVC_RDY_HOST: return "anfitrión";
    default: return "—";
    }
}

static uint16_t g_oseq = 0;
static long g_msgs = 0, g_lost = 0;
static long fsize(FILE *f)
{
    long cur = ftell(f);
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, cur, SEEK_SET);
    return n;
}
/* Un mensaje producido como lo haría acquireData: secuencia propia, packetData en el instante
   g_now y el resultado anotado (outcome). Sólo los PUBLICADOS se escriben en fm. */
/* Un mensaje producido como en acquireData; la cabecera (secuencia, timestampMs) es la que se
   pasa: en el calendario propio la del reloj, al reproducir la que construyó packetData. */
static int emit_one(FILE *fm, int outcome, uint16_t seq, uint32_t ts_ms)
{
    uint8_t m[ESKF_SVC_PAYLOAD_MAX];
    uint16_t len;
    if (!eskf_svc_produce(&svc, m, &len)) return 0;
    g_msgs++;
    g_oseq = (uint16_t)(seq + 1u);
    uint8_t p[64];
    mk_packet(p, svc.cfg.kpid_out, seq, ts_ms, m, len);
    if (fm && outcome == ESKF_SVC_OUT_PUBLISHED) fwrite(p, 1, 64, fm);
    if (outcome != ESKF_SVC_OUT_PUBLISHED) g_lost++;
    eskf_svc_note_output(&svc, outcome, m[1], len, seq, outcome == ESKF_SVC_OUT_PACKET_FAIL ? 0u : ts_ms);
    return 1;
}
#define EMIT_NOW(fm) emit_one((fm), ESKF_SVC_OUT_PUBLISHED, g_oseq, (uint32_t)(g_now / 1000))

static void drain_trace(FILE *ft)
{
    eskf_svc_trec_t r[64];
    uint32_t n;
    while ((n = eskf_svc_trace_read(&svc, r, 64)) > 0) trace_put(ft, r, n);
}

/* ---- reproducción: la traza original en memoria y el contraste con la que genera la reproducción */
static eskf_svc_trec_t *g_orig;
static long g_norig, g_cmp; /* registros originales; siguiente registro original a contrastar */
static const char *trname(uint8_t t)
{
    static const char *N[] = {"?", "DELIVER", "CONSUME", "RESTART", "CONFIG", "OUTPUT", "BAROREF", "END", "PHASE"};
    return t < 9 ? N[t] : "?";
}
static void trfmt(char *b, size_t n, const eskf_svc_trec_t *r)
{
    if (r->type == ESKF_SVC_TR_OUTPUT)
        snprintf(b, n, "OUTPUT resultado %d tipo %u longitud %u secuencia %u timestampMs %" PRId64, (int)r->res, r->a, r->b,
                 r->c, r->t);
    else if (r->type == ESKF_SVC_TR_DELIVER)
        snprintf(b, n, "DELIVER retorno %d KPID 0x%04x secuencia %u longitud %u recepción %" PRId64 " us", (int)r->res, r->a,
                 r->b, r->c, r->t);
    else if (r->type == ESKF_SVC_TR_CONSUME)
        snprintf(b, n, "CONSUME paquetes %u llamadas nav %u construidos 0x%x reloj %" PRId64 " us", r->a, r->b, r->c, r->t);
    else snprintf(b, n, "%s res %d a %u b %u c %u t %" PRId64, trname(r->type), (int)r->res, r->a, r->b, r->c, r->t);
}
/* Contrasta lo que ha registrado el servicio reproducido con la traza original (hasta el registro
   original «upto», el último cuya operación se ha reproducido). 0 o código 4. */
static int contrast(long upto)
{
    eskf_svc_trec_t r[64];
    uint32_t n;
    while ((n = eskf_svc_trace_read(&svc, r, 64)) > 0) {
        for (uint32_t i = 0; i < n; i++, g_cmp++) {
            char a[192], b[192];
            trfmt(b, sizeof b, &r[i]);
            if (g_cmp > upto || g_cmp >= g_norig) {
                fprintf(stderr, "svc_replay: tras el registro %ld la reproducción generó «%s», que la traza no tiene ahí\n", upto, b);
                return 4;
            }
            const eskf_svc_trec_t *o = &g_orig[g_cmp];
            if (o->type != r[i].type || o->res != r[i].res || o->a != r[i].a || o->b != r[i].b || o->c != r[i].c || o->t != r[i].t) {
                trfmt(a, sizeof a, o);
                fprintf(stderr, "svc_replay: registro %ld (byte %ld): metadatos distintos de la operación reproducida\n"
                                "  traza:        %s\n  reproducción: %s\n", g_cmp, g_cmp * 16, a, b);
                return 4;
            }
        }
    }
    return 0;
}

static int read_pkt(FILE *f, uint8_t p[64])
{
    while (fread(p, 1, 64, f) == 64) {
        int zero = 1;
        for (int k = 0; k < 64; k++) zero &= p[k] == 0;
        if (!zero) return 1; /* el relleno de ceros no son paquetes */
    }
    return 0;
}

int main(int argc, char **argv)
{
    int profile = ESKF_SVC_PROFILE_EXP, summary = 0, every = 1, hist = 0, kp_set = 0, telem = 0, partial = 0;
    int partial_done = 0;
    unsigned boot = 0;
    long long snap_us = -1, ops_us = -1;
    unsigned ki = 0, kb = 0, ko = 0;
    long long lag = 0;
    const char *in = NULL, *p_out = NULL, *p_in = NULL, *p_msg = NULL, *p_tr = NULL, *p_rep = NULL;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--dump-md")) {
            size_t n;
            const eskf_param_t *T = eskf_svc_params_table(&n);
            printf("| parámetro | unidad | estricto | estado estricto | experimental | procedencia experimental | grupo | "
                   "fuente / cómo actualizarlo |\n");
            printf("|---|---|---|---|---|---|---|---|\n");
            for (size_t k = 0; k < n; k++) {
                char f1[32], f2[32];
                if (isnan(T[k].strict)) snprintf(f1, sizeof f1, "—");
                else snprintf(f1, sizeof f1, "%.10g", T[k].strict);
                snprintf(f2, sizeof f2, "%.10g", T[k].exp);
                printf("| `%s` | %s | %s | %s | %s | %s | %s%s | %s |\n", T[k].name, T[k].unit, f1, st(T[k].st_strict), f2,
                       st(T[k].st_exp), grp(T[k].group), T[k].need_conf ? " (confirmar)" : "", T[k].source);
            }
            return 0;
        } else if (!strcmp(a, "--dump-cfg") && i + 1 < argc) {
            eskf_svc_config_t c;
            eskf_svc_default_config(profile_of(argv[++i]), &c);
            dump_cfg(stdout, &c);
            return 0;
        } else if (!strcmp(a, "--perfil") && i + 1 < argc) profile = profile_of(argv[++i]);
        else if (!strcmp(a, "--historico")) hist = 1;
        else if (!strcmp(a, "--kpid") && i + 1 < argc) {
            if (sscanf(argv[++i], "%i,%i,%i", (int *)&ki, (int *)&kb, (int *)&ko) != 3) return 2;
            kp_set = 1;
        } else if (!strcmp(a, "--retardo-us") && i + 1 < argc) lag = atoll(argv[++i]);
        else if (!strcmp(a, "--consumir-cada") && i + 1 < argc) every = atoi(argv[++i]);
        else if (!strcmp(a, "--salidas") && i + 1 < argc) p_out = argv[++i];
        else if (!strcmp(a, "--entradas") && i + 1 < argc) p_in = argv[++i];
        else if (!strcmp(a, "--mensajes") && i + 1 < argc) p_msg = argv[++i];
        else if (!strcmp(a, "--traza-val") && i + 1 < argc) p_tr = argv[++i];
        else if (!strcmp(a, "--reproducir") && i + 1 < argc) p_rep = argv[++i];
        else if (!strcmp(a, "--parcial")) partial = 1;
        else if (!strcmp(a, "--resumen")) summary = 1;
        else if (!strcmp(a, "--telemetria")) telem = 1;
        else if (!strcmp(a, "--estado-us") && i + 1 < argc) snap_us = atoll(argv[++i]);
        else if (!strcmp(a, "--compacta-us") && i + 1 < argc) ops_us = atoll(argv[++i]);
        else if (!strcmp(a, "--arranque") && i + 1 < argc) boot = (unsigned)atoi(argv[++i]);
        else if (a[0] != '-' && !in) in = a;
        else {
            fprintf(stderr, "uso: ver la cabecera de svc_replay.c (%s)\n", a);
            return 2;
        }
    }
    if (!in || every < 1 || (partial && !p_rep)) {
        fprintf(stderr, "uso: ver la cabecera de svc_replay.c\n");
        return 2;
    }
    eskf_svc_config_t cfg;
    eskf_svc_default_config(profile, &cfg);
    if (hist) eskf_svc_config_historical(&cfg);
    if (!kp_set) ki = hist ? 0x8u : 0x4u, kb = hist ? 0x4u : 0x8u, ko = hist ? 0x40u : 0x10u;
    cfg.kpid_imu = (uint16_t)ki, cfg.kpid_baro = (uint16_t)kb, cfg.kpid_out = (uint16_t)ko;
    cfg.boot_id = (uint16_t)boot;
    if (telem) eskf_svc_output_preset(&cfg, ESKF_OUT_TELEMETRY);
    if (snap_us >= 0) cfg.snapshot_period_us = snap_us;
    if (ops_us >= 0) cfg.ops_period_us = ops_us;
    if (p_tr || p_rep) cfg.trace_enable = 1;
    int64_t t_init = INT64_MIN;
    if (p_rep) { /* la traza original entera, antes de iniciar (el arranque sale de su CONFIG) */
        FILE *f = fopen(p_rep, "rb");
        if (!f) {
            perror(p_rep);
            return 2;
        }
        long tsz = fsize(f);
        if (tsz % 16) {
            if (!partial) {
                fprintf(stderr, "svc_replay: traza TRUNCADA: %ld B no es múltiplo de 16; el registro %ld está incompleto (%ld B desde el byte %ld)\n",
                        tsz, tsz / 16, tsz % 16, tsz - tsz % 16);
                return 5;
            }
            fprintf(stderr, "svc_replay: [parcial] traza: se ignoran los %ld B finales (registro %ld incompleto)\n", tsz % 16,
                    tsz / 16);
        }
        g_norig = tsz / 16;
        g_orig = (eskf_svc_trec_t *)calloc((size_t)(g_norig > 0 ? g_norig : 1), sizeof *g_orig);
        if (!g_orig) return 2;
        for (long k = 0; k < g_norig; k++) (void)trace_get(f, &g_orig[k]);
        fclose(f);
        if (g_norig == 0 || g_orig[0].type != ESKF_SVC_TR_CONFIG) {
            fprintf(stderr, "svc_replay: la traza no empieza con un registro CONFIG\n");
            return 6;
        }
        if (g_orig[0].res != ESKF_SVC_TRACE_VERSION) {
            fprintf(stderr, "svc_replay: traza de versión %d; esta herramienta reproduce la versión %d, que se cierra con un registro "
                            "END (recuento y CRC) y cuyo CONFIG guarda el reloj del inicio (del que depende el contrato de rampa). "
                            "Una traza v1/v2 no prueba que esté completa, la v1 no guarda la cabecera de los paquetes de salida, la "
                            "v3 no permite reproducir la fase y la v4 no registra sus transiciones: regenerarla con el pegamento "
                            "actual\n",
                    (int)g_orig[0].res, ESKF_SVC_TRACE_VERSION);
            return 6;
        }
        cfg.boot_id = g_orig[0].c; /* el arranque registrado: forma parte de cada mensaje */
        t_init = g_orig[0].t;      /* y el reloj del inicio: contrato de rampa */
    }
    const char *bad = eskf_svc_check_transport(&cfg);
    if (bad) {
        fprintf(stderr, "configuración: %s\n", bad);
        return 3;
    }
    FILE *fi = fopen(in, "rb");
    if (!fi) {
        perror(in);
        return 2;
    }
    hook_ctx_t hc = {p_out ? fopen(p_out, "w") : NULL, p_in ? fopen(p_in, "w") : NULL, 0};
    FILE *fm = p_msg ? fopen(p_msg, "wb") : NULL, *ft = p_tr ? fopen(p_tr, "wb") : NULL;
    int fr = p_rep != NULL;
    if ((p_msg && !fm) || (p_tr && !ft)) {
        perror("archivo");
        return 2;
    }
    if (hc.in) {
        fprintf(hc.in, "# svc_replay: llamadas a navigation.c del servicio (perfil %d)\n", profile);
        dump_cfg(hc.in, &cfg);
    }
    if (t_init == INT64_MIN) { /* calendario propio: el servicio arranca con el primer paquete del registro */
        uint8_t q[64];
        t_init = 0;
        if (read_pkt(fi, q))
            t_init = ((int64_t)((uint32_t)q[7] | ((uint32_t)q[8] << 8) | ((uint32_t)q[9] << 16) | ((uint32_t)q[10] << 24))) * 1000 +
                     lag;
        rewind(fi);
    }
    g_now = t_init;
    if (eskf_svc_init(&svc, &cfg, clk, NULL) != ESKF_SVC_OK) return 3;
    if (hc.out || hc.in) eskf_svc_set_hook(&svc, hook, &hc);
    uint8_t p[64];
    long pk = 0, rej = 0, defer = 0, outs = 0;
    if (fr) { /* comportamiento real registrado por el pegamento */
        long isz = fsize(fi);
        if (isz % 64) {
            if (!partial) {
                fprintf(stderr, "svc_replay: entradas TRUNCADAS: %ld B no es múltiplo de 64 (paquete %ld incompleto)\n", isz, isz / 64);
                return 5;
            }
            fprintf(stderr, "svc_replay: [parcial] entradas: se ignoran los %ld B finales (paquete %ld incompleto)\n", isz % 64,
                    isz / 64);
        }
        long idx, n_end;
        /* comprobaciones de la captura ANTES de reproducir: un único END, al final, con recuento y CRC */
        for (long k = 0; k + 1 < g_norig; k++)
            if (g_orig[k].type == ESKF_SVC_TR_END) {
                fprintf(stderr, "svc_replay: registro %ld (byte %ld): END en mitad de la traza (hay %ld registros)\n", k, k * 16,
                        g_norig);
                return 4;
            }
        int closed = g_orig[g_norig - 1].type == ESKF_SVC_TR_END;
        if (closed) {
            const eskf_svc_trec_t *e = &g_orig[g_norig - 1];
            uint16_t crc = 0xFFFFu;
            for (long k = 0; k < g_norig - 1; k++) crc = eskf_svc_trec_crc(&g_orig[k], crc);
            if (e->t != (int64_t)(g_norig - 1)) {
                fprintf(stderr, "svc_replay: captura INCOMPLETA: el END declara %" PRId64 " registros anteriores y la traza tiene %ld "
                                "(faltan o sobran registros completos)\n", e->t, g_norig - 1);
                return 5;
            }
            if (e->b != 0) {
                fprintf(stderr, "svc_replay: captura INCOMPLETA: la placa perdió %u registros por desbordamiento de la traza\n", e->b);
                return 5;
            }
            if (e->a != crc) {
                fprintf(stderr, "svc_replay: CRC de la traza 0x%04x, el END declara 0x%04x: registros alterados\n", crc, e->a);
                return 4;
            }
            n_end = g_norig - 1;
        } else if (!partial) {
            fprintf(stderr, "svc_replay: traza SIN registro de cierre (END): %ld registros; no prueba que la captura esté completa "
                            "(placa apagada antes de cerrar, captura cortada o registros finales eliminados). Para reproducir lo "
                            "registrado como PARCIAL: --parcial\n", g_norig);
            return 5;
        } else {
            n_end = g_norig;
            fprintf(stderr, "svc_replay: [parcial] traza sin cierre: se reproducen %ld registros; el resultado NO es una "
                            "reproducción completa\n", g_norig);
        }
        if (contrast(0)) { /* el CONFIG propio, con el arranque de la traza */
            fprintf(stderr, "svc_replay: la traza es de OTRA configuración (config_id 0x%04x perfil %u esquema %u arranque %u; "
                            "aquí 0x%04x %d %u)\n", g_orig[0].a, g_orig[0].b & 0xFFu, g_orig[0].b >> 8, g_orig[0].c, svc.config_id,
                    (int)cfg.profile, (unsigned)ESKF_MSG_VERSION);
            return 6;
        }
        for (idx = 1; idx < n_end; idx++) {
            eskf_svc_trec_t r = g_orig[idx];
            long off = idx * 16;
            if (r.type == ESKF_SVC_TR_DELIVER) {
                g_now = r.t;
                if (!read_pkt(fi, p)) {
                    fprintf(stderr, "svc_replay: registro %ld (byte %ld): DELIVER apid 0x%04x seq %u sin paquete en las entradas (agotadas tras %ld)\n",
                            idx, off, r.a, r.b, pk);
                    return 4;
                }
                uint16_t ap = (uint16_t)(p[1] | (p[2] << 8)), sq = (uint16_t)(p[3] | (p[4] << 8));
                if (r.a != ap || r.b != sq) {
                    fprintf(stderr, "svc_replay: registro %ld (byte %ld): DELIVER apid 0x%04x seq %u, pero la entrada %ld es apid 0x%04x seq %u\n",
                            idx, off, r.a, r.b, pk, ap, sq);
                    return 4;
                }
                if (eskf_svc_deliver_packet(&svc, p) != ESKF_SVC_OK) rej++;
                pk++;
            } else if (r.type == ESKF_SVC_TR_RESTART) {
                g_now = r.t;
                if (r.b == 0) eskf_svc_request_restart(&svc); /* pedido (aceptado o rechazado: lo decide el servicio) */
            } else if (r.type == ESKF_SVC_TR_BAROREF) {
                g_now = r.t;
                eskf_svc_notify_baro_ref_change(&svc);
            } else if (r.type == ESKF_SVC_TR_CONSUME) {
                g_now = r.t;
                eskf_svc_consume(&svc);
            } else if (r.type == ESKF_SVC_TR_PHASE) {
                /* la transición la genera el propio consume siguiente: se contrasta allí */
            } else if (r.type == ESKF_SVC_TR_OUTPUT) {
                if (r.res == ESKF_SVC_OUT_PUBLISHED || r.res == ESKF_SVC_OUT_PUB_FAIL || r.res == ESKF_SVC_OUT_PACKET_FAIL) {
                    if (r.c != g_oseq) {
                        fprintf(stderr, "svc_replay: registro %ld (byte %ld): secuencia de salida %u en la traza, %u en la reproducción\n",
                                idx, off, r.c, g_oseq);
                        return 4;
                    }
                    if (!emit_one(fm, r.res, r.c, (uint32_t)r.t)) { /* producido; publicado o perdido en el router */
                        fprintf(stderr, "svc_replay: registro %ld (byte %ld): OUTPUT tipo %u sin mensaje pendiente en la reproducción\n",
                                idx, off, r.a);
                        return 4;
                    }
                    outs++;
                } else if (r.res == ESKF_SVC_OUT_DEFER_QUEUE || r.res == ESKF_SVC_OUT_DEFER_POOL) {
                    if (!eskf_svc_pending(&svc)) { /* el pegamento sólo aplaza con un mensaje pendiente */
                        fprintf(stderr, "svc_replay: registro %ld (byte %ld): aplazamiento sin mensaje pendiente en la reproducción\n",
                                idx, off);
                        return 4;
                    }
                    defer++; /* aplazado: el mensaje sigue pendiente */
                    eskf_svc_note_output(&svc, r.res, 0, 0, 0, 0);
                } else {
                    fprintf(stderr, "svc_replay: registro %ld (byte %ld): resultado de salida desconocido %d\n", idx, off, (int)r.res);
                    return 4;
                }
            } else {
                fprintf(stderr, "svc_replay: registro %ld (byte %ld): tipo desconocido %u\n", idx, off, r.type);
                return 4;
            }
            int e = contrast(idx);
            if (e) return e;
        }
        long extra = 0;
        while (read_pkt(fi, p)) extra++;
        if (closed) {
            if (g_cmp != n_end) { /* registros sin operación reproducida que los genere */
                char a[192];
                trfmt(a, sizeof a, &g_orig[g_cmp]);
                fprintf(stderr, "svc_replay: registro %ld (byte %ld): «%s» no lo genera ninguna operación reproducida\n", g_cmp,
                        g_cmp * 16, a);
                return 4;
            }
            if (eskf_svc_trace_close(&svc) != ESKF_SVC_OK) return 4;
            int e = contrast(n_end); /* el END regenerado: mismo recuento y mismo CRC */
            if (e) return e;
            if (extra) {
                fprintf(stderr, "svc_replay: quedan %ld paquetes de entrada sin registro DELIVER en una traza cerrada de %ld registros\n",
                        extra, g_norig);
                return 4;
            }
            fprintf(stderr, "svc_replay: reproducción COMPLETA: traza v%d cerrada (%ld registros, CRC 0x%04x) contrastada entera; "
                            "%ld entradas, %ld salidas, %ld aplazamientos\n",
                    ESKF_SVC_TRACE_VERSION, g_norig, g_orig[g_norig - 1].a, pk, outs, defer);
        } else {
            partial_done = 1;
            fprintf(stderr, "svc_replay: REPRODUCCIÓN PARCIAL (no completa): %ld registros contrastados de una traza SIN cierre, "
                            "%ld entradas, %ld salidas; %ld registros finales sin operación que los regenere; %ld paquetes de "
                            "entrada posteriores sin DELIVER (no reproducidos)\n",
                    g_cmp, pk, outs, n_end - g_cmp, extra);
        }
    } else {
        while (read_pkt(fi, p)) {
            uint32_t ts = (uint32_t)p[7] | ((uint32_t)p[8] << 8) | ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);
            g_now = (int64_t)ts * 1000 + lag;
            if (eskf_svc_deliver_packet(&svc, p) != ESKF_SVC_OK) rej++;
            pk++;
            if (pk % every == 0) {
                eskf_svc_consume(&svc);
                (void)EMIT_NOW(fm); /* como acquireData: como mucho uno por vuelta */
                drain_trace(ft);
            }
        }
        for (int k = 0; k < 50; k++) { /* sin entradas: vaciado y caducidad */
            g_now += 10000;
            eskf_svc_consume(&svc);
            (void)EMIT_NOW(fm);
            drain_trace(ft);
        }
        while (EMIT_NOW(fm)) {
        }
        drain_trace(ft);
        if (ft && eskf_svc_trace_close(&svc) == ESKF_SVC_OK) drain_trace(ft); /* cierre verificable (END) */
    }
    fclose(fi);
    if (hc.out) fclose(hc.out);
    if (hc.in) fclose(hc.in);
    if (fm) fclose(fm);
    if (ft) fclose(ft);
    free(g_orig);
    if (summary) {
        eskf_svc_output_t o;
        eskf_svc_diag_t d;
        eskf_svc_get_output(&svc, &o);
        eskf_svc_get_diag(&svc, &d);
        printf("perfil=%d navega=%d motivo=%s why_code=%u readiness=0x%04x config_id=0x%04x\n", profile, svc.nav_enabled,
               svc.nav_enabled ? "-" : svc.why, o.why_code, o.readiness, o.config_id);
        printf("paquetes=%ld rechazados=%ld mensajes=%ld sizeof_servicio=%zu sesion=%u\n", pk, rej, g_msgs, sizeof svc,
               o.session);
        printf("delivered=%u mailbox_full=%u rejected_apid=%u rejected_len=%u rejected_crc=%u mailbox_hwm=%u\n", d.delivered,
               d.mailbox_full, d.rejected_apid, d.rejected_len, d.rejected_crc, d.mailbox_hwm);
        printf("imu_rx=%u baro_rx=%u imu_bad_len=%u imu_nonfinite=%u imu_out_of_range=%u reserved_nonzero=%u dmp_invalid=%u\n",
               d.imu_rx, d.baro_rx, d.imu_bad_len, d.imu_nonfinite, d.imu_out_of_range, d.imu_reserved_nonzero, d.dmp_invalid);
        printf("seq_lost_imu=%u seq_lost_baro=%u seq_dup=%u seq_old=%u seq_resync=%u ts_future=%u late_imu=%u late_baro=%u\n",
               d.seq_lost_imu, d.seq_lost_baro, d.seq_dup, d.seq_old, d.seq_resync, d.ts_future, d.late_imu, d.late_baro);
        printf("sat_cand_acc=%u sat_cand_gyro=%u sat_hw_confirmed=%u sat_out_of_model=%u clock_mismatch=%u lat_rx_max_us=%" PRId64
               " loss_recovered=%u\n",
               d.sat_cand_acc, d.sat_cand_gyro, d.sat_hw_confirmed, d.sat_out_of_model, d.clock_mismatch, d.lat_rx_max_us,
               d.loss_recovered);
        printf("nav_calls=%u nav_backlog_max=%u merge_full_waits=%u snapshots=%u snapshots_skipped=%u diag_msgs=%u ops_msgs=%u\n",
               d.nav_calls, d.nav_backlog_max, d.merge_full_waits, d.snapshots, d.snapshots_skipped, d.diag_msgs, d.ops_msgs);
        printf("bb: usados=%u descartados=%u reinicios=%u huecos=%u valido=%u congelada=%u h=%.3f vz=%.3f ref=%.3f\n", d.bb_used,
               d.bb_rejected, d.bb_reseeds, d.bb_gaps, o.bb.valid, o.bb.ref_frozen, (double)o.bb.h_m, (double)o.bb.vz_mps,
               (double)o.bb.ref_m);
        printf("nav_validez=%u svc_validez=%u motivos_nav=0x%04x motivos_svc=0x%04x caps=0x%02x fuente=%u fase=%u t_state_us=%" PRId64
               " accuracy=%g\n",
               o.nav_validity, o.svc_validity, o.nav.reasons, o.svc_reasons, o.caps, o.source, o.nav.phase, o.t_state_us,
               (double)d.dmp_accuracy_last);
        printf("publicados=%ld perdidos=%ld aplazados=%ld\n", g_msgs - g_lost, g_lost, defer);
        printf("vuelo=%d detector_vuelo=%u nav_cortado=%d muestras_retenidas=%u\n", svc.flight, d.flight_detector, svc.nav_cut,
               d.nav_cut_samples);
        if (svc.nav_enabled) {
            const nav_counters_t *c = &svc.nav.cnt;
            printf("nav: imu_rx=%u imu_applied=%u gaps=%u rest_breaks_gap=%u rest_windows_ok=%u align_ok=%u baro_used=%u "
                   "time_future=%u imu_acc_sat=%u ref_overflow=%u\n",
                   c->imu_rx, c->imu_applied, c->gaps, c->rest_breaks_gap, c->rest_windows_ok, c->align_ok, c->baro_used,
                   c->time_future, c->imu_acc_sat, c->ref_overflow);
        }
    }
    return partial_done ? 3 : 0;
}
