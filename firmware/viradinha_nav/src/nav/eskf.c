/*
 * eskf.c — Error-State Kalman Filter, 15 estados. Ver eskf.h para convenciones
 * y para el contrato de errores.
 *
 * Referencia: eskf.py (repositorio solaris-eskf). Cada función indica su
 * equivalente en Python y la ecuación de docs/ESKF.pdf.
 */
#include "eskf.h"

#include <math.h>
#include <stddef.h>

#if defined(__FAST_MATH__) || defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "eskf.c necesita isfinite(): no compilar con -ffast-math ni -ffinite-math-only"
#endif

#ifdef ESKF_USE_FLOAT
#define R_SQRT sqrtf
#define R_SIN sinf
#define R_COS cosf
#define R_ATAN2 atan2f
#define R_ASIN asinf
#define R_FABS fabsf
#define R_CEIL ceilf
#define RC(x) (x##f)
#else
#define R_SQRT sqrt
#define R_SIN sin
#define R_COS cos
#define R_ATAN2 atan2
#define R_ASIN asin
#define R_FABS fabs
#define R_CEIL ceil
#define RC(x) (x)
#endif

#define N ESKF_N
#define ESKF_MAGIC 0x45534B46u /* "ESKF" */
#define G0 RC(9.81)

/* ------------------------------------------------------------------------- */
/* Validación                                                                 */
/* ------------------------------------------------------------------------- */

static int fin(eskf_real x) { return isfinite(x) ? 1 : 0; }

static int fin3(const eskf_real v[3]) { return fin(v[0]) && fin(v[1]) && fin(v[2]); }

/* Desviación típica de MEDIDA: finita y > 0, y su cuadrado también (sin underflow/overflow). */
static int meas_var(eskf_real sigma, eskf_real *var)
{
    if (!fin(sigma) || !(sigma > RC(0.0))) return 0;
    eskf_real v = sigma * sigma;
    if (!fin(v) || !(v > RC(0.0))) return 0;
    *var = v;
    return 1;
}

/* Desviación típica de PROCESO o inicial: finita y >= 0 (0 se admite en ensayos deterministas). */
static int proc_std_ok(eskf_real s) { return fin(s) && s >= RC(0.0) && fin(s * s); }

/* ------------------------------------------------------------------------- */
/* Álgebra pequeña                                                            */
/* ------------------------------------------------------------------------- */

static eskf_real norm3(const eskf_real v[3])
{
    return R_SQRT(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void cross3(const eskf_real a[3], const eskf_real b[3], eskf_real out[3])
{
    eskf_real t0 = a[1] * b[2] - a[2] * b[1];
    eskf_real t1 = a[2] * b[0] - a[0] * b[2];
    eskf_real t2 = a[0] * b[1] - a[1] * b[0];
    out[0] = t0;
    out[1] = t1;
    out[2] = t2;
}

/* [x]× (eq. 33-34) */
static void skew(const eskf_real x[3], eskf_real S[3][3])
{
    S[0][0] = RC(0.0);  S[0][1] = -x[2];    S[0][2] = x[1];
    S[1][0] = x[2];     S[1][1] = RC(0.0);  S[1][2] = -x[0];
    S[2][0] = -x[1];    S[2][1] = x[0];     S[2][2] = RC(0.0);
}

/* out = Rᵀ v  (R sin const: en C99 T[3][3] -> const T[3][3] no es implícito) */
static void mat3T_vec(eskf_real R[3][3], const eskf_real v[3], eskf_real out[3])
{
    for (int i = 0; i < 3; i++)
        out[i] = R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2];
}

/* out = R v */
static void mat3_vec(eskf_real R[3][3], const eskf_real v[3], eskf_real out[3])
{
    for (int i = 0; i < 3; i++)
        out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
}

/* ------------------------------------------------------------------------- */
/* Cuaterniones (quaternion.py)                                               */
/* ------------------------------------------------------------------------- */

/* q1 ⊗ q2, Hamilton (sec. 2.2). Admite out == q1 o out == q2. */
static void quat_product(const eskf_real q1[4], const eskf_real q2[4], eskf_real out[4])
{
    eskf_real w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    eskf_real x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    eskf_real y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    eskf_real z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

/* Normaliza q. Devuelve 1 si lo consigue; si la norma no es finita o es demasiado
   pequeña (q no representa una rotación) devuelve 0 y NO modifica q: el llamador
   decide (rechazar o marcar fallo). Antes se sustituía en silencio por la identidad. */
static int quat_normalize(eskf_real q[4])
{
    eskf_real n = R_SQRT(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!fin(n) || !(n > RC(1e-6))) return 0;
    eskf_real inv = RC(1.0) / n;
    for (int i = 0; i < 4; i++) q[i] *= inv;
    return 1;
}

/* q{θ} (sec. 2.1). Por debajo de 1e-8 rad usa [1, θ/2] normalizado, como Python.
   rv debe ser finito; el resultado es siempre unitario. */
static void quat_from_rotvec(const eskf_real rv[3], eskf_real q[4])
{
    eskf_real th = norm3(rv);
    if (th < RC(1e-8)) {
        q[0] = RC(1.0);
        q[1] = rv[0] * RC(0.5);
        q[2] = rv[1] * RC(0.5);
        q[3] = rv[2] * RC(0.5);
        (void)quat_normalize(q); /* norma ∈ [1, 1+1e-16]: no puede fallar */
        return;
    }
    eskf_real s = R_SIN(th * RC(0.5)) / th;
    q[0] = R_COS(th * RC(0.5));
    q[1] = rv[0] * s;
    q[2] = rv[1] * s;
    q[3] = rv[2] * s;
}

/* R(q), cuerpo -> NED (sec. 2.3) */
void eskf_rotation_matrix(const eskf_real q[4], eskf_real R[3][3])
{
    eskf_real w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = RC(1.0) - RC(2.0) * (y * y + z * z);
    R[0][1] = RC(2.0) * (x * y - w * z);
    R[0][2] = RC(2.0) * (x * z + w * y);
    R[1][0] = RC(2.0) * (x * y + w * z);
    R[1][1] = RC(1.0) - RC(2.0) * (x * x + z * z);
    R[1][2] = RC(2.0) * (y * z - w * x);
    R[2][0] = RC(2.0) * (x * z - w * y);
    R[2][1] = RC(2.0) * (y * z + w * x);
    R[2][2] = RC(1.0) - RC(2.0) * (x * x + y * y);
}

/* Matriz de rotación -> cuaternión (Shepperd), w >= 0. Devuelve 0 si no es válido. */
static int quat_from_rotation_matrix(eskf_real R[3][3], eskf_real q[4])
{
    eskf_real tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > R[0][0] && tr > R[1][1] && tr > R[2][2]) {
        eskf_real s = R_SQRT(tr + RC(1.0)) * RC(2.0); /* s = 4w */
        q[0] = RC(0.25) * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        eskf_real s = R_SQRT(RC(1.0) + R[0][0] - R[1][1] - R[2][2]) * RC(2.0); /* 4x */
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = RC(0.25) * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        eskf_real s = R_SQRT(RC(1.0) + R[1][1] - R[0][0] - R[2][2]) * RC(2.0); /* 4y */
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = RC(0.25) * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        eskf_real s = R_SQRT(RC(1.0) + R[2][2] - R[0][0] - R[1][1]) * RC(2.0); /* 4z */
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = RC(0.25) * s;
    }
    if (q[0] < RC(0.0))
        for (int i = 0; i < 4; i++) q[i] = -q[i];
    return quat_normalize(q);
}

void eskf_quat_to_euler(const eskf_real q[4], eskf_real rpy[3])
{
    eskf_real w = q[0], x = q[1], y = q[2], z = q[3];
    eskf_real sp = RC(2.0) * (w * y - z * x);
    if (sp > RC(1.0)) sp = RC(1.0);
    if (sp < RC(-1.0)) sp = RC(-1.0);
    rpy[0] = R_ATAN2(RC(2.0) * (w * x + y * z), RC(1.0) - RC(2.0) * (x * x + y * y));
    rpy[1] = R_ASIN(sp);
    rpy[2] = R_ATAN2(RC(2.0) * (w * z + x * y), RC(1.0) - RC(2.0) * (y * y + z * z));
}

/* ------------------------------------------------------------------------- */
/* Estado lógico y diagnóstico                                                */
/* ------------------------------------------------------------------------- */

const char *eskf_status_str(int status)
{
    if (status > 0) return "OK (componentes aplicadas)";
    switch (status) {
    case ESKF_OK: return "OK";
    case ESKF_E_NULL: return "puntero nulo";
    case ESKF_E_NOT_INIT: return "filtro sin inicializar";
    case ESKF_E_CONFIG: return "configuración inválida o float/double incoherente";
    case ESKF_E_INPUT: return "medida o sigma inválida";
    case ESKF_E_DT: return "dt no finito o <= 0";
    case ESKF_E_DT_GAP: return "hueco temporal no puenteable";
    case ESKF_E_NOT_AT_REST: return "reposo no confirmado por el llamador";
    case ESKF_E_MOTION: return "la medida contradice el reposo";
    case ESKF_E_NO_REF: return "falta referencia magnética";
    case ESKF_E_NO_HEADING: return "marco horizontal sin rumbo conocido";
    case ESKF_E_GATED: return "rechazada por la puerta de innovación";
    case ESKF_E_DEGENERATE: return "geometría degenerada";
    case ESKF_E_NUMERIC: return "fallo numérico: filtro en fallo";
    case ESKF_E_FAULT: return "filtro en fallo";
    default: return "código desconocido";
    }
}

/* Comprobaciones comunes a las operaciones que modifican la estimación */
static int guard(const eskf_t *f)
{
    if (f == NULL) return ESKF_E_NULL;
    if (f->magic != ESKF_MAGIC) return ESKF_E_NOT_INIT;
    if (f->fault) return ESKF_E_FAULT;
    return ESKF_OK;
}

static int latch_fault(eskf_t *f)
{
    f->fault = 1;
    return ESKF_E_NUMERIC;
}

int eskf_check_health(const eskf_t *f)
{
    if (f == NULL) return ESKF_E_NULL;
    if (f->magic != ESKF_MAGIC) return ESKF_E_NOT_INIT;
    if (f->fault) return ESKF_E_FAULT; /* antes devolvía OK si los campos seguían finitos */
    if (!fin3(f->p) || !fin3(f->v) || !fin3(f->b_a) || !fin3(f->b_g)) return ESKF_E_NUMERIC;
    for (int i = 0; i < 4; i++)
        if (!fin(f->q[i])) return ESKF_E_NUMERIC;
    eskf_real qn = R_SQRT(f->q[0] * f->q[0] + f->q[1] * f->q[1] + f->q[2] * f->q[2] + f->q[3] * f->q[3]);
    if (!(R_FABS(qn - RC(1.0)) < RC(1e-3))) return ESKF_E_NUMERIC;
    for (int i = 0; i < N; i++) {
        if (!fin(f->P[i][i]) || f->P[i][i] < RC(0.0)) return ESKF_E_NUMERIC;
        for (int j = i + 1; j < N; j++)
            if (!fin(f->P[i][j]) || f->P[i][j] != f->P[j][i]) return ESKF_E_NUMERIC;
    }
    return ESKF_OK;
}

/* ------------------------------------------------------------------------- */
/* Inicialización                                                             */
/* ------------------------------------------------------------------------- */

int eskf_default_params_checked(eskf_params_t *prm, size_t sizeof_params)
{
    if (sizeof_params != sizeof(eskf_params_t)) return ESKF_E_CONFIG;
    if (prm == NULL) return ESKF_E_NULL;
    prm->sigma_a = RC(0.05);
    prm->sigma_w = RC(0.005);
    prm->sigma_aw = RC(1e-4);
    prm->sigma_ww = RC(1e-5);
    prm->mag_ref[0] = prm->mag_ref[1] = prm->mag_ref[2] = RC(0.0);
    prm->has_mag_ref = 0;
    prm->accel_threshold = RC(0.5);
    prm->innov_gate = RC(0.0);
    prm->dt_max = RC(0.0);  /* sin valor por defecto: depende de la IMU real */
    prm->gap_max = RC(0.0); /* no se puentean huecos salvo que se configure */
    prm->tau_max = RC(0.0); /* 0: el límite de desfase es dt_max */
    prm->align_acc_tol = RC(0.5);
    prm->align_gyro_max = RC(0.2);
    prm->p0_std = RC(1.0);
    prm->v0_std = RC(0.1);
    prm->th0_std = RC(0.05);
    prm->ba0_std = RC(0.2);
    prm->bg0_std = RC(0.01);
    return ESKF_OK;
}

int eskf_validate_params(const eskf_params_t *prm)
{
    if (prm == NULL) return ESKF_E_NULL;
    if (!proc_std_ok(prm->sigma_a) || !proc_std_ok(prm->sigma_w) || !proc_std_ok(prm->sigma_aw) ||
        !proc_std_ok(prm->sigma_ww))
        return ESKF_E_CONFIG;
    if (prm->has_mag_ref != 0 && prm->has_mag_ref != 1) return ESKF_E_CONFIG;
    if (prm->has_mag_ref) {
        if (!fin3(prm->mag_ref)) return ESKF_E_CONFIG;
        eskf_real mn = norm3(prm->mag_ref);
        if (!fin(mn) || !(mn > RC(0.0))) return ESKF_E_CONFIG;
    }
    if (!fin(prm->accel_threshold) || !(prm->accel_threshold > RC(0.0))) return ESKF_E_CONFIG;
    if (!fin(prm->innov_gate) || prm->innov_gate < RC(0.0)) return ESKF_E_CONFIG;
    if (!fin(prm->dt_max) || !(prm->dt_max > RC(0.0))) return ESKF_E_CONFIG;
    if (!fin(prm->gap_max) || prm->gap_max < RC(0.0)) return ESKF_E_CONFIG;
    if (prm->gap_max > RC(0.0)) {
        if (!(prm->gap_max > prm->dt_max)) return ESKF_E_CONFIG;
        if (prm->gap_max > prm->dt_max * (eskf_real)ESKF_MAX_BRIDGE_STEPS) return ESKF_E_CONFIG;
    }
    if (!fin(prm->tau_max) || prm->tau_max < RC(0.0)) return ESKF_E_CONFIG;
    if (!fin(prm->align_acc_tol) || !(prm->align_acc_tol > RC(0.0))) return ESKF_E_CONFIG;
    if (!fin(prm->align_gyro_max) || !(prm->align_gyro_max > RC(0.0))) return ESKF_E_CONFIG;
    if (!proc_std_ok(prm->p0_std) || !proc_std_ok(prm->v0_std) || !proc_std_ok(prm->th0_std) ||
        !proc_std_ok(prm->ba0_std) || !proc_std_ok(prm->bg0_std))
        return ESKF_E_CONFIG;
    return ESKF_OK;
}

static void set_diag_block(eskf_real P[N][N], int idx, eskf_real std)
{
    for (int i = 0; i < 3; i++) P[idx + i][idx + i] = std * std;
}

static void reset_covariance(eskf_t *f)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) f->P[i][j] = RC(0.0);
    set_diag_block(f->P, ESKF_IDX_P, f->prm.p0_std);
    set_diag_block(f->P, ESKF_IDX_V, f->prm.v0_std);
    set_diag_block(f->P, ESKF_IDX_TH, f->prm.th0_std);
    set_diag_block(f->P, ESKF_IDX_BA, f->prm.ba0_std);
    set_diag_block(f->P, ESKF_IDX_BG, f->prm.bg0_std);
}

static void clear_diag(eskf_t *f, int sensor)
{
    f->last.sensor = sensor;
    f->last.status = ESKF_OK;
    f->last.n_used = 0;
    f->last.n_gated = 0;
    f->last.n_numeric = 0;
    f->last.nis = RC(0.0);
}

int eskf_init_checked(eskf_t *f, const eskf_params_t *prm, size_t sizeof_filter, size_t sizeof_real)
{
    if (sizeof_filter != sizeof(eskf_t) || sizeof_real != sizeof(eskf_real)) return ESKF_E_CONFIG;
    if (f == NULL || prm == NULL) return ESKF_E_NULL;
    int st = eskf_validate_params(prm);
    if (st != ESKF_OK) {
        f->magic = 0; /* no dejar un filtro "inicializado" con parámetros inválidos */
        return st;
    }

    f->prm = *prm;
    for (int i = 0; i < 3; i++) {
        f->p[i] = f->v[i] = f->b_a[i] = f->b_g[i] = RC(0.0);
    }
    f->q[0] = RC(1.0);
    f->q[1] = f->q[2] = f->q[3] = RC(0.0);
    f->g[0] = RC(0.0);
    f->g[1] = RC(0.0);
    f->g[2] = -G0;
    reset_covariance(f);

    f->fault = 0;
    f->at_rest = 0;
    f->heading_known = 0;
    f->degraded = 0u;
    clear_diag(f, ESKF_SENSOR_NONE);
    f->cnt.n_predict = 0u;
    f->cnt.n_dt_rejected = 0u;
    f->cnt.n_gaps_bridged = 0u;
    f->magic = ESKF_MAGIC;
    return ESKF_OK;
}

int eskf_set_rest(eskf_t *f, int at_rest)
{
    if (f == NULL) return ESKF_E_NULL;
    if (f->magic != ESKF_MAGIC) return ESKF_E_NOT_INIT;
    f->at_rest = at_rest ? 1 : 0;
    return ESKF_OK;
}

int eskf_align_static(eskf_t *f, const eskf_real acc_mean[3], const eskf_real gyro_mean[3],
                      const eskf_real *mag_mean)
{
    if (f == NULL || acc_mean == NULL || gyro_mean == NULL) return ESKF_E_NULL;
    if (f->magic != ESKF_MAGIC) return ESKF_E_NOT_INIT;
    /* Se admite en fallo: la alineación es el camino de recuperación en rampa. */
    if (!f->at_rest) return ESKF_E_NOT_AT_REST;
    if (!fin3(acc_mean) || !fin3(gyro_mean)) return ESKF_E_INPUT;
    if (mag_mean != NULL && !fin3(mag_mean)) return ESKF_E_INPUT;

    /* Comprobaciones del VECTOR MEDIO. No demuestran reposo durante la ventana: eso lo
       hace el llamador antes de eskf_set_rest(f, 1). */
    eskf_real an = norm3(acc_mean);
    if (!fin(an) || !(R_FABS(an - G0) < f->prm.align_acc_tol)) return ESKF_E_MOTION;
    eskf_real wn = norm3(gyro_mean);
    if (!fin(wn) || wn > f->prm.align_gyro_max) return ESKF_E_MOTION;
    if (mag_mean != NULL && !f->prm.has_mag_ref) return ESKF_E_NO_REF;

    /* En reposo el acelerómetro apunta hacia arriba en ejes cuerpo: d = -a/‖a‖ */
    eskf_real d[3] = {-acc_mean[0] / an, -acc_mean[1] / an, -acc_mean[2] / an};

    eskf_real e[3];
    if (mag_mean != NULL) {
        eskf_real mn = norm3(mag_mean);
        if (!fin(mn) || !(mn > RC(0.0))) return ESKF_E_INPUT;
        eskf_real mu[3] = {mag_mean[0] / mn, mag_mean[1] / mn, mag_mean[2] / mn};
        cross3(d, mu, e); /* ‖e‖ = sen(ángulo entre la vertical y el campo) */
        if (!(norm3(e) > RC(1e-3))) return ESKF_E_DEGENERATE;
    } else {
        /* Marco local: N = proyección horizontal del eje de cuerpo más horizontal
           (|d_i| mínimo; a igualdad, el de menor índice). ‖e‖ >= sqrt(2/3). */
        int k = 0;
        eskf_real best = R_FABS(d[0]);
        for (int i = 1; i < 3; i++) {
            if (R_FABS(d[i]) < best) {
                best = R_FABS(d[i]);
                k = i;
            }
        }
        eskf_real ax[3] = {RC(0.0), RC(0.0), RC(0.0)};
        ax[k] = RC(1.0);
        cross3(d, ax, e);
    }
    eskf_real en = norm3(e);
    for (int i = 0; i < 3; i++) e[i] /= en;

    eskf_real n[3];
    cross3(e, d, n); /* e × d = n */

    /* R (cuerpo -> NED magnético o local) tiene por filas n, e, d expresados en cuerpo */
    eskf_real R[3][3];
    for (int i = 0; i < 3; i++) {
        R[0][i] = n[i];
        R[1][i] = e[i];
        R[2][i] = d[i];
    }

    /* Con magnetómetro, n apunta al norte magnético. Se gira la declinación
       D = atan2(m_E, m_N) de mag_ref para pasar al NED geográfico, coherente con la
       actualización de magnetómetro. */
    if (mag_mean != NULL) {
        eskf_real D = R_ATAN2(f->prm.mag_ref[1], f->prm.mag_ref[0]);
        eskf_real c = R_COS(D), s = R_SIN(D);
        for (int i = 0; i < 3; i++) {
            eskf_real r0 = R[0][i], r1 = R[1][i];
            R[0][i] = c * r0 - s * r1;
            R[1][i] = s * r0 + c * r1;
        }
    }
    eskf_real q[4];
    if (!quat_from_rotation_matrix(R, q)) return ESKF_E_NUMERIC; /* nada modificado */

    /* Éxito: reinicio completo de la estimación. Las correlaciones anteriores se
       descartan porque la nueva estimación procede de medidas nuevas en reposo. */
    for (int i = 0; i < 3; i++) {
        f->p[i] = RC(0.0);
        f->v[i] = RC(0.0);
        f->b_a[i] = RC(0.0);
        f->b_g[i] = gyro_mean[i];
    }
    for (int i = 0; i < 4; i++) f->q[i] = q[i];
    reset_covariance(f);
    f->heading_known = (mag_mean != NULL) ? 1 : 0;
    f->fault = 0;
    f->degraded = 0u; /* los huecos anteriores no afectan a la nueva estimación */
    return ESKF_OK;
}

/* ------------------------------------------------------------------------- */
/* Predicción (ESKF._predict, eq. 21-27, 36, 39)                              */
/* ------------------------------------------------------------------------- */

static void symmetrize(eskf_real P[N][N])
{
    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++) {
            eskf_real m = RC(0.5) * (P[i][j] + P[j][i]);
            P[i][j] = m;
            P[j][i] = m;
        }
}

/* Un paso de predicción con entradas ya validadas. Devuelve 0 si ‖q‖ no es normalizable.
   white_scale multiplica el ruido BLANCO de acelerómetro y giróscopo (ver eskf_predict):
   1 en un paso normal, nsub en los subpasos de un puente. */
static int predict_step(eskf_t *f, const eskf_real a_meas[3], const eskf_real w_meas[3], eskf_real dt,
                        eskf_real white_scale)
{
    eskf_real R[3][3];
    eskf_rotation_matrix(f->q, R);

    eskf_real a[3], th[3];
    for (int i = 0; i < 3; i++) {
        a[i] = a_meas[i] - f->b_a[i];
        th[i] = (w_meas[i] - f->b_g[i]) * dt;
    }

    eskf_real dq[4];
    quat_from_rotvec(th, dq);
    eskf_real Rdq[3][3];
    eskf_rotation_matrix(dq, Rdq);

    /* ---- Matriz de transición Φ (sec. 3.5.2), evaluada antes del paso ---- */
    eskf_real Phi[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) Phi[i][j] = (i == j) ? RC(1.0) : RC(0.0);

    eskf_real Sa[3][3];
    skew(a, Sa);
    for (int i = 0; i < 3; i++) {
        Phi[ESKF_IDX_P + i][ESKF_IDX_V + i] = dt;
        Phi[ESKF_IDX_TH + i][ESKF_IDX_BG + i] = -dt;
        for (int j = 0; j < 3; j++) {
            eskf_real RSa = R[i][0] * Sa[0][j] + R[i][1] * Sa[1][j] + R[i][2] * Sa[2][j];
            Phi[ESKF_IDX_V + i][ESKF_IDX_TH + j] = -RSa * dt;
            Phi[ESKF_IDX_V + i][ESKF_IDX_BA + j] = -R[i][j] * dt;
            Phi[ESKF_IDX_TH + i][ESKF_IDX_TH + j] = Rdq[j][i]; /* R{θ}ᵀ */
        }
    }

    /* ---- Estado nominal (Euler + retención de orden cero; aproximación deliberada,
       igual que eskf.py) ---- */
    eskf_real qn[4];
    quat_product(f->q, dq, qn);
    if (!quat_normalize(qn)) return 0;

    eskf_real Ra[3];
    mat3_vec(R, a, Ra);
    for (int i = 0; i < 3; i++) {
        f->p[i] += f->v[i] * dt;
        f->v[i] += (Ra[i] - f->g[i]) * dt;
    }
    for (int i = 0; i < 4; i++) f->q[i] = qn[i];

    /* ---- Covarianza: P = Φ P Φᵀ + F_i Q_i F_iᵀ ----
       Φ es muy dispersa (~45 no nulos de 225), así que se saltan los ceros. */
    eskf_real A[N][N]; /* A = Φ P */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) A[i][j] = RC(0.0);
        for (int k = 0; k < N; k++) {
            eskf_real phik = Phi[i][k];
            if (phik == RC(0.0)) continue;
            for (int j = 0; j < N; j++) A[i][j] += phik * f->P[k][j];
        }
    }
    for (int i = 0; i < N; i++) { /* P = A Φᵀ */
        for (int j = 0; j < N; j++) {
            eskf_real s = RC(0.0);
            for (int k = 0; k < N; k++) {
                eskf_real phjk = Phi[j][k];
                if (phjk != RC(0.0)) s += A[i][k] * phjk;
            }
            f->P[i][j] = s;
        }
    }

    /* Q_i (eq. 36): ruido blanco σ²Δt², random walk de bias σ²Δt. F_i = [0; I12]. */
    const eskf_params_t *p = &f->prm;
    eskf_real qv = p->sigma_a * p->sigma_a * dt * dt * white_scale;
    eskf_real qth = p->sigma_w * p->sigma_w * dt * dt * white_scale;
    eskf_real qba = p->sigma_aw * p->sigma_aw * dt;
    eskf_real qbg = p->sigma_ww * p->sigma_ww * dt;
    for (int i = 0; i < 3; i++) {
        f->P[ESKF_IDX_V + i][ESKF_IDX_V + i] += qv;
        f->P[ESKF_IDX_TH + i][ESKF_IDX_TH + i] += qth;
        f->P[ESKF_IDX_BA + i][ESKF_IDX_BA + i] += qba;
        f->P[ESKF_IDX_BG + i][ESKF_IDX_BG + i] += qbg;
    }
    symmetrize(f->P);
    return 1;
}

int eskf_predict(eskf_t *f, const eskf_real a_meas[3], const eskf_real w_meas[3], eskf_real dt)
{
    int st = guard(f);
    if (st != ESKF_OK) return st;
    if (a_meas == NULL || w_meas == NULL) return ESKF_E_NULL;
    if (!fin3(a_meas) || !fin3(w_meas)) return ESKF_E_INPUT;
    if (!fin(dt) || !(dt > RC(0.0))) {
        f->cnt.n_dt_rejected++;
        return ESKF_E_DT;
    }

    int nsub = 1;
    if (dt > f->prm.dt_max) {
        if (f->prm.gap_max > RC(0.0) && dt <= f->prm.gap_max) {
            eskf_real nreal = R_CEIL(dt / f->prm.dt_max); /* <= gap_max/dt_max <= 64 */
            nsub = (int)nreal;
            if (nsub < 1) nsub = 1;
            if (nsub > ESKF_MAX_BRIDGE_STEPS) nsub = ESKF_MAX_BRIDGE_STEPS;
            f->degraded |= ESKF_DEG_GAP_BRIDGED;
            f->cnt.n_gaps_bridged++;
        } else {
            f->degraded |= ESKF_DEG_GAP_LOST;
            f->cnt.n_dt_rejected++;
            return ESKF_E_DT_GAP;
        }
    }

    /* Ruido durante un puente. σ_a y σ_w son desviaciones típicas POR MUESTRA: en un paso
       normal el error e de la lectura actúa durante dt y aporta (σ·dt)² a v y a θ.
       En un puente la misma lectura se repite en nsub subpasos de h = dt/nsub: su error e
       es UNO solo, constante durante todo dt, y su efecto en v es e·dt, de varianza (σ·dt)²,
       igual que un paso de dt. Sumar (σ·h)² por subpaso como si fueran lecturas
       independientes daría (σ·dt)²/nsub: incertidumbre artificialmente baja. Se suma
       nsub·(σ·h)² por subpaso, que reproduce exactamente (σ·dt)² en v y θ. En p queda
       σ²dt⁴/3 frente al σ²dt⁴/4 exacto de un error constante (conservador, ≤ 4/3).
       Los random walk de los sesgos sí son incrementos independientes: σ²h por subpaso.
       NO se modela el cambio real de aceleración y giro durante el hueco (error de la
       retención de orden cero): por eso el puente está desactivado (gap_max = 0) en la
       configuración de primera integración. */
    eskf_real h = dt / (eskf_real)nsub;
    eskf_real white_scale = (eskf_real)nsub;
    for (int k = 0; k < nsub; k++)
        if (!predict_step(f, a_meas, w_meas, h, white_scale)) return latch_fault(f);

    f->cnt.n_predict++;
    if (eskf_check_health(f) != ESKF_OK) return latch_fault(f);
    return ESKF_OK;
}

int eskf_inflate_block(eskf_t *f, int block, const eskf_real std[3])
{
    int st = guard(f);
    if (st != ESKF_OK) return st;
    if (std == NULL) return ESKF_E_NULL;
    if (block != ESKF_IDX_P && block != ESKF_IDX_V && block != ESKF_IDX_TH && block != ESKF_IDX_BA &&
        block != ESKF_IDX_BG)
        return ESKF_E_INPUT;
    eskf_real add[3];
    for (int i = 0; i < 3; i++) {
        if (!proc_std_ok(std[i])) return ESKF_E_INPUT;
        add[i] = f->P[block + i][block + i] + std[i] * std[i];
        if (!fin(add[i])) return ESKF_E_INPUT;
    }
    for (int i = 0; i < 3; i++) f->P[block + i][block + i] = add[i];
    return ESKF_OK;
}

/* ------------------------------------------------------------------------- */
/* Actualización (ESKF._update, sec. 3.6, eq. 41-51)                          */
/* ------------------------------------------------------------------------- */

/*
 * Actualización escalar secuencial. Para la componente i de la medida:
 *   ν = (y_i − h_i(x)) − H_i·δx_acumulado
 *   S = H_i P H_iᵀ + r,  K = P H_iᵀ / S,  δx += K ν
 *   P = (I − K H_i) P (I − K H_i)ᵀ + K r Kᵀ      (Joseph, en O(n²))
 * Con V diagonal esto es exactamente la actualización apilada de Python.
 */
typedef struct {
    eskf_real dx[N];
} upd_ctx_t;

/* Prólogo común: reinicia el diagnóstico SIEMPRE (también en los retornos anticipados). */
static int upd_begin(eskf_t *f, int sensor, upd_ctx_t *c)
{
    if (f == NULL) return ESKF_E_NULL;
    if (f->magic != ESKF_MAGIC) return ESKF_E_NOT_INIT;
    clear_diag(f, sensor);
    for (int i = 0; i < N; i++) c->dx[i] = RC(0.0);
    if (f->fault) {
        f->last.status = ESKF_E_FAULT;
        return ESKF_E_FAULT;
    }
    return ESKF_OK;
}

static int upd_reject(eskf_t *f, int status)
{
    f->last.status = status;
    return status;
}

static void scalar_update(eskf_t *f, upd_ctx_t *c, const eskf_real h[N], eskf_real innov_raw, eskf_real r)
{
    eskf_real (*P)[N] = f->P;
    eskf_real b[N]; /* b = P hᵀ */
    eskf_real hdx = RC(0.0), hb = RC(0.0);
    for (int i = 0; i < N; i++) {
        eskf_real s = RC(0.0);
        for (int k = 0; k < N; k++)
            if (h[k] != RC(0.0)) s += P[i][k] * h[k];
        b[i] = s;
        hdx += h[i] * c->dx[i];
    }
    for (int i = 0; i < N; i++) hb += h[i] * b[i];
    eskf_real S = hb + r;
    eskf_real nu = innov_raw - hdx;
    if (!fin(S) || !(S > RC(0.0)) || !fin(nu)) { /* P no válida o desbordamiento */
        f->last.n_numeric++;
        return;
    }

    if (f->prm.innov_gate > RC(0.0) && nu * nu > f->prm.innov_gate * f->prm.innov_gate * S) {
        f->last.n_gated++;
        return;
    }

    eskf_real K[N];
    eskf_real invS = RC(1.0) / S;
    for (int i = 0; i < N; i++) {
        K[i] = b[i] * invS;
        if (!fin(K[i])) {
            f->last.n_numeric++;
            return;
        }
    }
    for (int i = 0; i < N; i++) c->dx[i] += K[i] * nu;

    /* Joseph evaluado en dos etapas, en el sitio:
         M  = (I − K h) P = P − K bᵀ
         P⁺ = M (I − K h)ᵀ + r K Kᵀ = M − (M hᵀ) Kᵀ + r K Kᵀ
       Calcular M primero es lo que conserva el término r K Kᵀ cuando P_hh ≫ r en float:
       con la expresión desarrollada de una sola vez, P + (−K bᵀ − … + r K Kᵀ) sumaba
       −1e7 + 0.25 antes de cancelar con P y perdía el 0.25. */
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) P[i][j] -= K[i] * b[j];
    eskf_real cM[N]; /* M hᵀ, calculado con la M ya redondeada */
    for (int i = 0; i < N; i++) {
        eskf_real s = RC(0.0);
        for (int k = 0; k < N; k++)
            if (h[k] != RC(0.0)) s += P[i][k] * h[k];
        cM[i] = s;
    }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) P[i][j] += -cM[i] * K[j] + r * K[i] * K[j];

    f->last.nis += nu * nu * invS;
    f->last.n_used++;
}

/* Inyección (eq. 48) y reset con G = I − [½δθ]× (eq. 49-51). Fija el estado de retorno. */
static int upd_finish(eskf_t *f, upd_ctx_t *c)
{
    symmetrize(f->P);
    if (f->last.n_numeric > 0) return upd_reject(f, latch_fault(f));
    if (f->last.n_used == 0) return upd_reject(f, ESKF_E_GATED); /* sólo queda la puerta */

    const eskf_real *dx = c->dx;
    for (int i = 0; i < N; i++)
        if (!fin(dx[i])) return upd_reject(f, latch_fault(f));

    eskf_real dq[4], qn[4];
    quat_from_rotvec(&dx[ESKF_IDX_TH], dq);
    quat_product(f->q, dq, qn);
    if (!quat_normalize(qn)) return upd_reject(f, latch_fault(f));

    for (int i = 0; i < 3; i++) {
        f->p[i] += dx[ESKF_IDX_P + i];
        f->v[i] += dx[ESKF_IDX_V + i];
        f->b_a[i] += dx[ESKF_IDX_BA + i];
        f->b_g[i] += dx[ESKF_IDX_BG + i];
    }
    for (int i = 0; i < 4; i++) f->q[i] = qn[i];

    /* G sólo difiere de I en el bloque θθ: Gθ = I − [½δθ]× */
    eskf_real hth[3] = {RC(0.5) * dx[ESKF_IDX_TH], RC(0.5) * dx[ESKF_IDX_TH + 1],
                        RC(0.5) * dx[ESKF_IDX_TH + 2]};
    eskf_real G[3][3];
    skew(hth, G);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) G[i][j] = ((i == j) ? RC(1.0) : RC(0.0)) - G[i][j];

    eskf_real (*P)[N] = f->P;
    eskf_real T[3][N];
    for (int i = 0; i < 3; i++) /* filas θ: Gθ P[θ,:] */
        for (int j = 0; j < N; j++)
            T[i][j] = G[i][0] * P[ESKF_IDX_TH][j] + G[i][1] * P[ESKF_IDX_TH + 1][j] +
                      G[i][2] * P[ESKF_IDX_TH + 2][j];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < N; j++) P[ESKF_IDX_TH + i][j] = T[i][j];
    for (int i = 0; i < N; i++) { /* columnas θ: P[:,θ] Gθᵀ */
        eskf_real r0 = P[i][ESKF_IDX_TH], r1 = P[i][ESKF_IDX_TH + 1], r2 = P[i][ESKF_IDX_TH + 2];
        for (int j = 0; j < 3; j++)
            P[i][ESKF_IDX_TH + j] = r0 * G[j][0] + r1 * G[j][1] + r2 * G[j][2];
    }
    symmetrize(P);

    if (eskf_check_health(f) != ESKF_OK) return upd_reject(f, latch_fault(f));
    f->last.status = f->last.n_used;
    return f->last.n_used;
}

static void zero_row(eskf_real h[N])
{
    for (int i = 0; i < N; i++) h[i] = RC(0.0);
}

/* Valida 3 sigmas de medida y devuelve sus varianzas */
static int meas_var3(const eskf_real sigma[3], eskf_real var[3])
{
    for (int i = 0; i < 3; i++)
        if (!meas_var(sigma[i], &var[i])) return 0;
    return 1;
}

/* h(x) = [p; v] (eq. 67-69) */
int eskf_update_gps(eskf_t *f, const eskf_real p_meas[3], const eskf_real v_meas[3],
                    const eskf_real sigma_p[3], const eskf_real sigma_v[3])
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_GPS, &c);
    if (st != ESKF_OK) return st;
    if (!p_meas || !v_meas || !sigma_p || !sigma_v) return upd_reject(f, ESKF_E_NULL);
    eskf_real rp[3], rv[3];
    if (!fin3(p_meas) || !fin3(v_meas) || !meas_var3(sigma_p, rp) || !meas_var3(sigma_v, rv))
        return upd_reject(f, ESKF_E_INPUT);
    if (!f->heading_known) return upd_reject(f, ESKF_E_NO_HEADING);

    eskf_real h[N];
    for (int i = 0; i < 3; i++) {
        zero_row(h);
        h[ESKF_IDX_P + i] = RC(1.0);
        scalar_update(f, &c, h, p_meas[i] - f->p[i], rp[i]);
    }
    for (int i = 0; i < 3; i++) {
        zero_row(h);
        h[ESKF_IDX_V + i] = RC(1.0);
        scalar_update(f, &c, h, v_meas[i] - f->v[i], rv[i]);
    }
    return upd_finish(f, &c);
}

static int velocity_update(eskf_t *f, upd_ctx_t *c, const eskf_real v_meas[3], const eskf_real rv[3])
{
    eskf_real h[N];
    for (int i = 0; i < 3; i++) {
        zero_row(h);
        h[ESKF_IDX_V + i] = RC(1.0);
        scalar_update(f, c, h, v_meas[i] - f->v[i], rv[i]);
    }
    return upd_finish(f, c);
}

int eskf_update_velocity(eskf_t *f, const eskf_real v_meas[3], const eskf_real sigma_v[3])
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_VEL, &c);
    if (st != ESKF_OK) return st;
    if (!v_meas || !sigma_v) return upd_reject(f, ESKF_E_NULL);
    eskf_real rv[3];
    if (!fin3(v_meas) || !meas_var3(sigma_v, rv)) return upd_reject(f, ESKF_E_INPUT);
    if (!f->heading_known) return upd_reject(f, ESKF_E_NO_HEADING);
    return velocity_update(f, &c, v_meas, rv);
}

int eskf_update_zupt(eskf_t *f, const eskf_real sigma_v[3])
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_ZUPT, &c);
    if (st != ESKF_OK) return st;
    if (!sigma_v) return upd_reject(f, ESKF_E_NULL);
    eskf_real rv[3];
    if (!meas_var3(sigma_v, rv)) return upd_reject(f, ESKF_E_INPUT);
    if (!f->at_rest) return upd_reject(f, ESKF_E_NOT_AT_REST);
    static const eskf_real zero[3] = {RC(0.0), RC(0.0), RC(0.0)};
    return velocity_update(f, &c, zero, rv);
}

/*
 * Medidas de un vector de referencia global r visto en cuerpo: h = Rᵀ r (+ b_a).
 * Con error local, R_t = R·Exp(δθ) ⇒ R_tᵀ r ≈ Rᵀ r + [Rᵀ r]× δθ, así que
 * H_δθ = [Rᵀ r]×. Es el mismo resultado que H_x·X_δx de Python (eq. 56, 47).
 */
static int vector_update(eskf_t *f, upd_ctx_t *c, const eskf_real ref[3], const eskf_real y[3],
                         const eskf_real var[3], int with_accel_bias)
{
    eskf_real R[3][3], hr[3], S[3][3];
    eskf_rotation_matrix(f->q, R);
    mat3T_vec(R, ref, hr);
    skew(hr, S);

    eskf_real h[N];
    for (int i = 0; i < 3; i++) {
        zero_row(h);
        for (int j = 0; j < 3; j++) h[ESKF_IDX_TH + j] = S[i][j];
        eskf_real pred = hr[i];
        if (with_accel_bias) {
            h[ESKF_IDX_BA + i] = RC(1.0);
            pred += f->b_a[i];
        }
        scalar_update(f, c, h, y[i] - pred, var[i]);
    }
    return upd_finish(f, c);
}

int eskf_update_mag(eskf_t *f, const eskf_real m_meas[3], const eskf_real sigma_mag[3])
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_MAG, &c);
    if (st != ESKF_OK) return st;
    if (!m_meas || !sigma_mag) return upd_reject(f, ESKF_E_NULL);
    eskf_real rm[3];
    if (!fin3(m_meas) || !meas_var3(sigma_mag, rm)) return upd_reject(f, ESKF_E_INPUT);
    if (!f->prm.has_mag_ref) return upd_reject(f, ESKF_E_NO_REF);
    if (!f->heading_known) return upd_reject(f, ESKF_E_NO_HEADING);
    return vector_update(f, &c, f->prm.mag_ref, m_meas, rm, 0);
}

int eskf_update_accel(eskf_t *f, const eskf_real a_meas[3], const eskf_real sigma_a[3])
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_ACCEL, &c);
    if (st != ESKF_OK) return st;
    if (!a_meas || !sigma_a) return upd_reject(f, ESKF_E_NULL);
    eskf_real ra[3];
    if (!fin3(a_meas) || !meas_var3(sigma_a, ra)) return upd_reject(f, ESKF_E_INPUT);
    if (!f->at_rest) return upd_reject(f, ESKF_E_NOT_AT_REST);
    eskf_real dg = norm3(a_meas) - norm3(f->g);
    if (!(R_FABS(dg) < f->prm.accel_threshold)) return upd_reject(f, ESKF_E_MOTION);
    return vector_update(f, &c, f->g, a_meas, ra, 1);
}

int eskf_update_baro_at(eskf_t *f, eskf_real z_meas, eskf_real sigma_baro, eskf_real tau)
{
    upd_ctx_t c;
    int st = upd_begin(f, ESKF_SENSOR_BARO, &c);
    if (st != ESKF_OK) return st;
    eskf_real r;
    if (!fin(z_meas) || !meas_var(sigma_baro, &r)) return upd_reject(f, ESKF_E_INPUT);
    eskf_real tau_lim = (f->prm.tau_max > RC(0.0)) ? f->prm.tau_max : f->prm.dt_max;
    if (!fin(tau) || R_FABS(tau) > tau_lim) return upd_reject(f, ESKF_E_INPUT);
    eskf_real h[N];
    zero_row(h);
    h[ESKF_IDX_P + 2] = RC(1.0);
    h[ESKF_IDX_V + 2] = tau; /* 0 en eskf_update_baro: idéntico a la versión sin desfase */
    scalar_update(f, &c, h, z_meas - (f->p[2] + f->v[2] * tau), r);
    return upd_finish(f, &c);
}

int eskf_update_baro(eskf_t *f, eskf_real z_meas, eskf_real sigma_baro)
{
    return eskf_update_baro_at(f, z_meas, sigma_baro, RC(0.0));
}
