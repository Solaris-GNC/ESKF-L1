/*
 * test_fw.c — piezas portables del firmware: ordenación de eventos (nav_events) y
 * conversión/saturación de la IMU (imu_conv). Regresiones 9 y 11 de la revisión del 19-oct.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_conv.h"
#include "nav_events.h"
#include "test_util.h"

static nav_ev_imu_t ev_imu(int64_t t)
{
    nav_ev_imu_t e;
    memset(&e, 0, sizeof e);
    e.t_us = t;
    e.s.t = (double)t * 1e-6;
    return e;
}
static nav_ev_baro_t ev_baro(int64_t t)
{
    nav_ev_baro_t e;
    memset(&e, 0, sizeof e);
    e.t_us = t;
    e.s.t = (double)t * 1e-6;
    return e;
}

static void prueba_orden_eventos(void)
{
    printf("9. eventos de IMU y baro en orden temporal aunque lleguen por lotes\n");
    static nav_merge_t m;
    nav_ev_imu_t ei;
    nav_ev_baro_t eb;
    nav_merge_init(&m);
    /* llega primero un baro (t=10 ms) y después un lote de IMU 0..20 ms (cada 4.444 ms) */
    nav_merge_push_baro(&m, &(nav_ev_baro_t){ev_baro(10000).t_us, 1, ev_baro(10000).s});
    for (int k = 0; k <= 4; k++) {
        nav_ev_imu_t e = ev_imu((int64_t)(k * 4444));
        nav_merge_push_imu(&m, &e);
    }
    int64_t now = 30000, holdoff = 20000, last = -1;
    int order_ok = 1, n_out = 0;
    char seq[32] = {0};
    nav_ev_kind_t kd;
    while ((kd = nav_merge_pop(&m, now, holdoff, &ei, &eb)) != NAV_EV_NONE) {
        int64_t t = (kd == NAV_EV_IMU) ? ei.t_us : eb.t_us;
        if (t < last) order_ok = 0;
        last = t;
        seq[n_out++] = (kd == NAV_EV_IMU) ? 'I' : 'B';
    }
    printf("   orden entregado: %s (ejemplo v3: toda la IMU antes del baro)\n", seq);
    CHECK(order_ok && strcmp(seq, "IIIB") == 0, "orden %s", seq);
    /* IMU 13.3 y 17.8 ms esperan: no hay baro posterior y no han pasado 20 ms */
    CHECK(m.imu_n == 2, "quedan 2 IMU a la espera (%u)", (unsigned)m.imu_n);
    CHECK(nav_merge_pop(&m, 34000, holdoff, &ei, &eb) == NAV_EV_IMU && ei.t_us == 13332, "a los 34 ms sale la de 13.3");

    /* espera: un evento solo no sale hasta holdoff */
    nav_merge_init(&m);
    nav_ev_baro_t b = ev_baro(100000);
    nav_merge_push_baro(&m, &b);
    CHECK(nav_merge_pop(&m, 110000, 20000, &ei, &eb) == NAV_EV_NONE, "baro solo, 10 ms < espera: no sale");
    CHECK(nav_merge_pop(&m, 120000, 20000, &ei, &eb) == NAV_EV_BARO, "a los 20 ms sale");

    /* empate: IMU primero */
    nav_merge_init(&m);
    nav_ev_imu_t i = ev_imu(5000);
    b = ev_baro(5000);
    nav_merge_push_baro(&m, &b);
    nav_merge_push_imu(&m, &i);
    CHECK(nav_merge_pop(&m, 100000, 0, &ei, &eb) == NAV_EV_IMU, "empate: IMU primero");

    /* desbordamiento contado */
    nav_merge_init(&m);
    for (int k = 0; k < NAV_EV_IMU_CAP + 3; k++) {
        i = ev_imu(k);
        nav_merge_push_imu(&m, &i);
    }
    CHECK(m.imu_overflow == 3 && m.imu_hwm == NAV_EV_IMU_CAP, "cola llena: 3 perdidos contados");

    /* aleatorio: dos flujos con retrasos distintos, salida siempre ordenada */
    nav_merge_init(&m);
    unsigned st = 5u;
    int64_t ti = 0, tb = 0, tnow = 0;
    int bad = 0, popped = 0;
    last = -1;
    for (int step = 0; step < 20000; step++) {
        tnow += 1000; /* 1 ms */
        while (ti + 4444 <= tnow - 2000 * (int64_t)(0.5 * (urand(&st) + 1) * 3)) { /* llegan con 0..6 ms */
            ti += 4444;
            i = ev_imu(ti);
            nav_merge_push_imu(&m, &i);
        }
        if (tb + 20000 <= tnow - 8000) { /* el baro llega 8 ms tarde */
            tb += 20000;
            b = ev_baro(tb);
            nav_merge_push_baro(&m, &b);
        }
        while ((kd = nav_merge_pop(&m, tnow, 12000, &ei, &eb)) != NAV_EV_NONE) {
            int64_t t = (kd == NAV_EV_IMU) ? ei.t_us : eb.t_us;
            if (t < last) bad++;
            last = t;
            popped++;
        }
    }
    CHECK(bad == 0 && popped > 5000 && m.imu_overflow == 0, "20 s intercalados: %d fuera de orden de %d", bad, popped);
}

static void prueba_conversion_saturacion(void)
{
    printf("11. conversión y saturación por código, en ambos extremos y en ejes del cohete\n");
    imu_conv_cfg_t c = {2048.0f, 16.4f, 9.80665f, {0, 1, 2}, {1, 1, 1}, 0};
    CHECK(imu_conv_check(&c) == 0, "identidad válida");
    int16_t a[3] = {32767, 0, -32768}, g[3] = {0, -32768, 100};
    float as[3], gs[3];
    uint8_t sat = imu_conv(&c, a, g, as, gs);
    CHECK(sat == (NAV_SAT_ACC_X | NAV_SAT_ACC_Z | NAV_SAT_GYRO_Y), "extremos + y −: 0x%02x", sat);
    CHECK(fabs((double)as[0] - 32767.0 / 2048.0 * 9.80665) < 1e-3 && fabs((double)as[2] + 16 * 9.80665) < 1e-3,
          "unidades m/s²");
    CHECK(fabs((double)gs[2] - 100.0 / 16.4 * 0.017453292519943295) < 1e-6, "unidades rad/s");
    int16_t a2[3] = {32000, -32000, 0}, g2[3] = {0, 0, 0};
    CHECK(imu_conv(&c, a2, g2, as, gs) == 0, "32000 LSB con margen 0: no saturado");
    c.sat_margin_lsb = 768; /* ≈ 97.7 %: margen por el filtro interno (política provisional) */
    CHECK(imu_conv(&c, a2, g2, as, gs) == (NAV_SAT_ACC_X | NAV_SAT_ACC_Y), "con margen 768: saturado");

    /* ejes: cohete x = −sensor z, y = sensor y, z = sensor x (rotación propia) */
    imu_conv_cfg_t r = {2048.0f, 16.4f, 9.80665f, {2, 1, 0}, {-1, 1, 1}, 0};
    CHECK(imu_conv_check(&r) == 0, "rotación propia aceptada");
    int16_t a3[3] = {0, 0, 32767};
    sat = imu_conv(&r, a3, g2, as, gs);
    CHECK(sat == NAV_SAT_ACC_X && as[0] < 0.0f, "la saturación se asigna al eje del cohete (0x%02x)", sat);
    imu_conv_cfg_t l = {2048.0f, 16.4f, 9.80665f, {2, 1, 0}, {1, 1, 1}, 0};
    CHECK(imu_conv_check(&l) != 0, "transformación con det −1 (espejo) rechazada");
    imu_conv_cfg_t d = {2048.0f, 16.4f, 9.80665f, {0, 0, 2}, {1, 1, 1}, 0};
    CHECK(imu_conv_check(&d) != 0, "eje repetido rechazado");
}

int main(void)
{
    prueba_orden_eventos();
    prueba_conversion_saturacion();
    return report("test_fw");
}
