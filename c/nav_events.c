#include "nav_events.h"

#include <string.h>

void nav_merge_init(nav_merge_t *m)
{
    if (m) memset(m, 0, sizeof *m);
}

int nav_merge_push_imu(nav_merge_t *m, const nav_ev_imu_t *e)
{
    if (m->imu_n >= NAV_EV_IMU_CAP) {
        m->imu_overflow++;
        return -1;
    }
    m->imu[(m->imu_head + m->imu_n) % NAV_EV_IMU_CAP] = *e;
    m->imu_n++;
    if (m->imu_n > m->imu_hwm) m->imu_hwm = m->imu_n;
    return 0;
}

int nav_merge_push_baro(nav_merge_t *m, const nav_ev_baro_t *e)
{
    if (m->baro_n >= NAV_EV_BARO_CAP) {
        m->baro_overflow++;
        return -1;
    }
    m->baro[(m->baro_head + m->baro_n) % NAV_EV_BARO_CAP] = *e;
    m->baro_n++;
    if (m->baro_n > m->baro_hwm) m->baro_hwm = m->baro_n;
    return 0;
}

nav_ev_kind_t nav_merge_pop(nav_merge_t *m, int64_t now_us, int64_t holdoff_us, nav_ev_imu_t *imu,
                            nav_ev_baro_t *baro)
{
    const nav_ev_imu_t *hi = m->imu_n ? &m->imu[m->imu_head] : 0;
    const nav_ev_baro_t *hb = m->baro_n ? &m->baro[m->baro_head] : 0;
    if (!hi && !hb) return NAV_EV_NONE;

    int take_imu;
    if (hi && hb) take_imu = hi->t_us <= hb->t_us; /* empate: IMU primero */
    else take_imu = hi != 0;

    if (take_imu) {
        /* seguro si el baro ya tiene un evento posterior, o si ha pasado la espera */
        if (!hb && now_us - hi->t_us < holdoff_us) return NAV_EV_NONE;
        *imu = *hi;
        m->imu_head = (m->imu_head + 1u) % NAV_EV_IMU_CAP;
        m->imu_n--;
        return NAV_EV_IMU;
    }
    if (!hi && now_us - hb->t_us < holdoff_us) return NAV_EV_NONE;
    *baro = *hb;
    m->baro_head = (m->baro_head + 1u) % NAV_EV_BARO_CAP;
    m->baro_n--;
    return NAV_EV_BARO;
}
