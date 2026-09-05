#include "svpwm.h"
#include "foc_math.h"

#include <math.h>
#include <stddef.h>

float svpwm_vmax_linear(float vdc)
{
    if (vdc <= 0.0f) {
        return 0.0f;
    }
    return vdc * FOC_MATH_1_SQRT3;
}

void svpwm_compute(float v_alpha, float v_beta, float vdc, float ts, svpwm_out_t *out)
{
    static const float k60 = 1.0471975511965976f; /* pi/3 */
    float vref = 0.0f;
    float theta = 0.0f;
    float theta_local = 0.0f;
    float m = 0.0f;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float t0 = 0.0f;
    float ta = 0.0f;
    float tb = 0.0f;
    float tc = 0.0f;
    uint32_t sector = 1U;
    uint8_t sat = 0U;

    if ((out == NULL) || (ts <= 0.0f) || (vdc <= 0.0f)) {
        return;
    }

    vref = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    theta = foc_wrap_2pi(atan2f(v_beta, v_alpha));

    /* Sector 1..6 from 60-degree slices. Guard theta==2pi edge. */
    sector = (uint32_t)(theta / k60) + 1U;
    if (sector < 1U) {
        sector = 1U;
    }
    if (sector > 6U) {
        sector = 6U;
    }
    theta_local = theta - ((float)(sector - 1U) * k60);

    /* m = sqrt(3)*Vref/Vdc; T1 = m*Ts*sin(60-th), T2 = m*Ts*sin(th). */
    m = (FOC_MATH_SQRT3 * vref) / vdc;
    t1 = m * ts * sinf(k60 - theta_local);
    t2 = m * ts * sinf(theta_local);
    if (t1 < 0.0f) {
        t1 = 0.0f;
    }
    if (t2 < 0.0f) {
        t2 = 0.0f;
    }

    /* Overmodulation: scale active vectors to fit Ts, zero T0. */
    if ((t1 + t2) > ts) {
        const float scale = ts / (t1 + t2);
        t1 *= scale;
        t2 *= scale;
        t0 = 0.0f;
        sat = 1U;
    } else {
        t0 = ts - t1 - t2;
    }

    /* Symmetric 7-segment center-aligned on-times. */
    {
        const float h0 = t0 * 0.25f;
        const float h1 = t1 * 0.5f;
        const float h2 = t2 * 0.5f;
        switch (sector) {
        case 1U:
            ta = h0 + h1 + h2 + (ts * 0.25f);
            tb = h0 + h2 + (ts * 0.0f) + (t0 * 0.0f);
            /* Recompute cleanly below; fallthrough avoided for clarity. */
            ta = (t1 + t2 + t0 * 0.5f) / ts;
            tb = (t2 + t0 * 0.5f) / ts;
            tc = (t0 * 0.5f) / ts;
            break;
        case 2U:
            ta = (t1 + t0 * 0.5f) / ts;
            tb = (t1 + t2 + t0 * 0.5f) / ts;
            tc = (t0 * 0.5f) / ts;
            break;
        case 3U:
            ta = (t0 * 0.5f) / ts;
            tb = (t1 + t2 + t0 * 0.5f) / ts;
            tc = (t2 + t0 * 0.5f) / ts;
            break;
        case 4U:
            ta = (t0 * 0.5f) / ts;
            tb = (t1 + t0 * 0.5f) / ts;
            tc = (t1 + t2 + t0 * 0.5f) / ts;
            break;
        case 5U:
            ta = (t2 + t0 * 0.5f) / ts;
            tb = (t0 * 0.5f) / ts;
            tc = (t1 + t2 + t0 * 0.5f) / ts;
            break;
        default: /* sector 6 */
            ta = (t1 + t2 + t0 * 0.5f) / ts;
            tb = (t0 * 0.5f) / ts;
            tc = (t1 + t0 * 0.5f) / ts;
            break;
        }
        (void)h0;
        (void)h1;
        (void)h2;
    }

    /* Clamp duties defensively. */
    if (ta < 0.0f) {
        ta = 0.0f;
    }
    if (ta > 1.0f) {
        ta = 1.0f;
    }
    if (tb < 0.0f) {
        tb = 0.0f;
    }
    if (tb > 1.0f) {
        tb = 1.0f;
    }
    if (tc < 0.0f) {
        tc = 0.0f;
    }
    if (tc > 1.0f) {
        tc = 1.0f;
    }

    out->sector = sector;
    out->t1 = t1;
    out->t2 = t2;
    out->t0 = t0;
    out->ta = ta;
    out->tb = tb;
    out->tc = tc;
    out->saturated = sat;
}
