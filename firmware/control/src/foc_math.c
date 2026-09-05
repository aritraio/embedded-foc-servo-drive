#include "foc_math.h"

#include <math.h>
#include <stddef.h>

void foc_clarke(float ia, float ib, float ic, foc_alphabeta_t *out)
{
    if (out == NULL) {
        return;
    }
    (void)ic; /* ic redundant under balance constraint; kept for API clarity */
    out->alpha = ia;
    out->beta = (ia + 2.0f * ib) * FOC_MATH_1_SQRT3;
}

void foc_clarke_2phase(float ia, float ib, foc_alphabeta_t *out)
{
    if (out == NULL) {
        return;
    }
    out->alpha = ia;
    out->beta = (ia + 2.0f * ib) * FOC_MATH_1_SQRT3;
}

void foc_sincos(float angle, float *s, float *c)
{
    float a = foc_wrap_pi(angle);
    /* double-promotion safe: compute in float via sinf/cosf. */
    if (s != NULL) {
        *s = sinf(a);
    }
    if (c != NULL) {
        *c = cosf(a);
    }
}

void foc_park(const foc_alphabeta_t *ab, float theta_e, foc_dq_t *out)
{
    float s = 0.0f;
    float c = 0.0f;

    if ((ab == NULL) || (out == NULL)) {
        return;
    }
    foc_sincos(theta_e, &s, &c);
    out->d = ab->alpha * c + ab->beta * s;
    out->q = -ab->alpha * s + ab->beta * c;
}

void foc_inv_park(const foc_dq_t *dq, float theta_e, foc_alphabeta_t *out)
{
    float s = 0.0f;
    float c = 0.0f;

    if ((dq == NULL) || (out == NULL)) {
        return;
    }
    foc_sincos(theta_e, &s, &c);
    out->alpha = dq->d * c - dq->q * s;
    out->beta = dq->d * s + dq->q * c;
}

float foc_wrap_pi(float angle)
{
    float a = angle;
    /* fmodf-based wrap, robust for large angles. */
    a = fmodf(a + FOC_MATH_PI, FOC_MATH_2PI);
    if (a < 0.0f) {
        a += FOC_MATH_2PI;
    }
    return a - FOC_MATH_PI;
}

float foc_wrap_2pi(float angle)
{
    float a = fmodf(angle, FOC_MATH_2PI);
    if (a < 0.0f) {
        a += FOC_MATH_2PI;
    }
    return a;
}

float foc_elec_angle(float theta_mech, uint32_t pole_pairs, float offset)
{
    float p = (float)pole_pairs;
    return foc_wrap_2pi(p * theta_mech - offset);
}
