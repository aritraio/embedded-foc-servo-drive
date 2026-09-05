#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

/* Space Vector PWM: sector ID, dwell times, center-aligned duties.
 * All times normalized so T1+T2+T0 == Ts. Duties in [0,1].
 * Linear modulation limit: |V*| <= Vdc/sqrt(3) (+15.5% over SPWM). */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sector; /* 1..6 */
    float t1;        /* active vector dwell [s] */
    float t2;        /* active vector dwell [s] */
    float t0;        /* zero vector dwell [s]  */
    float ta;        /* phase-A duty [0,1] */
    float tb;        /* phase-B duty [0,1] */
    float tc;        /* phase-C duty [0,1] */
    uint8_t saturated; /* 1 if overmodulation scaling applied */
} svpwm_out_t;

void svpwm_compute(float v_alpha, float v_beta, float vdc, float ts, svpwm_out_t *out);

float svpwm_vmax_linear(float vdc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
