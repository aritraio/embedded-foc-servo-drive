#ifndef FOC_MATH_H
#define FOC_MATH_H

/* Pure hardware-agnostic FOC vector math. No HAL, no heap, no statics.
 * All angles in radians, currents in Amperes, voltages in Volts. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FOC_MATH_PI
#define FOC_MATH_PI (3.14159265358979323846f)
#endif
#define FOC_MATH_2PI    (6.28318530717958647692f)
#define FOC_MATH_SQRT3  (1.73205080756887729353f)
#define FOC_MATH_1_SQRT3 (0.57735026918962576451f)

typedef struct {
    float alpha;
    float beta;
} foc_alphabeta_t;

typedef struct {
    float d;
    float q;
} foc_dq_t;

typedef struct {
    float a;
    float b;
    float c;
} foc_abc_t;

/* Clarke: 3-phase -> stationary alpha-beta.
 * ia+ib+ic==0 assumed; uses ia and ib (ic reconstructed when needed). */
void foc_clarke(float ia, float ib, float ic, foc_alphabeta_t *out);
void foc_clarke_2phase(float ia, float ib, foc_alphabeta_t *out);

/* Park / inverse Park using electrical angle theta_e [rad]. */
void foc_park(const foc_alphabeta_t *ab, float theta_e, foc_dq_t *out);
void foc_inv_park(const foc_dq_t *dq, float theta_e, foc_alphabeta_t *out);

/* Angle helpers. */
float foc_wrap_pi(float angle);
float foc_wrap_2pi(float angle);
float foc_elec_angle(float theta_mech, uint32_t pole_pairs, float offset);

/* Fast sin/cos (wraps to [-pi,pi] then libm; CORDIC hook for STM32). */
void foc_sincos(float angle, float *s, float *c);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
