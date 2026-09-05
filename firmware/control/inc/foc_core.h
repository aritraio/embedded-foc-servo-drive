#ifndef FOC_CORE_H
#define FOC_CORE_H

/* Deterministic 25 kHz FOC inner current loop.
 * Owns Clarke/Park, decoupled d/q PI + back-EMF feed-forward,
 * voltage-circle limitation, inverse Park + SVPWM. */

#include "foc_math.h"
#include "pid_controller.h"
#include "svpwm.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rs;
    float ld;
    float lq;
    float lambda_pm;
    uint32_t pole_pairs;
    float vdc_nom;
    float theta_offset;
} foc_motor_params_t;

typedef struct {
    foc_motor_params_t motor;
    pid_controller_t pid_d;
    pid_controller_t pid_q;
    float id_ref;
    float iq_ref;
    float theta_e;
    /* Last-step diagnostics. */
    float id;
    float iq;
    float vd;
    float vq;
    float vd_pi;
    float vq_pi;
    svpwm_out_t svpwm;
} foc_core_t;

void foc_core_init(foc_core_t *foc, const foc_motor_params_t *motor,
                   float id_kp, float id_ki, float iq_kp, float iq_ki,
                   float v_max);
void foc_core_reset(foc_core_t *foc);
void foc_core_set_refs(foc_core_t *foc, float id_ref, float iq_ref);
void foc_core_set_theta_offset(foc_core_t *foc, float offset);

/* Single 40 us step. ia/ib in A, theta_mech in rad, omega_e in rad/s(elec). */
void foc_core_step(foc_core_t *foc, float ia, float ib, float theta_mech,
                   float omega_e, float vdc, float dt);

/* Helpers also used by tests / SIL. */
float foc_reconstruct_ic(float ia, float ib);
float foc_circle_limit(float *vd, float *vq, float vmax);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CORE_H */
