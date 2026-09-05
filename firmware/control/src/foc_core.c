#include "foc_core.h"

#include <math.h>
#include <stddef.h>

float foc_reconstruct_ic(float ia, float ib)
{
    return -(ia + ib);
}

float foc_circle_limit(float *vd, float *vq, float vmax)
{
    float mag = 0.0f;
    float scale = 1.0f;

    if ((vd == NULL) || (vq == NULL) || (vmax <= 0.0f)) {
        return 1.0f;
    }
    mag = sqrtf((*vd) * (*vd) + (*vq) * (*vq));
    if (mag > vmax) {
        scale = vmax / mag;
        *vd *= scale;
        *vq *= scale;
    }
    return scale;
}

void foc_core_init(foc_core_t *foc, const foc_motor_params_t *motor,
                   float id_kp, float id_ki, float iq_kp, float iq_ki,
                   float v_max)
{
    pid_config_t cd;
    pid_config_t cq;

    if ((foc == NULL) || (motor == NULL)) {
        return;
    }
    foc->motor = *motor;

    cd.kp = id_kp;
    cd.ki = id_ki;
    cd.kd = 0.0f;
    cd.n = 0.0f;
    cd.out_min = -v_max;
    cd.out_max = v_max;
    cd.int_min = -v_max;
    cd.int_max = v_max;
    cd.kt = 0.0f;
    cd.d_on_measurement = 0U;

    cq.kp = iq_kp;
    cq.ki = iq_ki;
    cq.kd = 0.0f;
    cq.n = 0.0f;
    cq.out_min = -v_max;
    cq.out_max = v_max;
    cq.int_min = -v_max;
    cq.int_max = v_max;
    cq.kt = 0.0f;
    cq.d_on_measurement = 0U;

    pid_init(&foc->pid_d, &cd);
    pid_init(&foc->pid_q, &cq);

    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->theta_e = 0.0f;
    foc->id = 0.0f;
    foc->iq = 0.0f;
    foc->vd = 0.0f;
    foc->vq = 0.0f;
    foc->vd_pi = 0.0f;
    foc->vq_pi = 0.0f;
    foc->svpwm.sector = 1U;
    foc->svpwm.t1 = 0.0f;
    foc->svpwm.t2 = 0.0f;
    foc->svpwm.t0 = 0.0f;
    foc->svpwm.ta = 0.5f;
    foc->svpwm.tb = 0.5f;
    foc->svpwm.tc = 0.5f;
    foc->svpwm.saturated = 0U;
}

void foc_core_reset(foc_core_t *foc)
{
    if (foc == NULL) {
        return;
    }
    pid_reset(&foc->pid_d);
    pid_reset(&foc->pid_q);
    foc->id = 0.0f;
    foc->iq = 0.0f;
    foc->vd = 0.0f;
    foc->vq = 0.0f;
    foc->vd_pi = 0.0f;
    foc->vq_pi = 0.0f;
}

void foc_core_set_refs(foc_core_t *foc, float id_ref, float iq_ref)
{
    if (foc == NULL) {
        return;
    }
    foc->id_ref = id_ref;
    foc->iq_ref = iq_ref;
}

void foc_core_set_theta_offset(foc_core_t *foc, float offset)
{
    if (foc == NULL) {
        return;
    }
    foc->motor.theta_offset = offset;
}

void foc_core_step(foc_core_t *foc, float ia, float ib, float theta_mech,
                   float omega_e, float vdc, float dt)
{
    foc_alphabeta_t ab;
    foc_dq_t dq;
    foc_dq_t vdq_ref;
    foc_alphabeta_t vab;
    float ic = 0.0f;
    float vd_pi = 0.0f;
    float vq_pi = 0.0f;
    float vd = 0.0f;
    float vq = 0.0f;
    float ff_d = 0.0f;
    float ff_q = 0.0f;
    float vmax = 0.0f;

    if ((foc == NULL) || (dt <= 0.0f)) {
        return;
    }
    if (vdc <= 0.0f) {
        vdc = foc->motor.vdc_nom;
    }

    ic = foc_reconstruct_ic(ia, ib);
    foc->theta_e = foc_elec_angle(theta_mech, foc->motor.pole_pairs,
                                  foc->motor.theta_offset);

    foc_clarke(ia, ib, ic, &ab);
    foc_park(&ab, foc->theta_e, &dq);
    foc->id = dq.d;
    foc->iq = dq.q;

    /* Decoupled PI + feed-forward decoupling. */
    vd_pi = pid_update(&foc->pid_d, foc->id_ref, dq.d, dt);
    vq_pi = pid_update(&foc->pid_q, foc->iq_ref, dq.q, dt);
    foc->vd_pi = vd_pi;
    foc->vq_pi = vq_pi;

    ff_d = -omega_e * foc->motor.lq * dq.q;
    ff_q = omega_e * (foc->motor.ld * dq.d + foc->motor.lambda_pm);
    vd = vd_pi + ff_d;
    vq = vq_pi + ff_q;

    vmax = svpwm_vmax_linear(vdc);
    (void)foc_circle_limit(&vd, &vq, vmax);

    foc->vd = vd;
    foc->vq = vq;

    vdq_ref.d = vd;
    vdq_ref.q = vq;
    foc_inv_park(&vdq_ref, foc->theta_e, &vab);
    svpwm_compute(vab.alpha, vab.beta, vdc, dt, &foc->svpwm);
}
