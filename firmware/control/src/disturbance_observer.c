#include "disturbance_observer.h"

#include <stddef.h>

void dob_init(dob_state_t *dob, float inertia, float friction, float gain)
{
    if (dob == NULL) {
        return;
    }
    dob->inertia = (inertia > 0.0f) ? inertia : 1.0e-5f;
    dob->friction = (friction >= 0.0f) ? friction : 0.0f;
    dob->gain = (gain > 0.0f) ? gain : 50.0f;
    dob->t_hat = 0.0f;
    dob->w_hat = 0.0f;
}

void dob_reset(dob_state_t *dob)
{
    if (dob == NULL) {
        return;
    }
    dob->t_hat = 0.0f;
    dob->w_hat = 0.0f;
}

void dob_update(dob_state_t *dob, float torque_elec, float omega_meas, float dt)
{
    float w_err = 0.0f;
    float dw_hat = 0.0f;
    float dt_hat = 0.0f;
    float gamma = 0.0f;

    if ((dob == NULL) || (dt <= 0.0f)) {
        return;
    }
    /* Luenberger DOB, 2nd-order stable error dynamics:
     *   dw_hat/dt = (Te - t_hat - B*w_hat)/J + L*(w - w_hat)
     *   dt_hat/dt = -gamma*(w - w_hat), gamma = L^2*J/4 (critically damped). */
    w_err = omega_meas - dob->w_hat;
    gamma = dob->gain * dob->gain * dob->inertia * 0.25f;
    dw_hat = ((torque_elec - dob->t_hat - dob->friction * dob->w_hat) / dob->inertia) +
             dob->gain * w_err;
    dt_hat = -gamma * w_err;
    dob->w_hat += dw_hat * dt;
    dob->t_hat += dt_hat * dt;
    /* Clamp estimate to plausible load range. */
    if (dob->t_hat > 5.0f) {
        dob->t_hat = 5.0f;
    } else if (dob->t_hat < -5.0f) {
        dob->t_hat = -5.0f;
    }
}

float dob_get_torque(const dob_state_t *dob)
{
    if (dob == NULL) {
        return 0.0f;
    }
    return dob->t_hat;
}
