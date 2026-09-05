#include "pid_controller.h"

#include <stddef.h>

void pid_init(pid_controller_t *pid, const pid_config_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL)) {
        return;
    }
    pid->cfg = *cfg;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
    pid->saturated = 0U;
    pid->initialized = 0U;
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
    pid->saturated = 0U;
    pid->initialized = 0U;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
}

static float pid_clamp(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
    float error = 0.0f;
    float p_term = 0.0f;
    float d_raw = 0.0f;
    float d_term = 0.0f;
    float i_term = 0.0f;
    float u_unsat = 0.0f;
    float u_sat = 0.0f;

    if ((pid == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }

    error = setpoint - measurement;
    p_term = pid->cfg.kp * error;

    /* Derivative path. */
    if (pid->cfg.kd != 0.0f) {
        if (pid->cfg.d_on_measurement != 0U) {
            if (pid->initialized != 0U) {
                d_raw = -(measurement - pid->prev_meas) / dt;
            } else {
                d_raw = 0.0f;
            }
        } else {
            if (pid->initialized != 0U) {
                d_raw = (error - pid->prev_error) / dt;
            } else {
                d_raw = 0.0f;
            }
        }
        if (pid->cfg.n > 0.0f) {
            /* First-order LPF: d_filt += (d_raw - d_filt) * dt*N/(1+dt*N). */
            const float alpha = (dt * pid->cfg.n) / (1.0f + dt * pid->cfg.n);
            pid->d_filt += (d_raw - pid->d_filt) * alpha;
            d_term = pid->cfg.kd * pid->d_filt;
        } else {
            d_term = pid->cfg.kd * d_raw;
        }
    }

    i_term = pid->integrator;

    u_unsat = p_term + i_term + d_term;
    u_sat = pid_clamp(u_unsat, pid->cfg.out_min, pid->cfg.out_max);
    pid->saturated = (u_sat != u_unsat) ? 1U : 0U;

    /* Integrator update: conditional integration (freeze when saturated and
     * error pushes deeper) + back-calculation correction. */
    {
        const uint8_t can_integrate =
            (pid->saturated == 0U) ||
            ((u_unsat > pid->cfg.out_max) && (error < 0.0f)) ||
            ((u_unsat < pid->cfg.out_min) && (error > 0.0f));
        if (can_integrate != 0U) {
            pid->integrator += pid->cfg.ki * error * dt;
        }
        if (pid->cfg.kt > 0.0f) {
            pid->integrator += pid->cfg.kt * (u_sat - u_unsat) * dt;
        }
        pid->integrator = pid_clamp(pid->integrator, pid->cfg.int_min, pid->cfg.int_max);
        /* Recompute output with final integrator so freeze is instantaneous. */
        i_term = pid->integrator;
        u_unsat = p_term + i_term + d_term;
        u_sat = pid_clamp(u_unsat, pid->cfg.out_min, pid->cfg.out_max);
        pid->saturated = (u_sat != u_unsat) ? 1U : 0U;
    }

    pid->prev_error = error;
    pid->prev_meas = measurement;
    pid->initialized = 1U;
    return u_sat;
}

float pid_update_error(pid_controller_t *pid, float error, float dt)
{
    if ((pid == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }
    /* Route through setpoint/meas form with measurement = -error so that
     * derivative-on-measurement behaves as derivative-on-error. */
    if (pid->cfg.d_on_measurement != 0U) {
        /* Temporarily treat error directly to preserve semantics. */
        pid_controller_t *p = pid;
        float p_term = p->cfg.kp * error;
        float d_raw = 0.0f;
        float d_term = 0.0f;
        float u_unsat = 0.0f;
        float u_sat = 0.0f;
        if ((p->cfg.kd != 0.0f) && (p->initialized != 0U)) {
            d_raw = (error - p->prev_error) / dt;
            if (p->cfg.n > 0.0f) {
                const float alpha = (dt * p->cfg.n) / (1.0f + dt * p->cfg.n);
                p->d_filt += (d_raw - p->d_filt) * alpha;
                d_term = p->cfg.kd * p->d_filt;
            } else {
                d_term = p->cfg.kd * d_raw;
            }
        }
        u_unsat = p_term + p->integrator + d_term;
        u_sat = (u_unsat < p->cfg.out_min)
                    ? p->cfg.out_min
                    : ((u_unsat > p->cfg.out_max) ? p->cfg.out_max : u_unsat);
        p->saturated = (u_sat != u_unsat) ? 1U : 0U;
        {
            const uint8_t can_integrate =
                (p->saturated == 0U) ||
                ((u_unsat > p->cfg.out_max) && (error < 0.0f)) ||
                ((u_unsat < p->cfg.out_min) && (error > 0.0f));
            if (can_integrate != 0U) {
                p->integrator += p->cfg.ki * error * dt;
            }
            if (p->cfg.kt > 0.0f) {
                p->integrator += p->cfg.kt * (u_sat - u_unsat) * dt;
            }
            if (p->integrator < p->cfg.int_min) {
                p->integrator = p->cfg.int_min;
            }
            if (p->integrator > p->cfg.int_max) {
                p->integrator = p->cfg.int_max;
            }
            u_unsat = p_term + p->integrator + d_term;
            u_sat = (u_unsat < p->cfg.out_min)
                        ? p->cfg.out_min
                        : ((u_unsat > p->cfg.out_max) ? p->cfg.out_max : u_unsat);
            p->saturated = (u_sat != u_unsat) ? 1U : 0U;
        }
        p->prev_error = error;
        p->initialized = 1U;
        return u_sat;
    }
    return pid_update(pid, error, 0.0f, dt);
}

uint8_t pid_is_saturated(const pid_controller_t *pid)
{
    if (pid == NULL) {
        return 0U;
    }
    return pid->saturated;
}

float pid_get_integrator(const pid_controller_t *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }
    return pid->integrator;
}
