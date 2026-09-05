#include "motion_profiler.h"

#include <math.h>
#include <stddef.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void scurve_plan(scurve_plan_t *plan, float q0, float q1,
                 const traj_limits_t *limits)
{
    float dist = 0.0f;
    float dir = 1.0f;
    float v_max = 0.0f;
    float a_max = 0.0f;
    float j_max = 0.0f;
    float tj = 0.0f;
    float ta = 0.0f;
    float tv = 0.0f;
    float v_reach = 0.0f;

    if ((plan == NULL) || (limits == NULL)) {
        return;
    }
    plan->valid = 0U;

    v_max = limits->v_max > 0.0f ? limits->v_max : 1.0f;
    a_max = limits->a_max > 0.0f ? limits->a_max : 1.0f;
    j_max = limits->j_max > 0.0f ? limits->j_max : 1.0f;

    dist = q1 - q0;
    dir = (dist >= 0.0f) ? 1.0f : -1.0f;
    dist = fabsf(dist);

    if (dist < 1e-9f) {
        plan->q0 = q0;
        plan->q1 = q1;
        plan->tj = 0.0f;
        plan->ta = 0.0f;
        plan->tv = 0.0f;
        plan->t_total = 0.0f;
        plan->dir = 1.0f;
        plan->dist = 0.0f;
        plan->v_peak = 0.0f;
        plan->a_peak = 0.0f;
        plan->valid = 1U;
        return;
    }

    /* Phase 1: can Amax be reached given Vmax? Need Vmax*J >= Amax^2. */
    if ((v_max * j_max) < (a_max * a_max)) {
        /* Amax unreachable: Tj = sqrt(Vmax/J), Ta = 0. */
        tj = sqrtf(v_max / j_max);
        ta = 0.0f;
        v_reach = v_max;
    } else {
        tj = a_max / j_max;
        /* Velocity gained during accel ramp to Amax and back with Ta=0:
         * v_ramp = Amax^2 / J. Remaining accel time to hit Vmax: */
        ta = (v_max / a_max) - tj;
        if (ta < 0.0f) {
            ta = 0.0f;
        }
        v_reach = v_max;
    }

    /* Distance covered ramping 0 -> v_reach -> 0 (accel + decel, no cruise):
     * For S-curve accel phase with Tj,Ta: distance_accel = v_reach*(2*Tj+Ta)/... */
    {
        /* Velocity at end of accel phase: v = a_peak*(Ta+Tj)... derive:
         * With Tj,Ta: v_end_accel = a_peak*(Ta + Tj) where a_peak = J*Tj.
         * Distance for accel+decel phases combined (symmetric): */
        const float a_peak = j_max * tj;
        const float v_end = a_peak * (ta + tj);
        /* If v_end overshoots v_reach due to numeric path, clamp Ta. */
        if (v_end > v_reach) {
            /* Reduce Ta (already >= 0). */
            ta = (v_reach / a_peak) - tj;
            if (ta < 0.0f) {
                ta = 0.0f;
                /* Reduce Tj to hit v_reach with triangular accel:
                 * v = J*Tj^2 -> Tj = sqrt(v/J). */
                tj = sqrtf(v_reach / j_max);
            }
        }
    }

    /* Recompute distance with final tj,ta and check cruise need. */
    {
        const float a_peak = j_max * tj;
        const float v_end = a_peak * (ta + tj);
        /* Distance during one accel (0->v_end): s_acc = v_end*(Ta/2 + Tj).
         * Total accel+decel (no cruise) = 2*s_acc. */
        const float s_acc = v_end * (ta * 0.5f + tj);
        const float s_min = 2.0f * s_acc;
        if (dist >= s_min) {
            v_reach = v_end;
            tv = (dist - s_min) / v_reach;
        } else {
            /* Triangular: reduce peak velocity. Solve for Tj (Ta=0 case) or Ta.
             * General with Ta unknown is cubic; iterate: fix Tj at min(tj,
             * sqrt-based) and solve quadratic for Ta, else shrink Tj.
             * Simpler robust path: binary-search v_peak in (0, v_end]. */
            float lo = 0.0f;
            float hi = v_end;
            v_reach = v_end * 0.5f;
            for (int i = 0; i < 48; i++) {
                const float mid = (lo + hi) * 0.5f;
                /* For candidate vm: determine Tj',Ta' that reach vm with
                 * same J,Amax limits: */
                float tjc = tj;
                float tac = 0.0f;
                if ((mid * j_max) < (a_max * a_max)) {
                    tjc = sqrtf(mid / j_max);
                    tac = 0.0f;
                } else {
                    tjc = a_max / j_max;
                    tac = (mid / a_max) - tjc;
                    if (tac < 0.0f) {
                        tac = 0.0f;
                    }
                }
                const float ap = j_max * tjc;
                const float ve = ap * (tac + tjc);
                const float sa = ve * (tac * 0.5f + tjc);
                const float tot = 2.0f * sa;
                if (tot > dist) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            v_reach = (lo + hi) * 0.5f;
            if ((v_reach * j_max) < (a_max * a_max)) {
                tj = sqrtf(v_reach / j_max);
                ta = 0.0f;
            } else {
                tj = a_max / j_max;
                ta = (v_reach / a_max) - tj;
                if (ta < 0.0f) {
                    ta = 0.0f;
                }
            }
            tv = 0.0f;
        }
    }

    plan->q0 = q0;
    plan->q1 = q1;
    plan->tj = tj;
    plan->ta = ta;
    plan->tv = tv;
    plan->t_total = 4.0f * tj + 2.0f * ta + tv;
    plan->dir = dir;
    plan->dist = dist;
    plan->v_peak = v_reach;
    plan->a_peak = j_max * tj;
    plan->valid = 1U;
}

/* Distance covered during accel ramp 0..t within first accel half.
 * Piecewise with jerk J for Tj, const accel, then -J. We evaluate the
 * full profile by integrating jerk segments analytically via sampling
 * approach: build (t, pos) using closed-form per segment. */
traj_point_t scurve_sample(const scurve_plan_t *plan, float t)
{
    traj_point_t out = {0.0f, 0.0f, 0.0f};
    float tj = 0.0f;
    float ta = 0.0f;
    float tv = 0.0f;
    float tt = 0.0f;
    float j = 0.0f;
    float a_pk = 0.0f;
    float v_pk = 0.0f;

    if ((plan == NULL) || (plan->valid == 0U)) {
        return out;
    }
    if (plan->t_total <= 0.0f) {
        out.pos = plan->q1;
        return out;
    }
    tj = plan->tj;
    ta = plan->ta;
    tv = plan->tv;
    a_pk = plan->a_peak;
    /* Recompute consistent jerk from a_peak/tj. */
    j = (tj > 1e-9f) ? (a_pk / tj) : 0.0f;
    v_pk = plan->v_peak;

    if (t <= 0.0f) {
        out.pos = plan->q0;
        return out;
    }
    if (t >= plan->t_total) {
        out.pos = plan->q1;
        out.vel = 0.0f;
        out.acc = 0.0f;
        return out;
    }

    /* Segment boundaries (accel phase, cruise, decel phase). */
    {
        const float T1 = tj;              /* +J ramp up   */
        const float T2 = tj + ta;         /* const accel  */
        const float T3 = 2.0f * tj + ta;  /* -J ramp down */
        const float T4 = T3 + tv;         /* cruise       */
        const float T5 = T4 + tj;         /* -J ramp down (decel) */
        const float T6 = T5 + ta;         /* const decel  */
        /* T7 = T6 + tj = t_total */
        float s = 0.0f; /* distance from q0 (positive) */
        float v = 0.0f;
        float a = 0.0f;

        /* Precompute phase-end states. */
        const float v1 = j * tj * tj * 0.5f;
        const float s1 = j * tj * tj * tj / 6.0f;
        const float v2 = v1 + a_pk * ta;
        const float s2 = s1 + v1 * ta + a_pk * ta * ta * 0.5f;
        const float v3 = v2 + a_pk * tj - j * tj * tj * 0.5f; /* == v_pk */
        const float s3 = s2 + v2 * tj + a_pk * tj * tj * 0.5f - j * tj * tj * tj / 6.0f;
        const float s4 = s3 + v3 * tv;
        /* Decel mirror. */
        const float s5 = s4 + v3 * tj - j * tj * tj * tj / 6.0f;
        const float v5 = v3 - j * tj * tj * 0.5f;
        const float s6 = s5 + v5 * ta - a_pk * ta * ta * 0.5f;
        /* s7 = dist */
        (void)v_pk;

        if (t < T1) {
            tt = t;
            a = j * tt;
            v = j * tt * tt * 0.5f;
            s = j * tt * tt * tt / 6.0f;
        } else if (t < T2) {
            tt = t - T1;
            a = a_pk;
            v = v1 + a_pk * tt;
            s = s1 + v1 * tt + a_pk * tt * tt * 0.5f;
        } else if (t < T3) {
            tt = t - T2;
            a = a_pk - j * tt;
            v = v2 + a_pk * tt - j * tt * tt * 0.5f;
            s = s2 + v2 * tt + a_pk * tt * tt * 0.5f - j * tt * tt * tt / 6.0f;
        } else if (t < T4) {
            tt = t - T3;
            a = 0.0f;
            v = v3;
            s = s3 + v3 * tt;
        } else if (t < T5) {
            tt = t - T4;
            a = -j * tt;
            v = v3 - j * tt * tt * 0.5f;
            s = s4 + v3 * tt - j * tt * tt * tt / 6.0f;
        } else if (t < T6) {
            tt = t - T5;
            a = -a_pk;
            v = v5 - a_pk * tt;
            s = s5 + v5 * tt - a_pk * tt * tt * 0.5f;
        } else {
            tt = t - T6;
            a = -a_pk + j * tt;
            v = (v5 - a_pk * ta) + (-a_pk) * tt + j * tt * tt * 0.5f;
            /* v at T6 = v5 - a_pk*ta; then ramp back to 0. */
            const float v6 = v5 - a_pk * ta;
            v = v6 - a_pk * tt + j * tt * tt * 0.5f;
            s = s6 + v6 * tt - a_pk * tt * tt * 0.5f + j * tt * tt * tt / 6.0f;
            if (v < 0.0f) {
                v = 0.0f;
            }
        }
        if (s > plan->dist) {
            s = plan->dist;
        }
        out.pos = plan->q0 + plan->dir * s;
        out.vel = plan->dir * v;
        out.acc = plan->dir * a;
        return out;
    }
}

uint8_t scurve_is_done(const scurve_plan_t *plan, float t)
{
    if ((plan == NULL) || (plan->valid == 0U)) {
        return 1U;
    }
    return (t >= plan->t_total) ? 1U : 0U;
}

void vel_loop_init(vel_loop_state_t *st, float kp, float ki, float out_max)
{
    if (st == NULL) {
        return;
    }
    st->kp = kp;
    st->ki = ki;
    st->out_max = (out_max > 0.0f) ? out_max : 1.0f;
    st->integrator = 0.0f;
    st->int_max = st->out_max;
}

float vel_loop_step(vel_loop_state_t *st, float vel_ref, float vel_meas,
                    float acc_ff, float dt)
{
    float err = 0.0f;
    float p = 0.0f;
    float u = 0.0f;

    if ((st == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }
    err = vel_ref - vel_meas;
    p = st->kp * err;

    /* Conditional integration anti-windup. */
    u = p + st->integrator + acc_ff;
    {
        float sat = clampf(u, -st->out_max, st->out_max);
        const uint8_t saturated = (sat != u) ? 1U : 0U;
        const uint8_t can_int =
            (saturated == 0U) || ((u > st->out_max) && (err < 0.0f)) ||
            ((u < -st->out_max) && (err > 0.0f));
        if (can_int != 0U) {
            st->integrator += st->ki * err * dt;
            st->integrator = clampf(st->integrator, -st->int_max, st->int_max);
        }
        u = clampf(p + st->integrator + acc_ff, -st->out_max, st->out_max);
    }
    return u;
}

void vel_loop_reset(vel_loop_state_t *st)
{
    if (st == NULL) {
        return;
    }
    st->integrator = 0.0f;
}

float pos_loop_step(const pos_loop_config_t *cfg, float pos_ref, float pos_meas,
                    float vel_ff)
{
    float err = 0.0f;
    float v = 0.0f;

    if (cfg == NULL) {
        return 0.0f;
    }
    err = pos_ref - pos_meas;
    v = cfg->kp * err + cfg->ff_gain * vel_ff;
    return clampf(v, -cfg->vel_max, cfg->vel_max);
}
