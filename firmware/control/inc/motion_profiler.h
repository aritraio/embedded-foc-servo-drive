#ifndef FOC_MOTION_PROFILER_H
#define FOC_MOTION_PROFILER_H

/* 7-segment jerk-bounded S-curve trajectory generator.
 * Symmetric profile: Tj (jerk phases) + Ta (const accel) + Tv (cruise).
 * Re-planned per move; sampled at the position-loop rate with no heap. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float v_max;
    float a_max;
    float j_max;
} traj_limits_t;

typedef struct {
    float q0;       /* start position [rad] */
    float q1;       /* goal position [rad]  */
    float tj;       /* jerk phase duration [s] */
    float ta;       /* constant-accel duration [s] */
    float tv;       /* cruise duration [s] */
    float t_total;  /* 4*Tj + 2*Ta + Tv */
    float dir;      /* +1 / -1 */
    float dist;     /* |q1-q0| */
    float v_peak;   /* reached peak velocity */
    float a_peak;   /* reached peak accel */
    uint8_t valid;
} scurve_plan_t;

typedef struct {
    float pos;
    float vel;
    float acc;
} traj_point_t;

void scurve_plan(scurve_plan_t *plan, float q0, float q1,
                 const traj_limits_t *limits);
traj_point_t scurve_sample(const scurve_plan_t *plan, float t);
uint8_t scurve_is_done(const scurve_plan_t *plan, float t);

/* Cascaded outer loops (stateless helpers; caller holds integrators). */
typedef struct {
    float kp;
    float ff_gain;
    float vel_max;
} pos_loop_config_t;

typedef struct {
    float kp;
    float ki;
    float out_max;
    float integrator;
    float int_max;
} vel_loop_state_t;

void vel_loop_init(vel_loop_state_t *st, float kp, float ki, float out_max);
float vel_loop_step(vel_loop_state_t *st, float vel_ref, float vel_meas,
                    float acc_ff, float dt);
void vel_loop_reset(vel_loop_state_t *st);

float pos_loop_step(const pos_loop_config_t *cfg, float pos_ref, float pos_meas,
                    float vel_ff);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MOTION_PROFILER_H */
