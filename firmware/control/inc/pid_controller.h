#ifndef FOC_PID_CONTROLLER_H
#define FOC_PID_CONTROLLER_H

/* Parallel-form PI/PID with derivative LPF + conditional-integration and
 * back-calculation anti-windup. No heap, no statics; all state in struct. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float n;          /* derivative filter coefficient (rad/s), 0 = unfiltered */
    float out_min;
    float out_max;
    float int_min;    /* integrator clamp bounds */
    float int_max;
    float kt;         /* back-calculation gain (1/s); 0 disables back-calc */
    uint8_t d_on_measurement; /* 1: derivative on measurement (no kick) */
} pid_config_t;

typedef struct {
    pid_config_t cfg;
    float integrator;
    float prev_error;
    float prev_meas;
    float d_filt;
    uint8_t saturated;
    uint8_t initialized;
} pid_controller_t;

void pid_init(pid_controller_t *pid, const pid_config_t *cfg);
void pid_reset(pid_controller_t *pid);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/* setpoint/measurement form (preferred: enables derivative-on-measurement). */
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt);
/* error-direct form (derivative on error). */
float pid_update_error(pid_controller_t *pid, float error, float dt);

uint8_t pid_is_saturated(const pid_controller_t *pid);
float pid_get_integrator(const pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PID_CONTROLLER_H */
