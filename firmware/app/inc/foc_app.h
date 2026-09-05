#ifndef FOC_APP_H
#define FOC_APP_H

/* Top-level state machine + fault supervisor + multi-rate scheduler.
 * States: INIT -> CALIB -> RUN <-> FAULT (latched until cleared). */

#include "disturbance_observer.h"
#include "foc_core.h"
#include "motion_profiler.h"
#include "telemetry.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_STATE_INIT = 0,
    FOC_STATE_CALIB,
    FOC_STATE_RUN,
    FOC_STATE_FAULT
} foc_state_t;

typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_OVERCURRENT = 1,
    FOC_FAULT_BUS_OV = 2,
    FOC_FAULT_BUS_UV = 4,
    FOC_FAULT_ENCODER = 8,
    FOC_FAULT_OVERTEMP = 16,
    FOC_FAULT_GATE_DRIVER = 32
} foc_fault_t;

typedef struct {
    foc_core_t current;
    vel_loop_state_t velocity;
    pos_loop_config_t position;
    dob_state_t dob;
    scurve_plan_t traj;
    float traj_t;
    uint8_t traj_active;
    float pos_ref;
    float vel_ref;
    float acc_ref;
    float theta_mech;
    float omega_mech;
    float vbus;
    float temperature_c;
    foc_state_t state;
    uint32_t fault_bits;
    uint32_t tick_fast;
    uint32_t tick_vel;
    uint32_t tick_pos;
    uint32_t enc_fail_count;
    telemetry_queue_t telem;
} foc_app_t;

void foc_app_init(foc_app_t *app);
void foc_app_request_calibrate(foc_app_t *app);
void foc_app_command_position(foc_app_t *app, float q_target);
void foc_app_command_velocity(foc_app_t *app, float w_target);
void foc_app_command_current(foc_app_t *app, float id_ref, float iq_ref);

/* Rate steps called from ISRs / scheduler. */
void foc_app_step_fast(foc_app_t *app, float ia, float ib, float theta_mech,
                       float omega_e, float vbus, float dt);
void foc_app_step_velocity(foc_app_t *app, float dt);
void foc_app_step_position(foc_app_t *app, float dt);

/* Safety supervisor: returns nonzero fault bits; trips PWM on fault. */
uint32_t foc_app_supervisor(foc_app_t *app, float ia, float ib, float vbus,
                            uint8_t encoder_ok);
void foc_app_clear_fault(foc_app_t *app);

const char *foc_app_state_name(foc_state_t s);

/* ISR entry points (see isr_handlers.c). */
foc_app_t *isr_app_handle(void);
void isr_control_init(void);
void isr_fast_loop(void);

#ifdef __cplusplus
}
#endif

#endif
