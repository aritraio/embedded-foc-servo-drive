#include "foc_app.h"
#include "config_params.h"

#include "hal_pwm.h"

#include <math.h>
#include <stddef.h>

void foc_app_init(foc_app_t *app)
{
    foc_motor_params_t mp;

    if (app == NULL) {
        return;
    }
    mp.rs = MOTOR_RS_OHM;
    mp.ld = MOTOR_LD_H;
    mp.lq = MOTOR_LQ_H;
    mp.lambda_pm = MOTOR_LAMBDA_PM_VS;
    mp.pole_pairs = MOTOR_POLE_PAIRS;
    mp.vdc_nom = MOTOR_VDC_NOM_V;
    mp.theta_offset = 0.0f;

    foc_core_init(&app->current, &mp, ID_KP, ID_KI, IQ_KP, IQ_KI, ID_V_MAX_V);
    vel_loop_init(&app->velocity, VEL_KP, VEL_KI, VEL_OUT_MAX_A);
    app->position.kp = POS_KP;
    app->position.ff_gain = POS_VEL_FF;
    app->position.vel_max = POS_VEL_MAX_RAD_S;
    dob_init(&app->dob, MOTOR_INERTIA_KGM2, MOTOR_FRICTION_NMS, 120.0f);

    app->traj.valid = 0U;
    app->traj.t_total = 0.0f;
    app->traj_t = 0.0f;
    app->traj_active = 0U;
    app->pos_ref = 0.0f;
    app->vel_ref = 0.0f;
    app->acc_ref = 0.0f;
    app->theta_mech = 0.0f;
    app->omega_mech = 0.0f;
    app->vbus = MOTOR_VDC_NOM_V;
    app->temperature_c = 25.0f;
    app->state = FOC_STATE_INIT;
    app->fault_bits = (uint32_t)FOC_FAULT_NONE;
    app->tick_fast = 0U;
    app->tick_vel = 0U;
    app->tick_pos = 0U;
    app->enc_fail_count = 0U;
    telemetry_init(&app->telem);
}

void foc_app_request_calibrate(foc_app_t *app)
{
    if (app == NULL) {
        return;
    }
    if (app->state == FOC_STATE_INIT) {
        app->state = FOC_STATE_CALIB;
    }
}

void foc_app_command_position(foc_app_t *app, float q_target)
{
    traj_limits_t lim;

    if (app == NULL) {
        return;
    }
    lim.v_max = TRAJ_V_MAX_RAD_S;
    lim.a_max = TRAJ_A_MAX_RAD_S2;
    lim.j_max = TRAJ_J_MAX_RAD_S3;
    scurve_plan(&app->traj, app->pos_ref, q_target, &lim);
    app->traj_t = 0.0f;
    app->traj_active = 1U;
    if (app->state == FOC_STATE_INIT) {
        app->state = FOC_STATE_RUN;
    }
}

void foc_app_command_velocity(foc_app_t *app, float w_target)
{
    if (app == NULL) {
        return;
    }
    app->traj_active = 0U;
    app->vel_ref = w_target;
    if (app->state == FOC_STATE_INIT) {
        app->state = FOC_STATE_RUN;
    }
}

void foc_app_command_current(foc_app_t *app, float id_ref, float iq_ref)
{
    if (app == NULL) {
        return;
    }
    app->traj_active = 0U;
    foc_core_set_refs(&app->current, id_ref, iq_ref);
    if (app->state == FOC_STATE_INIT) {
        app->state = FOC_STATE_RUN;
    }
}

void foc_app_step_fast(foc_app_t *app, float ia, float ib, float theta_mech,
                       float omega_e, float vbus, float dt)
{
    if ((app == NULL) || (dt <= 0.0f)) {
        return;
    }
    app->theta_mech = theta_mech;
    app->omega_mech = omega_e / (float)MOTOR_POLE_PAIRS;
    app->vbus = vbus;
    if (app->state == FOC_STATE_FAULT) {
        return; /* outputs frozen, PWM already tri-stated */
    }
    foc_core_step(&app->current, ia, ib, theta_mech, omega_e, vbus, dt);
    hal_pwm_set_duties(app->current.svpwm.ta, app->current.svpwm.tb,
                       app->current.svpwm.tc);
    app->tick_fast++;
}

void foc_app_step_velocity(foc_app_t *app, float dt)
{
    float iq_cmd = 0.0f;
    float te_est = 0.0f;

    if ((app == NULL) || (dt <= 0.0f)) {
        return;
    }
    if (app->state != FOC_STATE_RUN) {
        return;
    }
    /* Disturbance observer stiffness feed-forward. */
    te_est = 1.5f * (float)MOTOR_POLE_PAIRS * MOTOR_LAMBDA_PM_VS * app->current.iq;
    dob_update(&app->dob, te_est, app->omega_mech, dt);
    {
        /* Convert estimated load torque to iq feed-forward: iq_ff = Tl/Kt. */
        const float kt = 1.5f * (float)MOTOR_POLE_PAIRS * MOTOR_LAMBDA_PM_VS;
        const float iq_ff = (kt > 1e-9f) ? (dob_get_torque(&app->dob) / kt) : 0.0f;
        iq_cmd = vel_loop_step(&app->velocity, app->vel_ref, app->omega_mech,
                               app->acc_ref * 0.0f + iq_ff * 0.05f, dt);
    }
    foc_core_set_refs(&app->current, 0.0f, iq_cmd);
    app->tick_vel++;
}

void foc_app_step_position(foc_app_t *app, float dt)
{
    if ((app == NULL) || (dt <= 0.0f)) {
        return;
    }
    if (app->state != FOC_STATE_RUN) {
        return;
    }
    if (app->traj_active != 0U) {
        traj_point_t pt = scurve_sample(&app->traj, app->traj_t);
        app->pos_ref = pt.pos;
        app->vel_ref = pos_loop_step(&app->position, pt.pos, app->theta_mech, pt.vel);
        app->acc_ref = pt.acc;
        app->traj_t += dt;
        if (scurve_is_done(&app->traj, app->traj_t) != 0U) {
            app->traj_active = 0U;
            app->vel_ref = 0.0f;
            app->acc_ref = 0.0f;
        }
    } else {
        app->vel_ref = pos_loop_step(&app->position, app->pos_ref, app->theta_mech, 0.0f);
    }
    app->tick_pos++;
}

uint32_t foc_app_supervisor(foc_app_t *app, float ia, float ib, float vbus,
                            uint8_t encoder_ok)
{
    uint32_t faults = (uint32_t)FOC_FAULT_NONE;
    float ic = 0.0f;

    if (app == NULL) {
        return faults;
    }
    ic = -(ia + ib);

    if ((fabsf(ia) > FAULT_OC_TRIP_A) || (fabsf(ib) > FAULT_OC_TRIP_A) ||
        (fabsf(ic) > FAULT_OC_TRIP_A)) {
        faults |= (uint32_t)FOC_FAULT_OVERCURRENT;
    }
    if (vbus > FAULT_OV_TRIP_V) {
        faults |= (uint32_t)FOC_FAULT_BUS_OV;
    }
    if (vbus < FAULT_UV_TRIP_V) {
        faults |= (uint32_t)FOC_FAULT_BUS_UV;
    }
    if (app->temperature_c > FAULT_TEMP_TRIP_C) {
        faults |= (uint32_t)FOC_FAULT_OVERTEMP;
    }
    if (encoder_ok == 0U) {
        app->enc_fail_count++;
        if (app->enc_fail_count > 5U) {
            faults |= (uint32_t)FOC_FAULT_ENCODER;
        }
    } else {
        app->enc_fail_count = 0U;
    }

    if (faults != (uint32_t)FOC_FAULT_NONE) {
        app->fault_bits |= faults;
        app->state = FOC_STATE_FAULT;
        hal_pwm_trip(); /* <1 us path on HW via comparator+break; SW latch here */
    }
    return faults;
}

void foc_app_clear_fault(foc_app_t *app)
{
    if (app == NULL) {
        return;
    }
    app->fault_bits = (uint32_t)FOC_FAULT_NONE;
    app->enc_fail_count = 0U;
    foc_core_reset(&app->current);
    vel_loop_reset(&app->velocity);
    dob_reset(&app->dob);
    hal_pwm_clear_fault();
    app->state = FOC_STATE_INIT;
}

const char *foc_app_state_name(foc_state_t s)
{
    switch (s) {
    case FOC_STATE_INIT:
        return "INIT";
    case FOC_STATE_CALIB:
        return "CALIB";
    case FOC_STATE_RUN:
        return "RUN";
    case FOC_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
