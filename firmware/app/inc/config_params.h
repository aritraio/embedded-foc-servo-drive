#ifndef FOC_CONFIG_PARAMS_H
#define FOC_CONFIG_PARAMS_H

/* Central type-safe physical configuration. All gains/units documented.
 * Zero heap allocation: every table below is a compile-time constant. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Loop rates (Hz) ---------- */
#define FOC_CURRENT_LOOP_HZ   (25000U)
#define FOC_VELOCITY_LOOP_HZ  (2500U)
#define FOC_POSITION_LOOP_HZ  (1000U)
#define FOC_TELEMETRY_HZ      (1000U)

#define FOC_DT_CURRENT   (1.0f / 25000.0f)   /* 40 us  */
#define FOC_DT_VELOCITY  (1.0f / 2500.0f)    /* 400 us */
#define FOC_DT_POSITION  (1.0f / 1000.0f)    /* 1 ms   */

/* ---------- Nominal PMSM (small gimbal / servo, e.g. GBM2804 class) ---------- */
#define MOTOR_RS_OHM        (0.35f)      /* phase resistance            */
#define MOTOR_LD_H          (0.00050f)   /* d-axis inductance  500 uH   */
#define MOTOR_LQ_H          (0.00050f)   /* q-axis inductance  500 uH   */
#define MOTOR_LAMBDA_PM_VS  (0.0080f)    /* PM flux linkage  8 mWb      */
#define MOTOR_POLE_PAIRS    (7U)         /* electrical = P * mechanical */
#define MOTOR_INERTIA_KGM2  (0.00002f)   /* rotor + light load          */
#define MOTOR_FRICTION_NMS  (0.00001f)   /* viscous damping B           */
#define MOTOR_VDC_NOM_V     (24.0f)      /* nominal DC bus              */
#define MOTOR_I_RATED_A     (5.0f)       /* rated phase current         */
#define MOTOR_I_MAX_A       (10.0f)      /* overcurrent trip threshold  */
#define MOTOR_VBUS_OV_V     (30.0f)      /* bus overvoltage trip        */
#define MOTOR_VBUS_UV_V     (10.0f)      /* bus undervoltage trip       */

/* ---------- Current-loop PI gains (pole-zero cancellation + BW target) ----------
 * Bandwidth ~1.5 kHz: Kp = L * w_bw, Ki = R * w_bw, w_bw = 2*pi*1500. */
#define ID_KP  (4.712f)
#define ID_KI  (3298.0f)
#define IQ_KP  (4.712f)
#define IQ_KI  (3298.0f)
#define ID_V_MAX_V (14.0f)
#define IQ_V_MAX_V (14.0f)

/* ---------- Velocity loop (2.5 kHz) ---------- */
#define VEL_KP  (0.060f)
#define VEL_KI  (1.200f)
#define VEL_OUT_MAX_A (8.0f)   /* iq* clamp */

/* ---------- Position loop (1 kHz) ---------- */
#define POS_KP  (28.0f)        /* 1/s: vel_cmd = Kp * pos_err */
#define POS_VEL_FF (1.0f)      /* velocity feed-forward gain  */
#define POS_VEL_MAX_RAD_S (60.0f)

/* ---------- S-curve limits ---------- */
#define TRAJ_V_MAX_RAD_S (40.0f)
#define TRAJ_A_MAX_RAD_S2 (400.0f)
#define TRAJ_J_MAX_RAD_S3 (4000.0f)

/* ---------- Inverter / sensing ---------- */
#define PWM_FREQ_HZ        (25000U)
#define PWM_DEADTIME_NS    (100U)
#define ADC_FULLSCALE_A    (20.0f)   /* ±20 A maps to 12-bit ADC */
#define ENCODER_BITS       (14U)
#define ENCODER_CPR        (16384U)

/* ---------- Safety ---------- */
#define FAULT_OC_TRIP_A      (10.0f)
#define FAULT_OV_TRIP_V      (30.0f)
#define FAULT_UV_TRIP_V      (10.0f)
#define FAULT_TEMP_TRIP_C    (85.0f)
#define FAULT_ENC_TIMEOUT_MS (10U)

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONFIG_PARAMS_H */
