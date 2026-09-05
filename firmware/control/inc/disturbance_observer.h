#ifndef FOC_DISTURBANCE_OBSERVER_H
#define FOC_DISTURBANCE_OBSERVER_H

/* Load-torque disturbance observer: estimates external torque so the
 * velocity loop can add stiffness feed-forward. Discrete Luenberger form. */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float inertia;   /* J [kg m^2] */
    float friction;  /* B [N m s]  */
    float gain;      /* observer gain L [1/s] */
    float t_hat;     /* estimated load torque [N m] */
    float w_hat;     /* estimated velocity [rad/s] */
} dob_state_t;

void dob_init(dob_state_t *dob, float inertia, float friction, float gain);
void dob_reset(dob_state_t *dob);
void dob_update(dob_state_t *dob, float torque_elec, float omega_meas, float dt);
float dob_get_torque(const dob_state_t *dob);

#ifdef __cplusplus
}
#endif

#endif /* FOC_DISTURBANCE_OBSERVER_H */
