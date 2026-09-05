#ifndef FOC_HAL_PWM_H
#define FOC_HAL_PWM_H

/* TIM1 center-aligned complementary PWM with dead-time + break.
 * Host builds use a register mock so SIL/tests compile and run. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t freq_hz;
    uint32_t deadtime_ns;
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t enabled;
    uint8_t fault_tripped;
} hal_pwm_state_t;

int hal_pwm_init(uint32_t freq_hz, uint32_t deadtime_ns);
void hal_pwm_set_duties(float da, float db, float dc);
void hal_pwm_enable(void);
void hal_pwm_disable(void);
/* Immediate tri-state (all FETs off) on fault; latches until re-enabled. */
void hal_pwm_trip(void);
void hal_pwm_clear_fault(void);
const hal_pwm_state_t *hal_pwm_state(void);

#ifdef __cplusplus
}
#endif

#endif
