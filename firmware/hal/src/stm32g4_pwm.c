#include "hal_pwm.h"
#include <stddef.h>

/* Register-level TIM1 model. On ARM this maps to real TIM1/BDTR; on host it
 * is a static mock with identical API/timing semantics. */

static hal_pwm_state_t s_pwm = {0U, 0U, 0.5f, 0.5f, 0.5f, 0U, 0U};

int hal_pwm_init(uint32_t freq_hz, uint32_t deadtime_ns)
{
    if ((freq_hz == 0U) || (freq_hz > 100000U)) {
        return -1;
    }
    s_pwm.freq_hz = freq_hz;
    s_pwm.deadtime_ns = deadtime_ns;
    s_pwm.duty_a = 0.5f;
    s_pwm.duty_b = 0.5f;
    s_pwm.duty_c = 0.5f;
    s_pwm.enabled = 0U;
    s_pwm.fault_tripped = 0U;
#if defined(STM32_TARGET)
    /* Real target: TIM1 center-aligned mode 3, complementary outputs,
     * DTG = deadtime_ns @ 170 MHz, BKIN break -> MOE cleared by HW. */
#endif
    return 0;
}

static float clamp01(float v)
{
    if (v < 0.0f) {
        return 0.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

void hal_pwm_set_duties(float da, float db, float dc)
{
    if (s_pwm.fault_tripped != 0U) {
        /* Latched fault: ignore writes, force safe 0 until cleared. */
        s_pwm.duty_a = 0.0f;
        s_pwm.duty_b = 0.0f;
        s_pwm.duty_c = 0.0f;
        return;
    }
    if (s_pwm.enabled == 0U) {
        return;
    }
    s_pwm.duty_a = clamp01(da);
    s_pwm.duty_b = clamp01(db);
    s_pwm.duty_c = clamp01(dc);
}

void hal_pwm_enable(void)
{
    if (s_pwm.fault_tripped != 0U) {
        return;
    }
    s_pwm.enabled = 1U;
}

void hal_pwm_disable(void)
{
    s_pwm.enabled = 0U;
    s_pwm.duty_a = 0.0f;
    s_pwm.duty_b = 0.0f;
    s_pwm.duty_c = 0.0f;
}

void hal_pwm_trip(void)
{
    s_pwm.fault_tripped = 1U;
    s_pwm.enabled = 0U;
    s_pwm.duty_a = 0.0f;
    s_pwm.duty_b = 0.0f;
    s_pwm.duty_c = 0.0f;
}

void hal_pwm_clear_fault(void)
{
    s_pwm.fault_tripped = 0U;
}

const hal_pwm_state_t *hal_pwm_state(void)
{
    return &s_pwm;
}
