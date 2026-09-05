#include "foc_app.h"
#include "config_params.h"

#include "hal_adc.h"
#include "hal_pwm.h"
#include "as5048a_encoder.h"

/* 25 kHz ADC/DMA complete ISR: sample valley currents + encoder, run fast
 * loop, update PWM compares, decimate to velocity/position/telemetry rates.
 * ISR budget: <1.5 us @ 170 MHz (all static, no heap, no printf). */

static foc_app_t s_app;
static uint8_t s_ready = 0U;

foc_app_t *isr_app_handle(void)
{
    return &s_app;
}

void isr_control_init(void)
{
    foc_app_init(&s_app);
    (void)hal_pwm_init(PWM_FREQ_HZ, PWM_DEADTIME_NS);
    (void)hal_adc_init(ADC_FULLSCALE_A);
    as5048a_init();
    hal_adc_start_dma();
    hal_pwm_enable();
    s_app.state = FOC_STATE_RUN;
    s_ready = 1U;
}

/* Called from TIM1 update / ADC DMA-complete interrupt context. */
void isr_fast_loop(void)
{
    float ia = 0.0f;
    float ib = 0.0f;
    float theta_mech = 0.0f;
    float omega_e = 0.0f;
    as5048a_reading_t enc;

    if (s_ready == 0U) {
        return;
    }
    /* Latest valley-sampled currents (zero CPU copy in HW via DMA). */
    if (hal_adc_read_latest(&ia, &ib) == 0U) {
        ia = 0.0f;
        ib = 0.0f;
    }
    /* Encoder: SPI read <2 us @ 10 MHz; parity fault -> supervisor counts. */
    if (as5048a_read(&enc) == 0) {
        const float steps = 16384.0f;
        theta_mech = ((float)enc.raw / steps) * 6.28318530717958647692f;
    } else {
        theta_mech = s_app.theta_mech; /* hold-last on dropout */
    }
    /* Speed from observer/track: reuse last mechanical speed. */
    omega_e = s_app.omega_mech * (float)MOTOR_POLE_PAIRS;

    (void)foc_app_supervisor(&s_app, ia, ib, s_app.vbus,
                             (enc.parity_ok != 0U) ? 1U : 0U);
    foc_app_step_fast(&s_app, ia, ib, theta_mech, omega_e, s_app.vbus,
                      FOC_DT_CURRENT);

    /* Decimation: 25 kHz -> 2.5 kHz velocity (every 10th), 1 kHz pos (every 25th). */
    if ((s_app.tick_fast % 10U) == 0U) {
        foc_app_step_velocity(&s_app, FOC_DT_VELOCITY);
    }
    if ((s_app.tick_fast % 25U) == 0U) {
        foc_app_step_position(&s_app, FOC_DT_POSITION);
    }
}
