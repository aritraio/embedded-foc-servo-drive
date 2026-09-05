#include <gtest/gtest.h>

extern "C" {
#include "as5048a_encoder.h"
#include "config_params.h"
#include "foc_app.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "hal_spi.h"
}

// Overcurrent must latch PWM tri-state (duties forced to 0, writes ignored).
TEST(HalFault, OvercurrentTripsPwm)
{
    foc_app_t app{};
    foc_app_init(&app);
    ASSERT_EQ(hal_pwm_init(25000, 100), 0);
    hal_pwm_enable();
    hal_pwm_set_duties(0.6f, 0.4f, 0.5f);
    EXPECT_GT(hal_pwm_state()->duty_a, 0.5f);

    const uint32_t f = foc_app_supervisor(&app, 12.0f, 0.0f, 24.0f, 1u);
    EXPECT_NE(f & (uint32_t)FOC_FAULT_OVERCURRENT, 0u);
    EXPECT_EQ(app.state, FOC_STATE_FAULT);
    const hal_pwm_state_t *st = hal_pwm_state();
    EXPECT_EQ(st->duty_a, 0.0f);
    EXPECT_EQ(st->duty_b, 0.0f);
    EXPECT_EQ(st->duty_c, 0.0f);
    // Latched: further duty writes ignored until cleared.
    hal_pwm_set_duties(0.9f, 0.9f, 0.9f);
    EXPECT_EQ(hal_pwm_state()->duty_a, 0.0f);
    // Clear path back to INIT.
    foc_app_clear_fault(&app);
    EXPECT_EQ(app.state, FOC_STATE_INIT);
    hal_pwm_enable();
    hal_pwm_set_duties(0.6f, 0.4f, 0.5f);
    EXPECT_NEAR(hal_pwm_state()->duty_a, 0.6f, 1e-6f);
}

TEST(HalFault, BusVoltageTrips)
{
    foc_app_t app{};
    foc_app_init(&app);
    (void)hal_pwm_init(25000, 100);
    hal_pwm_enable();
    EXPECT_NE(foc_app_supervisor(&app, 0.0f, 0.0f, 33.0f, 1u) & (uint32_t)FOC_FAULT_BUS_OV, 0u);
    EXPECT_EQ(app.state, FOC_STATE_FAULT);
    foc_app_clear_fault(&app);
    EXPECT_NE(foc_app_supervisor(&app, 0.0f, 0.0f, 5.0f, 1u) & (uint32_t)FOC_FAULT_BUS_UV, 0u);
    EXPECT_EQ(app.state, FOC_STATE_FAULT);
    foc_app_clear_fault(&app);
}

TEST(HalFault, EncoderDropoutTripsAfterDebounce)
{
    foc_app_t app{};
    foc_app_init(&app);
    (void)hal_pwm_init(25000, 100);
    // Up to 5 consecutive dropouts: no trip yet (debounce).
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(foc_app_supervisor(&app, 0.0f, 0.0f, 24.0f, 0u) & (uint32_t)FOC_FAULT_ENCODER, 0u);
    }
    EXPECT_NE(foc_app_supervisor(&app, 0.0f, 0.0f, 24.0f, 0u) & (uint32_t)FOC_FAULT_ENCODER, 0u);
    EXPECT_EQ(app.state, FOC_STATE_FAULT);
    foc_app_clear_fault(&app);
}

TEST(Encoder, ParityAndAngleDecode)
{
    as5048a_init();
    // Program mock frame: angle 0x2000 (=180 deg after mask? 8192/16384 = pi).
    const uint16_t frame = as5048a_build_frame(0x2000, 0);
    const uint8_t rx[2] = {(uint8_t)(frame >> 8), (uint8_t)(frame & 0xFF)};
    hal_spi_mock_set_rx(rx, 2);
    as5048a_reading_t r{};
    EXPECT_EQ(as5048a_read(&r), 0);
    EXPECT_EQ(r.parity_ok, 1u);
    EXPECT_EQ(r.raw, 0x2000u);
    EXPECT_NEAR(r.angle_rad, 3.14159265f, 1e-3f);

    // Corrupt parity: must flag fault.
    const uint16_t bad = (uint16_t)(frame ^ 0x8000u);
    const uint8_t rxb[2] = {(uint8_t)(bad >> 8), (uint8_t)(bad & 0xFF)};
    hal_spi_mock_set_rx(rxb, 2);
    EXPECT_NE(as5048a_read(&r), 0);
    EXPECT_EQ(r.parity_ok, 0u);
}
