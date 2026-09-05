#include <gtest/gtest.h>

extern "C" {
#include "pid_controller.h"
}

namespace {
pid_config_t DefaultCfg(float out_min = -14.0f, float out_max = 14.0f)
{
    pid_config_t c{};
    c.kp = 4.712f;
    c.ki = 3298.0f;
    c.kd = 0.0f;
    c.n = 0.0f;
    c.out_min = out_min;
    c.out_max = out_max;
    c.int_min = out_min;
    c.int_max = out_max;
    c.kt = 0.0f;
    c.d_on_measurement = 0;
    return c;
}
} // namespace

TEST(Pid, ZeroSteadyStateError)
{
    // PI on first-order plant P(s)=1/(Ls+R): step must converge, no offset.
    pid_controller_t pid{};
    pid_config_t cfg = DefaultCfg();
    pid_init(&pid, &cfg);
    const float dt = 1.0f / 25000.0f;
    const float R = 0.35f;
    const float L = 0.0005f;
    float i = 0.0f;
    const float ref = 5.0f;
    for (int k = 0; k < 25000; k++) {
        const float v = pid_update(&pid, ref, i, dt);
        // Plant Euler step: L di/dt = v - R i.
        i += dt / L * (v - R * i);
    }
    EXPECT_NEAR(i, ref, 0.02f);
}

TEST(Pid, IntegratorFreezeUnderSaturation)
{
    pid_controller_t pid{};
    pid_config_t cfg = DefaultCfg(-1.0f, 1.0f);
    pid_init(&pid, &cfg);
    const float dt = 1.0f / 25000.0f;
    // Huge persistent error: output pinned at +1.
    float u = 0.0f;
    for (int k = 0; k < 5000; k++) {
        u = pid_update(&pid, 100.0f, 0.0f, dt);
    }
    EXPECT_NEAR(u, 1.0f, 1e-6f);
    const float frozen = pid_get_integrator(&pid);
    // Keep saturating: integrator must not run away (bounded by int clamp
    // and conditional freeze).
    for (int k = 0; k < 5000; k++) {
        u = pid_update(&pid, 100.0f, 0.0f, dt);
    }
    EXPECT_NEAR(u, 1.0f, 1e-6f);
    EXPECT_NEAR(pid_get_integrator(&pid), frozen, 1e-3f);
    EXPECT_TRUE(pid_is_saturated(&pid));
    // Reverse error: must unwind immediately (no windup delay symptom:
    // output leaves rail within a few steps).
    int steps_to_unstick = -1;
    for (int k = 0; k < 25000; k++) {
        u = pid_update(&pid, -100.0f, 0.0f, dt);
        if (u < 0.99f) {
            steps_to_unstick = k;
            break;
        }
    }
    EXPECT_GT(steps_to_unstick, -1);
    EXPECT_LT(steps_to_unstick, 50);
}

TEST(Pid, DerivativeFilterStability)
{
    pid_controller_t pid{};
    pid_config_t cfg = DefaultCfg();
    cfg.kd = 0.001f;
    cfg.n = 2000.0f;
    cfg.d_on_measurement = 1;
    pid_init(&pid, &cfg);
    const float dt = 1.0f / 25000.0f;
    float meas = 0.0f;
    // Noisy measurement staircase; output must stay bounded.
    for (int k = 0; k < 5000; k++) {
        meas = (k % 2 == 0) ? 1.0f : 1.001f;
        const float u = pid_update(&pid, 1.0f, meas, dt);
        EXPECT_TRUE(u == u); // not NaN
        EXPECT_LT(u, 1e3f);
        EXPECT_GT(u, -1e3f);
    }
}
