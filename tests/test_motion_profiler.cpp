#include <gtest/gtest.h>

extern "C" {
#include "disturbance_observer.h"
#include "motion_profiler.h"
}

#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

TEST(MotionProfiler, FullRotationRespectsBounds)
{
    traj_limits_t lim{40.0f, 400.0f, 4000.0f};
    scurve_plan_t plan{};
    const float target = 2.0f * kPi; // 360 deg
    scurve_plan(&plan, 0.0f, target, &lim);
    ASSERT_EQ(plan.valid, 1u);
    EXPECT_GT(plan.t_total, 0.0f);

    const float dt = 0.001f;
    float prev_vel = 0.0f;
    float prev_acc = 0.0f;
    float max_v = 0.0f, max_a = 0.0f, max_j = 0.0f;
    bool first = true;
    for (float t = 0.0f; t <= plan.t_total + dt; t += dt) {
        traj_point_t p = scurve_sample(&plan, t);
        max_v = std::max(max_v, std::abs(p.vel));
        max_a = std::max(max_a, std::abs(p.acc));
        if (!first) {
            const float j = std::abs((p.acc - prev_acc) / dt);
            // Allow one-sample discretization spike at segment joins.
            if (t < plan.t_total - dt) {
                max_j = std::max(max_j, j);
            }
        }
        first = false;
        prev_vel = p.vel;
        prev_acc = p.acc;
        (void)prev_vel;
    }
    EXPECT_LE(max_v, lim.v_max * 1.02f) << "max_v=" << max_v;
    EXPECT_LE(max_a, lim.a_max * 1.05f) << "max_a=" << max_a;
    EXPECT_LE(max_j, lim.j_max * 2.5f) << "max_j=" << max_j; // discrete-diff margin

    // Endpoint lock: exact target, zero end velocity.
    traj_point_t end = scurve_sample(&plan, plan.t_total);
    EXPECT_NEAR(end.pos, target, 1e-3f);
    EXPECT_NEAR(end.vel, 0.0f, 1e-2f);
    EXPECT_TRUE(scurve_is_done(&plan, plan.t_total));
}

TEST(MotionProfiler, ShortMoveTriangular)
{
    traj_limits_t lim{40.0f, 400.0f, 4000.0f};
    scurve_plan_t plan{};
    scurve_plan(&plan, 0.0f, 0.05f, &lim); // tiny move: must not demand Vmax
    ASSERT_EQ(plan.valid, 1u);
    EXPECT_LT(plan.v_peak, lim.v_max);
    traj_point_t end = scurve_sample(&plan, plan.t_total + 0.01f);
    EXPECT_NEAR(end.pos, 0.05f, 1e-4f);
}

TEST(VelocityLoop, TracksWithFeedforward)
{
    vel_loop_state_t st{};
    vel_loop_init(&st, 0.06f, 1.2f, 8.0f);
    const float dt = 1.0f / 2500.0f;
    // Simulate rotor inertia driven by iq torque: J dw/dt = Kt*iq - B w.
    const float Kt = 1.5f * 7.0f * 0.0080f;
    const float J = 0.00002f;
    const float B = 0.00001f;
    float w = 0.0f;
    const float w_ref = 20.0f;
    for (int k = 0; k < 2500 * 3; k++) {
        const float iq = vel_loop_step(&st, w_ref, w, 0.0f, dt);
        const float dw = (Kt * iq - B * w) / J * dt;
        w += dw;
    }
    EXPECT_NEAR(w, w_ref, 0.5f);
}

TEST(PositionLoop, CascadedStepLocksWithoutRinging)
{
    // End-to-end: S-curve position ref -> P position -> PI velocity ->
    // torque -> inertia. Verify target lock, monotonic-ish approach, no ringing.
    traj_limits_t lim{40.0f, 400.0f, 4000.0f};
    scurve_plan_t plan{};
    scurve_plan(&plan, 0.0f, 2.0f * kPi, &lim);
    pos_loop_config_t pcfg{28.0f, 1.0f, 60.0f};
    vel_loop_state_t vel{};
    vel_loop_init(&vel, 0.06f, 1.2f, 8.0f);
    const float Kt = 1.5f * 7.0f * 0.0080f;
    const float J = 0.0002f;
    const float B = 0.00005f;
    float pos = 0.0f, w = 0.0f;
    const float dtp = 0.001f; // position rate
    const int sub = 10;       // velocity substeps per position step
    float t = 0.0f;
    float max_pos = 0.0f;
    const float target = 2.0f * kPi;
    const float t_end = plan.t_total + 1.0f;
    while (t < t_end) {
        traj_point_t pt = scurve_sample(&plan, t);
        const float w_cmd = pos_loop_step(&pcfg, pt.pos, pos, pt.vel);
        for (int k = 0; k < sub; k++) {
            const float dtv = dtp / static_cast<float>(sub);
            const float iq = vel_loop_step(&vel, w_cmd, w, 0.0f, dtv);
            w += (Kt * iq - B * w) / J * dtv;
            pos += w * dtv;
        }
        max_pos = std::max(max_pos, pos);
        t += dtp;
    }
    EXPECT_NEAR(pos, target, 0.05f);
    // Overshoot < 2% of move (no ringing).
    EXPECT_LT(max_pos - target, 0.02f * target);
}
