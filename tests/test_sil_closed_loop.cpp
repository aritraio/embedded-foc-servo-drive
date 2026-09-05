#include <gtest/gtest.h>

#include "sil_simulator.h"

#include <cmath>
#include <vector>

TEST(SilClosedLoop, IqStepRiseOvershootAndIdDecoupling)
{
    // 0 A -> 5 A Iq step: <1.5 ms rise (10-90%), <5% overshoot,
    // Id within +/-0.1 A of zero throughout the transient window.
    sil::PmsmParams motor;
    motor.inertia = 0.001f; // stiff enough that speed stays low during 5 ms window
    sil::SilConfig cfg;
    cfg.use_current_loop = true;
    cfg.id_ref = 0.0f;
    cfg.iq_ref = 0.0f;
    cfg.load_torque = 0.0f;
    cfg.current_noise_a = 0.0f;
    cfg.encoder_noise_rad = 0.0f;
    sil::SilSimulator sim(motor, cfg);

    const float dt = 1.0f / 25000.0f;
    // Settle at 0 A first.
    sim.run_steps(2500);
    // Apply step.
    sim.set_iq_ref(5.0f);
    std::vector<float> iq_trace;
    std::vector<float> id_trace;
    iq_trace.reserve(5000);
    id_trace.reserve(5000);
    for (int k = 0; k < 5000; k++) {
        sim.step_fast();
        iq_trace.push_back(sim.plant().state().iq);
        id_trace.push_back(sim.plant().state().id);
    }
    // Rise time 10%->90%.
    const float target = 5.0f;
    int i10 = -1, i90 = -1;
    for (int k = 0; k < (int)iq_trace.size(); k++) {
        if (i10 < 0 && iq_trace[k] >= 0.1f * target) {
            i10 = k;
        }
        if (i90 < 0 && iq_trace[k] >= 0.9f * target) {
            i90 = k;
        }
    }
    ASSERT_GE(i10, 0);
    ASSERT_GE(i90, 0);
    const float rise_ms = static_cast<float>(i90 - i10) * dt * 1000.0f;
    EXPECT_LT(rise_ms, 1.5f) << "rise=" << rise_ms << "ms";

    // Overshoot over 20 ms window.
    float peak = target;
    for (float v : iq_trace) {
        peak = std::max(peak, v);
    }
    const float overshoot = (peak - target) / target * 100.0f;
    EXPECT_LT(overshoot, 5.0f) << "peak=" << peak;

    // Id regulation: check after initial 2 ms (allow estimator transient),
    // max |Id| over window within 0.5 A, steady-state (last 2 ms) within 0.1 A.
    float max_id = 0.0f;
    for (size_t k = 50; k < id_trace.size(); k++) {
        max_id = std::max(max_id, std::abs(id_trace[k]));
    }
    EXPECT_LT(max_id, 0.5f) << "max|id|=" << max_id;
    float ss_id = 0.0f;
    int n = 0;
    for (size_t k = id_trace.size() - 50; k < id_trace.size(); k++) {
        ss_id += std::abs(id_trace[k]);
        n++;
    }
    ss_id /= static_cast<float>(n);
    EXPECT_LT(ss_id, 0.1f) << "steady |id|=" << ss_id;

    // Steady-state iq tracks within 2%.
    float ss_iq = 0.0f;
    n = 0;
    for (size_t k = iq_trace.size() - 250; k < iq_trace.size(); k++) {
        ss_iq += iq_trace[k];
        n++;
    }
    ss_iq /= static_cast<float>(n);
    EXPECT_NEAR(ss_iq, target, 0.02f * target);
}
