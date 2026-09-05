#pragma once

/* SIL simulator: wires FOC core + PMSM plant + outer loops at correct rates.
 * Deterministic discrete-time stepper used by tests and sil_main. */

#include "pmsm_model.h"

extern "C" {
#include "foc_core.h"
#include "motion_profiler.h"
#include "disturbance_observer.h"
}

namespace sil {

struct SilConfig {
    float dt_fast = 1.0f / 25000.0f; // current loop
    float iq_ref = 0.0f;
    float id_ref = 0.0f;
    float load_torque = 0.0f;
    bool use_current_loop = true; // false => direct vd/vq excitation (open loop)
    float vd_open = 0.0f;
    float vq_open = 0.0f;
    float current_noise_a = 0.0f; // 0 for deterministic tests
    float encoder_noise_rad = 0.0f;
};

class SilSimulator {
public:
    SilSimulator(const PmsmParams &motor, const SilConfig &cfg);

    void reset();
    // Advance one 40 us fast step.
    void step_fast();
    // Run N fast steps.
    void run_steps(int n);

    // Hooks for tests / telemetry.
    PmsmModel &plant() { return plant_; }
    const PmsmModel &plant() const { return plant_; }
    foc_core_t &controller() { return foc_; }
    const foc_core_t &controller() const { return foc_; }
    int step_count() const { return steps_; }
    float time_s() const;

    void set_iq_ref(float iq) { cfg_.iq_ref = iq; foc_core_set_refs(&foc_, cfg_.id_ref, iq); }
    void set_id_ref(float id) { cfg_.id_ref = id; foc_core_set_refs(&foc_, id, cfg_.iq_ref); }

private:
    PmsmParams motor_;
    SilConfig cfg_;
    PmsmModel plant_;
    foc_core_t foc_;
    int steps_ = 0;
};

} // namespace sil
