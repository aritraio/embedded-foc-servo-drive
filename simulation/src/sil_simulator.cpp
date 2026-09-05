#include "sil_simulator.h"

namespace sil {

SilSimulator::SilSimulator(const PmsmParams &motor, const SilConfig &cfg) : motor_(motor), cfg_(cfg), plant_(motor)
{
    foc_motor_params_t mp{};
    mp.rs = motor_.rs;
    mp.ld = motor_.ld;
    mp.lq = motor_.lq;
    mp.lambda_pm = motor_.lambda_pm;
    mp.pole_pairs = motor_.pole_pairs;
    mp.vdc_nom = motor_.vdc_nom;
    mp.theta_offset = 0.0f;
    // Current-loop gains matched to config_params.h defaults.
    foc_core_init(&foc_, &mp, 4.712f, 3298.0f, 4.712f, 3298.0f, 14.0f);
    foc_core_set_refs(&foc_, cfg_.id_ref, cfg_.iq_ref);
    plant_.reset();
}

void SilSimulator::reset()
{
    steps_ = 0;
    plant_.reset();
    foc_core_reset(&foc_);
    foc_core_set_refs(&foc_, cfg_.id_ref, cfg_.iq_ref);
}

void SilSimulator::step_fast()
{
    const float dt = cfg_.dt_fast;
    float vd = cfg_.vd_open;
    float vq = cfg_.vq_open;

    if (cfg_.use_current_loop) {
        float ia = 0.0f;
        float ib = 0.0f;
        plant_.sense_currents(ia, ib, cfg_.current_noise_a);
        const float th = plant_.sense_theta_mech(cfg_.encoder_noise_rad);
        const float we = plant_.electrical_speed();
        const float vbus = plant_.sense_vbus(0.0f);
        foc_core_step(&foc_, ia, ib, th, we, vbus, dt);
        vd = foc_.vd;
        vq = foc_.vq;
    }
    plant_.apply_inverter(vd, vq, motor_.vdc_nom);
    PmsmInputs in{};
    in.vd = vd;
    in.vq = vq;
    in.load_torque = cfg_.load_torque;
    in.vbus = motor_.vdc_nom;
    plant_.step(dt, in);
    steps_++;
}

void SilSimulator::run_steps(int n)
{
    for (int i = 0; i < n; i++) {
        step_fast();
    }
}

float SilSimulator::time_s() const
{
    return static_cast<float>(steps_) * cfg_.dt_fast;
}

} // namespace sil
