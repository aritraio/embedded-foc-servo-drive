#pragma once

/* High-fidelity PMSM model with RK4 integration, inverter non-idealities,
 * ADC / encoder quantization. Header-only parameter/state types + class. */

#include <cstdint>

namespace sil {

struct PmsmParams {
    float rs = 0.35f;          // phase resistance [ohm]
    float ld = 0.00050f;       // d inductance [H]
    float lq = 0.00050f;       // q inductance [H]
    float lambda_pm = 0.0080f; // PM flux [Wb]
    uint32_t pole_pairs = 7;   // P
    float inertia = 0.00002f;  // J [kg m^2]
    float friction = 0.00001f; // B [N m s]
    float vdc_nom = 24.0f;     // nominal bus [V]
    float dead_time_s = 100e-9f;
    float pwm_freq_hz = 25000.0f;
    float adc_fullscale_a = 20.0f; // +- full scale
    uint32_t encoder_bits = 14;
};

struct PmsmState {
    float id = 0.0f;      // d current [A]
    float iq = 0.0f;      // q current [A]
    float omega_m = 0.0f; // mech speed [rad/s]
    float theta_m = 0.0f; // mech angle [rad]
};

struct PmsmInputs {
    float vd = 0.0f;       // applied d voltage [V]
    float vq = 0.0f;       // applied q voltage [V]
    float load_torque = 0.0f;
    float vbus = 24.0f;
};

class PmsmModel {
public:
    explicit PmsmModel(const PmsmParams &p = PmsmParams{});

    void reset(const PmsmState &s = PmsmState{});
    // One RK4 physics step of dt seconds.
    void step(float dt, const PmsmInputs &in);

    // Accessors
    const PmsmState &state() const { return state_; }
    const PmsmParams &params() const { return params_; }
    float electrical_speed() const;
    float electrical_angle() const;
    float electromagnetic_torque() const;

    // Inverter: dead-time + saturation applied to commanded dq volts.
    void apply_inverter(float &vd, float &vq, float vbus) const;
    // Sensing: quantized + noisy readouts (use noise_std=0 for deterministic tests).
    void sense_currents(float &ia, float &ib, float noise_std_a) const;
    float sense_theta_mech(float noise_std_rad) const;
    float sense_vbus(float noise_std_v) const;

    // Phase-voltage helpers (for open-loop excitation tests).
    void phase_voltages(float vd, float vq, float theta_e, float &va, float &vb, float &vc) const;
    void phase_currents(float &ia, float &ib, float &ic) const;

    void set_state(const PmsmState &s) { state_ = s; }
    void set_vbus(float v) { vbus_ = v; }

private:
    struct Deriv {
        float did = 0.0f;
        float diq = 0.0f;
        float domega = 0.0f;
        float dtheta = 0.0f;
    };
    Deriv dynamics(const PmsmState &s, const PmsmInputs &in) const;

    PmsmParams params_;
    PmsmState state_;
    float vbus_ = 24.0f;
    mutable uint32_t rng_ = 0x12345678u; // xorshift for noise (mutable: sensing is const)
    float randn() const;                 // N(0,1)
};

} // namespace sil
