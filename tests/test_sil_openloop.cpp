#include <gtest/gtest.h>

#include "pmsm_model.h"
#include "sil_simulator.h"

#include <cmath>

TEST(SilOpenLoop, SinusoidalCurrentsAndAcceleration)
{
    // Open-loop V/f excitation: rotating voltage vector must produce smooth
    // sinusoidal phase currents and spin the rotor up from rest.
    sil::PmsmParams motor;
    sil::SilConfig cfg;
    cfg.use_current_loop = false;
    cfg.current_noise_a = 0.0f;
    cfg.encoder_noise_rad = 0.0f;
    sil::SilSimulator sim(motor, cfg);

    const float dt = 1.0f / 25000.0f;
    const float vmag = 6.0f;
    float we_cmd = 0.0f;
    float theta_e = 0.0f;
    float ia_first[5] = {0, 0, 0, 0, 0};

    // Ramp electrical frequency 0 -> 100 rad/s over 0.5 s, then hold.
    for (int k = 0; k < 25000; k++) {
        const float t = static_cast<float>(k) * dt;
        we_cmd = (t < 0.5f) ? (100.0f * t / 0.5f) : 100.0f;
        theta_e += we_cmd * dt;
        const float vd = 0.0f;
        const float vq = vmag;
        // Convert to stationary then to applied dq at TRUE rotor angle
        // (open-loop slip allowed): apply V in rotating command frame.
        const float c = std::cos(theta_e);
        const float s = std::sin(theta_e);
        // Command-frame voltage expressed in rotor frame isimplicitly handled
        // by plant; here excite directly with rotating alpha-beta:
        const float valpha = vmag * std::cos(theta_e);
        const float vbeta = vmag * std::sin(theta_e);
        // Project onto rotor frame for plant input:
        const float th_r = sim.plant().electrical_angle();
        const float cr = std::cos(th_r);
        const float sr = std::sin(th_r);
        const float vd_r = valpha * cr + vbeta * sr;
        const float vq_r = -valpha * sr + vbeta * cr;
        sil::PmsmInputs in{};
        in.vd = vd_r;
        in.vq = vq_r;
        in.load_torque = 0.0f;
        in.vbus = motor.vdc_nom;
        float vd_i = in.vd;
        float vq_i = in.vq;
        sim.plant().apply_inverter(vd_i, vq_i, motor.vdc_nom);
        in.vd = vd_i;
        in.vq = vq_i;
        sim.plant().step(dt, in);
        if (k < 5) {
            float ia = 0, ib = 0, ic = 0;
            sim.plant().phase_currents(ia, ib, ic);
            ia_first[k] = ia;
        }
        (void)c;
        (void)s;
        (void)vd;
        (void)vq;
    }

    const auto &st = sim.plant().state();
    // Rotor must have accelerated away from rest.
    EXPECT_GT(st.omega_m, 5.0f);
    // Currents must be AC (nonzero, bounded, not latched at rails).
    float ia = 0, ib = 0, ic = 0;
    sim.plant().phase_currents(ia, ib, ic);
    EXPECT_GT(std::abs(ia) + std::abs(ib) + std::abs(ic), 0.1f);
    EXPECT_LT(std::abs(ia), 30.0f);
    // KCL: ia+ib+ic == 0.
    EXPECT_NEAR(ia + ib + ic, 0.0f, 1e-4f);
    // Early samples near zero (smooth start, no impulse).
    EXPECT_LT(std::abs(ia_first[0]), 1.0f);
}

TEST(SilOpenLoop, Rk4SteadyStateMatchesAnalytic)
{
    // At standstill (locked rotor, we=0), DC Vd must give Id = Vd/Rs.
    sil::PmsmParams motor;
    motor.inertia = 1.0f; // effectively locked rotor: speed stays ~0
    sil::PmsmModel plant(motor);
    sil::PmsmInputs in{};
    in.vd = 1.75f; // 1.75/0.35 = 5 A
    in.vq = 0.0f;
    in.load_torque = 0.0f;
    const float dt = 1.0f / 25000.0f;
    for (int k = 0; k < 2500; k++) {
        plant.step(dt, in);
    }
    EXPECT_NEAR(plant.state().id, 5.0f, 0.15f);
}
