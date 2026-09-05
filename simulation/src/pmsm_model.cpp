#include "pmsm_model.h"

#include <cmath>
#include <cstdint>

namespace sil {

namespace {
constexpr float k2Pi = 6.28318530717958647692f;
} // namespace

PmsmModel::PmsmModel(const PmsmParams &p) : params_(p), vbus_(p.vdc_nom) {}

void PmsmModel::reset(const PmsmState &s)
{
    state_ = s;
    vbus_ = params_.vdc_nom;
    rng_ = 0x12345678u;
}

float PmsmModel::randn() const
{
    // Xorshift32 + Box-Muller (deterministic seed; const-safe via mutable).
    auto uni = [this]() -> float {
        rng_ ^= (rng_ << 13u);
        rng_ ^= (rng_ >> 17u);
        rng_ ^= (rng_ << 5u);
        // NOLINTNEXTLINE: intentional integer->float conversion
        return static_cast<float>(rng_) / 4294967296.0f;
    };
    float u1 = uni();
    float u2 = uni();
    if (u1 < 1e-9f) {
        u1 = 1e-9f;
    }
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(k2Pi * u2);
}

PmsmModel::Deriv PmsmModel::dynamics(const PmsmState &s, const PmsmInputs &in) const
{
    Deriv d{};
    const float we = static_cast<float>(params_.pole_pairs) * s.omega_m;
    const float vd = in.vd;
    const float vq = in.vq;
    d.did = (vd - params_.rs * s.id + we * params_.lq * s.iq) / params_.ld;
    d.diq = (vq - params_.rs * s.iq - we * params_.ld * s.id - we * params_.lambda_pm) / params_.lq;
    const float te =
        1.5f * static_cast<float>(params_.pole_pairs) * (params_.lambda_pm * s.iq + (params_.ld - params_.lq) * s.id * s.iq);
    d.domega = (te - in.load_torque - params_.friction * s.omega_m) / params_.inertia;
    d.dtheta = s.omega_m;
    return d;
}

void PmsmModel::step(float dt, const PmsmInputs &in)
{
    // Classic RK4 on (id, iq, omega_m, theta_m).
    const Deriv k1 = dynamics(state_, in);
    PmsmState s2{state_.id + k1.did * dt * 0.5f, state_.iq + k1.diq * dt * 0.5f,
                 state_.omega_m + k1.domega * dt * 0.5f, state_.theta_m + k1.dtheta * dt * 0.5f};
    const Deriv k2 = dynamics(s2, in);
    PmsmState s3{state_.id + k2.did * dt * 0.5f, state_.iq + k2.diq * dt * 0.5f,
                 state_.omega_m + k2.domega * dt * 0.5f, state_.theta_m + k2.dtheta * dt * 0.5f};
    const Deriv k3 = dynamics(s3, in);
    PmsmState s4{state_.id + k3.did * dt, state_.iq + k3.diq * dt, state_.omega_m + k3.domega * dt,
                 state_.theta_m + k3.dtheta * dt};
    const Deriv k4 = dynamics(s4, in);
    state_.id += dt / 6.0f * (k1.did + 2.0f * k2.did + 2.0f * k3.did + k4.did);
    state_.iq += dt / 6.0f * (k1.diq + 2.0f * k2.diq + 2.0f * k3.diq + k4.diq);
    state_.omega_m += dt / 6.0f * (k1.domega + 2.0f * k2.domega + 2.0f * k3.domega + k4.domega);
    state_.theta_m += dt / 6.0f * (k1.dtheta + 2.0f * k2.dtheta + 2.0f * k3.dtheta + k4.dtheta);
    // Wrap mechanical angle to [0, 2pi).
    state_.theta_m = std::fmod(state_.theta_m, k2Pi);
    if (state_.theta_m < 0.0f) {
        state_.theta_m += k2Pi;
    }
    vbus_ = in.vbus;
}

float PmsmModel::electrical_speed() const
{
    return static_cast<float>(params_.pole_pairs) * state_.omega_m;
}

float PmsmModel::electrical_angle() const
{
    float a = std::fmod(static_cast<float>(params_.pole_pairs) * state_.theta_m, k2Pi);
    if (a < 0.0f) {
        a += k2Pi;
    }
    return a;
}

float PmsmModel::electromagnetic_torque() const
{
    return 1.5f * static_cast<float>(params_.pole_pairs) *
           (params_.lambda_pm * state_.iq + (params_.ld - params_.lq) * state_.id * state_.iq);
}

void PmsmModel::apply_inverter(float &vd, float &vq, float vbus) const
{
    // 1) DC-bus sag already reflected in vbus argument; 2) circle saturation
    // at Vbus/sqrt(3); 3) dead-time voltage error ~ (Tdead/Tpwm)*Vbus opposing
    // current direction, projected on dq (approximate resistive drop).
    const float vmax = (vbus > 0.0f ? vbus : params_.vdc_nom) / 1.7320508075688772f;
    const float mag = std::sqrt(vd * vd + vq * vq);
    if (mag > vmax && mag > 1e-9f) {
        const float s = vmax / mag;
        vd *= s;
        vq *= s;
    }
    const float vdead = params_.dead_time_s * params_.pwm_freq_hz * vbus;
    // Dead-time acts like a current-polarity-dependent drop; approximate as
    // small resistive term so open/closed-loop SIL stays stable yet visible.
    const float id = state_.id;
    const float iq = state_.iq;
    const float im = std::sqrt(id * id + iq * iq);
    if (im > 0.05f) {
        vd -= vdead * 0.5f * (id / im);
        vq -= vdead * 0.5f * (iq / im);
    }
}

void PmsmModel::phase_voltages(float vd, float vq, float theta_e, float &va, float &vb, float &vc) const
{
    const float c = std::cos(theta_e);
    const float s = std::sin(theta_e);
    const float valpha = vd * c - vq * s;
    const float vbeta = vd * s + vq * c;
    va = valpha;
    vb = -0.5f * valpha + 0.8660254037844386f * vbeta;
    vc = -0.5f * valpha - 0.8660254037844386f * vbeta;
    (void)this;
}

void PmsmModel::phase_currents(float &ia, float &ib, float &ic) const
{
    // Inverse Clarke/Park of (id,iq) at true electrical angle.
    const float th = electrical_angle();
    const float c = std::cos(th);
    const float s = std::sin(th);
    const float ialpha = state_.id * c - state_.iq * s;
    const float ibeta = state_.id * s + state_.iq * c;
    ia = ialpha;
    ib = -0.5f * ialpha + 0.8660254037844386f * ibeta;
    ic = -ia - ib;
}

void PmsmModel::sense_currents(float &ia, float &ib, float noise_std_a) const
{
    float ic = 0.0f;
    phase_currents(ia, ib, ic);
    if (noise_std_a > 0.0f) {
        ia += randn() * noise_std_a;
        ib += randn() * noise_std_a;
    }
    // 12-bit quantization over +-fullscale.
    const float lsb = (2.0f * params_.adc_fullscale_a) / 4096.0f;
    ia = std::round(ia / lsb) * lsb;
    ib = std::round(ib / lsb) * lsb;
}

float PmsmModel::sense_theta_mech(float noise_std_rad) const
{
    float th = state_.theta_m;
    if (noise_std_rad > 0.0f) {
        th += randn() * noise_std_rad;
    }
    // 14-bit quantization.
    const float steps = static_cast<float>(1u << params_.encoder_bits);
    float wrapped = std::fmod(th, k2Pi);
    if (wrapped < 0.0f) {
        wrapped += k2Pi;
    }
    const float q = std::floor(wrapped / k2Pi * steps) / steps * k2Pi;
    return q;
}

float PmsmModel::sense_vbus(float noise_std_v) const
{
    float v = vbus_;
    if (noise_std_v > 0.0f) {
        v += randn() * noise_std_v;
    }
    return v;
}

} // namespace sil
