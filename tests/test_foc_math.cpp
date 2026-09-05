#include <gtest/gtest.h>

extern "C" {
#include "foc_math.h"
#include "svpwm.h"
}

#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTol = 1e-4f;
} // namespace

TEST(FocMath, ClarkeBalanced)
{
    // ia=1, ib=-0.5, ic=-0.5 (balanced): ialpha=1, ibeta=0.
    foc_alphabeta_t ab{};
    foc_clarke(1.0f, -0.5f, -0.5f, &ab);
    EXPECT_NEAR(ab.alpha, 1.0f, kTol);
    EXPECT_NEAR(ab.beta, 0.0f, kTol);

    // ia=0, ib=1, ic=-1: ibeta = (0+2)/sqrt3.
    foc_clarke(0.0f, 1.0f, -1.0f, &ab);
    EXPECT_NEAR(ab.beta, 2.0f / std::sqrt(3.0f), kTol);
}

TEST(FocMath, ParkInvParkRoundtrip)
{
    for (int deg = 0; deg < 360; deg += 15) {
        const float th = static_cast<float>(deg) * kPi / 180.0f;
        foc_alphabeta_t ab{0.7f, -0.3f};
        foc_dq_t dq{};
        foc_alphabeta_t back{};
        foc_park(&ab, th, &dq);
        foc_inv_park(&dq, th, &back);
        EXPECT_NEAR(back.alpha, ab.alpha, 1e-5f) << "deg=" << deg;
        EXPECT_NEAR(back.beta, ab.beta, 1e-5f) << "deg=" << deg;
    }
}

TEST(FocMath, ParkKnownAngle)
{
    // alpha=1,beta=0 at theta=0 -> d=1,q=0; at 90deg -> d=0,q=-1.
    foc_alphabeta_t ab{1.0f, 0.0f};
    foc_dq_t dq{};
    foc_park(&ab, 0.0f, &dq);
    EXPECT_NEAR(dq.d, 1.0f, kTol);
    EXPECT_NEAR(dq.q, 0.0f, kTol);
    foc_park(&ab, kPi / 2.0f, &dq);
    EXPECT_NEAR(dq.d, 0.0f, kTol);
    EXPECT_NEAR(dq.q, -1.0f, kTol);
}

TEST(FocMath, AngleWrap)
{
    EXPECT_NEAR(foc_wrap_2pi(7.0f * kPi), kPi, 1e-5f);
    EXPECT_NEAR(foc_wrap_pi(3.0f * kPi), -kPi + 2.0f * kPi - 2.0f * kPi + 0.0f + kPi - 2.0f * kPi + kPi, 1e-4f);
    // Electrical angle: P=7, theta_m=2pi -> 0.
    EXPECT_NEAR(foc_elec_angle(2.0f * kPi, 7, 0.0f), 0.0f, 1e-4f);
}

TEST(Svpwm, FullCircleSweep)
{
    const float vdc = 24.0f;
    const float ts = 1.0f / 25000.0f;
    const float vmag = 5.0f; // well inside linear region (Vmax=13.85)
    bool seen[7] = {false, false, false, false, false, false, false};

    for (int deg = 0; deg < 360; deg++) {
        const float th = static_cast<float>(deg) * kPi / 180.0f;
        svpwm_out_t o{};
        svpwm_compute(vmag * std::cos(th), vmag * std::sin(th), vdc, ts, &o);
        ASSERT_GE(o.sector, 1u);
        ASSERT_LE(o.sector, 6u);
        seen[o.sector] = true;
        // Dwell-time conservation with zero error.
        EXPECT_NEAR(o.t1 + o.t2 + o.t0, ts, 1e-9f) << "deg=" << deg;
        EXPECT_GE(o.t1, 0.0f);
        EXPECT_GE(o.t2, 0.0f);
        EXPECT_GE(o.t0, 0.0f);
        EXPECT_GE(o.ta, 0.0f);
        EXPECT_LE(o.ta, 1.0f);
        EXPECT_GE(o.tb, 0.0f);
        EXPECT_LE(o.tb, 1.0f);
        EXPECT_GE(o.tc, 0.0f);
        EXPECT_LE(o.tc, 1.0f);
        EXPECT_EQ(o.saturated, 0u);
        // Sector consistency.
        const uint32_t expect = static_cast<uint32_t>(deg / 60) + 1u;
        EXPECT_EQ(o.sector, expect) << "deg=" << deg;
    }
    for (int s = 1; s <= 6; s++) {
        EXPECT_TRUE(seen[s]) << "sector " << s << " never hit";
    }
}

TEST(Svpwm, OvermodulationClamps)
{
    const float vdc = 24.0f;
    const float ts = 1.0f / 25000.0f;
    svpwm_out_t o{};
    // 20 V >> Vmax(13.86): must saturate, T0=0, duties still in [0,1].
    svpwm_compute(20.0f, 0.0f, vdc, ts, &o);
    EXPECT_EQ(o.saturated, 1u);
    EXPECT_NEAR(o.t0, 0.0f, 1e-9f);
    EXPECT_NEAR(o.t1 + o.t2, ts, 1e-9f);
}

TEST(Svpwm, VmaxExtension)
{
    // SVPWM linear limit Vdc/sqrt(3) vs SPWM Vdc/2: ratio 2/sqrt(3) = +15.47%.
    const float vdc = 24.0f;
    EXPECT_NEAR(svpwm_vmax_linear(vdc), vdc / std::sqrt(3.0f), 1e-5f);
    EXPECT_NEAR(svpwm_vmax_linear(vdc) / (vdc / 2.0f), 2.0f / std::sqrt(3.0f), 1e-5f);
}
