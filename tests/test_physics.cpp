#include "PhysicsEngine.hpp"

#include <gtest/gtest.h>

namespace {

constexpr double kG = -9.81;
constexpr double kEps = 1e-9;

}  // namespace

TEST(PhysicsEngine, ZeroThrustFreeFallMatchesEuler) {
    const double mass = 2.0;
    const double dt = 0.1;
    const double h0 = 100.0;
    PhysicsEngine p(mass, h0);

    const double thrust = 0.0;
    const double alt = p.update(thrust, 0.0, dt);

    const double a = kG + thrust / mass;
    const double v_expected = 0.0 + a * dt;
    const double h_expected = h0 + v_expected * dt;

    EXPECT_NEAR(p.getVelocity(), v_expected, kEps);
    EXPECT_NEAR(alt, h_expected, kEps);
    EXPECT_NEAR(p.getAltitude(), h_expected, kEps);
}

TEST(PhysicsEngine, ThrustBalancesWeightHoversAcceleration) {
    const double mass = 2.0;
    const double dt = 0.05;
    PhysicsEngine p(mass, 50.0);
    const double thrust = -kG * mass;  // cancel gravity
    const double v0 = p.getVelocity();

    p.update(thrust, 0.0, dt);

    EXPECT_NEAR(p.getVelocity(), v0, 1e-6);
}

TEST(PhysicsEngine, DownwardDisturbanceAcceleratesDownward) {
    const double mass = 2.0;
    const double dt = 0.1;
    PhysicsEngine p(mass, 50.0);
    const double thrust = -kG * mass;
    const double downward_gust_n = -10.0;

    p.update(thrust, downward_gust_n, dt);

    const double a_expected = kG + thrust / mass + downward_gust_n / mass;
    EXPECT_NEAR(p.getVelocity(), 0.0 + a_expected * dt, kEps);
}

TEST(PhysicsEngine, GroundStopsAtZero) {
    PhysicsEngine p(1.0, 0.5);
    // Large downward velocity in one step — should not go below ground.
    for (int i = 0; i < 20; ++i) {
        p.update(0.0, 0.0, 0.2);
    }
    EXPECT_GE(p.getAltitude(), 0.0);
    EXPECT_NEAR(p.getAltitude(), 0.0, kEps);
    EXPECT_NEAR(p.getVelocity(), 0.0, kEps);
}
