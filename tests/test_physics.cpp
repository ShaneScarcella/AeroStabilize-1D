#include "PhysicsEngine.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

constexpr double kG = 9.81;
constexpr double kEps = 1e-9;

}  // namespace

TEST(PhysicsEngine, ZeroThrustFreeFallMatchesEuler) {
    const double mass = 2.0;
    const double iyy = 0.05;
    const double dt = 0.1;
    const double h0 = 100.0;
    PhysicsEngine p(mass, iyy, h0);

    const double thrust = 0.0;
    const Vector2D pos = p.update(thrust, 0.0, 0.0, dt);

    const double a_z = -kG + thrust / mass;
    const double v_expected_z = 0.0 + a_z * dt;
    const double h_expected = h0 + v_expected_z * dt;

    EXPECT_NEAR(p.getVelocity().z, v_expected_z, kEps);
    EXPECT_NEAR(pos.z, h_expected, kEps);
    EXPECT_NEAR(p.getPosition().z, h_expected, kEps);
}

TEST(PhysicsEngine, ThrustBalancesWeightHoversAcceleration) {
    const double mass = 2.0;
    const double iyy = 0.05;
    const double dt = 0.05;
    PhysicsEngine p(mass, iyy, 50.0);
    const double thrust = kG * mass;
    const double vz0 = p.getVelocity().z;

    p.update(thrust, 0.0, 0.0, dt);

    EXPECT_NEAR(p.getVelocity().z, vz0, 1e-6);
}

TEST(PhysicsEngine, DownwardDisturbanceAcceleratesDownward) {
    const double mass = 2.0;
    const double iyy = 0.05;
    const double dt = 0.1;
    PhysicsEngine p(mass, iyy, 50.0);
    const double thrust = kG * mass;
    const double downward_gust_n = -10.0;

    p.update(thrust, 0.0, downward_gust_n, dt);

    const double a_z = -kG + thrust / mass + downward_gust_n / mass;
    EXPECT_NEAR(p.getVelocity().z, 0.0 + a_z * dt, kEps);
}

TEST(PhysicsEngine, GroundStopsAtZero) {
    PhysicsEngine p(1.0, 0.05, 0.5);
    for (int i = 0; i < 20; ++i) {
        p.update(0.0, 0.0, 0.0, 0.2);
    }
    EXPECT_GE(p.getPosition().z, 0.0);
    EXPECT_NEAR(p.getPosition().z, 0.0, kEps);
    EXPECT_NEAR(p.getVelocity().z, 0.0, kEps);
}

TEST(PhysicsEngine, PositivePitchSteersThrustIntoPositiveX) {
    const double mass = 2.0;
    const double iyy = 0.05;
    const double pitch_rad = 0.3;
    const double thrust_n = 40.0;
    const double dt = 0.02;

    PhysicsEngine p(mass, iyy, 50.0, pitch_rad);

    p.update(thrust_n, 0.0, 0.0, dt);

    const double accel_x_expected = thrust_n * std::sin(pitch_rad) / mass;
    const double vx_expected = accel_x_expected * dt;

    EXPECT_NEAR(p.getVelocity().x, vx_expected, 1e-6);
    EXPECT_GT(p.getVelocity().x, 0.0);
}
