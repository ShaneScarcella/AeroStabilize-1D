#pragma once

#include "Vector2D.hpp"

class PhysicsEngine {
public:
    PhysicsEngine(double mass_kg, double moment_of_inertia_kg_m2, double initial_altitude_m = 0.0,
                  double initial_pitch_rad = 0.0);

    /** Planar rigid-body step: thrust and disturbance are in inertial XZ; pitch is measured from +Z toward +X. */
    Vector2D update(double thrust_n, double torque_n_m, double disturbance_force_n, double dt);

    Vector2D getPosition() const;
    Vector2D getVelocity() const;
    double getPitchRad() const;
    double getPitchRateRadS() const;

private:
    static constexpr double kGravityMps2 = 9.81;

    double _mass_kg;
    double _moment_of_inertia_kg_m2;

    Vector2D _position{};
    Vector2D _velocity{};
    double _pitch_rad = 0.0;
    double _pitch_rate_rad_s = 0.0;
};