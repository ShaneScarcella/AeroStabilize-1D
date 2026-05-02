#include "PhysicsEngine.hpp"

#include <cmath>

namespace {

// On ground contact, horizontal slip and rotation decay exponentially over dt so the body settles without tuning per dt.
constexpr double kGroundHorizontalDampingPerS = 10.0;
constexpr double kGroundPitchRateDampingPerS = 15.0;

}  // namespace

PhysicsEngine::PhysicsEngine(double mass_kg, double moment_of_inertia_kg_m2, double initial_altitude_m,
                             double initial_pitch_rad)
    : _mass_kg(mass_kg),
      _moment_of_inertia_kg_m2(moment_of_inertia_kg_m2),
      _position{0.0, initial_altitude_m},
      _velocity{},
      _pitch_rad(initial_pitch_rad),
      _pitch_rate_rad_s(0.0) {
}

Vector2D PhysicsEngine::update(double thrust_n, double torque_n_m, double disturbance_force_n, double dt) {
    const double inv_mass = 1.0 / _mass_kg;
    const double sin_p = std::sin(_pitch_rad);
    const double cos_p = std::cos(_pitch_rad);

    // Thrust acts along the body axis: in inertial XZ it splits by pitch (nose toward +X when pitch > 0).
    const double accel_x = (thrust_n * sin_p) * inv_mass;
    const double accel_z =
        ((thrust_n * cos_p) + disturbance_force_n - (_mass_kg * kGravityMps2)) * inv_mass;

    const double pitch_accel_rad_s2 = torque_n_m / _moment_of_inertia_kg_m2;

    _pitch_rate_rad_s += pitch_accel_rad_s2 * dt;
    _pitch_rad += _pitch_rate_rad_s * dt;

    _velocity.x += accel_x * dt;
    _velocity.z += accel_z * dt;

    _position.x += _velocity.x * dt;
    _position.z += _velocity.z * dt;

    if (_position.z <= 0.0) {
        _position.z = 0.0;
        _velocity.z = 0.0;
        const double horizontal_decay = std::exp(-kGroundHorizontalDampingPerS * dt);
        const double spin_decay = std::exp(-kGroundPitchRateDampingPerS * dt);
        _velocity.x *= horizontal_decay;
        _pitch_rate_rad_s *= spin_decay;
    }

    return _position;
}

Vector2D PhysicsEngine::getPosition() const {
    return _position;
}

Vector2D PhysicsEngine::getVelocity() const {
    return _velocity;
}

double PhysicsEngine::getPitchRad() const {
    return _pitch_rad;
}

double PhysicsEngine::getPitchRateRadS() const {
    return _pitch_rate_rad_s;
}
