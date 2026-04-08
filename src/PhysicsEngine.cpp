#include "PhysicsEngine.hpp"

PhysicsEngine::PhysicsEngine(double mass, double initial_altitude)
    : _mass(mass), _altitude(initial_altitude), _velocity(0.0) {
}

double PhysicsEngine::update(double thrust_n, double disturbance_force_n, double dt) {
    double thrust_acceleration = thrust_n / _mass;
    double disturbance_acceleration = disturbance_force_n / _mass;

    double total_acceleration = _gravity + thrust_acceleration + disturbance_acceleration;

    _velocity += total_acceleration * dt;
    _altitude += _velocity * dt;

    if (_altitude <= 0.0) {
        _altitude = 0.0;
        _velocity = 0.0;
    }

    return _altitude;
}

double PhysicsEngine::getAltitude() const {
    return _altitude;
}

double PhysicsEngine::getVelocity() const {
    return _velocity;
}