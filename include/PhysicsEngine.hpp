#pragma once

class PhysicsEngine {
public:
    PhysicsEngine(double mass, double initial_altitude = 0.0);

    double update(double thrust_n, double dt);

    double getAltitude() const;
    double getVelocity() const;

private:
    const double _gravity = -9.81;
    double _mass;
    
    double _altitude;
    double _velocity;
};