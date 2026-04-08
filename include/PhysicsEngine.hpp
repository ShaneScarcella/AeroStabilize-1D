#pragma once

class PhysicsEngine {
public:
    PhysicsEngine(double mass, double initial_altitude = 0.0);

    /** @param thrust_n vertical thrust (N), positive upward
     *  @param disturbance_force_n extra vertical force from environment (N), positive upward; e.g. wind gust downward = negative */
    double update(double thrust_n, double disturbance_force_n, double dt);

    double getAltitude() const;
    double getVelocity() const;

private:
    const double _gravity = -9.81;
    double _mass;
    
    double _altitude;
    double _velocity;
};