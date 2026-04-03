#include <iostream>
#include <iomanip>
#include "PhysicsEngine.hpp"
#include "PIDController.hpp"

int main() {
    PhysicsEngine drone(1.5, 0.0);
    
    PIDController flightComputer(10.0, 1.0, 5.0, 0.0, 30.0);
    
    double target_altitude = 10.0;
    double dt = 0.1;
    double elapsed_time = 0.0;

    std::cout << "Time(s)\tTarget\tAlt(m)\tVel(m/s)\tThrust(N)\n";
    std::cout << "---------------------------------------------------------\n";

    for (int i = 0; i < 50; ++i) {
        double current_altitude = drone.getAltitude();
        
        double thrust = flightComputer.calculate(target_altitude, current_altitude, dt);
        
        drone.update(thrust, dt);
        elapsed_time += dt;

        std::cout << std::fixed << std::setprecision(2) 
                  << elapsed_time << "\t" 
                  << target_altitude << "\t"
                  << drone.getAltitude() << "\t" 
                  << drone.getVelocity() << "\t\t"
                  << thrust << "\n";
    }

    return 0;
}