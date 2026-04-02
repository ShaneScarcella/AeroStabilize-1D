#include <iostream>
#include <iomanip>
#include "PhysicsEngine.hpp"

int main() {
    PhysicsEngine drone(1.5, 10.0);
    
    double dt = 0.1;
    double elapsed_time = 0.0;

    std::cout << "Time(s)\tAlt(m)\tVel(m/s)\n";
    std::cout << "--------------------------\n";

    for (int i = 0; i < 20; ++i) {
        double thrust = 0.0;
        
        drone.update(thrust, dt);
        elapsed_time += dt;

        std::cout << std::fixed << std::setprecision(2) 
                  << elapsed_time << "\t" 
                  << drone.getAltitude() << "\t" 
                  << drone.getVelocity() << "\n";
    }

    return 0;
}