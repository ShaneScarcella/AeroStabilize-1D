#include <iostream>
#include <iomanip>
#include <stdexcept>
#include "PhysicsEngine.hpp"
#include "PIDController.hpp"
#include "TelemetryLogger.hpp"

int main() {
    try {
        PhysicsEngine drone(1.5, 0.0);

        // Altitude loop: PID output is thrust (N), clamped to [min, max]. P and D act
        // on error and its rate; I integrates so the command can converge to hover (mg).
        PIDController flightComputer(12.0, 4.0, 7.0, 0.0, 30.0);

        double target_altitude = 10.0;
        double dt = 0.1;
        double elapsed_time = 0.0;

        TelemetryLogger telemetry("flight_data.csv");

        std::cout << "Time(s)\tTarget\tAlt(m)\tVel(m/s)\tThrust(N)\n";
        std::cout << "---------------------------------------------------------\n";

        constexpr int kSimulationSteps = 100;
        for (int i = 0; i < kSimulationSteps; ++i) {
            double current_altitude = drone.getAltitude();

            double thrust = flightComputer.calculate(target_altitude, current_altitude, dt);

            drone.update(thrust, dt);
            elapsed_time += dt;

            telemetry.logState(elapsed_time, target_altitude, drone.getAltitude(),
                               drone.getVelocity(), thrust);

            std::cout << std::fixed << std::setprecision(2)
                      << elapsed_time << "\t"
                      << target_altitude << "\t"
                      << drone.getAltitude() << "\t"
                      << drone.getVelocity() << "\t\t"
                      << thrust << "\n";
        }

        std::cout << "Wrote flight_data.csv\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        std::cerr << "Hint: close flight_data.csv in Excel/other apps if it is open, or run from a writable folder.\n";
        return 1;
    }
    return 0;
}