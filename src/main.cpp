#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "Config.hpp"
#include "PhysicsEngine.hpp"
#include "PIDController.hpp"
#include "TelemetryLogger.hpp"

namespace {

constexpr const char* kDefaultConfigPath = "config.txt";

}  // namespace

int main(int argc, char* argv[]) {
    try {
        std::string config_path = kDefaultConfigPath;
        if (argc >= 2 && argv[1][0] != '\0') {
            config_path = argv[1];
        }

        const Config cfg = Config::loadFromFile(config_path);

        PhysicsEngine drone(cfg.mass_kg, cfg.initial_altitude_m);

        PIDController flightComputer(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd,
                                   cfg.pid_min_thrust_n, cfg.pid_max_thrust_n);

        const double target_altitude = cfg.target_altitude_m;
        const double dt = cfg.dt_s;
        double elapsed_time = 0.0;

        TelemetryLogger telemetry(cfg.telemetry_csv);

        std::cout << "Time(s)\tTarget\tAlt(m)\tVel(m/s)\tThrust(N)\n";
        std::cout << "---------------------------------------------------------\n";

        for (int i = 0; i < cfg.simulation_steps; ++i) {
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

        std::cout << "Wrote " << cfg.telemetry_csv << "\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        std::cerr << "Hint: close the telemetry CSV in Excel/other apps if it is open, or run from a writable folder.\n";
        return 1;
    }
    return 0;
}