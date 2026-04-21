#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <cstddef>

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

        const double dt = cfg.dt_s;
        double elapsed_time = 0.0;
        double current_target_altitude = cfg.waypoints.front().altitude_m;
        std::size_t next_waypoint_index = 0;

        TelemetryLogger telemetry(cfg.telemetry_csv);

        std::cout << "Time(s)\tTarget\tAlt(m)\tVel(m/s)\tThrust(N)\tDist(N)\n";
        std::cout << "-----------------------------------------------------------------------------\n";

        for (int i = 0; i < cfg.simulation_steps; ++i) {
            while (next_waypoint_index < cfg.waypoints.size() &&
                   elapsed_time >= cfg.waypoints[next_waypoint_index].time_s) {
                current_target_altitude = cfg.waypoints[next_waypoint_index].altitude_m;
                ++next_waypoint_index;
            }

            double current_altitude = drone.getAltitude();

            double thrust = flightComputer.calculate(current_target_altitude, current_altitude, dt);

            double disturbance_n = 0.0;
            if (cfg.gust_force_n != 0.0 && cfg.gust_duration_steps > 0 &&
                i >= cfg.gust_start_step &&
                i < cfg.gust_start_step + cfg.gust_duration_steps) {
                disturbance_n = cfg.gust_force_n;
            }

            drone.update(thrust, disturbance_n, dt);
            elapsed_time += dt;

            telemetry.logState(elapsed_time, current_target_altitude, drone.getAltitude(),
                               drone.getVelocity(), thrust, disturbance_n);

            std::cout << std::fixed << std::setprecision(2)
                      << elapsed_time << "\t"
                      << current_target_altitude << "\t"
                      << drone.getAltitude() << "\t"
                      << drone.getVelocity() << "\t\t"
                      << thrust << "\t\t"
                      << disturbance_n << "\n";
        }

        std::cout << "Wrote " << cfg.telemetry_csv << "\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        std::cerr << "Hint: close the telemetry CSV in Excel/other apps if it is open, or run from a writable folder.\n";
        return 1;
    }
    return 0;
}