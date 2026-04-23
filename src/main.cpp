#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cstddef>

#include "Config.hpp"
#include "PhysicsEngine.hpp"
#include "PIDController.hpp"
#include "PerformanceAnalyzer.hpp"
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

        TelemetryLogger telemetry(cfg.telemetry_csv, cfg.system_log_level);
        telemetry.print(LogLevel::INFO, "Starting simulation");
        telemetry.print(LogLevel::DEBUG, "Time(s)\tTarget\tAlt(m)\tVel(m/s)\tThrust(N)\tDist(N)");
        telemetry.print(LogLevel::DEBUG, "-----------------------------------------------------------------------------");

        for (int i = 0; i < cfg.simulation_steps; ++i) {
            while (next_waypoint_index < cfg.waypoints.size() &&
                   elapsed_time >= cfg.waypoints[next_waypoint_index].time_s) {
                current_target_altitude = cfg.waypoints[next_waypoint_index].altitude_m;
                std::ostringstream waypoint_message;
                waypoint_message << std::fixed << std::setprecision(2)
                                 << "Crossed waypoint at t=" << cfg.waypoints[next_waypoint_index].time_s
                                 << "s, target altitude now "
                                 << current_target_altitude << " m";
                telemetry.print(LogLevel::INFO, waypoint_message.str());
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

            std::ostringstream tick_row;
            tick_row << std::fixed << std::setprecision(2)
                     << elapsed_time << "\t"
                     << current_target_altitude << "\t"
                     << drone.getAltitude() << "\t"
                     << drone.getVelocity() << "\t\t"
                     << thrust << "\t\t"
                     << disturbance_n;
            telemetry.print(LogLevel::DEBUG, tick_row.str());
        }

        PerformanceAnalyzer analyzer;
        const FlightReport report = analyzer.generateReport(
            telemetry.getTimes(), telemetry.getTargets(), telemetry.getAltitudes());

        telemetry.print(LogLevel::INFO, "");
        telemetry.print(LogLevel::INFO, "Performance Report");
        telemetry.print(LogLevel::INFO, "------------------");
        std::ostringstream report_rows;
        report_rows << std::fixed << std::setprecision(4)
                    << "Max overshoot (m):      " << report.max_overshoot_m << "\n"
                    << "Settling time (s):      " << report.settling_time_s << "\n"
                    << "Steady-state error (m): " << report.steady_state_error_m << "\n"
                    << "RMSE (m):               " << report.rmse;
        telemetry.print(LogLevel::INFO, report_rows.str());

        telemetry.print(LogLevel::INFO, "Wrote " + cfg.telemetry_csv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        std::cerr << "Hint: close the telemetry CSV in Excel/other apps if it is open, or run from a writable folder.\n";
        return 1;
    }
    return 0;
}