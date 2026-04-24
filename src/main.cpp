#include <chrono>
#include <iostream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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
        telemetry.print(
            LogLevel::DEBUG,
            "Time(s)\tTarget\tAlt(m)\tSensed(m)\tVel(m/s)\tThrust(N)\tDist(N)");
        telemetry.print(LogLevel::DEBUG,
                        "-----------------------------------------------------------------------------"
                        "-----");

        // One RNG stream per run keeps the error sequence a single draw from the noise model. MSVC
        // checks σ > 0 in debug when constructing std::normal_distribution, so a missing key or
        // explicit 0.0 (noiseless sensor) must skip the distribution and add zero error instead.
        std::random_device random_seed;
        std::mt19937 rng(
            static_cast<std::mt19937::result_type>(random_seed()));
        std::optional<std::normal_distribution<double>> altitude_noise;
        if (cfg.sensor_noise_stddev > 0.0) {
            altitude_noise.emplace(0.0, cfg.sensor_noise_stddev);
        }

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

            // Sample physics truth at the start of this tick; the controller only receives a
            // perturbed view to model noisy sensors, and we log that same instant before integration.
            const double true_altitude = drone.getAltitude();
            const double true_velocity = drone.getVelocity();
            const double noise_m = altitude_noise ? (*altitude_noise)(rng) : 0.0;
            const double noisy_altitude = true_altitude + noise_m;

            double disturbance_n = 0.0;
            if (cfg.gust_force_n != 0.0 && cfg.gust_duration_steps > 0 &&
                i >= cfg.gust_start_step &&
                i < cfg.gust_start_step + cfg.gust_duration_steps) {
                disturbance_n = cfg.gust_force_n;
            }

            const double thrust =
                flightComputer.calculate(current_target_altitude, noisy_altitude, dt);

            telemetry.logState(elapsed_time, current_target_altitude, true_altitude, noisy_altitude,
                               true_velocity, thrust, disturbance_n);

            std::ostringstream tick_row;
            tick_row << std::fixed << std::setprecision(2)
                     << elapsed_time << "\t"
                     << current_target_altitude << "\t"
                     << true_altitude << "\t"
                     << noisy_altitude << "\t"
                     << true_velocity << "\t\t"
                     << thrust << "\t\t"
                     << disturbance_n;
            telemetry.print(LogLevel::DEBUG, tick_row.str());

            drone.update(thrust, disturbance_n, dt);
            elapsed_time += dt;
            if (cfg.realtime_multiplier > 0.0) {
                const auto step_wall =
                    std::chrono::duration<double>(dt / cfg.realtime_multiplier);
                const auto as_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(step_wall);
                std::this_thread::sleep_for(as_ns);
            }
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