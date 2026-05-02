#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <cstddef>
#include <numbers>

#include "Config.hpp"
#include "PhysicsEngine.hpp"
#include "PIDController.hpp"
#include "PerformanceAnalyzer.hpp"
#include "TelemetryLogger.hpp"
#include "Vector2D.hpp"

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

        PhysicsEngine drone(cfg.mass_kg, cfg.moment_of_inertia_kg_m2, cfg.initial_altitude_m);

        // Cascaded architecture: outer X-position loop → desired pitch; inner pitch loop → torque;
        // altitude loop runs in parallel and commands total body thrust (with tilt compensation).
        PIDController alt_pid(cfg.alt_kp, cfg.alt_ki, cfg.alt_kd, cfg.pid_min_thrust_n,
                              cfg.pid_max_thrust_n);
        PIDController pos_pid(cfg.pos_kp, cfg.pos_ki, cfg.pos_kd, -cfg.pos_max_pitch_rad,
                              cfg.pos_max_pitch_rad);
        PIDController pitch_pid(cfg.pitch_kp, cfg.pitch_ki, cfg.pitch_kd,
                                -cfg.pitch_max_torque_n_m, cfg.pitch_max_torque_n_m);

        const double dt = cfg.dt_s;
        double elapsed_time = 0.0;
        double current_target_altitude = cfg.waypoints.front().position.z;
        double current_target_x = cfg.waypoints.front().position.x;
        std::size_t next_waypoint_index = 0;

        TelemetryLogger telemetry(cfg.telemetry_csv, cfg.system_log_level);
        telemetry.print(LogLevel::INFO, "Starting simulation");

        telemetry.print(LogLevel::DEBUG,
                        "Time(s)\tTgtX\tTgtZ\tTrueX\tTrueZ\tSensZ\tVelX\tVelZ\tPitchDeg\t"
                        "Thrust\tTorque\tDist");
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

        // Horizontal and vertical position filters start at the initial pose so the first EMA outputs
        // match the first noisy samples when the drone begins at rest at the configured altitude.
        double filtered_x = drone.getPosition().x;
        double filtered_z = drone.getPosition().z;

        for (int i = 0; i < cfg.simulation_steps; ++i) {
            while (next_waypoint_index < cfg.waypoints.size() &&
                   elapsed_time >= cfg.waypoints[next_waypoint_index].time_s) {
                current_target_altitude =
                    cfg.waypoints[next_waypoint_index].position.z;
                current_target_x = cfg.waypoints[next_waypoint_index].position.x;
                std::ostringstream waypoint_message;
                waypoint_message << std::fixed << std::setprecision(2)
                                 << "Crossed waypoint at t=" << cfg.waypoints[next_waypoint_index].time_s
                                 << "s, target altitude now "
                                 << current_target_altitude << " m";
                telemetry.print(LogLevel::INFO, waypoint_message.str());
                ++next_waypoint_index;
            }

            const Vector2D true_pos = drone.getPosition();
            const double pitch_rad = drone.getPitchRad();

            // Altitude uses the configured Gaussian error model; horizontal position and pitch are taken as
            // noise-free here so the single σ knob continues to mean altitude uncertainty only.
            const double noise_z_m = altitude_noise ? (*altitude_noise)(rng) : 0.0;
            const double noisy_x = true_pos.x;
            const double noisy_z = true_pos.z + noise_z_m;

            // First-order low-pass (EMA) on measured horizontal and vertical position reduces altitude
            // and X noise before the outer loops. Pitch is not low-passed: blending attitude would add
            // phase lag in the inner loop and hurt stability, so the attitude PID always sees the
            // current body pitch from the simulation.
            filtered_x =
                cfg.filter_alpha * noisy_x + (1.0 - cfg.filter_alpha) * filtered_x;
            filtered_z =
                cfg.filter_alpha * noisy_z + (1.0 - cfg.filter_alpha) * filtered_z;
            const double filtered_pitch_rad = pitch_rad;

            double disturbance_n = 0.0;
            if (cfg.gust_force_n != 0.0 && cfg.gust_duration_steps > 0 &&
                i >= cfg.gust_start_step &&
                i < cfg.gust_start_step + cfg.gust_duration_steps) {
                disturbance_n = cfg.gust_force_n;
            }

            // Outer loop (horizontal position): X error → desired pitch; +X error requires +pitch (nose toward +X).
            const double desired_pitch_rad =
                pos_pid.calculate(current_target_x, filtered_x, dt);

            // Inner loop (pitch attitude): pitch error → aerodynamic / control torque about body Y (planar model).
            const double torque_n_m =
                pitch_pid.calculate(desired_pitch_rad, filtered_pitch_rad, dt);

            // Altitude loop: Z error → body-axis thrust from alt PID; divide by cos(pitch) so vertical lift matches demand when tilted.
            const double alt_thrust_body_n =
                alt_pid.calculate(current_target_altitude, filtered_z, dt);
            constexpr double kMinAbsCos = 1e-5;
            const double cos_pitch = std::cos(pitch_rad);
            const double cos_safe =
                std::abs(cos_pitch) < kMinAbsCos ? (cos_pitch >= 0.0 ? kMinAbsCos : -kMinAbsCos)
                                                  : cos_pitch;
            double thrust = alt_thrust_body_n / cos_safe;
            thrust = std::clamp(thrust, cfg.pid_min_thrust_n, cfg.pid_max_thrust_n);

            const Vector2D target_pos{current_target_x, current_target_altitude};
            const Vector2D velocity = drone.getVelocity();
            telemetry.logState(elapsed_time, target_pos, true_pos, noisy_z, velocity,
                               pitch_rad, thrust, torque_n_m, disturbance_n);

            const double pitch_deg = pitch_rad * (180.0 / std::numbers::pi_v<double>);
            std::ostringstream tick_row;
            tick_row << std::fixed << std::setprecision(2)
                     << elapsed_time << "\t"
                     << target_pos.x << "\t"
                     << target_pos.z << "\t"
                     << true_pos.x << "\t"
                     << true_pos.z << "\t"
                     << noisy_z << "\t"
                     << velocity.x << "\t"
                     << velocity.z << "\t"
                     << pitch_deg << "\t"
                     << thrust << "\t"
                     << torque_n_m << "\t"
                     << disturbance_n;
            telemetry.print(LogLevel::DEBUG, tick_row.str());

            drone.update(thrust, torque_n_m, disturbance_n, dt);
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
