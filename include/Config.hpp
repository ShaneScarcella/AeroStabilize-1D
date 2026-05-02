#pragma once

#include <vector>
#include <string>

#include "TelemetryLogger.hpp"
#include "Vector2D.hpp"

struct Waypoint {
    double time_s = 0.0;
    // Full planar pose is stored now so config stays stable when horizontal control lands; 1D flight still reads Z only.
    Vector2D position{};
};

struct Config {
    double mass_kg = 0.0;
    /** About-axis moment of inertia for pitch (kg·m²), used with aerodynamic torque in the planar model. */
    double moment_of_inertia_kg_m2 = 0.05;
    double initial_altitude_m = 0.0;
    std::vector<Waypoint> waypoints;
    double dt_s = 0.0;
    int simulation_steps = 0;
    /** When zero, no real-time pacing. When positive, wall delay per step is dt_s / realtime_multiplier. */
    double realtime_multiplier = 0.0;

    double alt_kp = 0.0;
    double alt_ki = 0.0;
    double alt_kd = 0.0;
    double pid_min_thrust_n = 0.0;
    double pid_max_thrust_n = 0.0;

    /** Inner attitude loop: torque (N·m) from pitch error; defaults match typical small-quad tuning. */
    double pitch_kp = 5.0;
    double pitch_ki = 0.0;
    double pitch_kd = 1.0;
    double pitch_max_torque_n_m = 1.0;

    /** Outer horizontal loop: desired pitch (rad) from X position error; saturates to ±pos_max_pitch_rad. */
    double pos_kp = 0.1;
    double pos_ki = 0.0;
    double pos_kd = 0.1;
    double pos_max_pitch_rad = 0.5;

    /** Optional wind / disturbance: constant force (N) for gust_duration_steps starting at gust_start_step (0-based). Zero gust_force_n or duration disables. */
    double gust_force_n = 0.0;
    int gust_start_step = 0;
    int gust_duration_steps = 0;

    /** Standard deviation of zero-mean Gaussian error (m) on altitude used only by the simulated flight computer; omitted in config = perfect sensor. */
    double sensor_noise_stddev = 0.0;

    /**
     * Weight on the latest noisy sample in the exponential moving average (EMA) low-pass applied to
     * simulated position and attitude measurements before the cascaded PIDs. Valid range is (0, 1]:
     * values near 1 respond quickly; smaller values smooth more strongly.
     */
    double filter_alpha = 1.0;

    LogLevel system_log_level = LogLevel::INFO;
    std::string telemetry_csv;

    /** Loads and validates settings from a key=value file. Throws std::runtime_error on failure. */
    static Config loadFromFile(const std::string& path);
};
