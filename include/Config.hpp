#pragma once

#include <vector>
#include <string>

#include "TelemetryLogger.hpp"

struct Waypoint {
    double time_s = 0.0;
    double altitude_m = 0.0;
};

struct Config {
    double mass_kg = 0.0;
    double initial_altitude_m = 0.0;
    std::vector<Waypoint> waypoints;
    double dt_s = 0.0;
    int simulation_steps = 0;

    double pid_kp = 0.0;
    double pid_ki = 0.0;
    double pid_kd = 0.0;
    double pid_min_thrust_n = 0.0;
    double pid_max_thrust_n = 0.0;

    /** Optional wind / disturbance: constant force (N) for gust_duration_steps starting at gust_start_step (0-based). Zero gust_force_n or duration disables. */
    double gust_force_n = 0.0;
    int gust_start_step = 0;
    int gust_duration_steps = 0;

    /** Standard deviation of zero-mean Gaussian error (m) on altitude used only by the simulated flight computer; omitted in config = perfect sensor. */
    double sensor_noise_stddev = 0.0;

    LogLevel system_log_level = LogLevel::INFO;
    std::string telemetry_csv;

    /** Loads and validates settings from a key=value file. Throws std::runtime_error on failure. */
    static Config loadFromFile(const std::string& path);
};
