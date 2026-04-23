#include "Config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace {

// Remove leading and trailing whitespace from a string
std::string trim(std::string s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.erase(0, 1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.pop_back();
    }
    return s;
}

double parseDouble(const std::string& key, const std::string& value) {
    try {
        size_t idx = 0;
        double v = std::stod(value, &idx);
        if (idx != value.size()) {
            throw std::invalid_argument("trailing junk");
        }
        return v;
    } catch (const std::exception&) {
        throw std::runtime_error("Config: invalid number for '" + key + "': " + value);
    }
}

int parseInt(const std::string& key, const std::string& value) {
    try {
        size_t idx = 0;
        long v = std::stol(value, &idx, 10);
        if (idx != value.size()) {
            throw std::invalid_argument("trailing junk");
        }
        if (v > static_cast<long>(std::numeric_limits<int>::max()) ||
            v < static_cast<long>(std::numeric_limits<int>::min())) {
            throw std::out_of_range("out of int range");
        }
        return static_cast<int>(v);
    } catch (const std::exception&) {
        throw std::runtime_error("Config: invalid integer for '" + key + "': " + value);
    }
}

Waypoint parseWaypoint(const std::string& value) {
    const auto comma = value.find(',');
    if (comma == std::string::npos) {
        throw std::runtime_error("Config: invalid waypoint format, expected 'time_s, altitude_m': " +
                                 value);
    }

    const std::string time_raw = trim(value.substr(0, comma));
    const std::string altitude_raw = trim(value.substr(comma + 1));
    if (time_raw.empty() || altitude_raw.empty()) {
        throw std::runtime_error("Config: invalid waypoint format, expected non-empty time and altitude: " +
                                 value);
    }

    const double time_s = parseDouble("waypoint.time_s", time_raw);
    const double altitude_m = parseDouble("waypoint.altitude_m", altitude_raw);
    return Waypoint{time_s, altitude_m};
}

LogLevel parseLogLevelOrDefaultInfo(const std::string& raw_value) {
    std::string normalized = trim(raw_value);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (normalized == "DEBUG") {
        return LogLevel::DEBUG;
    }
    if (normalized == "INFO") {
        return LogLevel::INFO;
    }
    if (normalized == "WARN") {
        return LogLevel::WARN;
    }
    if (normalized == "ERROR") {
        return LogLevel::ERROR;
    }
    if (normalized == "NONE") {
        return LogLevel::NONE;
    }

    // Falling back keeps older configs working while still allowing strict parsing elsewhere.
    return LogLevel::INFO;
}

}  // namespace

// Loads and validates settings from a key=value file. Throws std::runtime_error on failure.
Config Config::loadFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Config: cannot open file: " + path);
    }

    std::unordered_map<std::string, std::string> raw;
    std::string line;
    unsigned line_no = 0;

    while (std::getline(in, line)) {
        ++line_no;
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            throw std::runtime_error("Config: line " + std::to_string(line_no) +
                                     ": expected key = value, got: " + line);
        }

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if (key.empty()) {
            throw std::runtime_error("Config: line " + std::to_string(line_no) + ": empty key");
        }
        if (key == "waypoint") {
            continue;
        }
        if (raw.contains(key)) {
            throw std::runtime_error("Config: duplicate key '" + key + "' at line " +
                                     std::to_string(line_no));
        }
        raw.emplace(std::move(key), std::move(value));
    }

    auto require = [&](const char* key) -> const std::string& {
        auto it = raw.find(key);
        if (it == raw.end()) {
            throw std::runtime_error(std::string("Config: missing key '") + key + "' in " + path);
        }
        return it->second;
    };

    Config c;
    c.mass_kg = parseDouble("mass_kg", require("mass_kg"));
    c.initial_altitude_m = parseDouble("initial_altitude_m", require("initial_altitude_m"));
    c.dt_s = parseDouble("dt_s", require("dt_s"));
    c.simulation_steps = parseInt("simulation_steps", require("simulation_steps"));
    c.pid_kp = parseDouble("pid_kp", require("pid_kp"));
    c.pid_ki = parseDouble("pid_ki", require("pid_ki"));
    c.pid_kd = parseDouble("pid_kd", require("pid_kd"));
    c.pid_min_thrust_n = parseDouble("pid_min_thrust_n", require("pid_min_thrust_n"));
    c.pid_max_thrust_n = parseDouble("pid_max_thrust_n", require("pid_max_thrust_n"));
    c.telemetry_csv = require("telemetry_csv");

    auto optionalDouble = [&](const char* key, double default_val) -> double {
        auto it = raw.find(key);
        if (it == raw.end()) {
            return default_val;
        }
        return parseDouble(key, it->second);
    };
    auto optionalInt = [&](const char* key, int default_val) -> int {
        auto it = raw.find(key);
        if (it == raw.end()) {
            return default_val;
        }
        return parseInt(key, it->second);
    };

    c.gust_force_n = optionalDouble("gust_force_n", 0.0);
    c.gust_start_step = optionalInt("gust_start_step", 0);
    c.gust_duration_steps = optionalInt("gust_duration_steps", 0);
    c.sensor_noise_stddev = optionalDouble("sensor_noise_stddev", 0.0);
    c.system_log_level = parseLogLevelOrDefaultInfo(
        raw.contains("log_level") ? raw.at("log_level") : "INFO");

    in.clear();
    in.seekg(0);
    line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        if (key == "waypoint") {
            c.waypoints.push_back(parseWaypoint(value));
        }
    }

    std::sort(c.waypoints.begin(), c.waypoints.end(),
              [](const Waypoint& a, const Waypoint& b) { return a.time_s < b.time_s; });
    if (c.waypoints.empty()) {
        throw std::runtime_error("Config: at least one waypoint is required");
    }

    if (c.mass_kg <= 0.0) {
        throw std::runtime_error("Config: mass_kg must be positive");
    }
    if (c.dt_s <= 0.0 || !std::isfinite(c.dt_s)) {
        throw std::runtime_error("Config: dt_s must be a positive finite number");
    }
    if (c.simulation_steps <= 0) {
        throw std::runtime_error("Config: simulation_steps must be positive");
    }
    if (c.pid_max_thrust_n < c.pid_min_thrust_n) {
        throw std::runtime_error("Config: pid_max_thrust_n must be >= pid_min_thrust_n");
    }
    if (c.gust_duration_steps < 0) {
        throw std::runtime_error("Config: gust_duration_steps must be >= 0");
    }
    if (c.gust_start_step < 0) {
        throw std::runtime_error("Config: gust_start_step must be >= 0");
    }
    if (c.sensor_noise_stddev < 0.0 || !std::isfinite(c.sensor_noise_stddev)) {
        throw std::runtime_error(
            "Config: sensor_noise_stddev must be non-negative and finite (std::normal_distribution "
            "rejects a negative standard deviation)");
    }

    return c;
}
