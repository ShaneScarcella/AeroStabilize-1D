#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "Vector2D.hpp"

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3,
    NONE = 4
};

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath, LogLevel system_log_level);
    ~TelemetryLogger();

    TelemetryLogger(const TelemetryLogger&) = delete;
    TelemetryLogger& operator=(const TelemetryLogger&) = delete;

    /** Logs planar state; CSV stores pitch in degrees (PitchDeg). In-memory series keep target/true altitude (Z) for reporting. */
    void logState(double time_s, Vector2D target_pos, Vector2D true_pos, double sensed_alt_m,
                  Vector2D velocity, double pitch_rad, double thrust_n, double torque_n_m,
                  double disturbance_n);
    void print(LogLevel level, const std::string& message) const;

    const std::vector<double>& getTimes() const { return _times_s; }
    const std::vector<double>& getTargets() const { return _targets_m; }
    const std::vector<double>& getAltitudes() const { return _altitudes_m; }
    const std::vector<double>& getSensedAltitudes() const { return _sensed_altitudes_m; }

private:
    std::ofstream _file;
    LogLevel _system_log_level;
    std::vector<double> _times_s;
    std::vector<double> _targets_m;
    std::vector<double> _altitudes_m;
    std::vector<double> _sensed_altitudes_m;
};
