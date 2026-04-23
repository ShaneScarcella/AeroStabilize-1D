#pragma once

#include <fstream>
#include <string>
#include <vector>

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

    /** true_alt_m: physics altitude at the logged instant. sensed_alt_m: value passed to the controller (may include sensor noise). */
    void logState(double time_s, double target_alt_m, double true_alt_m, double sensed_alt_m,
                  double velocity_m_s, double thrust_n, double disturbance_n);
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
