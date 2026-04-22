#pragma once

#include <fstream>
#include <string>
#include <vector>

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    TelemetryLogger(const TelemetryLogger&) = delete;
    TelemetryLogger& operator=(const TelemetryLogger&) = delete;

    void logState(double time_s, double target_alt_m, double altitude_m,
                  double velocity_m_s, double thrust_n, double disturbance_n);

    const std::vector<double>& getTimes() const { return _times_s; }
    const std::vector<double>& getTargets() const { return _targets_m; }
    const std::vector<double>& getAltitudes() const { return _altitudes_m; }

private:
    std::ofstream _file;
    std::vector<double> _times_s;
    std::vector<double> _targets_m;
    std::vector<double> _altitudes_m;
};
