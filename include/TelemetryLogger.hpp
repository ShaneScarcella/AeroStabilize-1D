#pragma once

#include <fstream>
#include <string>

class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filepath);
    ~TelemetryLogger();

    TelemetryLogger(const TelemetryLogger&) = delete;
    TelemetryLogger& operator=(const TelemetryLogger&) = delete;

    void logState(double time_s, double target_alt_m, double altitude_m,
                  double velocity_m_s, double thrust_n, double disturbance_n);

private:
    std::ofstream _file;
};
