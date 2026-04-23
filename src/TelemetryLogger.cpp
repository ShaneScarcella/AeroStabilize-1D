#include "TelemetryLogger.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

TelemetryLogger::TelemetryLogger(const std::string& filepath, LogLevel system_log_level)
    : _file(filepath, std::ios::out | std::ios::trunc),
      _system_log_level(system_log_level) {
    if (!_file.is_open()) {
        throw std::runtime_error("TelemetryLogger: could not open " + filepath);
    }
    _file << "Time_s,Target_m,Altitude_m,SensedAlt_m,Velocity_m_s,Thrust_N,Disturbance_N\n";
    _file.flush();
}

TelemetryLogger::~TelemetryLogger() {
    if (_file.is_open()) {
        _file.flush();
    }
}

void TelemetryLogger::logState(double time_s, double target_alt_m, double true_alt_m,
                               double sensed_alt_m, double velocity_m_s, double thrust_n,
                               double disturbance_n) {
    // Keep an in-memory trace so analysis can run after simulation without reparsing CSV.
    _times_s.push_back(time_s);
    _targets_m.push_back(target_alt_m);
    _altitudes_m.push_back(true_alt_m);
    _sensed_altitudes_m.push_back(sensed_alt_m);

    _file << std::fixed << std::setprecision(6)
          << time_s << ','
          << target_alt_m << ','
          << true_alt_m << ','
          << sensed_alt_m << ','
          << velocity_m_s << ','
          << thrust_n << ','
          << disturbance_n << '\n';
    _file.flush();
}

void TelemetryLogger::print(LogLevel level, const std::string& message) const {
    // Use one threshold gate for console output so verbosity can be tuned per run.
    if (level == LogLevel::NONE || level < _system_log_level) {
        return;
    }
    std::cout << message << '\n';
}
