#include "TelemetryLogger.hpp"

#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>

TelemetryLogger::TelemetryLogger(const std::string& filepath, LogLevel system_log_level)
    : _file(filepath, std::ios::out | std::ios::trunc),
      _system_log_level(system_log_level) {
    if (!_file.is_open()) {
        throw std::runtime_error("TelemetryLogger: could not open " + filepath);
    }
    _file << "Time_s,TargetX,TargetZ,TrueX,TrueZ,SensedZ,VelX,VelZ,PitchDeg,Thrust,Torque,"
             "Disturbance\n";
    _file.flush();
}

TelemetryLogger::~TelemetryLogger() {
    if (_file.is_open()) {
        _file.flush();
    }
}

void TelemetryLogger::logState(double time_s, Vector2D target_pos, Vector2D true_pos,
                               double sensed_alt_m, Vector2D velocity, double pitch_rad,
                               double thrust_n, double torque_n_m, double disturbance_n) {
    // Mirror key scalars in RAM so PerformanceAnalyzer can summarize the run without reparsing CSV.
    _times_s.push_back(time_s);
    _targets_m.push_back(target_pos.z);
    _altitudes_m.push_back(true_pos.z);
    _sensed_altitudes_m.push_back(sensed_alt_m);

    const double pitch_deg = pitch_rad * (180.0 / std::numbers::pi_v<double>);

    _file << std::fixed << std::setprecision(6)
          << time_s << ','
          << target_pos.x << ','
          << target_pos.z << ','
          << true_pos.x << ','
          << true_pos.z << ','
          << sensed_alt_m << ','
          << velocity.x << ','
          << velocity.z << ','
          << pitch_deg << ','
          << thrust_n << ','
          << torque_n_m << ','
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
