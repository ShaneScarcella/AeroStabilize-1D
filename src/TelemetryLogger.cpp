#include "TelemetryLogger.hpp"

#include <iomanip>
#include <stdexcept>

TelemetryLogger::TelemetryLogger(const std::string& filepath)
    : _file(filepath, std::ios::out | std::ios::trunc) {
    if (!_file.is_open()) {
        throw std::runtime_error("TelemetryLogger: could not open " + filepath);
    }
    _file << "Time_s,Target_m,Altitude_m,Velocity_m_s,Thrust_N,Disturbance_N\n";
    _file.flush();
}

TelemetryLogger::~TelemetryLogger() {
    if (_file.is_open()) {
        _file.flush();
    }
}

void TelemetryLogger::logState(double time_s, double target_alt_m, double altitude_m,
                               double velocity_m_s, double thrust_n, double disturbance_n) {
    // Keep an in-memory trace so analysis can run after simulation without reparsing CSV.
    _times_s.push_back(time_s);
    _targets_m.push_back(target_alt_m);
    _altitudes_m.push_back(altitude_m);

    _file << std::fixed << std::setprecision(6)
          << time_s << ','
          << target_alt_m << ','
          << altitude_m << ','
          << velocity_m_s << ','
          << thrust_n << ','
          << disturbance_n << '\n';
    _file.flush();
}
