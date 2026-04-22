#include "PerformanceAnalyzer.hpp"

#include <cmath>
#include <stdexcept>

namespace {

constexpr double kSettlingBandMeters = 0.1;

}  // namespace

FlightReport PerformanceAnalyzer::generateReport(const std::vector<double>& times,
                                                 const std::vector<double>& targets,
                                                 const std::vector<double>& actuals) {
    if (times.size() != targets.size() || times.size() != actuals.size()) {
        throw std::invalid_argument("PerformanceAnalyzer: input vectors must have the same length");
    }
    if (times.empty()) {
        throw std::invalid_argument("PerformanceAnalyzer: input vectors must not be empty");
    }

    FlightReport report{};

    double squared_error_sum = 0.0;
    for (std::size_t i = 0; i < times.size(); ++i) {
        const double error = actuals[i] - targets[i];
        squared_error_sum += error * error;

        // Overshoot is only meaningful when altitude goes above the commanded target.
        const double overshoot = actuals[i] - targets[i];
        if (overshoot > report.max_overshoot_m) {
            report.max_overshoot_m = overshoot;
        }
    }

    report.rmse = std::sqrt(squared_error_sum / static_cast<double>(times.size()));
    report.steady_state_error_m = std::abs(actuals.back() - targets.back());

    bool found_settled_window = false;
    for (std::size_t i = 0; i < times.size(); ++i) {
        bool remains_in_band = true;
        for (std::size_t j = i; j < times.size(); ++j) {
            if (std::abs(actuals[j] - targets[j]) > kSettlingBandMeters) {
                remains_in_band = false;
                break;
            }
        }

        // First index that stays in-band to the end is the settling time by definition.
        if (remains_in_band) {
            report.settling_time_s = times[i];
            found_settled_window = true;
            break;
        }
    }

    if (!found_settled_window) {
        // If it never settles, expose the full run duration to signal unmet stability.
        report.settling_time_s = times.back();
    }

    return report;
}
