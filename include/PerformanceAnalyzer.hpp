#pragma once

#include <vector>

struct FlightReport {
    double max_overshoot_m = 0.0;
    double settling_time_s = 0.0;
    double steady_state_error_m = 0.0;
    double rmse = 0.0;
};

class PerformanceAnalyzer {
public:
    FlightReport generateReport(const std::vector<double>& times,
                                const std::vector<double>& targets,
                                const std::vector<double>& actuals);
};
