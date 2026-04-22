#include "PerformanceAnalyzer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

constexpr double kEps = 1e-9;

}  // namespace

TEST(PerformanceAnalyzer, ComputesAllMetricsForTypicalResponse) {
    const std::vector<double> times{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> targets{10.0, 10.0, 10.0, 10.0, 10.0};
    const std::vector<double> actuals{0.0, 12.0, 10.2, 10.05, 10.0};

    PerformanceAnalyzer analyzer;
    const FlightReport report = analyzer.generateReport(times, targets, actuals);

    const double expected_rmse =
        std::sqrt((100.0 + 4.0 + 0.04 + 0.0025 + 0.0) / static_cast<double>(times.size()));

    EXPECT_NEAR(report.max_overshoot_m, 2.0, kEps);
    EXPECT_NEAR(report.settling_time_s, 3.0, kEps);
    EXPECT_NEAR(report.steady_state_error_m, 0.0, kEps);
    EXPECT_NEAR(report.rmse, expected_rmse, kEps);
}

TEST(PerformanceAnalyzer, ReturnsEndTimeWhenNeverSettled) {
    const std::vector<double> times{0.0, 1.0, 2.0};
    const std::vector<double> targets{5.0, 5.0, 5.0};
    const std::vector<double> actuals{4.6, 5.2, 4.8};

    PerformanceAnalyzer analyzer;
    const FlightReport report = analyzer.generateReport(times, targets, actuals);

    EXPECT_NEAR(report.settling_time_s, 2.0, kEps);
}

TEST(PerformanceAnalyzer, HandlesNoPositiveOvershoot) {
    const std::vector<double> times{0.0, 1.0, 2.0};
    const std::vector<double> targets{8.0, 8.0, 8.0};
    const std::vector<double> actuals{7.0, 7.5, 8.0};

    PerformanceAnalyzer analyzer;
    const FlightReport report = analyzer.generateReport(times, targets, actuals);

    EXPECT_NEAR(report.max_overshoot_m, 0.0, kEps);
    EXPECT_NEAR(report.steady_state_error_m, 0.0, kEps);
}

TEST(PerformanceAnalyzer, ThrowsForMismatchedVectorSizes) {
    const std::vector<double> times{0.0, 1.0};
    const std::vector<double> targets{1.0};
    const std::vector<double> actuals{1.0, 1.0};

    PerformanceAnalyzer analyzer;
    EXPECT_THROW((void)analyzer.generateReport(times, targets, actuals), std::invalid_argument);
}
