#include "PIDController.hpp"

#include <gtest/gtest.h>

namespace {

constexpr double kEps = 1e-9;

}  // namespace

TEST(PIDController, ClampsOutputToMax) {
    PIDController pid(1.0, 0.0, 0.0, 0.0, 30.0);
    const double out = pid.calculate(1000.0, 0.0, 0.01);
    EXPECT_NEAR(out, 30.0, kEps);
}

TEST(PIDController, ClampsOutputToMin) {
    PIDController pid(1.0, 0.0, 0.0, 0.0, 30.0);
    const double out = pid.calculate(-1000.0, 0.0, 0.01);
    EXPECT_NEAR(out, 0.0, kEps);
}

TEST(PIDController, SymmetricLimits) {
    PIDController pid(1.0, 0.0, 0.0, -10.0, 10.0);
    EXPECT_NEAR(pid.calculate(1000.0, 0.0, 0.01), 10.0, kEps);
    EXPECT_NEAR(pid.calculate(-1000.0, 0.0, 0.01), -10.0, kEps);
}

TEST(PIDController, ResetClearsState) {
    PIDController pid(0.0, 1.0, 0.0, 0.0, 100.0);
    (void)pid.calculate(10.0, 0.0, 0.1);
    pid.reset();
    // After reset, first derivative term should be zero again (no spike from stale _last_error).
    const double out = pid.calculate(10.0, 0.0, 0.1);
    EXPECT_NEAR(out, 1.0, 1e-6);  // Ki * integral only: error=10, int=10*0.1=1 -> 1.0
}

TEST(PIDController, OutputStaysWithinLimitsUnderSaturation) {
    PIDController pid(1.0, 5.0, 0.0, 0.0, 30.0);
    for (int i = 0; i < 500; ++i) {
        const double out = pid.calculate(1000.0, 0.0, 0.01);
        EXPECT_GE(out, 0.0);
        EXPECT_LE(out, 30.0);
    }
}
