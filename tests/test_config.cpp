#include "Config.hpp"

#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

#include <filesystem>
namespace fs = std::filesystem;

namespace {

fs::path makeTempConfigPath() {
    return fs::temp_directory_path() / "aerostabilize_test_config.txt";
}

void writeFile(const fs::path& path, const std::string& contents) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(out.is_open());
    out << contents;
}

}  // namespace

TEST(Config, MissingFileThrows) {
    EXPECT_THROW(
        (void)Config::loadFromFile("nonexistent_config_file_9f3a2c1e.txt"),
        std::runtime_error);
}

TEST(Config, InvalidNumberThrows) {
    const fs::path path = makeTempConfigPath();
    writeFile(path, R"(mass_kg = not_a_number
initial_altitude_m = 0.0
target_altitude_m = 1.0
dt_s = 0.1
simulation_steps = 10
pid_kp = 1.0
pid_ki = 0.0
pid_kd = 0.0
pid_min_thrust_n = 0.0
pid_max_thrust_n = 10.0
telemetry_csv = out.csv
)");
    EXPECT_THROW((void)Config::loadFromFile(path.string()), std::runtime_error);
    fs::remove(path);
}

TEST(Config, DuplicateKeyThrows) {
    const fs::path path = makeTempConfigPath();
    writeFile(path, R"(mass_kg = 1.0
mass_kg = 2.0
initial_altitude_m = 0.0
target_altitude_m = 1.0
dt_s = 0.1
simulation_steps = 10
pid_kp = 1.0
pid_ki = 0.0
pid_kd = 0.0
pid_min_thrust_n = 0.0
pid_max_thrust_n = 10.0
telemetry_csv = out.csv
)");
    EXPECT_THROW((void)Config::loadFromFile(path.string()), std::runtime_error);
    fs::remove(path);
}

TEST(Config, ValidFileLoads) {
    const fs::path path = makeTempConfigPath();
    writeFile(path, R"(# comment
mass_kg = 2.5
initial_altitude_m = 1.0
target_altitude_m = 10.0
dt_s = 0.05
simulation_steps = 100
pid_kp = 3.0
pid_ki = 0.5
pid_kd = 1.0
pid_min_thrust_n = 0.0
pid_max_thrust_n = 25.0
telemetry_csv = flight.csv
)");
    const Config c = Config::loadFromFile(path.string());
    EXPECT_NEAR(c.mass_kg, 2.5, 1e-12);
    EXPECT_NEAR(c.initial_altitude_m, 1.0, 1e-12);
    EXPECT_NEAR(c.target_altitude_m, 10.0, 1e-12);
    EXPECT_NEAR(c.dt_s, 0.05, 1e-12);
    EXPECT_EQ(c.simulation_steps, 100);
    EXPECT_NEAR(c.pid_kp, 3.0, 1e-12);
    EXPECT_NEAR(c.pid_ki, 0.5, 1e-12);
    EXPECT_NEAR(c.pid_kd, 1.0, 1e-12);
    EXPECT_NEAR(c.pid_min_thrust_n, 0.0, 1e-12);
    EXPECT_NEAR(c.pid_max_thrust_n, 25.0, 1e-12);
    EXPECT_EQ(c.telemetry_csv, "flight.csv");
    EXPECT_NEAR(c.gust_force_n, 0.0, 1e-12);
    EXPECT_EQ(c.gust_start_step, 0);
    EXPECT_EQ(c.gust_duration_steps, 0);
    fs::remove(path);
}

TEST(Config, OptionalGustKeysLoad) {
    const fs::path path = makeTempConfigPath();
    writeFile(path, R"(mass_kg = 2.0
initial_altitude_m = 10.0
target_altitude_m = 10.0
dt_s = 0.1
simulation_steps = 50
pid_kp = 12.0
pid_ki = 4.0
pid_kd = 7.0
pid_min_thrust_n = 0.0
pid_max_thrust_n = 30.0
gust_force_n = -10.0
gust_start_step = 5
gust_duration_steps = 3
telemetry_csv = out.csv
)");
    const Config c = Config::loadFromFile(path.string());
    EXPECT_NEAR(c.gust_force_n, -10.0, 1e-12);
    EXPECT_EQ(c.gust_start_step, 5);
    EXPECT_EQ(c.gust_duration_steps, 3);
    fs::remove(path);
}
