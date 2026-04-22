add_executable(aerostabilize_tests
    test_pid.cpp
    test_physics.cpp
    test_config.cpp
    test_analyzer.cpp
)
target_link_libraries(aerostabilize_tests PRIVATE aerostabilize_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(aerostabilize_tests)
