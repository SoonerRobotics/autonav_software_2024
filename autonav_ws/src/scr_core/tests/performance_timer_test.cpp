#include "gtest/gtest.h"
#include "scr_core/performance_timer.hpp"
#include <chrono>
#include <thread>

TEST(PerformanceTimerTests, false_stop_test) {
    SCR::PerformanceTimer test_timer = SCR::PerformanceTimer("test_timer");

    double false_stop_time = test_timer.stop();

    ASSERT_EQ(false_stop_time, 0.0);
}

TEST(PerformanceTimerTests, false_lap_test) {
    SCR::PerformanceTimer test_timer = SCR::PerformanceTimer("test_timer");

    double false_lap_time = test_timer.lap();

    ASSERT_EQ(false_lap_time, 0.0);
}

TEST(PerformanceTimerTests, quarter_second_stop_test) {
    SCR::PerformanceTimer test_timer = SCR::PerformanceTimer("test_timer");
    test_timer.start();
    std::this_thread::sleep_for(std::chrono::microseconds(250000));
    test_timer.stop();

    ASSERT_NEAR(test_timer.end_time_seconds, .25, 1e-3);
}

TEST(PerformanceTimerTests, quarter_second_lap_test) {
    SCR::PerformanceTimer test_timer = SCR::PerformanceTimer("test_timer");
    test_timer.start();

    std::this_thread::sleep_for(std::chrono::microseconds(250000));

    test_timer.lap();

    ASSERT_NEAR(test_timer.lap_times_microseconds[0], .25, 1e-2);
}

TEST(PerformanceTimerTests, multi_lap_test) {
    SCR::PerformanceTimer test_timer = SCR::PerformanceTimer("test_timer");
    test_timer.start();

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
    
    test_timer.lap();

    std::this_thread::sleep_for(std::chrono::microseconds(150000));

    test_timer.lap();

    ASSERT_NEAR(test_timer.lap_times_microseconds[0], .10, 1e-3);

    ASSERT_NEAR(test_timer.lap_times_microseconds[1], .25, 1e-3);

}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}