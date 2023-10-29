#include "gtest/gtest.h"
#include "autonav_core/performance_timer.hpp"
#include <chrono>
#include <thread>

TEST(PerformanceTimerTests, false_stop_test) {
    PerformanceTimer test_timer = PerformanceTimer("test_timer");

    double false_stop_time = test_timer.stop();

    ASSERT_EQ(false_stop_time, 0.0);
}

TEST(PerformanceTimerTests, false_lap_test) {
    PerformanceTimer test_timer = PerformanceTimer("test_timer");

    double false_stop_time = test_timer.lap();

    ASSERT_EQ(false_stop_time, 0.0);
}

TEST(PerformanceTimerTests, one_second_stop_test) {
    PerformanceTimer test_timer = PerformanceTimer("test_timer");
    test_timer.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    test_timer.stop();

    ASSERT_EQ(test_timer.end_time_seconds, 1);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}