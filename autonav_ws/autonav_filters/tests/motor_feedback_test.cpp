#include "gtest/gtest.h"
#include "autonav_filters/motor_feedback_example.hpp"

TEST(ParticleFilterTests, MotorFeedbackTest) {
    PositionPublisher position_publisher;
    EXPECT_TRUE(false);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}