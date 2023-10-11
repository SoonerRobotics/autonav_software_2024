#include "gtest/gtest.h"
#include "autonav_filters/motor_feedback_example.hpp"
#include "autonav_messages/msg/motor_feedback.hpp"

TEST(ParticleFilterTests, MotorFeedbackTest) {
    ASSERT_TRUE(true);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}