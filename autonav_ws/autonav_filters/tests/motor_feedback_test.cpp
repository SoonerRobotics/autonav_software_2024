#include "gtest/gtest.h"
#include "autonav_filters/particle_filter.hpp"

TEST(ParticleFilterTests, MotorFeedbackTest) {
    float latitudeLength = 111086.2;
    float longitudeLength = 81978.2;
    ParticleFilter particle_filter{latitudeLength, longitudeLength};
    EXPECT_TRUE(false);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}