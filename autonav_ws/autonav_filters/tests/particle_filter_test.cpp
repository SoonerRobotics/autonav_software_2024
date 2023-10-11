#include "gtest/gtest.h"
#include "autonav_filters/particle_filter.hpp"

TEST(ParticleFilterTests, initialization_test) {
    float latitudeLength = 111086.2;
    float longitudeLength = 81978.2;
    ParticleFilter particle_filter = ParticleFilter(latitudeLength, longitudeLength);
    ASSERT_EQ(particle_filter.latitudeLength, latitudeLength);
    ASSERT_EQ(particle_filter.longitudeLength, longitudeLength);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
