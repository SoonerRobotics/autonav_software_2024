#include "gtest/gtest.h"
#include "autonav_core/logger.hpp"

TEST(LoggerTests, archive_test) {
    ASSERT_TRUE(true);
};

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}