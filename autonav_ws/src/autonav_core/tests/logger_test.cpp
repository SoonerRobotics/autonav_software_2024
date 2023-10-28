#include "gtest/gtest.h"
#include "autonav_core/logger.hpp"

TEST(LoggerTests, archive_test) {
    Logger my_logger = Logger();

    struct person {
        std::string name;
        std::string mom;
        std::string dad;
    } person_1;

    person_1.name = "Henry";
    person_1.mom = "Jane";
    person_1.dad = "John";

    my_logger.archive(person_1, "Henry.json");
};