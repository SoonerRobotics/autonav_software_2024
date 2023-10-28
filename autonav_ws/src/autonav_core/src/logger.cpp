#include "autonav_core/logger.hpp"
#include <fstream>

void Logger::jsonToFile(nlohmann::json json_object, std::string filename) {
    std::ofstream o(filename);
    o << std::setw(4) << json_object << std::endl;
}