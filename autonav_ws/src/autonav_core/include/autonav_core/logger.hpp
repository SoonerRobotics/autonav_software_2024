#include "autonav_core/json.hpp"
#include <fstream>

class Logger {
    public:
        // constructor
        Logger() {}

        // log an object to a file
        template<typename T>
        void archive(T object, std::string filename) {
            nlohmann::json json_object = toJson(object);
            jsonToFile(json_object, filename);
        }

    private:
        // convert an object to json
        template<typename T>
        nlohmann::json toJson(T object) {
            nlohmann::json json_object = object;
            return object;
        }

        // log a json object to a file
        void jsonToFile(nlohmann::json json_object, std::string filename) {
            std::ofstream o(filename);
            o << std::setw(4) << json_object << std::endl;
        }
};