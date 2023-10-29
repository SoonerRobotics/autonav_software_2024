#include <string>
#include <ctime>
#include <chrono>
#include <vector>

class PerformanceTimer {
    public:
        std::vector<double> lap_times_seconds;
        double end_time_seconds;

        // constructor
        PerformanceTimer(std::string name) {
            this->name = name;
        }

        void start();

        double lap();

        double stop();

    private:
        std::string name;
        std::time_t start_time_seconds_internal;
        std::time_t lap_time_seconds_internal;
        std::time_t end_time_seconds_internal;

        bool started = false;
};