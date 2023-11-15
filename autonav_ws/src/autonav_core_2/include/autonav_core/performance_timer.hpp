#include <string>
#include <ctime>
#include <chrono>
#include <vector>

namespace SCR {
    class PerformanceTimer {
        public:
            std::vector<double> lap_times_microseconds;
            double end_time_seconds;

            // constructor
            PerformanceTimer(std::string name) {
                this->name = name;
            }

            void start();

            std::chrono::milliseconds::rep lap();

            std::chrono::milliseconds::rep stop();

        private:
            std::string name;
            std::chrono::high_resolution_clock::time_point start_time_microseconds_internal;
            std::chrono::high_resolution_clock::time_point lap_time_microseconds_internal;
            std::chrono::high_resolution_clock::time_point end_time_microseconds_internal;

            bool started = false;
    };
}