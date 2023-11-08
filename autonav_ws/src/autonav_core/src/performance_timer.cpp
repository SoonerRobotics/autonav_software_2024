#include "autonav_core/performance_timer.hpp"
#include <chrono>
#include <iostream>

namespace SCR {
    void PerformanceTimer::start() {
        this->started = true;
        auto start = std::chrono::system_clock::now();
        this->start_time_seconds_internal = std::chrono::system_clock::to_time_t(start);
    }

    double PerformanceTimer::lap() {
        if (this->started) {
            auto lap = std::chrono::system_clock::now();
            this->lap_time_seconds_internal = std::chrono::system_clock::to_time_t(lap);
            double time_delta_seconds = difftime(this->lap_time_seconds_internal, this->start_time_seconds_internal);

            this->lap_times_seconds.push_back(time_delta_seconds);
            return time_delta_seconds;
        } else {
            std::cout << "timer " << this->name << " not started!" << std::endl;
            return 0;
        }
    }

    double PerformanceTimer::stop() {
        if (this->started) {
            auto stop = std::chrono::system_clock::now();
            this->end_time_seconds_internal = std::chrono::system_clock::to_time_t(stop);
            double time_delta_seconds = difftime(this->end_time_seconds_internal, this->start_time_seconds_internal);
            started = false;

            this->end_time_seconds = time_delta_seconds;
            return time_delta_seconds;
        } else {
            std::cout << "timer " << this->name << " not started!" << std::endl;
            return 0;
        }
    }
}