#include "scr_core/performance_timer.hpp"
#include <chrono>
#include <iostream>

namespace SCR {
    void PerformanceTimer::start() {
        this->started = true;
        auto start = std::chrono::high_resolution_clock::now();
        this->start_time_microseconds_internal = start;
    }

    std::chrono::milliseconds::rep PerformanceTimer::lap() {
        if (this->started) {
            auto lap = std::chrono::high_resolution_clock::now();
            this->lap_time_microseconds_internal = lap;

            auto difference = this->lap_time_microseconds_internal - this->start_time_microseconds_internal;
            std::chrono::milliseconds::rep time_delta_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(difference).count();
            double time_delta_seconds = time_delta_microseconds * 1e-6;

            this->lap_times_microseconds.push_back(time_delta_seconds);
            return time_delta_microseconds;
        } else {
            std::cout << "timer " << this->name << " not started!" << std::endl;
            return 0;
        }
    }

    std::chrono::milliseconds::rep PerformanceTimer::stop() {
        if (this->started) {
            auto stop = std::chrono::high_resolution_clock::now();
            this->end_time_microseconds_internal = stop;

            auto difference = this->end_time_microseconds_internal - this->start_time_microseconds_internal;
            std::chrono::milliseconds::rep time_delta_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(difference).count();
            double time_delta_seconds = time_delta_microseconds * 1e-6;

            started = false;
            this->end_time_seconds = time_delta_seconds;
            return time_delta_seconds;

        } else {
            std::cout << "timer " << this->name << " not started!" << std::endl;
            return 0;
        }
    }
}