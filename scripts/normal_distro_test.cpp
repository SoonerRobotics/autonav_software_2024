#include <random>
#include <iostream>
#include <chrono>

int main() {
    std::normal_distribution<double> myDistribution(0, 0.1);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::cout << myDistribution(generator) << std::endl;
    std::cout << myDistribution(generator) << std::endl;
}