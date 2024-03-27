#include <random>
#include <iostream>
#include <ostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

int num_generator() {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::vector<int> range = {1, 5, 10, 15, 20, 25};
    std::discrete_distribution<int> discrete(range.begin(), range.end());
    int index = discrete(generator);
    int value = range[index];
    
    printf("index: %d\n", index);
    printf("value: %d\n", value);
    return index;
}

int main() {
    // distance
    double particles_x;
    double particles_y;

    double gps_x;
    double gps_y;

    double distance = sqrt(pow((particles_x - gps_x), double(2)) + pow((particles_y- gps_y), double(2)));

    printf("distance: %f\n", distance);

    // random shit

    std::random_device rd;
    std::mt19937 generator(rd());
    std::vector<int> range = {1, 5, 10, 15, 20, 25};
    std::discrete_distribution<int> discrete(range.begin(), range.end());
    std::vector<double> p = discrete.probabilities();
    double sum = 0.0;
    for (auto n : p) {
        sum = sum + n;
        std::cout << n << ' ';
    }
    
    std::cout << '\n';
    std::cout << sum;
    for (int i = 0; i < 1; i++) {
        int index = discrete(generator);
        int value = range[index];
        printf("index: %d\n", index);
        printf("value: %d\n", value);
    }
    std::vector<int> indexes;
    for (int i = 0; i < 10; i++) {
        indexes.push_back(num_generator());
    }

    // accumulate
    std::vector<double> weights;
    weights.push_back(0.993333);

    double weights_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    printf("weights sum: %f\n", weights_sum);

    //exponential
    double dist_sqrt = .063345;
    double exponential_result = exp(-1 * dist_sqrt / (2 * pow(0.45, 2)));

    printf("exponential_result: %f\n", exponential_result);

}