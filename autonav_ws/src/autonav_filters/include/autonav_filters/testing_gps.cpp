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
    std::vector<int> range = {1, 1, 1, 1, 1, 100};
    std::discrete_distribution<int> discrete(range.begin(), range.end());
    int index = discrete(generator);
    int value = range[index];
    
    printf("index: %d\n", index);
    printf("value: %d\n", value);
    return index;
}

double pymod(double n, double M) {
    return fmodl(((fmodl(n, M)) + M), M);
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
    std::vector<int> range = {10, 30, 20, 25, 15};
    std::discrete_distribution<int> discrete(range.begin(), range.end());
    std::vector<double> p = discrete.probabilities();
    double sum = 0.0;
    for (auto n : p) {
        sum = sum + n;
        std::cout << n << ' ';
    }
    std::cout << "sum: " << sum << std::endl;
    
    std::vector<int> indices;
    std::cout << '\n';
    std::cout << sum;
    for (int i = 0; i < 1000; i++) {
        int index = discrete(generator);
        int value = range[index];
        indices.push_back(index);
        //printf("index: %d\n", index);
        //printf("value: %d\n", value);
    }

    int one_count;
    int twos_count;
    int threes_count;
    int fours_count;
    int fives_count;
    for (int idx : indices) {
        if (idx == 0) {
            one_count++;
        }
        else if (idx == 1) {
            twos_count++;
        }
        else if (idx == 2) {
            threes_count++;
        }  
        else if (idx == 3) {
            fours_count++;
        }
        else if (idx == 4) {
            fives_count++;
        }
    }

    std::cout << "number of ones: " << one_count << std::endl;
    std::cout << "number of twos: " << twos_count << std::endl;
    std::cout << "number of threes: " << threes_count << std::endl;
    std::cout << "number of fours: " << fours_count << std::endl;    
    std::cout << "number of fives: " << fives_count << std::endl;    
    //std::vector<int> indexes;
    //for (int i = 0; i < 10; i++) {
    //    indexes.push_back(num_generator());
    // }

    // accumulate
    std::vector<double> weights;
    weights.push_back(0.993333);

    double weights_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    printf("weights sum: %f\n", weights_sum);

    //exponential
    double dist_sqrt = .063345;
    double exponential_result = exp(-1 * dist_sqrt / (2 * pow(0.45, 2)));

    printf("exponential_result: %f\n", exponential_result);

    double theta = 0.0;

    for (int i=0; i < 10; i++) {
        std::normal_distribution<> normal_distribution_theta{theta, 0.05};
        double theta = pymod(normal_distribution_theta(generator), (2* M_PI));
        //theta = 2.0;
        printf("theta: %f\n", theta);
    }

    std::vector<int> test_vec;

    test_vec.push_back(1);
    test_vec.push_back(2);

    std::cout << test_vec[0] << std::endl;
    std::cout << test_vec[1] << std::endl;

}