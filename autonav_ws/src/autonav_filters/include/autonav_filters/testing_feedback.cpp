#include <random>
#include <iostream>
#include <ostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

double pymod(double n, double M) {
            return fmod(((fmod(n, M)) + M), M);
}

int main() {
    // pymod good
    int n = 16;
    int M = 10;

    int result = pymod(n, M);

    printf("%d\n", result);

    // trig good
    double cosine_result = cos(2*M_PI);
    printf("%f\n", cosine_result);

    // sum theta lol

    double sum_theta_x = -7.771561172376096e-16;
    double sum_theta_y = -6.286637876939949e-15;
    double sum_weight = 2;
    double avg_theta = pymod(atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight), 2 * M_PI);

    printf("%f\n", avg_theta);

}