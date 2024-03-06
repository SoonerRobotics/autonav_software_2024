#include <random>
#include <iostream>
#include <ostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

int main() {
    // distance
    double particles_x;
    double particles_y;

    double gps_x;
    double gps_y;

    double distance = sqrt(pow((particles_x - gps_x), double(2)) + pow((particles_y- gps_y), double(2)));

    printf("distance: %f", distance);
}