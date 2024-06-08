#include <random>
#include <iostream>
#include <ostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <iomanip>
#include <numeric>

#define _USE_MATH_DEFINES
#include <math.h>


int main() {
    double particle_x;
    for (int i=0;i<10;i++) {
        particle_x += 0.03 * 1.2 * cos(M_PI);
    }
    printf("%f\n", particle_x);
    return 0;
}