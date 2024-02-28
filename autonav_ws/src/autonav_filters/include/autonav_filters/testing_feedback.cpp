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


}