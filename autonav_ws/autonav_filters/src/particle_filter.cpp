#include <random>
#include <iostream>
#include <chrono>

#define _USE_MATH_DEFINES
#include <math.h>

class particle {
    public:
        float x;
        float y;
        float theta;
        float weight;

        void printParticle() {
            std::cout << "x: " << this->x << " y: " << this->y << " theta: "; 
            std::cout << this->theta << " weight: " << this->weight << std::endl;
        }

    // constructor
    particle(float x = 0, float y = 0, float theta = 0, float weight = 1) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->weight = weight;
    }
};

class particleFilter {
    public:
        static const int num_particles = 750;
        float gps_noise = 0.45;
        float odom_noise[3] = {0.05, 0.05, 0.01};
        particle particles[num_particles];
        float latitudeLength;
        float longitudeLength;

        // constructor
        particleFilter(float latitudeLength, float longitudeLength) {
            this->latitudeLength = latitudeLength;
            this->longitudeLength = longitudeLength;
        };

        void init_particles() {
            for (int i=0; i<this->num_particles; i++) {
                particles[i] = particle(i, i, (float)i / this->num_particles * 2 * M_PI);
            }
        }

        void printParticles() {
            for (particle part : particles) {
                part.printParticle();
            }
        }
};

int main () {
    particle testParticle(10.0, 10.0, (float)3 / (float)750 * (float)2 * M_PI, 1);
    testParticle.printParticle();

    particleFilter myFilter(1.0, 1.0);
    myFilter.init_particles();
    myFilter.printParticles();
}
