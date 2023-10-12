#include <random>
#include <iostream>
#include <chrono>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "autonav_messages/msg/motor_feedback.hpp"
#include "autonav_messages/msg/gps_feedback.hpp"

class Particle {
    public:
        float x;
        float y;
        float theta;
        float weight;

        void printParticle() {
            std::cout << "x: " << this->x << " y: " << this->y << " theta: "; 
            std::cout << this->theta << " weight: " << this->weight << std::endl;
        }

        std::vector<float> get_particle_data() {
            std::vector<float> particle_data = {this->x, this->y, this->theta, this->weight};
            return particle_data;
        }

        // constructor
        Particle(float x = 0, float y = 0, float theta = 0, float weight = 1) {
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->weight = weight;
        }
};

class ParticleFilter {
    public:
        // constructor
        ParticleFilter(float latitudeLength, float longitudeLength) {
            this->latitudeLength = latitudeLength;
            this->longitudeLength = longitudeLength;
        };

        void init_particles() {
            for (int i=0; i<this->num_particles; i++) {
                particles.push_back(Particle(i, i, (float)i / this->num_particles * 2 * M_PI));
            }
        }

        void printParticles() {
            for (Particle part : particles) {
                part.printParticle();
            }
        }

        std::vector<float> feedback(autonav_messages::msg::MotorFeedback feedback) {
            float sum_x = 0;
            float sum_y = 0;
            float sum_theta_x = 0;
            float sum_theta_y = 0;
            float sum_weight = 0;

            for (Particle particle : this->particles) {
                particle.x = feedback.delta_x * 1.2 * cos(particle.theta) + feedback.delta_y * sin(particle.theta);
                particle.y = feedback.delta_x * 1.2 * sin(particle.theta) + feedback.delta_y * cos(particle.theta);
                particle.theta += feedback.delta_theta;
                particle.theta = fmod(particle.theta,(2 * M_PI));
                float weight = pow(particle.weight, 2);
                sum_x = particle.x * weight;
                sum_y = particle.y * weight;
                sum_theta_x += cos(particle.theta) * weight;
                sum_theta_y += sin(particle.theta) * weight;
                sum_weight += weight;
            }

            if (sum_weight < 0.000001) {
                sum_weight = 0.000001;
            }

            float avg_x = sum_x / sum_weight;
            float avg_y = sum_y / sum_weight;
            float avg_theta = fmod(atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight), 2 * M_PI);

            std::vector<float> feedback_vector = {avg_x, avg_y, avg_theta};
            return feedback_vector;
        }

        std::vector<float> gps(autonav_messages::msg::GPSFeedback gps) {
            if (this->first_gps_received == false) {
                this->first_gps = gps;
                this->first_gps_received = true;
            }

            float gps_x = (gps.latitude - this->first_gps.latitude) * this->latitudeLength;
            float gps_y = (this->first_gps.longitude - gps.longitude) * this->longitudeLength;

            for (Particle particle : this->particles) {
                float distance = sqrt(pow((particle.x - gps_x), 2) + pow((particle.y - gps_y), 2));
                particle.weight = exp(-1 * distance / 2 * pow(this->gps_noise[0], 2));
            }

            resample();

            std::vector<float> gps_vector = {gps_x, gps_y};
            return gps_vector;
        }

        void resample() {
            std::vector<float> weights;
            for (int i = 0; i < int(std::size(particles)); i++) {
                weights.push_back(particles[i].weight);
            }
            float weights_sum = std::accumulate(weights.begin(), weights.end(), 0);
            if (weights_sum < 0.0001) {
                weights_sum = 0.0001;
            }
            std::vector<float> temp_weights;
            for (int i = 0; i < int(std::size(weights)); i++) {
                temp_weights.push_back(weights[i] / weights_sum);
            }
            weights = temp_weights;
            std::discrete_distribution<int> discrete(weights.begin(), weights.end());
            std::vector<Particle> new_particles;

            for (int i = 0;i < num_particles; i++) {
                int index = discrete(generator);
                Particle selected_particle = particles[index];
                new_particles.push_back(selected_particle);
            }

            particles.clear();

            std::normal_distribution<> normal_distribution_x{0, this->odom_noise[0]};
            std::normal_distribution<> normal_distribution_y{0, this->odom_noise[1]};
            for (Particle p: new_particles) {
                float random_x = normal_distribution_x(generator);
                float random_y = normal_distribution_y(generator);

                float x = p.x + random_x * cos(p.theta) + random_y * sin(p.theta);
                float y= p.y + random_x * sin(p.theta) + random_y * cos(p.theta);

                std::normal_distribution<> normal_distribution_theta{p.theta, this->odom_noise[2]};
                float theta = normal_distribution_theta(generator);
                particles.push_back(Particle(x, y, theta, p.weight));
            }
        }

        #pragma region Getters
        
        int get_num_particles() {
            return this->num_particles;
        }

        float * get_gps_noise() {
            return this->gps_noise;
        }

        float * get_odom_noise() {
            return this->odom_noise;
        }

        std::vector<Particle> get_particles() {
            return this->particles;
        }

        float get_latitudeLength() {
            return this->latitudeLength;
        }

        float get_longitudeLength() {
            return this->longitudeLength;
        }

        autonav_messages::msg::GPSFeedback get_first_gps() {
            return this->first_gps;
        }

        std::vector<std::vector<float>> get_particles_data() {
            std::vector<std::vector<float>> particle_as_floats_collection;
            for (Particle part : particles) {
                std::vector<float> particle_data = part.get_particle_data();
                particle_as_floats_collection.push_back(particle_data);
            }
            return particle_as_floats_collection;
        }

        #pragma endregion Getters

    private:
        static const int num_particles = 750;
        float gps_noise[1] = {0.45};
        float odom_noise[3] = {0.05, 0.05, 0.01};
        std::vector<Particle> particles;
        float latitudeLength;
        float longitudeLength;
        bool first_gps_received = false;
        autonav_messages::msg::GPSFeedback first_gps;

        // random generator for distributions
        std::random_device rd;
        std::mt19937 generator;
        std::normal_distribution<> normal_distribution{0, 1};
};
