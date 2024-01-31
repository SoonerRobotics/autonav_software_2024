#pragma once

#include <random>
#include <iostream>
#include <chrono>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"

class Particle {
    public:
        double x;
        double y;
        double theta;
        double weight;

        void printParticle() {
            std::cout << "x: " << this->x << " y: " << this->y << " theta: "; 
            std::cout << this->theta << " weight: " << this->weight << std::endl;
        }

        std::vector<double> get_particle_data() {
            std::vector<double> particle_data = {this->x, this->y, this->theta, this->weight};
            return particle_data;
        }

        // constructor
        Particle(double x = 0, double y = 0, double theta = 0, double weight = 0.000001) {
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->weight = weight;
        }
};

class ParticleFilter {
    public:
        // constructor
        ParticleFilter(double latitudeLength, double longitudeLength) {
            this->latitudeLength = latitudeLength;
            this->longitudeLength = longitudeLength;
        };

        void init_particles() {
            particles.clear();
            for (int i=0; i<this->num_particles; i++) {
                particles.push_back(Particle(0, 0, (double)i / this->num_particles * 2 * M_PI));
            }
        }

        void printParticles() {
            for (Particle part : particles) {
                part.printParticle();
            }
        }

        std::vector<double> feedback(autonav_msgs::msg::MotorFeedback feedback) {
            double sum_x = 0;
            double sum_y = 0;
            double sum_theta_x = 0;
            double sum_theta_y = 0;
            double sum_weight = 0;

            for (int i = 0; i < this->particles.size(); i++) {
                this->particles[i].x += feedback.delta_x * 1.2 * cos(this->particles[i].theta) + feedback.delta_y * sin(this->particles[i].theta);
                this->particles[i].y += feedback.delta_x * 1.2 * sin(this->particles[i].theta) + feedback.delta_y * cos(this->particles[i].theta);
                this->particles[i].theta += feedback.delta_theta;
                this->particles[i].theta = pymod(this->particles[i].theta, (2 * M_PI));
                double weight = pow(this->particles[i].weight, 2);
                sum_x += this->particles[i].x * weight;
                sum_y += this->particles[i].y * weight;
                sum_theta_x += cos(this->particles[i].theta) * weight;
                sum_theta_y += sin(this->particles[i].theta) * weight;
                sum_weight += weight;
            }

            if (sum_weight < 0.000001) {
                sum_weight = 0.000001;
            }

            double avg_x = sum_x / sum_weight;
            double avg_y = sum_y / sum_weight;
            double avg_theta = pymod(atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight), 2 * M_PI);

            std::vector<double> feedback_vector = {avg_x, avg_y, avg_theta};
            return feedback_vector;
        }

        std::vector<double> gps(autonav_msgs::msg::GPSFeedback gps) {
            if (this->first_gps_received == false) {
                this->first_gps = gps;
                this->first_gps_received = true;
            }

            double gps_x = (gps.latitude - this->first_gps.latitude) * this->latitudeLength;
            double gps_y = (this->first_gps.longitude - gps.longitude) * this->longitudeLength;

            for (Particle particle : this->particles) {
                double distance = sqrt(pow((particle.x - gps_x), 2) + pow((particle.y - gps_y), 2));
                particle.weight = exp(-1 * distance / 2 * pow(this->gps_noise[0], 2));
            }

            resample();

            std::vector<double> gps_vector = {gps_x, gps_y};
            //printf("gps_x: %f \n", gps_x);
            //printf("gps_y: %f \n", gps_y);
            return gps_vector;
        }

        void resample() {
            std::vector<double> weights;
            for (int i = 0; i < int(std::size(particles)); i++) {
                weights.push_back(particles[i].weight);
            }
            double weights_sum = std::accumulate(weights.begin(), weights.end(), 0);
            if (weights_sum < 0.0001) {
                weights_sum = 0.0001;
            }
            std::vector<double> temp_weights;
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
            for (Particle p : new_particles) {
                double random_x = normal_distribution_x(generator);
                double random_y = normal_distribution_y(generator);

                double x = p.x + random_x * cos(p.theta) + random_y * sin(p.theta);
                double y = p.y + random_x * sin(p.theta) + random_y * cos(p.theta);

                std::normal_distribution<> normal_distribution_theta{p.theta, this->odom_noise[2]};
                double theta = normal_distribution_theta(generator);
                particles.push_back(Particle(x, y, theta, p.weight));
            }
        }

        #pragma region Getters
        
        int get_num_particles() {
            return this->num_particles;
        }

        double * get_gps_noise() {
            return this->gps_noise;
        }

        double * get_odom_noise() {
            return this->odom_noise;
        }

        std::vector<Particle> get_particles() {
            return this->particles;
        }

        double get_latitudeLength() {
            return this->latitudeLength;
        }

        double get_longitudeLength() {
            return this->longitudeLength;
        }

        autonav_msgs::msg::GPSFeedback get_first_gps() {
            return this->first_gps;
        }

        std::vector<std::vector<double>> get_particles_data() {
            std::vector<std::vector<double>> particle_data_collection;
            for (Particle part : particles) {
                std::vector<double> particle_data = part.get_particle_data();
                particle_data_collection.push_back(particle_data);
            }
            return particle_data_collection;
        }

        #pragma endregion Getters

    private:
        static const int num_particles = 750;
        double gps_noise[1] = {0.45};
        double odom_noise[3] = {0.05, 0.05, 0.01};
        std::vector<Particle> particles;
        double latitudeLength;
        double longitudeLength;
        bool first_gps_received = false;
        autonav_msgs::msg::GPSFeedback first_gps;

        // random generator for distributions
        std::random_device rd;
        std::mt19937 generator;
        std::normal_distribution<> normal_distribution{0, 1};

        double pymod(double n, double M) {
            return fmod(((fmod(n, M)) + M), M);
        }
};
