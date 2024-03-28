#pragma once

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

#include "rclcpp/rclcpp.hpp"
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
        Particle(double x = 0, double y = 0, double theta = 0, double weight = 1) {
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
            std::random_device rd;
            //std::mt19937 generator(std::chrono::high_resolution_clock::now().time_since_epoch().count());
            std::mt19937 generator(rd());
            std::vector<int> range = {1, 5, 10, 15, 20, 25};
            std::discrete_distribution<int> discrete(range.begin(), range.end());
             for (int i = 0; i < 10; i++) {
                int index = discrete(generator);
                int value = range[index];
                printf("index: %d\n", index);
                printf("value: %d\n", value);
            }
            this->generator = generator;
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
                printf("particle data before : %f, %f, %f, %f\n\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
                this->particles[i].x += feedback.delta_x * 1.2 * cos(this->particles[i].theta) + feedback.delta_y * sin(this->particles[i].theta);
                
                // particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
                this->particles[i].y += feedback.delta_x * 1.2 * sin(this->particles[i].theta) + feedback.delta_y * cos(this->particles[i].theta);
                this->particles[i].theta += feedback.delta_theta;
                this->particles[i].theta = pymod(this->particles[i].theta, (2 * M_PI));
                printf("particle data after updates: %f, %f, %f, %f\n\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
                double weight = pow(this->particles[i].weight, 2);
                sum_x += this->particles[i].x * weight;
                sum_y += this->particles[i].y * weight;
                sum_theta_x += cos(this->particles[i].theta) * weight;
                sum_theta_y += sin(this->particles[i].theta) * weight;
                sum_weight += weight;

                printf("summation data: %f, %f, %f, %f, %f\n\n", sum_x, sum_y, sum_theta_x, sum_theta_y, sum_weight);
                printf("iteration number: %d\n\n", i);
            }

            if (sum_weight < 0.000001) {
                sum_weight = 0.000001;
            }

            double avg_x = sum_x / sum_weight;
            double avg_y = sum_y / sum_weight;
            double avg_theta = pymod(atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight), 2 * M_PI);

            printf("average data: %f, %f, %f\n\n", avg_x, avg_y, avg_theta);

            std::vector<double> feedback_vector = {avg_x, avg_y, avg_theta};

            for (int i = 0; i<int(std::size(this->particles)); i++)
            printf("new particles in feedback: %f, %f, %f, %f\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
            printf("\n====== END FEEDBACK ======\n\n");
            return feedback_vector;
        }

        std::vector<double> gps(autonav_msgs::msg::GPSFeedback gps) {
            if (this->first_gps_received == false) {
                this->first_gps = gps;
                this->first_gps_received = true;
            }

            double gps_x = (gps.latitude - this->first_gps.latitude) * this->latitudeLength;
            double gps_y = (this->first_gps.longitude - gps.longitude) * this->longitudeLength;

            printf("gps_x, gps_y: %f, %f\n", gps_x, gps_y);

            for (int i = 0; i < this->particles.size(); i++) {
                printf("particle_x, particle_y: %f, %f\n", this->particles[i].x, this->particles[i].y);
                double distance = sqrt(pow((this->particles[i].x - gps_x), double(2)) + pow((this->particles[i].y - gps_y), double(2)));
                printf("distance: %f\n", distance);
                this->particles[i].weight = exp(-1 * distance / (2 * pow(this->gps_noise[0], 2)));
                printf("particle weight after reassignment: %f\n", this->particles[i].weight);
            }

            resample();

            std::vector<double> gps_vector = {gps_x, gps_y};
            // printf("gps_x: %f \n", gps_x);
            // printf("gps_y: %f \n", gps_y);
            // printf("gps_vector in particle_filter header: x %f, y: %f\n", gps_vector[0], gps_vector[1]);
            //
            std::string individual_particles_string = "";
            for (Particle particle : this->particles) {
                individual_particles_string = individual_particles_string + std::to_string(particle.x) + 
                ", " + std::to_string(particle.y) + ", ";
            }
            
            std::ofstream gps_log_file;
            gps_log_file.open("/home/tony/Documents/gps_log_file.txt", std::ios::app);
            if (gps_log_file.is_open()) {
                printf("gps log file open");
            }
            else {
                printf("log file not open");
            }
            gps_log_file << individual_particles_string << gps_vector[0] << ", " << gps_vector[1] << std::endl;
            gps_log_file.close();
            printf("====== END GPS ======\n\n");

            return gps_vector;
        }

        void resample() {
            std::vector<double> weights;
            for (int i = 0; i < int(std::size(this->particles)); i++) {
                weights.push_back(this->particles[i].weight);
                printf("weight: %f\n", weights[i]);
            }
            double weights_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
            printf("weight sum: %f\n", weights_sum);
            if (weights_sum < 0.0001) {
                weights_sum = 0.0001;
            }
            std::vector<double> temp_weights;
            for (int i = 0; i < int(std::size(weights)); i++) {
                temp_weights.push_back(weights[i] / weights_sum);
                printf("weight after sum: %f\n", temp_weights[i]);
            }
            weights = temp_weights;
            std::discrete_distribution<int> discrete(weights.begin(), weights.end());
            std::vector<Particle> new_particles;

            for (int i = 0;i < num_particles; i++) {
                int index = discrete(generator);
                //index = 0;
                printf("index %i\n", index);
                Particle selected_particle = this->particles[index];
                new_particles.push_back(selected_particle);
            }

            for (int i = 0; i < int(std::size(new_particles)); i++) {
                printf("new particles: %f, %f, %f, %f\n", new_particles[i].x, new_particles[i].y, new_particles[i].theta, new_particles[i].weight);
            }

            this->particles.clear();

            std::normal_distribution<> normal_distribution_x{0, this->odom_noise[0]};
            std::normal_distribution<> normal_distribution_y{0, this->odom_noise[1]};
            for (int i = 0; i < int(std::size(new_particles)); i++) {
                double random_x = normal_distribution_x(generator);
                random_x = 0.03;
                double random_y = normal_distribution_y(generator);
                random_y = 0.05;

                printf("random_x, random_y: %f, %f\n", random_x, random_y);

                double x = new_particles[i].x + random_x * cos(new_particles[i].theta) + random_y * sin(new_particles[i].theta);
                double y = new_particles[i].y + random_x * sin(new_particles[i].theta) + random_y * cos(new_particles[i].theta);

                printf("x, y, theta: %f, %f, %f\n", x, y, new_particles[i].theta);

                std::normal_distribution<> normal_distribution_theta{new_particles[i].theta, this->odom_noise[2]};
                double theta = pymod(normal_distribution_theta(generator), (2 * M_PI));
                //double theta = 6.0;
                printf("theta: %f\n", theta);
                this->particles.push_back(Particle(x, y, theta, new_particles[i].weight));
            }
            for (int i = 0; i < int(std::size(this->particles)); i++) {
                printf("new particles after resample: %f, %f, %f, %f\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
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
        std::mt19937 generator;
        static const int num_particles = 10;
        double gps_noise[1] = {0.45};
        double odom_noise[3] = {0.05, 0.05, 0.01};
        std::vector<Particle> particles;
        double latitudeLength;
        double longitudeLength;
        bool first_gps_received = false;
        autonav_msgs::msg::GPSFeedback first_gps;

        // random generator for distributions
        std::normal_distribution<> normal_distribution{0, 1};

        double pymod(double n, double M) {
            return fmodl(((fmodl(n, M)) + M), M);
        }
};
