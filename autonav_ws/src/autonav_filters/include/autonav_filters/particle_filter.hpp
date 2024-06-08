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
        ParticleFilter(int num_particles, double latitudeLength, double longitudeLength, double gps_noise, double odom_noise_x, double odom_noise_y, double odom_noise_theta) {
            this->num_particles = num_particles;
            //printf("num partices: %d\n", this->num_particles);
            this->latitudeLength = latitudeLength;
            //printf("lat len: %f\n", this->latitudeLength);
            this->longitudeLength = longitudeLength;
            //printf("long len %f\n", this->longitudeLength);
            this->gps_noise = {gps_noise};
            this->odom_noise = {odom_noise_x, odom_noise_y, odom_noise_theta};

            std::random_device rd;
            //std::mt19937 generator(std::chrono::high_resolution_clock::now().time_since_epoch().count());
            std::mt19937 generator(rd());
            this->generator = generator;
            this->particles = particles;
        };

        // empty constructor
        ParticleFilter() {
            this->num_particles = 0;
            this->latitudeLength = 0;
            this->longitudeLength = 0;
            std::random_device rd;
            std::mt19937 generator(rd());
            this->generator = generator;
        }

        void init_particles() {
            this->particles.clear();
            for (int i=0; i<this->num_particles; i++) {
                //printf("PF INITIALIZED\n");
                //printf("%d\n", std::size(this->particles));
                //particles.push_back(Particle(0, 0, (double)i / this->num_particles * 2 * M_PI));
                //Particle particle = Particle(0, 0, (double)i / this->num_particles * 2 * M_PI);
                //particle.printParticle();
                this->particles.push_back(Particle(0, 0, (double)i / this->num_particles * 2 * M_PI));
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
                //printf("particle data before : %f, %f, %f, %f\n\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
                
                this->particles[i].x += feedback.delta_x * 1.2 * cos(this->particles[i].theta) + feedback.delta_y * sin(this->particles[i].theta);
                //printf("particle.x start of feedback %f\n", this->particles[i].x);
                // particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
                this->particles[i].y += feedback.delta_x * 1.2 * sin(this->particles[i].theta) + feedback.delta_y * cos(this->particles[i].theta);
                this->particles[i].theta += feedback.delta_theta;
                this->particles[i].theta = pymod(this->particles[i].theta, (2 * M_PI));
                
                // !! stuff below this point in this function is irrelevant to the problem
                //printf("particle data after updates: %f, %f, %f, %f\n\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
                double weight = copysign(1.0, this->particles[i].weight) * pow(this->particles[i].weight, 2);
                sum_x += this->particles[i].x * weight;
                sum_y += this->particles[i].y * weight;
                sum_theta_x += cos(this->particles[i].theta) * weight;
                sum_theta_y += sin(this->particles[i].theta) * weight;
                sum_weight += weight;

                //printf("summation data: %f, %f, %f, %f, %f\n\n", sum_x, sum_y, sum_theta_x, sum_theta_y, sum_weight);
                //printf("iteration number: %d\n\n", i);
            }

            if (sum_weight < 0.00001) {
                sum_weight = 0.00001;
            }

            double avg_x = sum_x / sum_weight;
            double avg_y = sum_y / sum_weight;
            double avg_theta = pymod(atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight), 2 * M_PI);

            //printf("average data: %f, %f, %f\n\n", avg_x, avg_y, avg_theta);

            this->log_particles();
            std::vector<double> feedback_vector = {avg_x, avg_y, avg_theta};

            //for (int i = 0; i<int(std::size(this->particles)); i++)
            //printf("new particles in feedback: %f, %f, %f, %f\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
            //printf("\n====== END FEEDBACK ======\n\n");

            std::ofstream averages_log_file;
            averages_log_file.open("/home/tony/Documents/averages_log_file_long.txt", std::ios::app);
            if (averages_log_file.is_open()) {
                //printf("gps log file open");
            }
            else {
                //printf("log file not open");
            }
            averages_log_file << feedback_vector[0] << ", " << feedback_vector[1] << ", " << feedback_vector[2] << std::endl;
            averages_log_file.close();
            //FILE OUTPUT SECTION
            //printf("====== END FEEDBACK ======\n\n");

            return feedback_vector;
        };

        std::vector<double> gps(autonav_msgs::msg::GPSFeedback gps) {
            int particles_size = this->particles.size();
            //printf("particles size: %d\n", particles_size);
            if (this->first_gps_received == false) {
                this->first_gps = gps;
                this->first_gps_received = true;
            }

            double gps_x = (gps.latitude - this->first_gps.latitude) * this->latitudeLength;
            //gps_x = -1;
            //gps_x = abs(gps_x);
            double gps_y = (this->first_gps.longitude - gps.longitude) * this->longitudeLength;

            //printf("gps_x, gps_y inside pf: %f, %f\n", gps_x, gps_y);

            double mean_distance = 0.0;
            double mean_weight = 0.0;
            double average_x = 0.0;
            for (int i = 0; i < this->particles.size(); i++) {
                double distance = sqrt(pow((this->particles[i].x - gps_x), double(2)) + pow((this->particles[i].y - gps_y), double(2)));
                //printf("distance: %f\n", distance);
                this->particles[i].weight = exp(-1 * distance / (2 * copysign(1.0, this->gps_noise[0]) * pow(this->gps_noise[0], 2)));
                //printf("particle weight: %f\n", this->particles[i].weight);
                mean_distance += distance;
                mean_weight += this->particles[i].weight;
                average_x += this->particles[i].x;
            }
            mean_distance /= this->num_particles;
            //printf("mean distance: %f\n", mean_distance);
            mean_weight /= this->num_particles;
            //printf("mean weight: %f\n", mean_weight);
            average_x /= num_particles;
            //printf("average x %f\n", average_x);

            resample();

            std::vector<double> gps_vector = {gps_x, gps_y};
            // printf("gps_x: %f \n", gps_x);
            // printf("gps_y: %f \n", gps_y);
            // printf("gps_vector in particle_filter header: x %f, y: %f\n", gps_vector[0], gps_vector[1]);
            // FILE OUTPUT SECTION
            // std::string individual_particles_string = "";
            // for (Particle particle : this->particles) {
            //     individual_particles_string = individual_particles_string + std::to_string(particle.x) + 
            //     ", " + std::to_string(particle.y) + ", ";
            // }
            
            //std::ofstream gps_log_file;
            //gps_log_file.open("/home/tony/Documents/gps_log_file.txt", std::ios::app);
            // if (gps_log_file.is_open()) {
            //     printf("gps log file open");
            // }
            // else {
            //     printf("log file not open");
            // }
            // gps_log_file << individual_particles_string << gps_vector[0] << ", " << gps_vector[1] << std::endl;
            // gps_log_file.close();
            // FILE OUTPUT SECTION
            // printf("====== END GPS ======\n\n");

            return gps_vector;
        }

        void resample() {
            int particles_size = this->particles.size();
            //printf("particles size: %d\n", particles_size);

            std::vector<double> weights;
            for (int i = 0; i < int(std::size(this->particles)); i++) {
                weights.push_back(this->particles[i].weight);
            }
            double weights_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
            if (weights_sum < 0.0001) {
                weights_sum = 0.0001;
            }
            
            for (int i=0;i<int(std::size(weights));i++) {
                //printf("weight %i: %f\n", i, weights[i]);
            }

            std::vector<double> temp_weights;
            for (int i = 0; i < int(std::size(weights)); i++) {
                temp_weights.push_back(weights[i] / weights_sum);
                //printf("temp_weights %i: %f\n", i, temp_weights[i]);
            }
            weights = temp_weights;
            int weights_size = weights.size();
            //printf("WEIGHTS SIZE: %d\n", weights_size);
            std::discrete_distribution<int> discrete(weights.begin(), weights.end());
            std::vector<Particle> new_particles;

            int accum_index;
            // if (int(std::size(weights) == 0)) {
            //     return;
            // }

            for (int i = 0;i < num_particles; i++) {
                int index = discrete(generator);
                // if (index == 0) {
                //     new_particles = this->particles;
                //     break;
                // }
                accum_index += index;
                //printf("index: %i\n", index);
                Particle selected_particle = this->particles[index];
                //elected_particle.printParticle();
                new_particles.push_back(selected_particle);
            }
            double mean_index = (double)mean_index / 262500.0;
            //printf("mean_index: %f\n", mean_index);

            for (int i = 0; i < int(std::size(new_particles)); i++) {
                //printf("weight after selecting new particles: %f\n", this->particles[i].weight);
                //printf("new particles: %f, %f, %f, %f\n", new_particles[i].x, new_particles[i].y, new_particles[i].theta, new_particles[i].weight);
            }

            this->particles.clear();

            std::normal_distribution<> normal_distribution_x{0, this->odom_noise[0]};
            std::normal_distribution<> normal_distribution_y{0, this->odom_noise[1]};
            double average_x = 0;
            double average_weight = 0;
            for (int i = 0; i < int(std::size(new_particles)); i++) {
                double random_x = normal_distribution_x(generator);
                //printf("random_x %f\n", random_x);
                //random_x = 0.03;
                double random_y = normal_distribution_y(generator);
                //random_y = 0.05;

                //printf("random_x, random_y: %f, %f\n", random_x, random_y);
                //printf("new_particles[i].x before adding noise %f\n", new_particles[i].x);
                double x = new_particles[i].x + random_x * cos(new_particles[i].theta) + random_y * sin(new_particles[i].theta);
                //printf("x in resample %f\n", x);
                double y = new_particles[i].y + random_x * sin(new_particles[i].theta) + random_y * cos(new_particles[i].theta);

                //printf("x, y, theta: %f, %f, %f\n", x, y, new_particles[i].theta);

                std::normal_distribution<> normal_distribution_theta{new_particles[i].theta, this->odom_noise[2]};
                double theta = pymod(normal_distribution_theta(generator), (2 * M_PI));
                //theta = 6.0;
                //printf("theta: %f\n", theta);
                this->particles.push_back(Particle(x, y, theta, new_particles[i].weight));
                average_x += x;
                average_weight += new_particles[i].weight;
            }

            average_x /= this->num_particles;
            //printf("average_x: %f\n", average_x);
            average_weight /= this->num_particles;
            //printf("average_weight %f\n", average_weight);

            for (int i = 0; i < int(std::size(this->particles)); i++) {
                //printf("new particles after resample: %f, %f, %f, %f\n", this->particles[i].x, this->particles[i].y, this->particles[i].theta, this->particles[i].weight);
            }
        }

        #pragma region Getters
        
        int get_num_particles() {
            return this->num_particles;
        }

        std::array<double, 1> get_gps_noise() {
            return this->gps_noise;
        }

        std::array<double, 3> get_odom_noise() {
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

        double* get_mean_values() {
            double means[3];
            for (Particle particle : this->particles) {
                means[0] += particle.x;
                means[1] += particle.y;
                means[2] += particle.theta;
            }
            means[0] /= this->num_particles;
            means[1] /= this->num_particles;
            means[2] /= this->num_particles;

            return means;
        }

        std::vector<std::vector<double>> get_particles_data() {
            std::vector<std::vector<double>> particle_data_collection;
            for (Particle part : particles) {
                std::vector<double> particle_data = part.get_particle_data();
                particle_data_collection.push_back(particle_data);
            }
            return particle_data_collection;
        }

        void log_particles() {
            //printf("in log particles\n");
            std::ofstream particles_x_log_file;
            std::ofstream particles_y_log_file;
            std::ofstream particles_theta_log_file;
            std::ofstream particles_mean_log_file;
            particles_x_log_file.open("/home/tony/Documents/particles_x_log_file.txt", std::ios::app);
            particles_y_log_file.open("/home/tony/Documents/particles_y_log_file.txt", std::ios::app);
            particles_theta_log_file.open("/home/tony/Documents/particles_theta_log_file.txt", std::ios::app);
            for (Particle particle : this->particles) {
                particles_x_log_file << particle.x << ", ";
                particles_y_log_file << particle.y << ", ";
                particles_theta_log_file << particle.theta << ", ";
            }
            particles_x_log_file << std::endl;
            particles_y_log_file << std::endl;
            particles_theta_log_file << std::endl;

            particles_x_log_file.close();
            particles_y_log_file.close();
            particles_theta_log_file.close();
        }

        #pragma endregion Getters

    private:
        std::mt19937 generator;
        //int num_particles = 750;
        int num_particles = 750;
        //double gps_noise[1] = {0.8};
        std::array<double, 1> gps_noise;
        //double odom_noise[3] = {0.05, 0.05, 0.01};
        std::array<double, 3> odom_noise;
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
