from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
import numpy as np
import math
import random


class Particle:
    def __init__(self, x=0, y=0, theta=0, weight=1) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class ParticleFilter:
    def __init__(self, latitudeLength, longitudeLength) -> None:
        self.num_particles = 1
        self.gps_noise = [0.45]
        self.odom_noise = [0.05, 0.05, 0.1]
        self.init_particles()
        self.first_gps = None

        self.latitude_length = latitudeLength
        self.longitude_length = longitudeLength

    def init_particles(self, seedHeading: float = 0.0, useSeedHeading: bool = False):
        self.first_gps = None
        if useSeedHeading:
            self.particles = [Particle(0, 0, seedHeading + np.random.normal(0, 0.1)) for i in range(self.num_particles)]
        else:
            self.particles = [Particle(0, 0, i / self.num_particles * 2 * math.pi) for i in range(self.num_particles)]

    def feedback(self, feedback: MotorFeedback) -> list[float]:
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0

        for particle in self.particles:
            particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            #print(f"particle.x start of feedback: {particle.x}")
            particle.y += feedback.delta_x * 1.2 * math.sin(particle.theta) + feedback.delta_y * math.cos(particle.theta)
            particle.theta += feedback.delta_theta
            particle.theta = particle.theta % (2 * math.pi)
            weight = particle.weight ** 2
            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += math.cos(particle.theta) * weight
            sum_theta_y += math.sin(particle.theta) * weight
            sum_weight += weight

        if sum_weight < 0.000001:
            sum_weight = 0.000001

        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = math.atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * math.pi)

        return [avg_x, avg_y, avg_theta]

    def gps(self, gps: GPSFeedback) -> list[float]:
        if self.first_gps is None:
            self.first_gps = gps

        gps_x = (gps.latitude - self.first_gps.latitude) * self.latitude_length
        gps_y = (self.first_gps.longitude - gps.longitude) * self.longitude_length

        print(f"gps_x, gps_y: {gps_x}, {gps_y}")
        mean_dist = 0
        mean_weight = 0
        average_x = 0
        for particle in self.particles:
            dist_sqrt = np.sqrt((particle.x - gps_x) ** 2 + (particle.y - gps_y) ** 2)
            #print(f"dist_sqrt {dist_sqrt}")
            particle.weight = math.exp(-dist_sqrt / (2 * self.gps_noise[0] ** 2))
            #print(f"particle.weight {particle.weight}")
            mean_dist += dist_sqrt
            mean_weight += particle.weight
            average_x += particle.x


        mean_dist /= self.num_particles
        mean_weight /= self.num_particles
        average_x /= self.num_particles
        print(f"mean_dist {mean_dist}")
        print(f"mean_weight {mean_weight}")
        print(f"average_x {average_x}")

        self.resample()
        return [gps_x, gps_y]

    def resample(self) -> None:
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        if weights_sum <= 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]

        new_particles = random.choices(self.particles, weights, k=self.num_particles)
        self.particles = []

        average_x = 0
        for particle in new_particles:
            rand_x = np.random.normal(0, self.odom_noise[0])
            #print(f"rand_x {rand_x}")
            #rand_x = 0.03
            rand_y = np.random.normal(0, self.odom_noise[1])
            #rand_y = 0.05
            #print(f"particle.x before adding random noise {particle.x}")
            x = particle.x + rand_x * math.cos(particle.theta) + rand_y * math.sin(particle.theta)
            #print(f"x in resample {x}")
            y = particle.y + rand_x * math.sin(particle.theta) + rand_y * math.cos(particle.theta)
            theta = np.random.normal(particle.theta, self.odom_noise[2]) % (2 * math.pi)
            theta = 6.0
            self.particles.append(Particle(x, y, theta, particle.weight))

            average_x += x

        average_x /= self.num_particles
        print(f"average_x {average_x}")
