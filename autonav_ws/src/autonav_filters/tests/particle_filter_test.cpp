#include "gtest/gtest.h"
#include "autonav_filters/particle_filter.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include "autonav_filters/csv_utils.hpp"
#include "autonav_filters/rapidcsv.h"

TEST(ParticleFilterTests, initialization_test) {
    GTEST_SKIP() << "skipping init test";
    int num_particles = 750;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);
    ASSERT_EQ(particle_filter.get_latitudeLength(), latitudeLength);
    ASSERT_EQ(particle_filter.get_longitudeLength(), longitudeLength);

    particle_filter.init_particles();

    #pragma region PythonData
    std::vector<std::vector<double>> python_data = {{0, 0, 0.0, 0.0}, {0, 0, 0.00838, 0.0}, {0, 0, 0.01676, 0.0}, {0, 0, 0.02513, 0.0}, {0, 0, 0.03351, 0.0}, {0, 0, 0.04189, 0.0}, {0, 0, 0.05027, 0.0}, {0, 0, 0.05864, 0.0}, {0, 0, 0.06702, 0.0}, {0, 0, 0.0754, 0.0}, {0, 0, 0.08378, 0.0}, {0, 0, 0.09215, 0.0}, {0, 0, 0.10053, 0.0}, {0, 0, 0.10891, 0.0}, {0, 0, 0.11729, 0.0}, {0, 0, 0.12566, 0.0}, {0, 0, 0.13404, 0.0}, {0, 0, 0.14242, 0.0}, {0, 0, 0.1508, 0.0}, {0, 0, 0.15917, 0.0}, {0, 0, 0.16755, 0.0}, {0, 0, 0.17593, 0.0}, {0, 0, 0.18431, 0.0}, {0, 0, 0.19268, 0.0}, {0, 0, 0.20106, 0.0}, {0, 0, 0.20944, 0.0}, {0, 0, 0.21782, 0.0}, {0, 0, 0.22619, 0.0}, {0, 0, 0.23457, 0.0}, {0, 0, 0.24295, 0.0}, {0, 0, 0.25133, 0.0}, {0, 0, 0.2597, 0.0}, {0, 0, 0.26808, 0.0}, {0, 0, 0.27646, 0.0}, {0, 0, 0.28484, 0.0}, {0, 0, 0.29322, 0.0}, {0, 0, 0.30159, 0.0}, {0, 0, 0.30997, 0.0}, {0, 0, 0.31835, 0.0}, {0, 0, 0.32673, 0.0}, {0, 0, 0.3351, 0.0}, {0, 0, 0.34348, 0.0}, {0, 0, 0.35186, 0.0}, {0, 0, 0.36024, 0.0}, {0, 0, 0.36861, 0.0}, {0, 0, 0.37699, 0.0}, {0, 0, 0.38537, 0.0}, {0, 0, 0.39375, 0.0}, {0, 0, 0.40212, 0.0}, {0, 0, 0.4105, 0.0}, {0, 0, 0.41888, 0.0}, {0, 0, 0.42726, 0.0}, {0, 0, 0.43563, 0.0}, {0, 0, 0.44401, 0.0}, {0, 0, 0.45239, 0.0}, {0, 0, 0.46077, 0.0}, {0, 0, 0.46914, 0.0}, {0, 0, 0.47752, 0.0}, {0, 0, 0.4859, 0.0}, {0, 0, 0.49428, 0.0}, {0, 0, 0.50265, 0.0}, {0, 0, 0.51103, 0.0}, {0, 0, 0.51941, 0.0}, {0, 0, 0.52779, 0.0}, {0, 0, 0.53617, 0.0}, {0, 0, 0.54454, 0.0}, {0, 0, 0.55292, 0.0}, {0, 0, 0.5613, 0.0}, {0, 0, 0.56968, 0.0}, {0, 0, 0.57805, 0.0}, {0, 0, 0.58643, 0.0}, {0, 0, 0.59481, 0.0}, {0, 0, 0.60319, 0.0}, {0, 0, 0.61156, 0.0}, {0, 0, 0.61994, 0.0}, {0, 0, 0.62832, 0.0}, {0, 0, 0.6367, 0.0}, {0, 0, 0.64507, 0.0}, {0, 0, 0.65345, 0.0}, {0, 0, 0.66183, 0.0}, {0, 0, 0.67021, 0.0}, {0, 0, 0.67858, 0.0}, {0, 0, 0.68696, 0.0}, {0, 0, 0.69534, 0.0}, {0, 0, 0.70372, 0.0}, {0, 0, 0.71209, 0.0}, {0, 0, 0.72047, 0.0}, {0, 0, 0.72885, 0.0}, {0, 0, 0.73723, 0.0}, {0, 0, 0.7456, 0.0}, {0, 0, 0.75398, 0.0}, {0, 0, 0.76236, 0.0}, {0, 0, 0.77074, 0.0}, {0, 0, 0.77911, 0.0}, {0, 0, 0.78749, 0.0}, {0, 0, 0.79587, 0.0}, {0, 0, 0.80425, 0.0}, {0, 0, 0.81263, 0.0}, {0, 0, 0.821, 0.0}, {0, 0, 0.82938, 0.0}, {0, 0, 0.83776, 0.0}, {0, 0, 0.84614, 0.0}, {0, 0, 0.85451, 0.0}, {0, 0, 0.86289, 0.0}, {0, 0, 0.87127, 0.0}, {0, 0, 0.87965, 0.0}, {0, 0, 0.88802, 0.0}, {0, 0, 0.8964, 0.0}, {0, 0, 0.90478, 0.0}, {0, 0, 0.91316, 0.0}, {0, 0, 0.92153, 0.0}, {0, 0, 0.92991, 0.0}, {0, 0, 0.93829, 0.0}, {0, 0, 0.94667, 0.0}, {0, 0, 0.95504, 0.0}, {0, 0, 0.96342, 0.0}, {0, 0, 0.9718, 0.0}, {0, 0, 0.98018, 0.0}, {0, 0, 0.98855, 0.0}, {0, 0, 0.99693, 0.0}, {0, 0, 1.00531, 0.0}, {0, 0, 1.01369, 0.0}, {0, 0, 1.02206, 0.0}, {0, 0, 1.03044, 0.0}, {0, 0, 1.03882, 0.0}, {0, 0, 1.0472, 0.0}, {0, 0, 1.05558, 0.0}, {0, 0, 1.06395, 0.0}, {0, 0, 1.07233, 0.0}, {0, 0, 1.08071, 0.0}, {0, 0, 1.08909, 0.0}, {0, 0, 1.09746, 0.0}, {0, 0, 1.10584, 0.0}, {0, 0, 1.11422, 0.0}, {0, 0, 1.1226, 0.0}, {0, 0, 1.13097, 0.0}, {0, 0, 1.13935, 0.0}, {0, 0, 1.14773, 0.0}, {0, 0, 1.15611, 0.0}, {0, 0, 1.16448, 0.0}, {0, 0, 1.17286, 0.0}, {0, 0, 1.18124, 0.0}, {0, 0, 1.18962, 0.0}, {0, 0, 1.19799, 0.0}, {0, 0, 1.20637, 0.0}, {0, 0, 1.21475, 0.0}, {0, 0, 1.22313, 0.0}, {0, 0, 1.2315, 0.0}, {0, 0, 1.23988, 0.0}, {0, 0, 1.24826, 0.0}, {0, 0, 1.25664, 0.0}, {0, 0, 1.26501, 0.0}, {0, 0, 1.27339, 0.0}, {0, 0, 1.28177, 0.0}, {0, 0, 1.29015, 0.0}, {0, 0, 1.29852, 0.0}, {0, 0, 1.3069, 0.0}, {0, 0, 1.31528, 0.0}, {0, 0, 1.32366, 0.0}, {0, 0, 1.33204, 0.0}, {0, 0, 1.34041, 0.0}, {0, 0, 1.34879, 0.0}, {0, 0, 1.35717, 0.0}, {0, 0, 1.36555, 0.0}, {0, 0, 1.37392, 0.0}, {0, 0, 1.3823, 0.0}, {0, 0, 1.39068, 0.0}, {0, 0, 1.39906, 0.0}, {0, 0, 1.40743, 0.0}, {0, 0, 1.41581, 0.0}, {0, 0, 1.42419, 0.0}, {0, 0, 1.43257, 0.0}, {0, 0, 1.44094, 0.0}, {0, 0, 1.44932, 0.0}, {0, 0, 1.4577, 0.0}, {0, 0, 1.46608, 0.0}, {0, 0, 1.47445, 0.0}, {0, 0, 1.48283, 0.0}, {0, 0, 1.49121, 0.0}, {0, 0, 1.49959, 0.0}, {0, 0, 1.50796, 0.0}, {0, 0, 1.51634, 0.0}, {0, 0, 1.52472, 0.0}, {0, 0, 1.5331, 0.0}, {0, 0, 1.54147, 0.0}, {0, 0, 1.54985, 0.0}, {0, 0, 1.55823, 0.0}, {0, 0, 1.56661, 0.0}, {0, 0, 1.57499, 0.0}, {0, 0, 1.58336, 0.0}, {0, 0, 1.59174, 0.0}, {0, 0, 1.60012, 0.0}, {0, 0, 1.6085, 0.0}, {0, 0, 1.61687, 0.0}, {0, 0, 1.62525, 0.0}, {0, 0, 1.63363, 0.0}, {0, 0, 1.64201, 0.0}, {0, 0, 1.65038, 0.0}, {0, 0, 1.65876, 0.0}, {0, 0, 1.66714, 0.0}, {0, 0, 1.67552, 0.0}, {0, 0, 1.68389, 0.0}, {0, 0, 1.69227, 0.0}, {0, 0, 1.70065, 0.0}, {0, 0, 1.70903, 0.0}, {0, 0, 1.7174, 0.0}, {0, 0, 1.72578, 0.0}, {0, 0, 1.73416, 0.0}, {0, 0, 1.74254, 0.0}, {0, 0, 1.75091, 0.0}, {0, 0, 1.75929, 0.0}, {0, 0, 1.76767, 0.0}, {0, 0, 1.77605, 0.0}, {0, 0, 1.78442, 0.0}, {0, 0, 1.7928, 0.0}, {0, 0, 1.80118, 0.0}, {0, 0, 1.80956, 0.0}, {0, 0, 1.81793, 0.0}, {0, 0, 1.82631, 0.0}, {0, 0, 1.83469, 0.0}, {0, 0, 1.84307, 0.0}, {0, 0, 1.85145, 0.0}, {0, 0, 1.85982, 0.0}, {0, 0, 1.8682, 0.0}, {0, 0, 1.87658, 0.0}, {0, 0, 1.88496, 0.0}, {0, 0, 1.89333, 0.0}, {0, 0, 1.90171, 0.0}, {0, 0, 1.91009, 0.0}, {0, 0, 1.91847, 0.0}, {0, 0, 1.92684, 0.0}, {0, 0, 1.93522, 0.0}, {0, 0, 1.9436, 0.0}, {0, 0, 1.95198, 0.0}, {0, 0, 1.96035, 0.0}, {0, 0, 1.96873, 0.0}, {0, 0, 1.97711, 0.0}, {0, 0, 1.98549, 0.0}, {0, 0, 1.99386, 0.0}, {0, 0, 2.00224, 0.0}, {0, 0, 2.01062, 0.0}, {0, 0, 2.019, 0.0}, {0, 0, 2.02737, 0.0}, {0, 0, 2.03575, 0.0}, {0, 0, 2.04413, 0.0}, {0, 0, 2.05251, 0.0}, {0, 0, 2.06088, 0.0}, {0, 0, 2.06926, 0.0}, {0, 0, 2.07764, 0.0}, {0, 0, 2.08602, 0.0}, {0, 0, 2.0944, 0.0}, {0, 0, 2.10277, 0.0}, {0, 0, 2.11115, 0.0}, {0, 0, 2.11953, 0.0}, {0, 0, 2.12791, 0.0}, {0, 0, 2.13628, 0.0}, {0, 0, 2.14466, 0.0}, {0, 0, 2.15304, 0.0}, {0, 0, 2.16142, 0.0}, {0, 0, 2.16979, 0.0}, {0, 0, 2.17817, 0.0}, {0, 0, 2.18655, 0.0}, {0, 0, 2.19493, 0.0}, {0, 0, 2.2033, 0.0}, {0, 0, 2.21168, 0.0}, {0, 0, 2.22006, 0.0}, {0, 0, 2.22844, 0.0}, {0, 0, 2.23681, 0.0}, {0, 0, 2.24519, 0.0}, {0, 0, 2.25357, 0.0}, {0, 0, 2.26195, 0.0}, {0, 0, 2.27032, 0.0}, {0, 0, 2.2787, 0.0}, {0, 0, 2.28708, 0.0}, {0, 0, 2.29546, 0.0}, {0, 0, 2.30383, 0.0}, {0, 0, 2.31221, 0.0}, {0, 0, 2.32059, 0.0}, {0, 0, 2.32897, 0.0}, {0, 0, 2.33734, 0.0}, {0, 0, 2.34572, 0.0}, {0, 0, 2.3541, 0.0}, {0, 0, 2.36248, 0.0}, {0, 0, 2.37086, 0.0}, {0, 0, 2.37923, 0.0}, {0, 0, 2.38761, 0.0}, {0, 0, 2.39599, 0.0}, {0, 0, 2.40437, 0.0}, {0, 0, 2.41274, 0.0}, {0, 0, 2.42112, 0.0}, {0, 0, 2.4295, 0.0}, {0, 0, 2.43788, 0.0}, {0, 0, 2.44625, 0.0}, {0, 0, 2.45463, 0.0}, {0, 0, 2.46301, 0.0}, {0, 0, 2.47139, 0.0}, {0, 0, 2.47976, 0.0}, {0, 0, 2.48814, 0.0}, {0, 0, 2.49652, 0.0}, {0, 0, 2.5049, 0.0}, {0, 0, 2.51327, 0.0}, {0, 0, 2.52165, 0.0}, {0, 0, 2.53003, 0.0}, {0, 0, 2.53841, 0.0}, {0, 0, 2.54678, 0.0}, {0, 0, 2.55516, 0.0}, {0, 0, 2.56354, 0.0}, {0, 0, 2.57192, 0.0}, {0, 0, 2.58029, 0.0}, {0, 0, 2.58867, 0.0}, {0, 0, 2.59705, 0.0}, {0, 0, 2.60543, 0.0}, {0, 0, 2.61381, 0.0}, {0, 0, 2.62218, 0.0}, {0, 0, 2.63056, 0.0}, {0, 0, 2.63894, 0.0}, {0, 0, 2.64732, 0.0}, {0, 0, 2.65569, 0.0}, {0, 0, 2.66407, 0.0}, {0, 0, 2.67245, 0.0}, {0, 0, 2.68083, 0.0}, {0, 0, 2.6892, 0.0}, {0, 0, 2.69758, 0.0}, {0, 0, 2.70596, 0.0}, {0, 0, 2.71434, 0.0}, {0, 0, 2.72271, 0.0}, {0, 0, 2.73109, 0.0}, {0, 0, 2.73947, 0.0}, {0, 0, 2.74785, 0.0}, {0, 0, 2.75622, 0.0}, {0, 0, 2.7646, 0.0}, {0, 0, 2.77298, 0.0}, {0, 0, 2.78136, 0.0}, {0, 0, 2.78973, 0.0}, {0, 0, 2.79811, 0.0}, {0, 0, 2.80649, 0.0}, {0, 0, 2.81487, 0.0}, {0, 0, 2.82324, 0.0}, {0, 0, 2.83162, 0.0}, {0, 0, 2.84, 0.0}, {0, 0, 2.84838, 0.0}, {0, 0, 2.85675, 0.0}, {0, 0, 2.86513, 0.0}, {0, 0, 2.87351, 0.0}, {0, 0, 2.88189, 0.0}, {0, 0, 2.89027, 0.0}, {0, 0, 2.89864, 0.0}, {0, 0, 2.90702, 0.0}, {0, 0, 2.9154, 0.0}, {0, 0, 2.92378, 0.0}, {0, 0, 2.93215, 0.0}, {0, 0, 2.94053, 0.0}, {0, 0, 2.94891, 0.0}, {0, 0, 2.95729, 0.0}, {0, 0, 2.96566, 0.0}, {0, 0, 2.97404, 0.0}, {0, 0, 2.98242, 0.0}, {0, 0, 2.9908, 0.0}, {0, 0, 2.99917, 0.0}, {0, 0, 3.00755, 0.0}, {0, 0, 3.01593, 0.0}, {0, 0, 3.02431, 0.0}, {0, 0, 3.03268, 0.0}, {0, 0, 3.04106, 0.0}, {0, 0, 3.04944, 0.0}, {0, 0, 3.05782, 0.0}, {0, 0, 3.06619, 0.0}, {0, 0, 3.07457, 0.0}, {0, 0, 3.08295, 0.0}, {0, 0, 3.09133, 0.0}, {0, 0, 3.0997, 0.0}, {0, 0, 3.10808, 0.0}, {0, 0, 3.11646, 0.0}, {0, 0, 3.12484, 0.0}, {0, 0, 3.13322, 0.0}, {0, 0, 3.14159, 0.0}, {0, 0, 3.14997, 0.0}, {0, 0, 3.15835, 0.0}, {0, 0, 3.16673, 0.0}, {0, 0, 3.1751, 0.0}, {0, 0, 3.18348, 0.0}, {0, 0, 3.19186, 0.0}, {0, 0, 3.20024, 0.0}, {0, 0, 3.20861, 0.0}, {0, 0, 3.21699, 0.0}, {0, 0, 3.22537, 0.0}, {0, 0, 3.23375, 0.0}, {0, 0, 3.24212, 0.0}, {0, 0, 3.2505, 0.0}, {0, 0, 3.25888, 0.0}, {0, 0, 3.26726, 0.0}, {0, 0, 3.27563, 0.0}, {0, 0, 3.28401, 0.0}, {0, 0, 3.29239, 0.0}, {0, 0, 3.30077, 0.0}, {0, 0, 3.30914, 0.0}, {0, 0, 3.31752, 0.0}, {0, 0, 3.3259, 0.0}, {0, 0, 3.33428, 0.0}, {0, 0, 3.34265, 0.0}, {0, 0, 3.35103, 0.0}, {0, 0, 3.35941, 0.0}, {0, 0, 3.36779, 0.0}, {0, 0, 3.37616, 0.0}, {0, 0, 3.38454, 0.0}, {0, 0, 3.39292, 0.0}, {0, 0, 3.4013, 0.0}, {0, 0, 3.40968, 0.0}, {0, 0, 3.41805, 0.0}, {0, 0, 3.42643, 0.0}, {0, 0, 3.43481, 0.0}, {0, 0, 3.44319, 0.0}, {0, 0, 3.45156, 0.0}, {0, 0, 3.45994, 0.0}, {0, 0, 3.46832, 0.0}, {0, 0, 3.4767, 0.0}, {0, 0, 3.48507, 0.0}, {0, 0, 3.49345, 0.0}, {0, 0, 3.50183, 0.0}, {0, 0, 3.51021, 0.0}, {0, 0, 3.51858, 0.0}, {0, 0, 3.52696, 0.0}, {0, 0, 3.53534, 0.0}, {0, 0, 3.54372, 0.0}, {0, 0, 3.55209, 0.0}, {0, 0, 3.56047, 0.0}, {0, 0, 3.56885, 0.0}, {0, 0, 3.57723, 0.0}, {0, 0, 3.5856, 0.0}, {0, 0, 3.59398, 0.0}, {0, 0, 3.60236, 0.0}, {0, 0, 3.61074, 0.0}, {0, 0, 3.61911, 0.0}, {0, 0, 3.62749, 0.0}, {0, 0, 3.63587, 0.0}, {0, 0, 3.64425, 0.0}, {0, 0, 3.65263, 0.0}, {0, 0, 3.661, 0.0}, {0, 0, 3.66938, 0.0}, {0, 0, 3.67776, 0.0}, {0, 0, 3.68614, 0.0}, {0, 0, 3.69451, 0.0}, {0, 0, 3.70289, 0.0}, {0, 0, 3.71127, 0.0}, {0, 0, 3.71965, 0.0}, {0, 0, 3.72802, 0.0}, {0, 0, 3.7364, 0.0}, {0, 0, 3.74478, 0.0}, {0, 0, 3.75316, 0.0}, {0, 0, 3.76153, 0.0}, {0, 0, 3.76991, 0.0}, {0, 0, 3.77829, 0.0}, {0, 0, 3.78667, 0.0}, {0, 0, 3.79504, 0.0}, {0, 0, 3.80342, 0.0}, {0, 0, 3.8118, 0.0}, {0, 0, 3.82018, 0.0}, {0, 0, 3.82855, 0.0}, {0, 0, 3.83693, 0.0}, {0, 0, 3.84531, 0.0}, {0, 0, 3.85369, 0.0}, {0, 0, 3.86206, 0.0}, {0, 0, 3.87044, 0.0}, {0, 0, 3.87882, 0.0}, {0, 0, 3.8872, 0.0}, {0, 0, 3.89557, 0.0}, {0, 0, 3.90395, 0.0}, {0, 0, 3.91233, 0.0}, {0, 0, 3.92071, 0.0}, {0, 0, 3.92909, 0.0}, {0, 0, 3.93746, 0.0}, {0, 0, 3.94584, 0.0}, {0, 0, 3.95422, 0.0}, {0, 0, 3.9626, 0.0}, {0, 0, 3.97097, 0.0}, {0, 0, 3.97935, 0.0}, {0, 0, 3.98773, 0.0}, {0, 0, 3.99611, 0.0}, {0, 0, 4.00448, 0.0}, {0, 0, 4.01286, 0.0}, {0, 0, 4.02124, 0.0}, {0, 0, 4.02962, 0.0}, {0, 0, 4.03799, 0.0}, {0, 0, 4.04637, 0.0}, {0, 0, 4.05475, 0.0}, {0, 0, 4.06313, 0.0}, {0, 0, 4.0715, 0.0}, {0, 0, 4.07988, 0.0}, {0, 0, 4.08826, 0.0}, {0, 0, 4.09664, 0.0}, {0, 0, 4.10501, 0.0}, {0, 0, 4.11339, 0.0}, {0, 0, 4.12177, 0.0}, {0, 0, 4.13015, 0.0}, {0, 0, 4.13852, 0.0}, {0, 0, 4.1469, 0.0}, {0, 0, 4.15528, 0.0}, {0, 0, 4.16366, 0.0}, {0, 0, 4.17204, 0.0}, {0, 0, 4.18041, 0.0}, {0, 0, 4.18879, 0.0}, {0, 0, 4.19717, 0.0}, {0, 0, 4.20555, 0.0}, {0, 0, 4.21392, 0.0}, {0, 0, 4.2223, 0.0}, {0, 0, 4.23068, 0.0}, {0, 0, 4.23906, 0.0}, {0, 0, 4.24743, 0.0}, {0, 0, 4.25581, 0.0}, {0, 0, 4.26419, 0.0}, {0, 0, 4.27257, 0.0}, {0, 0, 4.28094, 0.0}, {0, 0, 4.28932, 0.0}, {0, 0, 4.2977, 0.0}, {0, 0, 4.30608, 0.0}, {0, 0, 4.31445, 0.0}, {0, 0, 4.32283, 0.0}, {0, 0, 4.33121, 0.0}, {0, 0, 4.33959, 0.0}, {0, 0, 4.34796, 0.0}, {0, 0, 4.35634, 0.0}, {0, 0, 4.36472, 0.0}, {0, 0, 4.3731, 0.0}, {0, 0, 4.38147, 0.0}, {0, 0, 4.38985, 0.0}, {0, 0, 4.39823, 0.0}, {0, 0, 4.40661, 0.0}, {0, 0, 4.41498, 0.0}, {0, 0, 4.42336, 0.0}, {0, 0, 4.43174, 0.0}, {0, 0, 4.44012, 0.0}, {0, 0, 4.4485, 0.0}, {0, 0, 4.45687, 0.0}, {0, 0, 4.46525, 0.0}, {0, 0, 4.47363, 0.0}, {0, 0, 4.48201, 0.0}, {0, 0, 4.49038, 0.0}, {0, 0, 4.49876, 0.0}, {0, 0, 4.50714, 0.0}, {0, 0, 4.51552, 0.0}, {0, 0, 4.52389, 0.0}, {0, 0, 4.53227, 0.0}, {0, 0, 4.54065, 0.0}, {0, 0, 4.54903, 0.0}, {0, 0, 4.5574, 0.0}, {0, 0, 4.56578, 0.0}, {0, 0, 4.57416, 0.0}, {0, 0, 4.58254, 0.0}, {0, 0, 4.59091, 0.0}, {0, 0, 4.59929, 0.0}, {0, 0, 4.60767, 0.0}, {0, 0, 4.61605, 0.0}, {0, 0, 4.62442, 0.0}, {0, 0, 4.6328, 0.0}, {0, 0, 4.64118, 0.0}, {0, 0, 4.64956, 0.0}, {0, 0, 4.65793, 0.0}, {0, 0, 4.66631, 0.0}, {0, 0, 4.67469, 0.0}, {0, 0, 4.68307, 0.0}, {0, 0, 4.69145, 0.0}, {0, 0, 4.69982, 0.0}, {0, 0, 4.7082, 0.0}, {0, 0, 4.71658, 0.0}, {0, 0, 4.72496, 0.0}, {0, 0, 4.73333, 0.0}, {0, 0, 4.74171, 0.0}, {0, 0, 4.75009, 0.0}, {0, 0, 4.75847, 0.0}, {0, 0, 4.76684, 0.0}, {0, 0, 4.77522, 0.0}, {0, 0, 4.7836, 0.0}, {0, 0, 4.79198, 0.0}, {0, 0, 4.80035, 0.0}, {0, 0, 4.80873, 0.0}, {0, 0, 4.81711, 0.0}, {0, 0, 4.82549, 0.0}, {0, 0, 4.83386, 0.0}, {0, 0, 4.84224, 0.0}, {0, 0, 4.85062, 0.0}, {0, 0, 4.859, 0.0}, {0, 0, 4.86737, 0.0}, {0, 0, 4.87575, 0.0}, {0, 0, 4.88413, 0.0}, {0, 0, 4.89251, 0.0}, {0, 0, 4.90088, 0.0}, {0, 0, 4.90926, 0.0}, {0, 0, 4.91764, 0.0}, {0, 0, 4.92602, 0.0}, {0, 0, 4.93439, 0.0}, {0, 0, 4.94277, 0.0}, {0, 0, 4.95115, 0.0}, {0, 0, 4.95953, 0.0}, {0, 0, 4.96791, 0.0}, {0, 0, 4.97628, 0.0}, {0, 0, 4.98466, 0.0}, {0, 0, 4.99304, 0.0}, {0, 0, 5.00142, 0.0}, {0, 0, 5.00979, 0.0}, {0, 0, 5.01817, 0.0}, {0, 0, 5.02655, 0.0}, {0, 0, 5.03493, 0.0}, {0, 0, 5.0433, 0.0}, {0, 0, 5.05168, 0.0}, {0, 0, 5.06006, 0.0}, {0, 0, 5.06844, 0.0}, {0, 0, 5.07681, 0.0}, {0, 0, 5.08519, 0.0}, {0, 0, 5.09357, 0.0}, {0, 0, 5.10195, 0.0}, {0, 0, 5.11032, 0.0}, {0, 0, 5.1187, 0.0}, {0, 0, 5.12708, 0.0}, {0, 0, 5.13546, 0.0}, {0, 0, 5.14383, 0.0}, {0, 0, 5.15221, 0.0}, {0, 0, 5.16059, 0.0}, {0, 0, 5.16897, 0.0}, {0, 0, 5.17734, 0.0}, {0, 0, 5.18572, 0.0}, {0, 0, 5.1941, 0.0}, {0, 0, 5.20248, 0.0}, {0, 0, 5.21086, 0.0}, {0, 0, 5.21923, 0.0}, {0, 0, 5.22761, 0.0}, {0, 0, 5.23599, 0.0}, {0, 0, 5.24437, 0.0}, {0, 0, 5.25274, 0.0}, {0, 0, 5.26112, 0.0}, {0, 0, 5.2695, 0.0}, {0, 0, 5.27788, 0.0}, {0, 0, 5.28625, 0.0}, {0, 0, 5.29463, 0.0}, {0, 0, 5.30301, 0.0}, {0, 0, 5.31139, 0.0}, {0, 0, 5.31976, 0.0}, {0, 0, 5.32814, 0.0}, {0, 0, 5.33652, 0.0}, {0, 0, 5.3449, 0.0}, {0, 0, 5.35327, 0.0}, {0, 0, 5.36165, 0.0}, {0, 0, 5.37003, 0.0}, {0, 0, 5.37841, 0.0}, {0, 0, 5.38678, 0.0}, {0, 0, 5.39516, 0.0}, {0, 0, 5.40354, 0.0}, {0, 0, 5.41192, 0.0}, {0, 0, 5.42029, 0.0}, {0, 0, 5.42867, 0.0}, {0, 0, 5.43705, 0.0}, {0, 0, 5.44543, 0.0}, {0, 0, 5.4538, 0.0}, {0, 0, 5.46218, 0.0}, {0, 0, 5.47056, 0.0}, {0, 0, 5.47894, 0.0}, {0, 0, 5.48732, 0.0}, {0, 0, 5.49569, 0.0}, {0, 0, 5.50407, 0.0}, {0, 0, 5.51245, 0.0}, {0, 0, 5.52083, 0.0}, {0, 0, 5.5292, 0.0}, {0, 0, 5.53758, 0.0}, {0, 0, 5.54596, 0.0}, {0, 0, 5.55434, 0.0}, {0, 0, 5.56271, 0.0}, {0, 0, 5.57109, 0.0}, {0, 0, 5.57947, 0.0}, {0, 0, 5.58785, 0.0}, {0, 0, 5.59622, 0.0}, {0, 0, 5.6046, 0.0}, {0, 0, 5.61298, 0.0}, {0, 0, 5.62136, 0.0}, {0, 0, 5.62973, 0.0}, {0, 0, 5.63811, 0.0}, {0, 0, 5.64649, 0.0}, {0, 0, 5.65487, 0.0}, {0, 0, 5.66324, 0.0}, {0, 0, 5.67162, 0.0}, {0, 0, 5.68, 0.0}, {0, 0, 5.68838, 0.0}, {0, 0, 5.69675, 0.0}, {0, 0, 5.70513, 0.0}, {0, 0, 5.71351, 0.0}, {0, 0, 5.72189, 0.0}, {0, 0, 5.73027, 0.0}, {0, 0, 5.73864, 0.0}, {0, 0, 5.74702, 0.0}, {0, 0, 5.7554, 0.0}, {0, 0, 5.76378, 0.0}, {0, 0, 5.77215, 0.0}, {0, 0, 5.78053, 0.0}, {0, 0, 5.78891, 0.0}, {0, 0, 5.79729, 0.0}, {0, 0, 5.80566, 0.0}, {0, 0, 5.81404, 0.0}, {0, 0, 5.82242, 0.0}, {0, 0, 5.8308, 0.0}, {0, 0, 5.83917, 0.0}, {0, 0, 5.84755, 0.0}, {0, 0, 5.85593, 0.0}, {0, 0, 5.86431, 0.0}, {0, 0, 5.87268, 0.0}, {0, 0, 5.88106, 0.0}, {0, 0, 5.88944, 0.0}, {0, 0, 5.89782, 0.0}, {0, 0, 5.90619, 0.0}, {0, 0, 5.91457, 0.0}, {0, 0, 5.92295, 0.0}, {0, 0, 5.93133, 0.0}, {0, 0, 5.9397, 0.0}, {0, 0, 5.94808, 0.0}, {0, 0, 5.95646, 0.0}, {0, 0, 5.96484, 0.0}, {0, 0, 5.97321, 0.0}, {0, 0, 5.98159, 0.0}, {0, 0, 5.98997, 0.0}, {0, 0, 5.99835, 0.0}, {0, 0, 6.00673, 0.0}, {0, 0, 6.0151, 0.0}, {0, 0, 6.02348, 0.0}, {0, 0, 6.03186, 0.0}, {0, 0, 6.04024, 0.0}, {0, 0, 6.04861, 0.0}, {0, 0, 6.05699, 0.0}, {0, 0, 6.06537, 0.0}, {0, 0, 6.07375, 0.0}, {0, 0, 6.08212, 0.0}, {0, 0, 6.0905, 0.0}, {0, 0, 6.09888, 0.0}, {0, 0, 6.10726, 0.0}, {0, 0, 6.11563, 0.0}, {0, 0, 6.12401, 0.0}, {0, 0, 6.13239, 0.0}, {0, 0, 6.14077, 0.0}, {0, 0, 6.14914, 0.0}, {0, 0, 6.15752, 0.0}, {0, 0, 6.1659, 0.0}, {0, 0, 6.17428, 0.0}, {0, 0, 6.18265, 0.0}, {0, 0, 6.19103, 0.0}, {0, 0, 6.19941, 0.0}, {0, 0, 6.20779, 0.0}, {0, 0, 6.21616, 0.0}, {0, 0, 6.22454, 0.0}, {0, 0, 6.23292, 0.0}, {0, 0, 6.2413, 0.0}, {0, 0, 6.24967, 0.0}, {0, 0, 6.25805, 0.0}, {0, 0, 6.26643, 0.0}, {0, 0, 6.27481, 0.0}};

    #pragma endregion PythonData

    std::vector<std::vector<double>> cpp_data = particle_filter.get_particles_data();

    for(int i = 0; i < cpp_data.size(); i++) {
        ASSERT_NEAR(cpp_data[i][0], python_data[i][0], 0.00001);
        ASSERT_NEAR(cpp_data[i][1], python_data[i][1], 0.00001);
        ASSERT_NEAR(cpp_data[i][2], python_data[i][2], 0.00001);
    }
}

TEST(ParticleFilterTests, feedback_test) {
    GTEST_SKIP() << "skipping feedback test";
    rapidcsv::Document motor_feedback_sheet("/home/tony/autonav_software_2024/autonav_ws/src/autonav_filters/tests/ENTRY_FEEDBACK.csv");

    std::vector<double> delta_xs = motor_feedback_sheet.GetColumn<double>(2);
    std::vector<double> delta_ys = motor_feedback_sheet.GetColumn<double>(3);
    std::vector<double> delta_thetas = motor_feedback_sheet.GetColumn<double>(4);

    //printf("delta_xs size: %ld\n", delta_xs.size());

    std::vector sliced_delta_xs(delta_xs.begin(), delta_xs.begin() + 1402);
    std::vector sliced_delta_ys(delta_ys.begin(), delta_ys.begin() + 1402);
    std::vector sliced_delta_thetas(delta_thetas.begin(), delta_thetas.begin() + 1402);
    // printf("sliced_delta_xs size: %ld\n", sliced_delta_xs.size());
    // printf("sliced_delta_xs size: %ld\n", sliced_delta_xs.size());
    // printf("sliced_delta_xs size: %ld\n", sliced_delta_xs.size());

    int num_particles = 750;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);
    particle_filter.init_particles();

    // construct feedback messages

    autonav_msgs::msg::MotorFeedback motor_message;
    autonav_msgs::msg::GPSFeedback gps_message;
    std::vector<double> position_vector; 
    for (int i = 0; i < 1402; i++) {
        motor_message.delta_x = sliced_delta_xs[i];
        motor_message.delta_y = sliced_delta_ys[i];
        motor_message.delta_theta = sliced_delta_thetas[i];

        position_vector = particle_filter.feedback(motor_message);
    }

    // printf("%f\n", position_vector[0]);
    // printf("%f\n", position_vector[1]);
    // printf("%f\n", position_vector[2]);

    EXPECT_EQ(position_vector[0], 1.4589810840940724e-15);
    EXPECT_EQ(position_vector[1], -1.1226575225009583e-14);
    EXPECT_EQ(position_vector[2], 3.4728605908690113);
}

TEST(ParticleFilterTests, feedback_sign_combine_test) {
    int num_particles = 750;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);
    particle_filter.init_particles();
    autonav_msgs::msg::MotorFeedback motor_message;
    double xs[2] = {4, -4};
    double ys[2] = {3, -3};
    double thetas[2] = {2, -2};

    // a triple for loop timed out gtest so im doing this garbage instead
    motor_message.delta_x = xs[0];
    motor_message.delta_y = ys[0];
    motor_message.delta_theta = thetas[0];

    std::vector<double> position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], -1.8071470246165216e-15);
    EXPECT_EQ(position_vector[1], 4.407141318552021e-15);
    EXPECT_EQ(position_vector[2], 4.409343000989507);
    

    motor_message.delta_x = xs[0];
    motor_message.delta_y = ys[0];
    motor_message.delta_theta = thetas[1];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], -1.8900436771218666e-15);
    EXPECT_EQ(position_vector[1], -5.459336686423437e-16);
    EXPECT_EQ(position_vector[2], 2.0845519307911555);

    motor_message.delta_x = xs[0];
    motor_message.delta_y = ys[1];
    motor_message.delta_theta = thetas[0];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], 1.5631940186722204e-15);
    EXPECT_EQ(position_vector[1], 1.1114072625180901e-15);
    EXPECT_EQ(position_vector[2], 4.409343000989507);

    motor_message.delta_x = xs[0];
    motor_message.delta_y = ys[1];
    motor_message.delta_theta = thetas[1];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], 1.3381888190148554e-16);
    EXPECT_EQ(position_vector[1], -9.710750722054703e-17);
    EXPECT_EQ(position_vector[2], 2.0845519307911555);


    motor_message.delta_x = xs[1];
    motor_message.delta_y = ys[0];
    motor_message.delta_theta = thetas[0];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], -1.0646298657472169e-15);
    EXPECT_EQ(position_vector[1], -1.2055541750063033e-15);
    EXPECT_EQ(position_vector[2], 4.409343000989507);

    motor_message.delta_x = xs[1];
    motor_message.delta_y = ys[0];
    motor_message.delta_theta = thetas[1];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], -1.7360927510405115e-15);
    EXPECT_EQ(position_vector[1], -2.0487315547749555e-16);
    EXPECT_EQ(position_vector[2], 2.0845519307911555);

    motor_message.delta_x = xs[1];
    motor_message.delta_y = ys[1];
    motor_message.delta_theta = thetas[0];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], 1.8237263551175905e-16);
    EXPECT_EQ(position_vector[1], -2.1257070178156331e-16);
    EXPECT_EQ(position_vector[2], 4.409343000989507);

    motor_message.delta_x = xs[1];
    motor_message.delta_y = ys[1];
    motor_message.delta_theta = thetas[1];
    position_vector = particle_filter.feedback(motor_message);
    EXPECT_EQ(position_vector[0], -6.010007306637514e-17);
    EXPECT_EQ(position_vector[1], -1.3914795241968628e-17);
    EXPECT_EQ(position_vector[2], 2.0845519307911555);

}

TEST(ParticleFilterTests, gps_test) {
    GTEST_SKIP() << "Skipping gps_test";
    int num_particles = 750;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);

    particle_filter.init_particles();

    autonav_msgs::msg::GPSFeedback gps_feedback_1 = autonav_msgs::msg::GPSFeedback();
    gps_feedback_1.latitude = 42.6681254;
    gps_feedback_1.longitude = -83.2188876;
    gps_feedback_1.altitude = 234.891;
    gps_feedback_1.gps_fix = 4;
    gps_feedback_1.is_locked = true;
    gps_feedback_1.satellites = 16;

    std::vector<double> python_gps_vector_1 = {0.0, 0.0};

    std::vector<double> gps_vector_1 = particle_filter.gps(gps_feedback_1);

    ASSERT_EQ(gps_vector_1, python_gps_vector_1);
}

TEST(ParticleFilterTests, gps_combine_test) {
    int num_particles = 1;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);

    particle_filter.init_particles();

    autonav_msgs::msg::GPSFeedback gps_feedback = autonav_msgs::msg::GPSFeedback();
    gps_feedback.latitude = 42.6681254;
    gps_feedback.longitude = -83.2188876;
    gps_feedback.altitude = 234.891;
    gps_feedback.gps_fix = 4;
    gps_feedback.is_locked = true;
    gps_feedback.satellites = 16;

    std::vector<double> python_gps_vector = {0.0, 0.0};

    std::vector<double> gps_vector = particle_filter.gps(gps_feedback);

    ASSERT_EQ(gps_vector[0], python_gps_vector[0]);
    ASSERT_EQ(gps_vector[1], python_gps_vector[1]);

    gps_feedback.latitude = 42.66;

    python_gps_vector = {-902.6198094804877, 0.0};

    gps_vector = particle_filter.gps(gps_feedback);

    ASSERT_EQ(gps_vector[0], python_gps_vector[0]);
    ASSERT_EQ(gps_vector[1], python_gps_vector[1]);
}

TEST(ParticleFilterTests, complete_test) {
    GTEST_SKIP() << "skipping test";
    // This test will fail sometimes because the particle filter is non-deterministic
    int num_particles = 750;
    double latitudeLength = 111086.2;
    double longitudeLength = 81978.2;
    double gps_noise = 0.8;
    double odom_noise[3] = {0.5, 0.5, 0.1};
    ParticleFilter particle_filter = ParticleFilter(num_particles, latitudeLength, longitudeLength, gps_noise, odom_noise[0], odom_noise[1], odom_noise[2]);

    particle_filter.init_particles();

    rapidcsv::Document motor_feedback_sheet("/home/tony/autonav_software_2024/autonav_ws/src/autonav_filters/tests/ENTRY_FEEDBACK.csv");
    rapidcsv::Document gps_feedback_sheet("/home/tony/autonav_software_2024/autonav_ws/src/autonav_filters/tests/ENTRY_GPS.csv");

    std::vector<double> delta_xs = motor_feedback_sheet.GetColumn<double>(2);
    std::vector<double> delta_ys = motor_feedback_sheet.GetColumn<double>(3);
    std::vector<double> delta_thetas = motor_feedback_sheet.GetColumn<double>(4);

    std::vector<double> latitudes = gps_feedback_sheet.GetColumn<double>(2);
    std::vector<double> longitudes = gps_feedback_sheet.GetColumn<double>(3);
    std::vector<double> altitudes = gps_feedback_sheet.GetColumn<double>(4);
    std::vector<double> gps_fix = gps_feedback_sheet.GetColumn<double>(5);
    std::vector<std::string> is_locked_str = gps_feedback_sheet.GetColumn<std::string>(6);
    std::vector<bool> is_locked;
    std::vector<double> satellites = gps_feedback_sheet.GetColumn<double>(7);

    for (std::string s : is_locked_str) {
        is_locked.push_back(csv_utils::to_bool(s));
    };

    // cut down the deltas until they are the same length as the gps values
    
    int n = latitudes.size();
    //n = 1;

    std::vector sliced_delta_xs(delta_xs.begin(), delta_xs.begin() + n);
    std::vector sliced_delta_ys(delta_ys.begin(), delta_ys.begin() + n);
    std::vector sliced_delta_thetas(delta_thetas.begin(), delta_thetas.begin() + n);

    // construct feedback messages

    autonav_msgs::msg::MotorFeedback motor_message;
    autonav_msgs::msg::GPSFeedback gps_message;
    std::vector<double> position_vector;
    std::vector<double> gps_vector;
    for (int i = 0; i < n; i++) {
        motor_message.delta_x = sliced_delta_xs[i];
        //printf("motor_mesage delta_x %f\n", motor_message.delta_x);
        motor_message.delta_y = sliced_delta_ys[i];
        //printf("motor_mesage delta_y %f\n", motor_message.delta_y);
        motor_message.delta_theta = sliced_delta_thetas[i];
        //printf("motor_mesage delta_theta %f\n\n", motor_message.delta_theta);

        gps_message.latitude = latitudes[i];
        gps_message.longitude = longitudes[i];
        gps_message.altitude = altitudes[i];
        gps_message.gps_fix = gps_fix[i];
        gps_message.is_locked = is_locked[i];
        gps_message.satellites = satellites[i];

        position_vector = particle_filter.feedback(motor_message);
        gps_vector = particle_filter.gps(gps_message);
    }

    printf("%f\n", position_vector[0]);
    printf("%f\n", position_vector[1]);
    printf("%f\n", position_vector[2]);

    ASSERT_NEAR(position_vector[0], 17.507143650199076, 1.0);
    ASSERT_NEAR(position_vector[1], 17.762181504279035, 1.0);
    ASSERT_NEAR(position_vector[2], 4.3224496833375365, 1.0);

    /*std::filesystem::path cwd = std::filesystem::current_path();
    std::string cwd_str = cwd.string();
    printf(cwd_str.c_str()); */
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
