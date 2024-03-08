#include <math.h>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>
#include <unordered_map>
#include <filesystem>

#include <bits/stdc++.h>


struct GPSPoint {
    double lat;
    double lon;
};


// main initilization method (because we don't really have a constructor for reasons)
void init() {
    const std::string WAYPOINTS_FILENAME = "./waypoints.csv";
    std::ifstream waypointsFile;
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of a list of double tuples

    // === read waypoints from file ===
    waypointsFile.open(WAYPOINTS_FILENAME);
    int numWaypoints = 0;
    bool firstLine = false;

    // loop through the lines in the file
    std::string line;
    if (waypointsFile.is_open()) {
        std::cout << "YES FILE IS OPEN" << std::endl;
        while (getline(waypointsFile, line) ) {
            if (!firstLine) { // first line is the one with the labels, we need to skip it
                firstLine = true;
                continue;
            }

            // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
            std::vector<std::string> tokens;
            std::stringstream strstream(line);
            std::string intermediate;
            while(getline(strstream, intermediate, ',')) {
                tokens.push_back(intermediate);
            }

            GPSPoint point;
            point.lat = std::stod(tokens[1]); //https://cplusplus.com/reference/string/stod/
            point.lon = std::stod(tokens[2]);

            // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
            waypointsDict[label].push_back(point);
            numWaypoints++;
        }
    }
    std::cout << "NUM WAYPOINTS: " + std::to_string(numWaypoints) << std::endl;

    waypointsFile.close();
    std::cout << "WAYPOINTS READ" << std::endl;
    // === /read waypoints ===
}

int main(int argc, char *argv[]) {
    init();
    return 0;
}