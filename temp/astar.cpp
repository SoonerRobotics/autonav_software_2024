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


struct GPSPoint {
    double lat;
    double lon;
};


// main initilization method (because we don't really have a constructor for reasons)
void init() {
    GPSPoint gps_position; // for smellification algorithm
    const std::string WAYPOINTS_FILENAME = "./waypoints.csv";
    std::ifstream waypointsFile;
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of a list of double tuples
    std::vector<GPSPoint> waypoints; // gps waypoints we PID to //TODO this needs to be how it is and the other thing needs to be renamed

    std::cout << "INITIALIZATION STARTED" << std::endl;
    std::cout << "STARTING LISTING ALL FILES" << std::endl;

    std::string path = ".";
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        std::cout << entry.path().string() << std::endl;
    }
    std::cout << "ALL FILES LISTED" << std::endl;

    // === read waypoints from file ===
    waypointsFile.open(WAYPOINTS_FILENAME);
    double numWaypoints = 0;
    bool firstLine = false;

    // loop through the lines in the file
    std::string line;
    if (waypointsFile.is_open()) {
        std::cout << "YES FILE IS OPEN" << std::endl;
        while (getline(waypointsFile, line) ) {
            if (!firstLine) { // first line is the one with the labels, we can skip it
                firstLine = true;
                continue;
            }
            // label, latitude, longitude = line.split(",")
            // format is like label,lat,lon, right?
            // so label is [0:first comma]
            // latitude is [first comma:second to last comma] (because remember there's a trailing comma before the newline)
            // longitude is [second to last comma:end of string-1] (so we don't catch that last comma at the end)
            std::cout << "LINE: " + line << std::endl;
            
            std::string label = line.substr(0, line.find(",")); //https://cplusplus.com/reference/string/string/find/
            
            // alright so what if we look for [first comma:second comma] ?
            auto firstCommaIndex = line.find(",");
            auto secondCommaIndex = line.substr(firstCommaIndex+1, line.length()).find(",");
            auto thirdCommaIndex = line.substr(secondCommaIndex+2, line.length()).find(","); // do we even need this?
          
            auto latString = line.substr(firstCommaIndex+1, secondCommaIndex);
            auto lonString = line.substr(secondCommaIndex+1, thirdCommaIndex);
          
            std::cout << "LAT: " + latString << std::endl;
            std::cout << "LON: " + lonString << std::endl;
            std::cout << std::endl;
           
            // double lat = std::stod(latString); //https://cplusplus.com/reference/string/stod/
            // double lon = std::stod(lonString);
            numWaypoints++;

            // if the vector doesn't exist yet in the dictionary, make it
            // if (!waypoints[label]) {
            //     waypoints[label] = std::vector<std::vector<double>>(5);
            // }

            // waypoints are stored like {"north":[(lat, lon), (lat, lon)]}
            // with lat, lon in sequential order
            // waypointsDict[label].push_back({lat, lon});
            // waypointsDict[label].push_back(GPSPoint(lat, lon));
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