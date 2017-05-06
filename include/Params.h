//
// Created by mikhail on 06.05.17.
//

#ifndef CARMENCOVERTERTOROSBAG_PARAMS_H
#define CARMENCOVERTERTOROSBAG_PARAMS_H

#include <unordered_map>

class Params {
public:
    static std::unordered_map<std::string, double> old_laser_params;
    static std::unordered_map<std::string, double> new_laser_params; /// Change laser_i?
    static std::unordered_map<std::string, double> robot_params;
    static std::unordered_map<std::string, double> gps_params;
    static std::unordered_map<std::string, double> base_params;
    static std::unordered_map<std::string, double> arm_params;
    static std::unordered_map<std::string, double> segway_params;
};

#endif //CARMENCOVERTERTOROSBAG_PARAMS_H
