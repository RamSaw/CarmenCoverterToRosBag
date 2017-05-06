//
// Created by mikhail on 06.05.17.
//

#ifndef CARMENCOVERTERTOROSBAG_PARAMS_H
#define CARMENCOVERTERTOROSBAG_PARAMS_H

#include <unordered_map>

class Params {
public:
    static std::unordered_map<std::string, std::string> old_laser_params;
    static std::unordered_map<std::string, std::string> new_laser_params; /// Change laser_i?
    static std::unordered_map<std::string, std::string> robot_params;
    static std::unordered_map<std::string, std::string> gps_params;
    static std::unordered_map<std::string, std::string> base_params;
    static std::unordered_map<std::string, std::string> arm_params;
    static std::unordered_map<std::string, std::string> segway_params;

    static std::string laserId2ParamPrefix(std::string &laser_id);
};

#endif //CARMENCOVERTERTOROSBAG_PARAMS_H
