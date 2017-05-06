//
// Created by mikhail on 06.05.17.
//

#ifndef CARMENCOVERTERTOROSBAG_PARAMS_H
#define CARMENCOVERTERTOROSBAG_PARAMS_H

#include <unordered_map>

class Params {
public:
    static std::unordered_map<std::string, float> old_laser_params;
    static std::unordered_map<std::string, float> new_laser_params; /// Change laser_i?
    static std::unordered_map<std::string, float> robot_params;
    static std::unordered_map<std::string, float> gps_params;
    static std::unordered_map<std::string, float> base_params;
    static std::unordered_map<std::string, float> arm_params;
    static std::unordered_map<std::string, float> segway_params;

    static std::string laserId2ParamPrefix(std::string &laser_id);
};

#endif //CARMENCOVERTERTOROSBAG_PARAMS_H
