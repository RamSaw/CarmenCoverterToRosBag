//
// Created by mikhail on 06.05.17.
//

#include "Params.h"

std::unordered_map<std::string, float> Params::old_laser_params = {{"front_laser_dev", float()},
                                                                    {"rear_laser_dev", float()},
                                                                    {"laser3_dev", float()},
                                                                    {"laser4_dev", float()},
                                                                    {"laser5_dev", float()},
                                                                    {"front_laser_type", float()},
                                                                    {"front_laser_resolution", float()},
                                                                    {"front_laser_use_remission", float()},
                                                                    {"front_laser_fov", float()},
                                                                    {"front_laser_baud", float()},
                                                                    {"front_laser_flipped", float()},
                                                                    {"rear_laser_type", float()},
                                                                    {"rear_laser_resolution", float()},
                                                                    {"rear_laser_use_remission", float()},
                                                                    {"rear_laser_fov", float()},
                                                                    {"rear_laser_baud", float()},
                                                                    {"rear_laser_flipped", float()},
                                                                    {"laser3_type", float()},
                                                                    {"laser3_resolution", float()},
                                                                    {"laser3_use_remission", float()},
                                                                    {"laser3_fov", float()},
                                                                    {"laser3_baud", float()},
                                                                    {"laser3_flipped", float()},
                                                                    {"laser4_type", float()},
                                                                    {"laser4_resolution", float()},
                                                                    {"laser4_use_remission", float()},
                                                                    {"laser4_fov", float()},
                                                                    {"laser4_baud", float()},
                                                                    {"laser4_flipped", float()},
                                                                    {"laser5_type", float()},
                                                                    {"laser5_resolution", float()},
                                                                    {"laser5_use_remission", float()},
                                                                    {"laser5_fov", float()},
                                                                    {"laser5_baud", float()},
                                                                    {"laser5_flipped", float()}};

std::unordered_map<std::string, float> Params::new_laser_params = {{"laser1_flipped", float()},
                                                                    {"laser2_flipped", float()},
                                                                    {"laser3_flipped", float()},
                                                                    {"laser4_flipped", float()},
                                                                    {"laser5_flipped", float()},
                                                                    {"laser6_flipped", float()},
                                                                    {"laser7_flipped", float()},
                                                                    {"laser8_flipped", float()},
                                                                    {"laser9_flipped", float()}};

std::unordered_map<std::string, float> Params::robot_params = {{"callow_rear_motion", float()},
                                                                {"rectangular", float()},
                                                                {"use_laser", float()},
                                                                {"use_sonar", float()},
                                                                {"converge", float()},
                                                                {"timeout", float()},
                                                                {"sensor_timeout", float()},
                                                                {"collision_avoidance", float()},
                                                                {"collision_avoidance_frequency", float()},
                                                                {"laser_bearing_skip_rate", float()},
                                                                {"turn_before_driving_if_heading_bigger", float()},
                                                                {"backwards", float()},
                                                                {"length", float()},
                                                                {"width", float()},
                                                                {"frontlaser_use", float()},
                                                                {"frontlaser_id", float()},
                                                                {"frontlaser_offset", float()},
                                                                {"frontlaser_side_offset", float()},
                                                                {"frontlaser_angular_offset", float()},
                                                                {"rearlaser_use", float()},
                                                                {"rearlaser_id", float()},
                                                                {"rearlaser_offset", float()},
                                                                {"rearlaser_side_offset", float()},
                                                                {"rearlaser_angular_offset", float()},
                                                                {"front_laser_max", float()},
                                                                {"min_approach_dist", float()},
                                                                {"min_side_dist", float()},
                                                                {"acceleration", float()},
                                                                {"deceleration", float()},
                                                                {"reaction_time", float()},
                                                                {"max_t_vel", float()},
                                                                {"max_r_vel", float()},
                                                                {"curvature", float()},
                                                                {"theta_gain", float()},
                                                                {"displacement_gain", float()},
                                                                {"use_bumper", float()},
                                                                {"bumper_offsets", float()},
                                                                {"odometry_inverted", float()}};

std::unordered_map<std::string, float> Params::gps_params = {{"nmea_dev", float()},
                                                              {"nmea_baud", float()},
                                                              {"originlat", float()},
                                                              {"originlon", float()},
                                                              {"integrate_with_odometry", float()},
                                                              {"initialtheta", float()},
                                                              {"initialthetastd", float()},
                                                              {"odomdiststdper1m", float()},
                                                              {"odomthetastdper1m", float()},
                                                              {"odomthetastdper1rad", float()},
                                                              {"gpsxystdper1precdil", float()}};

std::unordered_map<std::string, float> Params::base_params = {{"type", float()},
                                                               {"model", float()},
                                                               {"motion_timeout", float()},
                                                               {"dev", float()},
                                                               {"use_hardware_integrator", float()},
                                                               {"relative_wheelsize", float()},
                                                               {"relative_wheelbase", float()}};

std::unordered_map<std::string, float> Params::arm_params = {{"num_joints", float()},
                                                              {"joint_types", float()},
                                                              {"dev", float()}};

std::unordered_map<std::string, float> Params::segway_params = {{"accel_limit", float()},
                                                                 {"torque_limit", float()},
                                                                 {"gain_schedule", float()}};

std::string Params::laserId2ParamPrefix(std::string &laser_id) {
    if (laser_id == "FLASER")
        return "front_laser";
    else if (laser_id == "RLASER")
        return "rear_laser";
    else if (laser_id == "LASER3" || laser_id == "RAWLASER3")
        return "laser3";
    else if (laser_id == "LASER4" || laser_id == "RAWLASER4")
        return "laser4";
    else if (laser_id == "LASER5" || laser_id == "RAWLASER5")
        return "laser5";
    else if (laser_id == "RAWLASER1")
        return "laser1";
    else if (laser_id == "RAWLASER2")
        return "laser2";
    return "";
}
