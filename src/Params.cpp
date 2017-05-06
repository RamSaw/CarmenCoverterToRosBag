//
// Created by mikhail on 06.05.17.
//

#include "Params.h"

std::unordered_map<std::string, std::string> Params::old_laser_params = {{"front_laser_dev", std::string()},
                                                                    {"rear_laser_dev", std::string()},
                                                                    {"laser3_dev", std::string()},
                                                                    {"laser4_dev", std::string()},
                                                                    {"laser5_dev", std::string()},
                                                                    {"front_laser_type", std::string()},
                                                                    {"front_laser_resolution", std::string()},
                                                                    {"front_laser_use_remission", std::string()},
                                                                    {"front_laser_fov", std::string()},
                                                                    {"front_laser_baud", std::string()},
                                                                    {"front_laser_flipped", std::string()},
                                                                    {"rear_laser_type", std::string()},
                                                                    {"rear_laser_resolution", std::string()},
                                                                    {"rear_laser_use_remission", std::string()},
                                                                    {"rear_laser_fov", std::string()},
                                                                    {"rear_laser_baud", std::string()},
                                                                    {"rear_laser_flipped", std::string()},
                                                                    {"laser3_type", std::string()},
                                                                    {"laser3_resolution", std::string()},
                                                                    {"laser3_use_remission", std::string()},
                                                                    {"laser3_fov", std::string()},
                                                                    {"laser3_baud", std::string()},
                                                                    {"laser3_flipped", std::string()},
                                                                    {"laser4_type", std::string()},
                                                                    {"laser4_resolution", std::string()},
                                                                    {"laser4_use_remission", std::string()},
                                                                    {"laser4_fov", std::string()},
                                                                    {"laser4_baud", std::string()},
                                                                    {"laser4_flipped", std::string()},
                                                                    {"laser5_type", std::string()},
                                                                    {"laser5_resolution", std::string()},
                                                                    {"laser5_use_remission", std::string()},
                                                                    {"laser5_fov", std::string()},
                                                                    {"laser5_baud", std::string()},
                                                                    {"laser5_flipped", std::string()}};

std::unordered_map<std::string, std::string> Params::new_laser_params = {{"laser1_flipped", std::string()},
                                                                    {"laser2_flipped", std::string()},
                                                                    {"laser3_flipped", std::string()},
                                                                    {"laser4_flipped", std::string()},
                                                                    {"laser5_flipped", std::string()},
                                                                    {"laser6_flipped", std::string()},
                                                                    {"laser7_flipped", std::string()},
                                                                    {"laser8_flipped", std::string()},
                                                                    {"laser9_flipped", std::string()}};

std::unordered_map<std::string, std::string> Params::robot_params = {{"callow_rear_motion", std::string()},
                                                                {"rectangular", std::string()},
                                                                {"use_laser", std::string()},
                                                                {"use_sonar", std::string()},
                                                                {"converge", std::string()},
                                                                {"timeout", std::string()},
                                                                {"sensor_timeout", std::string()},
                                                                {"collision_avoidance", std::string()},
                                                                {"collision_avoidance_frequency", std::string()},
                                                                {"laser_bearing_skip_rate", std::string()},
                                                                {"turn_before_driving_if_heading_bigger", std::string()},
                                                                {"backwards", std::string()},
                                                                {"length", std::string()},
                                                                {"width", std::string()},
                                                                {"frontlaser_use", std::string()},
                                                                {"frontlaser_id", std::string()},
                                                                {"frontlaser_offset", std::string()},
                                                                {"frontlaser_side_offset", std::string()},
                                                                {"frontlaser_angular_offset", std::string()},
                                                                {"rearlaser_use", std::string()},
                                                                {"rearlaser_id", std::string()},
                                                                {"rearlaser_offset", std::string()},
                                                                {"rearlaser_side_offset", std::string()},
                                                                {"rearlaser_angular_offset", std::string()},
                                                                {"front_laser_max", std::string()},
                                                                {"min_approach_dist", std::string()},
                                                                {"min_side_dist", std::string()},
                                                                {"acceleration", std::string()},
                                                                {"deceleration", std::string()},
                                                                {"reaction_time", std::string()},
                                                                {"max_t_vel", std::string()},
                                                                {"max_r_vel", std::string()},
                                                                {"curvature", std::string()},
                                                                {"theta_gain", std::string()},
                                                                {"displacement_gain", std::string()},
                                                                {"use_bumper", std::string()},
                                                                {"bumper_offsets", std::string()},
                                                                {"odometry_inverted", std::string()}};

std::unordered_map<std::string, std::string> Params::gps_params = {{"nmea_dev", std::string()},
                                                              {"nmea_baud", std::string()},
                                                              {"originlat", std::string()},
                                                              {"originlon", std::string()},
                                                              {"integrate_with_odometry", std::string()},
                                                              {"initialtheta", std::string()},
                                                              {"initialthetastd", std::string()},
                                                              {"odomdiststdper1m", std::string()},
                                                              {"odomthetastdper1m", std::string()},
                                                              {"odomthetastdper1rad", std::string()},
                                                              {"gpsxystdper1precdil", std::string()}};

std::unordered_map<std::string, std::string> Params::base_params = {{"type", std::string()},
                                                               {"model", std::string()},
                                                               {"motion_timeout", std::string()},
                                                               {"dev", std::string()},
                                                               {"use_hardware_integrator", std::string()},
                                                               {"relative_wheelsize", std::string()},
                                                               {"relative_wheelbase", std::string()}};

std::unordered_map<std::string, std::string> Params::arm_params = {{"num_joints", std::string()},
                                                              {"joint_types", std::string()},
                                                              {"dev", std::string()}};

std::unordered_map<std::string, std::string> Params::segway_params = {{"accel_limit", std::string()},
                                                                 {"torque_limit", std::string()},
                                                                 {"gain_schedule", std::string()}};

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
