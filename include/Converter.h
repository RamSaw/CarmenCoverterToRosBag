//
// Created by mikhail on 05.05.17.
//

#ifndef CARMENCOVERTERTOROSBAG_CONVERTER_H
#define CARMENCOVERTERTOROSBAG_CONVERTER_H


#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include "Params.h"

// Converts degrees to radians.
#define radians(angleDegrees) (angleDegrees * static_cast<float>(M_PI) / 180)

class Converter {
private:
    std::ifstream carmen_file;
    std::ofstream bag_file;

    /* static constants defines */
    static const std::vector<std::string> LASER_MESSAGE_DEFINED;
    static const std::vector<std::string> OLD_LASER_MESSAGE_DEFINED;
    static const std::vector<std::string> ROBOT_LASER_MESSAGE_DEFINED;
    static const std::vector<std::string> GPS_MESSAGE_DEFINED;
    static const std::string ODOM_DEFINED;
    static const std::string TRUEPOS_DEFINED;
    static const std::string PARAM_DEFINED;
    static const std::string SYNC_DEFINED;

    std::vector<std::string> unknown_entries;

    ros::Time stamp;
    nav_msgs::Odometry pose_msg;
    nav_msgs::Odometry true_pose_msg;

    sensor_msgs::LaserScan laser_msg;

    geometry_msgs::TransformStamped tf_odom_robot_msg;
    geometry_msgs::TransformStamped tf_laser_robot_msg;
    tf::tfMessage tf2_msg;

    tf::Transformer tf_tr;

    bool pause;

    int rate; /// double ??

    /* If false(default) then ODOM is published on tf, true - ROBOTLASER1 */
    bool publish_corrected;

    std::unordered_map<std::string, std::string> topics;
    std::unordered_map<std::string, std::string> links;
    std::unordered_map<std::string, std::unordered_map<std::string, float>> params;

    void fillUpOdomMessage(std::vector<std::string> &words);
    void fillUpLaserMessage(std::vector<std::string> &words);
    void fillUpOldLaserMessage(std::vector<std::string> &words);
    void fillUpRobotLaserMessage(std::vector<std::string> &words);
    void increment_stamp();

public:
    Converter(int argc, char** argv);

    void convert(std::string input_filename, std::string output_filename);
};


#endif //CARMENCOVERTERTOROSBAG_CONVERTER_H
