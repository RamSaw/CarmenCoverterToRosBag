//
// Created by mikhail on 05.05.17.
//

#include "Converter.h"

#include "rosbag/bag.h"

Converter::Converter(int argc, char** argv) : tf_tr(true, ros::Duration(2.0)) {
    ros::init(argc, argv, "CarmenConverterToRosBag", ros::init_options::AnonymousName); /// Here or in main?
    ros::NodeHandle node;

    stamp = ros::Time::now();

    pause = false;
    
    ros::param::param<int>("~global_rate", rate, 40); /// Double ???


    ros::param::param<std::string>("~RAWLASER1_topic", RAWLASER1_topic, "/RAWLASER1");
    ros::param::param<std::string>("~RAWLASER2_topic", RAWLASER2_topic, "/RAWLASER2");
    ros::param::param<std::string>("~RAWLASER3_topic", RAWLASER3_topic, "/RAWLASER3");
    ros::param::param<std::string>("~RAWLASER4_topic", RAWLASER4_topic, "/RAWLASER4");

    ros::param::param<std::string>("~FLASER_topic", FLASER_topic, "/FLASER");
    ros::param::param<std::string>("~RLASER_topic", RLASER_topic, "/RLASER");
    ros::param::param<std::string>("~LASER3_topic", LASER3_topic, "/LASER3");
    ros::param::param<std::string>("~LASER4_topic", LASER4_topic, "/LASER4");

    ros::param::param<std::string>("~ROBOTLASER1_topic", ROBOTLASER1_topic, "/ROBOTLASER1");
    ros::param::param<std::string>("~ROBOTLASER2_topic", ROBOTLASER2_topic, "/ROBOTLASER2");

    ros::param::param<std::string>("~NMEAGGA_topic", NMEAGGA_topic, "/NMEAGGA");
    ros::param::param<std::string>("~NMEARMC_topic", NMEARMC_topic, "/NMEARMC");

    ros::param::param<std::string>("~ODOM_topic", ODOM_topic, "/ODOM");
    ros::param::param<std::string>("~TRUEPOS_topic", TRUEPOS_topic, "/TRUEPOS");

    ros::param::param<std::string>("~tf_topic", tf_topic, "/tf");

    ros::param::param<std::string>("~robot_link", robot_link, "base_link");
    ros::param::param<std::string>("~odom_link", odom_link, "odom");
    ros::param::param<std::string>("~odom_robot_link", odom_robot_link, "odom_robot_link");
    ros::param::param<std::string>("~trueodom_link", true_odom_link, "gt_odom");
    ros::param::param<std::string>("~ROBOTLASER1_link", ROBOTLASER1_link, "ROBOTLASER1_link");
    ros::param::param<std::string>("~ROBOTLASER2_link", ROBOTLASER2_link, "ROBOTLASER2_link");

    ros::param::param<bool>("~pub_corrected", publish_corrected, false);

    topics = {{"RAWLASER1", RAWLASER1_topic},
              {"RAWLASER2", RAWLASER2_topic},
              {"RAWLASER3", RAWLASER3_topic},
              {"RAWLASER4", RAWLASER4_topic},
              {"FLASER", FLASER_topic},
              {"RLASER", RLASER_topic},
              {"LASER3", LASER3_topic},
              {"LASER4", LASER4_topic},
              {"ROBOTLASER1", ROBOTLASER1_topic},
              {"ROBOTLASER2", ROBOTLASER2_topic},
              {"NMEAGGA", NMEAGGA_topic},
              {"NMEARMC", NMEARMC_topic},
              {"ODOM", ODOM_topic},
              {"TRUEPOS", TRUEPOS_topic},
              {"TF", tf_topic}};

    links = {{"ROBOT",     robot_link},
             {"ODOM",      odom_link},
             {"ROBOTODOM", odom_robot_link},
             {"TRUEPOS",   true_odom_link},
             {"ROBOTLASER1", ROBOTLASER1_link},
             {"ROBOTLASER2", ROBOTLASER2_link}};


    params = {{"laser", Params::old_laser_params},
              {"newlaser", Params::new_laser_params},
              {"robot", Params::robot_params},
              {"gps", Params::gps_params},
              {"base", Params::base_params},
              {"arm", Params::arm_params},
              {"segway", Params::segway_params}};
}

void Converter::convert(std::string input_filename, std::string output_filename) {
    carmen_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    carmen_file.open(input_filename);

    rosbag::Bag bag(output_filename, rosbag::bagmode::Write);

    ROS_INFO("Reading data from : %s", input_filename.c_str());
    ROS_INFO("Writing rosbag to : %s", output_filename.c_str());
    ROS_INFO("Converting started!");

    std::cout << "Reading data from : " + input_filename << std::endl;
    std::cout << "Writing rosbag to : " + output_filename << std::endl;
    std::cout << "Converting started!" << std::endl;

    std::string line;
    while (carmen_file.peek() != std::istream::traits_type::eof()) {
        std::getline(carmen_file, line);

        /* Split line on words */
        std::istringstream iss(line);
        std::vector<std::string> words{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};

        if (pause) { /// If statement is not needed
            while (pause)
                ros::Duration(1 / rate).sleep();
        }

        if (std::find(LASER_MESSAGE_DEFINED.begin(), LASER_MESSAGE_DEFINED.end(), words[0])
            != LASER_MESSAGE_DEFINED.end()) {

            std::cout << "Converting new NEW LASER message" << std::endl;
            fillUpLaserMessage(words);

            std::string topic = topics[words[0]];
            bag.write(topic, laser_msg.header.stamp, laser_msg);

            laser_msg.header.seq = laser_msg.header.seq + 1;
        }
        else if (words[0] == ODOM_DEFINED) {
            std::cout << "Converting new ODOM message" << std::endl;
            fillUpOdomMessage(words);

            if (pose_msg.header.stamp.sec != 0 && !publish_corrected) {
                std::string topic = topics["ODOM"];
                bag.write(topic, pose_msg.header.stamp, pose_msg);

                topic = topics["TF"];
                bag.write(topic, pose_msg.header.stamp, tf2_msg);

                pose_msg.header.seq = pose_msg.header.seq + 1;
                tf_odom_robot_msg.header.seq = tf_odom_robot_msg.header.seq + 1;
            }
        }

        increment_stamp();
        tf2_msg = tf::tfMessage();
    }

    bag.close();

    std::cout << "Converted!" << std::endl;
}

void Converter::fillUpOdomMessage(std::vector<std::string> &words) {
    double timestamp = ::atof(words[7].c_str()); /// double?
    if (timestamp < 0)
        return;

    pose_msg.header.stamp = ros::Time(timestamp);
    geometry_msgs::Point position;
    position.x = ::atof(words[1].c_str());
    position.y = ::atof(words[2].c_str());
    position.z = 0.0;
    /* Write position to pose_msg */
    pose_msg.pose.pose.position.x = position.x;
    pose_msg.pose.pose.position.y = position.y;
    pose_msg.pose.pose.position.z = position.z;

    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, ::atof(words[3].c_str()));
    pose_msg.pose.pose.orientation.x = quaternion.x();
    pose_msg.pose.pose.orientation.y = quaternion.y();
    pose_msg.pose.pose.orientation.z = quaternion.z();
    pose_msg.pose.pose.orientation.w = quaternion.w();

    std::string odom_link = links["ODOM"];
    std::string robot_link = links["ROBOT"];

    pose_msg.header.frame_id = odom_link;
    pose_msg.child_frame_id  = robot_link;

    ///pose_msg.twist.linear  = ::atof(words[4].c_str())
    ///pose_msg.twist.angular = ::atof(words[5].c_str())

    if (!publish_corrected) {
        tf_odom_robot_msg.header.stamp = pose_msg.header.stamp;
        tf_odom_robot_msg.header.frame_id = odom_link;
        tf_odom_robot_msg.child_frame_id = robot_link;

        /* tf_odom_robot_msg.transform.translation = position; */
        tf_odom_robot_msg.transform.translation.x = position.x;
        tf_odom_robot_msg.transform.translation.y = position.y;
        tf_odom_robot_msg.transform.translation.z = position.z;

        tf_odom_robot_msg.transform.rotation.x  = quaternion.x();
        tf_odom_robot_msg.transform.rotation.y  = quaternion.y();
        tf_odom_robot_msg.transform.rotation.z  = quaternion.z();
        tf_odom_robot_msg.transform.rotation.w  = quaternion.w();

        tf2_msg.transforms.push_back(tf_odom_robot_msg);
    }
}

void Converter::fillUpLaserMessage(std::vector<std::string> &words) {
    laser_msg.header.frame_id = "base_link";

    laser_msg.angle_increment = std::stof(words[4].c_str());
    laser_msg.angle_min = std::stof(words[2].c_str());
    laser_msg.angle_max = std::stof(words[2].c_str()) + std::stof(words[3].c_str());

    laser_msg.range_min = 0;
    laser_msg.range_max = std::stof(words[5].c_str());

    std::vector<float> ranges;
    int num_range_readings = ::atoi(words[8].c_str());
    int last_range_reading = num_range_readings + 8;

    for (int i = 9; i < last_range_reading + 1; i++)
        ranges.push_back(std::stof(words[i].c_str()));

    laser_msg.ranges = ranges;

    // min-max angle fitting, Karto need
    float factor_angle_fitting = laser_msg.angle_increment / 2;
    while ((round((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1) != num_range_readings) {
        if ((round((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1) > num_range_readings)
            laser_msg.angle_min = laser_msg.angle_min + factor_angle_fitting;
        else
            laser_msg.angle_max = laser_msg.angle_max - factor_angle_fitting;

        factor_angle_fitting = factor_angle_fitting / 2;
    }

    ranges.clear();

    int num_emisison_readings = ::atoi(words[last_range_reading + 1].c_str());
    int last_emission_reading = num_emisison_readings + last_range_reading;

    for (int i = last_range_reading + 1; i < last_emission_reading + 1; i++)
        ranges.push_back(std::stof(words[i].c_str()));

    laser_msg.intensities = ranges;

    laser_msg.header.stamp = ros::Time(atof(words[last_emission_reading + 2].c_str())); /// float or double
}

void Converter::increment_stamp() {
    stamp = stamp + ros::Duration(1.0 / rate);
}
