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

    std::string RAWLASER1_topic;
    std::string RAWLASER2_topic;
    std::string RAWLASER3_topic;
    std::string RAWLASER4_topic;

    std::string FLASER_topic;
    std::string RLASER_topic;
    std::string LASER3_topic;
    std::string LASER4_topic;

    std::string ROBOTLASER1_topic;
    std::string ROBOTLASER2_topic;

    std::string NMEAGGA_topic;
    std::string NMEARMC_topic;

    std::string ODOM_topic;
    std::string TRUEPOS_topic;

    std::string tf_topic;

    std::string robot_link;
    std::string odom_link;
    std::string odom_robot_link;
    std::string true_odom_link;
    std::string ROBOTLASER1_link;
    std::string ROBOTLASER2_link;

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

        while (pause)
            ros::Duration(1 / rate).sleep();

        if (std::find(LASER_MESSAGE_DEFINED.begin(), LASER_MESSAGE_DEFINED.end(), words[0])
            != LASER_MESSAGE_DEFINED.end()) {

            std::cout << "Converting new NEW LASER message" << std::endl;
            fillUpLaserMessage(words);

            std::string topic = topics[words[0]];
            bag.write(topic, laser_msg.header.stamp, laser_msg);

            laser_msg.header.seq = laser_msg.header.seq + 1;
        }
        else if (std::find(OLD_LASER_MESSAGE_DEFINED.begin(), OLD_LASER_MESSAGE_DEFINED.end(), words[0])
                 != OLD_LASER_MESSAGE_DEFINED.end()) {

            std::cout << "Converting new OLD LASER message" << std::endl;
            fillUpOldLaserMessage(words);

            std::string topic = topics[words[0]];

            if (laser_msg.header.stamp.sec != 0) {
                bag.write(topic, laser_msg.header.stamp, laser_msg);

                /* Store TF in laser */
                if (publish_corrected) {
                    topic = topics["TF"];
                    bag.write(topic, laser_msg.header.stamp,  tf2_msg);

                }
                laser_msg.header.seq = laser_msg.header.seq + 1;
            }
        }
        else if (std::find(ROBOT_LASER_MESSAGE_DEFINED.begin(), ROBOT_LASER_MESSAGE_DEFINED.end(), words[0])
                 != ROBOT_LASER_MESSAGE_DEFINED.end()) {

            std::cout << "Converting new ROBOT LASER message" << std::endl;
            fillUpRobotLaserMessage(words);

            std::string topic = topics[words[0]];
            bag.write(topic, laser_msg.header.stamp, laser_msg);

            if (publish_corrected) {
                topic = topics["TF"];
                bag.write(topic, laser_msg.header.stamp, tf2_msg);

                topic = topics["ODOM"];
                bag.write(topic, pose_msg.header.stamp, pose_msg);

                pose_msg.header.seq = pose_msg.header.seq + 1;
                tf_laser_robot_msg.header.seq = tf_laser_robot_msg.header.seq + 1;
                tf_odom_robot_msg.header.seq = tf_odom_robot_msg.header.seq + 1;
            }
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
        else if (TRUEPOS_DEFINED == words[0]) {
            fillUpTruePoseMessage(words);

            std::string topic = topics[words[0]];
            bag.write(topic, true_pose_msg.header.stamp, true_pose_msg);

            topic = topics["TF"];
            bag.write(topic, true_pose_msg.header.stamp, tf2_msg);

            pose_msg.header.seq = true_pose_msg.header.seq + 1;
        }
        else if (PARAM_DEFINED == words[0])
            parse_robot_param(words);
        else {
            if (std::find(unknown_entries.begin(), unknown_entries.end(), words[0]) == unknown_entries.end()) {
                unknown_entries.push_back(words[0]);
                if (words[0] != "#")
                    std::cerr << "Uknown entry " << words[0];
            }

        }
        increment_stamp();
        tf2_msg = tf::tfMessage();
    }

    bag.close();

    std::cout << "Converted!" << std::endl;
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
            laser_msg.angle_max = laser_msg.angle_max - factor_angle_fitting; /// +

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

void Converter::fillUpOldLaserMessage(std::vector<std::string> &words) {
    laser_msg.header.frame_id = "base_link";

    std::vector<float> ranges;

    int num_range_readings = ::atoi(words[1].c_str());
    int last_range_reading = num_range_readings + 1;
    for (int i = 2; i < last_range_reading + 1; i++)
        ranges.push_back(std::stof(words[i].c_str()));

    std::string laser_id = Params::laserId2ParamPrefix(words[0]);

    /* Set to Default better, test file converts incorrectly on params */
    /* Get FoV */
    float ang_range;
    if (params["laser"][laser_id + "_fov"].c_str() == "")
        ang_range = 180;
    else
        ang_range = radians(std::stof(params["laser"][laser_id + "_fov"].c_str()));

    /* Get angular resolution */
    float ang_res;
    if (params["laser"][laser_id + "_resolution"] == "")
        ang_res = radians(ang_range) / num_range_readings;
    else
        ang_res = radians(std::stof(params["laser"][laser_id + "_resolution"].c_str()));

    /* Get max reading */
    float max_reading;
    if (params["robot"][laser_id + "_max"] == "")
        max_reading = 20;
    else
        max_reading = std::stof(params["robot"][laser_id + "_max"].c_str());

    /* Init laser msg */
    float ang_min = radians(-ang_range / 2);
    float ang_max = radians(ang_range / 2) - ang_res;

    laser_msg.angle_min = ang_min;
    laser_msg.angle_max = ang_max;
    laser_msg.angle_increment = ang_res;

    laser_msg.range_min = 0;
    laser_msg.range_max = max_reading;

    laser_msg.ranges = ranges;
    laser_msg.header.stamp = ros::Time(atof(words[last_range_reading + 7].c_str()));

    std::string robot_link = links["ROBOT"];
    std::string odom_link  = links["ROBOTODOM"];

    // tf needs to be publish a little bit in the future
    tf_odom_robot_msg.header.stamp = laser_msg.header.stamp + ros::Duration(0.01);

    geometry_msgs::Point position;
    position.x = atof(words[last_range_reading + 4].c_str());
    position.y = atof(words[last_range_reading + 5].c_str());
    position.z = 0;

    tf::Quaternion quaternion;
    quaternion.setEuler(0, 0, atof(words[last_range_reading + 6].c_str()));

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

void Converter::fillUpRobotLaserMessage(std::vector<std::string> &words) {
    std::string laser_link = links[words[0]];
    std::string robot_link = links["ROBOT"];
    std::string odom_link  = links["ROBOTODOM"];

    laser_msg.header.frame_id = robot_link; //laser_link

    laser_msg.angle_increment = std::stof(words[4].c_str());
    laser_msg.angle_min = std::stof((words[2].c_str())) + laser_msg.angle_increment / 2;
    laser_msg.angle_max = std::stof(words[2].c_str()) + std::stof(words[3].c_str()) - laser_msg.angle_increment / 2;

    laser_msg.range_min = 0;
    laser_msg.range_max = std::stof(words[5].c_str());

    std::vector<float> ranges;
    int num_range_readings = atoi(words[8].c_str());
    int last_range_reading = num_range_readings + 8;

    for (int i = 9; i < last_range_reading + 1; i++)
        ranges.push_back(std::stof(words[i].c_str()));

    laser_msg.ranges = ranges;

    /* min-max angle fitting, Karto need */
    float factor_angle_fitting = laser_msg.angle_increment / 2;
    while ((round((laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment) + 1) != num_range_readings) {
        if ((round((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1) > num_range_readings)
           laser_msg.angle_min = laser_msg.angle_min + factor_angle_fitting;
        else
            laser_msg.angle_max = laser_msg.angle_max + factor_angle_fitting;
        factor_angle_fitting = factor_angle_fitting / 2;
    }

    laser_msg.header.stamp = ros::Time(atof(words[last_range_reading + 13].c_str()));

    geometry_msgs::Point position;
    position.x = atof(words[last_range_reading + 2].c_str());
    position.y = atof(words[last_range_reading + 3].c_str());
    position.z = 0;

    tf::Quaternion quaternion;
    quaternion.setEuler(0, 0, atof(words[last_range_reading + 4].c_str()));

    tf_laser_robot_msg.header.stamp = laser_msg.header.stamp;

    tf_laser_robot_msg.header.frame_id = robot_link;
    tf_laser_robot_msg.child_frame_id = laser_link;

    /* tf_laser_robot_msg.transform.translation = position */
    tf_laser_robot_msg.transform.translation.x = position.x;
    tf_laser_robot_msg.transform.translation.y = position.y;
    tf_laser_robot_msg.transform.translation.z = position.z;

    tf_laser_robot_msg.transform.rotation.x  = quaternion.x();
    tf_laser_robot_msg.transform.rotation.y  = quaternion.y();
    tf_laser_robot_msg.transform.rotation.z  = quaternion.z();
    tf_laser_robot_msg.transform.rotation.w  = quaternion.w();

    /*
     * This transform is actually odom->laser_link
     * need to compute base_link->laser_link
     * tf2_msg.transforms.append(tf_laser_robot_msg)
     */
    position.x = atof(words[last_range_reading + 5].c_str());
    position.y = atof(words[last_range_reading + 6].c_str());
    position.z = 0;

    quaternion.setEuler(0, 0, atof(words[last_range_reading + 7].c_str()));

    /* tf needs to be publish a little bit in the future */
    tf_odom_robot_msg.header.stamp = laser_msg.header.stamp + ros::Duration(0.01);

    tf_odom_robot_msg.header.frame_id = odom_link;
    tf_odom_robot_msg.child_frame_id = robot_link;

    /* tf_odom_robot_msg.transform.translation = position */
    tf_odom_robot_msg.transform.translation.x = position.x;
    tf_odom_robot_msg.transform.translation.y = position.y;
    tf_odom_robot_msg.transform.translation.z = position.z;

    tf_odom_robot_msg.transform.rotation.x  = quaternion.x();
    tf_odom_robot_msg.transform.rotation.y  = quaternion.y();
    tf_odom_robot_msg.transform.rotation.z  = quaternion.z();
    tf_odom_robot_msg.transform.rotation.w  = quaternion.w();

    tf2_msg.transforms.push_back(tf_odom_robot_msg);

    pose_msg.header.stamp = laser_msg.header.stamp;

    /* pose_msg.pose.pose.position = position; */
    pose_msg.pose.pose.position = position;

    pose_msg.pose.pose.orientation.x = quaternion.x();
    pose_msg.pose.pose.orientation.y = quaternion.y();
    pose_msg.pose.pose.orientation.z = quaternion.z();
    pose_msg.pose.pose.orientation.w = quaternion.w();

    pose_msg.header.frame_id = odom_link;
    pose_msg.child_frame_id  = robot_link;
}

void Converter::fillUpOdomMessage(std::vector<std::string> &words) {
    double timestamp = ::atof(words[7].c_str()); /// double?
    if (timestamp < 0)
        return;

    pose_msg.header.stamp = ros::Time(timestamp);
    geometry_msgs::Point position;
    position.x = atof(words[1].c_str());
    position.y = atof(words[2].c_str());
    position.z = 0.0;
    /* Write position to pose_msg */
    pose_msg.pose.pose.position = position;

    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, atof(words[3].c_str()));
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

void Converter::fillUpTruePoseMessage(std::vector<std::string> &words) {
    true_pose_msg.header.stamp = stamp;

    geometry_msgs::Point position;
    position.x = atof(words[1].c_str());
    position.y = atof(words[2].c_str());
    position.z = 0.0;
    true_pose_msg.pose.pose.position = position;

    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, atof(words[3].c_str()));
    true_pose_msg.pose.pose.orientation.x = quaternion.x();
    true_pose_msg.pose.pose.orientation.y = quaternion.y();
    true_pose_msg.pose.pose.orientation.z = quaternion.z();
    true_pose_msg.pose.pose.orientation.w = quaternion.w();

    std::string odom_link  = links[words[0]];
    std::string robot_link = links["ROBOT"];

    pose_msg.header.frame_id = odom_link; /// or true_pose_msg?


    ///pose_msg.child_frame_id  = ?_link
    ///pose_msg.twist.linear  = float(words[4])
    ///pose_msg.twist.angular = float(words[5])

    tf_odom_robot_msg.header.stamp = true_pose_msg.header.stamp;
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

    ///tf2_msg.transforms[0] = tf_odom_robot_msg
    tf2_msg.transforms.push_back(tf_odom_robot_msg);
}

void Converter::parse_robot_param(std::vector<std::string> &words) {
    std::size_t i_s = words[1].find("_");
    std::string prefix(words[1].begin(), words[1].begin() + i_s);
    std::string actual_param(words[1].begin() + i_s + 1, words[1].end());

    if (params.find(prefix) == params.end()) {
        std::cerr << "Unknown param prefix '" + prefix + "' param '" + actual_param + "' ignored." << std::endl;
        return;
    }
    std::unordered_map<std::string, std::string>& params_by_prefix = params[prefix];

    if (params_by_prefix.find(actual_param) == params_by_prefix.end()) {
        std::cerr << "Unknown param " << actual_param << std::endl;
        return;
    }

    params_by_prefix[actual_param] = words[2];
    std::cout << "Param " << words[1] << " has value: " << words[2] << std::endl;
}

void Converter::increment_stamp() {
    stamp = stamp + ros::Duration(1.0 / rate);
}
