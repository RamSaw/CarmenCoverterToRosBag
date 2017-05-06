//
// Created by mikhail on 05.05.17.
//

#include "Converter.h"

const std::vector<std::string> Converter::LASER_MESSAGE_DEFINED = {"RAWLASER1", "RAWLASER2", "RAWLASER3", "RAWLASER4"};
const std::vector<std::string> Converter::OLD_LASER_MESSAGE_DEFINED   = {"FLASER", "RLASER", "LASER3", "LASER4"};
const std::vector<std::string> Converter::ROBOT_LASER_MESSAGE_DEFINED = {"ROBOTLASER1", "ROBOTLASER2"};
const std::vector<std::string> Converter::GPS_MESSAGE_DEFINED = {"NMEAGGA", "NMEARMC"};
const std::string Converter::ODOM_DEFINED = "ODOM";
const std::string Converter::TRUEPOS_DEFINED = "TRUEPOS";
const std::string Converter::PARAM_DEFINED = "PARAM";
const std::string Converter::SYNC_DEFINED = "SYNC";