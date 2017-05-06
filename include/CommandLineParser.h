//
// Created by mikhail on 05.05.17.
//

#ifndef CARMENCOVERTERTOROSBAG_COMMANDLINEPARSER_H
#define CARMENCOVERTERTOROSBAG_COMMANDLINEPARSER_H

#include <vector>
#include <string>

struct parameters {
    std::string in_filename, out_filename;

    parameters(const std::string &in_filename, const std::string &out_filename) :
            in_filename(in_filename), out_filename(out_filename) {}
};

parameters parse_parameters(const std::vector<std::string> &input_parameters);

#endif //CARMENCOVERTERTOROSBAG_COMMANDLINEPARSER_H
