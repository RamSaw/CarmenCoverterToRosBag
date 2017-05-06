//
// Created by mikhail on 05.05.17.
//

#include <stdexcept>
#include "CommandLineParser.h"

parameters parse_parameters(const std::vector<std::string> &input_parameters) {
    std::string in_f, out_f;

    std::size_t i = 1;
    while (i < input_parameters.size()) {
        if ("-o" == input_parameters[i]) {
            if (i + 1 < input_parameters.size())
                out_f = input_parameters[i + 1]; /// ??? strcpy
            else
                throw std::runtime_error("Invalid arguments: output filename is missing");
            i++;
        }
        else if ("-i" == input_parameters[i]) {
            if (i + 1 < input_parameters.size())
                in_f = input_parameters[i + 1];
            else
                throw std::runtime_error("Invalid arguments: input filename is missing");
            i++;
        }
        else {
            throw std::runtime_error("Invalid arguments: unknown key");
        }
        i++;
    }

    return parameters(in_f, out_f);
}