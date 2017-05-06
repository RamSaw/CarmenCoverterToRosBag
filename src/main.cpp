//
// Created by mikhail on 05.05.17.
//

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "CommandLineParser.h"
#include "Converter.h"

#include "rosbag/bag.h"

int main(int argc, char** argv) {
    try {
        parameters parsed_parameters = parse_parameters(std::vector<std::string>(argv, argv + argc));

        /* Opening files */
        std::ifstream carmen_file;
        std::ofstream bag_file;

        /*
        bag_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        bag_file.open(parsed_parameters.out_filename, std::ios::binary);
        */

        Converter converter(argc, argv);

        converter.convert(parsed_parameters.in_filename, parsed_parameters.out_filename);
        ///ros::NodeHandle node_handle("CarmenConverterToRosBag");
    }
    catch (std::fstream::failure &fstream_error) {
        std::cerr << fstream_error.what() << " Error from reading/writing from/to file\n";
    }
    catch (std::runtime_error &re){
        std::cerr << re.what() << "."
                " -i - input file, -o - output file.\n";
    }
    catch (std::exception &except) {
        std::cerr << except.what();
    }
    return 0;
}