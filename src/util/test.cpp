//std/stl
#include <iostream>
#include <experimental/filesystem>
#include <memory>  // unique_ptr
#include <string>
#include <vector>
#include <getopt.h>
namespace fs = std::experimental::filesystem;

//YARR
#include "logging.h"
#include "LoggingConfig.h"
#include "Bookkeeper.h"
#include "Rd53b.h"
#include "ScanHelper.h"
#include "SpecController.h"

//itkpix_dataflow
#include "rd53b_helpers.h"


#define LOGGER(x) spdlog::x

struct option longopts_t[] = {{"hw", required_argument, NULL, 'c'},
                              {"chip", required_argument, NULL, 'r'},
                              {"debug", no_argument, NULL, 'd'},
                              {"help", no_argument, NULL, 'h'},
                              {0, 0, 0, 0}};

void print_help() {
    std::cout << "=========================================================="
              << std::endl;
	std::cout << " ITkPix dataflow test" << std::endl;
    std::cout << std::endl;
    std::cout << " Usage: [CMD] [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << " Options:" << std::endl;
    std::cout << "   --hw       JSON configuration file for hw controller"
              << std::endl;
    std::cout << "   --chip     JSON connectivity configuration for RD53B"
              << std::endl;
    std::cout << "   -d|--debug turn on debug-level" << std::endl;
    std::cout << "   -h|--help  print this help message" << std::endl;
    std::cout << "=========================================================="
              << std::endl;
}



int main(int argc, char* argv[]) {
	std::string defaultLogPattern = "[%T:%e]%^[%=8l]:%$ %v";
	spdlog::set_pattern(defaultLogPattern);

    std::string chip_config_filename = "";
    std::string hw_config_filename = "";
	bool verbose = false;
    int c;
    while ((c = getopt_long(argc, argv, "c:dr:h", longopts_t, NULL)) != -1) {
        switch (c) {
            case 'c':
                hw_config_filename = optarg;
                break;
            case 'r':
                chip_config_filename = optarg;
                break;
            case 'd':
				verbose = true;
                break;
            case 'h':
                print_help();
                return 0;
                break;
            case '?':
            default:
				LOGGER(error)("Invalid command-line argument provided: {}", char(c));
                return 1;
        }  // switch
    }      // while

    // check the inputs
    fs::path hw_config_path(hw_config_filename);
    fs::path chip_config_path(chip_config_filename);
    if (!fs::exists(hw_config_path)) {
		LOGGER(error)("Provided HW config file (=\"{}\") does not exist!", hw_config_filename);
        return 1;
    }
    if (!fs::exists(chip_config_path)) {
		LOGGER(error)("Provided chip config file (=\"{}\") does not exist!", chip_config_filename);
        return 1;
    }

    namespace rh = rd53b::helpers;
    auto hw = rh::spec_init(hw_config_filename);
    auto fe = rh::rd53b_init(hw, chip_config_filename);

	rh::clear_tot_memories(hw, fe);
	rh::rd53b_reset(hw, fe);

    return 0;
}
