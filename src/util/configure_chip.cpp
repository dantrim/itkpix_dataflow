//std/stl
#include <iostream>
#include <experimental/filesystem>
#include <memory>  // unique_ptr
#include <string>
#include <vector>
#include <getopt.h>
#include <bitset>
#include <iomanip>
namespace fs = std::experimental::filesystem;

//YARR
#include "logging.h"
#include "LoggingConfig.h"
#include "Bookkeeper.h"
#include "Rd53b.h"
#include "ScanHelper.h"
#include "SpecController.h"
#include "RawData.h"
#include "LUT_PlainHMapToColRow.h"
#include "LUT_BinaryTreeRowHMap.h"
#include "LUT_BinaryTreeHitMap.h"

//itkpix_dataflow
#include "rd53b_helpers.h"

#define LOGGER(x) spdlog::x

struct option longopts_t[] = {{"hw", required_argument, NULL, 'r'},
                              {"chip", required_argument, NULL, 'c'},
                              {"debug", no_argument, NULL, 'd'},
                              {"help", no_argument, NULL, 'h'},
                              {0, 0, 0, 0}};

void wait(std::unique_ptr<SpecController>& hw) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    while(!hw->isCmdEmpty()) {}
}

void send_reset(std::unique_ptr<SpecController>& hw, std::unique_ptr<Rd53b>& fe, unsigned signal) {
    LOGGER(info)("Sending reset signal to Chip {}: {:x}", fe->getChipId(), 0xffff & signal);
    fe->writeRegister(&Rd53b::GlobalPulseConf, signal);
    fe->writeRegister(&Rd53b::GlobalPulseWidth, 10);
    wait(hw);
    fe->sendGlobalPulse(fe->getChipId());
    wait(hw);
    fe->writeRegister(&Rd53b::GlobalPulseConf, 0);
}

void set_cores(std::unique_ptr<Rd53b>& fe, std::array<uint16_t, 4> cores, bool use_ptot = false) {
    namespace rh = rd53b::helpers;
    rh::set_core_columns(fe, cores);
    if(use_ptot) {
        fe->writeRegister(&Rd53b::PtotCoreColEn0, cores[0]);
        fe->writeRegister(&Rd53b::PtotCoreColEn1, cores[1]);
        fe->writeRegister(&Rd53b::PtotCoreColEn2, cores[2]);
        fe->writeRegister(&Rd53b::PtotCoreColEn3, cores[3]);
    }
}

void set_pixels_enable(std::unique_ptr<Rd53b>& fe, std::vector<std::pair<unsigned, unsigned>> pixel_addresses, bool use_ptot = false) {
    for(auto pix_address : pixel_addresses) {
        auto col = std::get<0>(pix_address);
        auto row = std::get<1>(pix_address);
        LOGGER(warn)("CHIP[{}]: Enabling pix (col,row) = ({},{})", fe->getChipId(), col, row);
        fe->setEn(col, row, use_ptot ? 0 : 1);
        fe->setInjEn(col, row, 1);
        fe->setHitbus(col, row, use_ptot ? 1 : 0);
    } // pix_address
    fe->configurePixels();
}


void write_config(const json& config, std::unique_ptr<Rd53b>& fe) {
    for(auto j: config.items()) {
        fe->writeNamedRegister(j.key(), j.value());
    }
}

void print_help() {
    std::cout << "=========================================================="
              << std::endl;
	std::cout << " ITkPix link sharing test" << std::endl;
    std::cout << std::endl;
    std::cout << " Usage: [CMD] [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << " Options:" << std::endl;
    std::cout << "   --hw            JSON configuration file for hw controller"
              << std::endl;
    std::cout << "   -c|--chip       JSON configuration for the chip" << std::endl;
    std::cout << "   -d|--debug      turn on debug-level" << std::endl;
    std::cout << "   -h|--help       print this help message" << std::endl;
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
    while ((c = getopt_long(argc, argv, "r:c:hd", longopts_t, NULL)) != -1) {
        switch (c) {
            case 'r':
                hw_config_filename = optarg;
                break;
            case 'c':
                chip_config_filename = optarg;
                break;
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
        LOGGER(error)("Provided chip config (=\"{}\") does not exist!", chip_config_filename);
        return 1;
    }

    namespace rh = rd53b::helpers;
    auto hw = rh::spec_init(hw_config_filename);
    auto fe = rh::rd53b_init(hw, chip_config_filename);

    

    // Sync CMD decoder
    hw->setCmdEnable(fe->getTxChannel());
    hw->setTrigEnable(0x0);
    hw->setRxEnable(fe->getRxChannel());
    hw->flushBuffer();
    hw->runMode();

    unsigned chip_id = fe->getChipId();
    //rh::rd53b_configure(hw, fe);

    fe->writeRegister(&Rd53b::GcrDefaultConfig, 0xAC75);
    fe->writeRegister(&Rd53b::GcrDefaultConfigB, 0x538A);
    wait(hw);

    LOGGER(info)("Writing global configuration to chip with ID = {}", fe->getChipId());
    fe->configureGlobal();
//    int result = fe->checkCom();
//    if(result != 1) {
//        LOGGER(error)("Could not confirm checkCom after writing global configuration");
//        return 1;
//    }
    LOGGER(info)("Done");
    return 0;
}
