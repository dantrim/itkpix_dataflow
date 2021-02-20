#include "rd53b_helpers.h"

// labremote
//#include "Logger.h"

// json
// using json = nlohmann::json; // this is picked up from YARR, which does it at
// global scope!!

// yarr
//#include "SpecController.h"
//#include "Rd53b.h"
#include "ScanHelper.h"  // openJsonFile, loadController

// std/stl
#include <array>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>  // setw
#include <vector>
namespace fs = std::experimental::filesystem;
#include <chrono>
#include <thread>  // this_thread
#include <tuple>   // pair


std::unique_ptr<SpecController> rd53b::helpers::spec_init(std::string config) {
    std::unique_ptr<HwController> hw;
    json hw_config;
    try {
        hw_config = ScanHelper::openJsonFile(config);
        hw = ScanHelper::loadController(hw_config);
        // hw =
        // std::make_unique<SpecController>(ScanHelper::loadController(hw_config));
    } catch (std::exception& e) {
        std::cout << "Failed to initialize SPEC, exception caught: "
                         << e.what() << std::endl;
        return nullptr;
    }

    //hw->runMode();
    hw->setupMode();
    hw->setTrigEnable(0);
    hw->disableRx();
    HwController* p = hw.release();
    return std::unique_ptr<SpecController>(dynamic_cast<SpecController*>(p));
}

bool rd53b::helpers::spec_init_trigger(std::unique_ptr<SpecController>& hw,
                                       json trigger_config) {
    uint32_t m_trigDelay = trigger_config.at("delay");
    float m_trigTime = trigger_config.at("time");
    float m_trigFreq = trigger_config.at("frequency");
    uint32_t m_trigWordLength = 32;
    bool m_noInject = trigger_config.at("noInject");
    bool m_edgeMode = trigger_config.at("edgeMode");
    bool m_extTrig = trigger_config.at("extTrigger");
    uint32_t m_edgeDuration = trigger_config.at("edgeDuration");
    uint32_t m_pulseDuration = 8;
    uint32_t m_trigMultiplier = 16;
    std::array<uint32_t, 32> m_trigWord;

    ////////////////////////////////////////////////////////////////
    // SET TRIGGER DELAY
    ////////////////////////////////////////////////////////////////
    m_trigWord.fill(0xaaaaaaaa);
    std::array<uint16_t, 3> calWords = Rd53b::genCal(16, 0, 0, 1, 0, 0);
    m_trigWord[31] = 0xaaaa0000 | calWords[0];
    m_trigWord[30] = ((uint32_t)calWords[1] << 16) | calWords[2];

    uint64_t trigStream = 0;

    uint64_t one = 1;
    for (unsigned i = 0; i < m_trigMultiplier; i++) {
        trigStream |= (one << i);
    }  // i
    trigStream = trigStream << m_trigDelay % 8;

    for (unsigned i = 0; i < (m_trigMultiplier / 8) + 1; i++) {
        if (((30 - (m_trigDelay / 8) - i) > 2) && m_trigDelay > 30) {
            uint32_t bc1 = (trigStream >> (2 * i * 4)) & 0xf;
            uint32_t bc2 = (trigStream >> ((2 * i * 4) + 4)) & 0xf;
            m_trigWord[30 - (m_trigDelay / 8) - i] =
                ((uint32_t)Rd53b::genTrigger(bc1, 2 * i)[0] << 16) |
                Rd53b::genTrigger(bc2, (2 * i) + 1)[0];
        } else {
            std::cout << "Delay is either too small or too large!" << std::endl;
        }
    }  // i

    // rearm
    std::array<uint16_t, 3> armWords = Rd53b::genCal(16, 1, 0, 0, 0, 0);
    m_trigWord[1] = 0xaaaa0000 | armWords[0];
    m_trigWord[0] = ((uint32_t)armWords[1] << 16) | armWords[2];

    ////////////////////////////////////////////////////////////////
    // SET EDGE MODE
    ////////////////////////////////////////////////////////////////
    calWords = Rd53b::genCal(16, 1, 0, m_edgeDuration, 0, 0);
    m_trigWord[31] = 0xaaaa0000 | calWords[0];
    m_trigWord[30] = ((uint32_t)calWords[1] << 16) | calWords[2];

    ////////////////////////////////////////////////////////////////
    // SET TRIGGER MODE
    ////////////////////////////////////////////////////////////////
    hw->setTrigConfig(INT_COUNT);
    hw->setTrigCnt(trigger_config.at("count"));

    ////////////////////////////////////////////////////////////////
    // REMAINING
    ////////////////////////////////////////////////////////////////
    hw->setTrigFreq(m_trigFreq);
    hw->setTrigWord(&m_trigWord[0], 32);
    hw->setTrigWordLength(m_trigWordLength);
    hw->setTrigTime(m_trigTime);
    return true;
}

bool rd53b::helpers::spec_trigger_loop(std::unique_ptr<SpecController>& hw) {
    while (!hw->isCmdEmpty()) {
    }
    hw->flushBuffer();
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    hw->setTrigEnable(0x1);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (!hw->isTrigDone()) {
    }
    hw->setTrigEnable(0x0);
    return true;
}

std::unique_ptr<Rd53b> rd53b::helpers::rd53b_init(
    std::unique_ptr<SpecController>& hw, std::string config) {
    std::unique_ptr<Rd53b> fe = std::make_unique<Rd53b>(&*hw);
    auto cfg = dynamic_cast<FrontEndCfg*>(fe.get());

    fs::path chip_config(config);
    json json_config;
    if (fs::exists(chip_config)) {
        try {
            json_config = ScanHelper::openJsonFile(config);
        } catch (std::exception& e) {
			std::cout
                << "Failed to load Rd53b chip config, exception caught: "
                << e.what() << std::endl;
            return nullptr;
        }

        auto chip_configs = json_config["chips"];
        if (chip_configs.size() != 1) {
            std::cout << "Can only load chip configuration for a single "
                                "front-end, the provided configuration"
                             << " (\"" << config << "\") has "
                             << chip_configs.size() << " chips" << std::endl;
            return nullptr;
        }
        // auto chip_config = json_config["chips"]["config"];
        auto chip_config = chip_configs.at(0);
        fe->init(&*hw, chip_config["tx"], chip_config["rx"]);
        auto chip_register_file_path = chip_config["config"];
        auto chip_register_json =
            ScanHelper::openJsonFile(chip_register_file_path);
        cfg->fromFileJson(chip_register_json);
    } else {
        std::cout << "WARNING: "  << "Creating new Rd53b configuration file";
        std::ofstream new_cfg_file(config);
        fe->toFileJson(json_config);
        new_cfg_file << std::setw(4) << json_config;
        new_cfg_file.close();
    }

    std::cout << "Initialized RD53b with TX/RX = " << cfg->getTxChannel()
                    << "/" << cfg->getRxChannel() << std::endl;

    return fe;
}

void rd53b::helpers::configure_init(std::unique_ptr<SpecController>& hw, std::unique_ptr<Rd53b>& fe) {
    ////logger->debug("Initiliasing chip ...");

    // TODO this should only be done once per TX!
    // Send low number of transitions for at least 10us to put chip in reset state

    //logger->debug(" ... asserting CMD reset via low activity");
    //for (unsigned int i=0; i<400; i++) {
    //    // Pattern corresponds to approx. 0.83MHz
    //    core->writeFifo(0xFFFFFFFF);
    //    core->writeFifo(0xFFFFFFFF);
    //    core->writeFifo(0xFFFFFFFF);
    //    core->writeFifo(0x00000000);
    //    core->writeFifo(0x00000000);
    //    core->writeFifo(0x00000000);
    //}
    //core->releaseFifo();
    //while(!core->isCmdEmpty());

    // Wait for at least 1000us before chip is release from reset
    //logger->debug(" ... waiting for CMD reset to be released");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Sync CMD decoder
    //logger->debug(" ... send syncs");
    for(unsigned int i=0; i<32; i++)
        hw->writeFifo(0x817E817E);
    hw->releaseFifo();
    while(!hw->isCmdEmpty());

    // Enable register writing to do more resetting
    //logger->debug(" ... set global register in writeable mode");
    fe->writeRegister(&Rd53b::GcrDefaultConfig, 0xAC75);
    fe->writeRegister(&Rd53b::GcrDefaultConfigB, 0x538A);
    while(!hw->isCmdEmpty());

    // Send a global pulse to reset multiple things
    //logger->debug(" ... send resets via global pulse");
    fe->writeRegister(&Rd53b::GlobalPulseConf, 0x0FFF);
    fe->writeRegister(&Rd53b::GlobalPulseWidth, 10);
    while(!hw->isCmdEmpty());
    fe->sendGlobalPulse(fe->getChipId());
    while(!hw->isCmdEmpty());
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    // Reset register
    fe->writeRegister(&Rd53b::GlobalPulseConf, 0);
}

void rd53b::helpers::configure_global(std::unique_ptr<SpecController>& hw, std::unique_ptr<Rd53b>& fe) {
    //logger->debug("Configuring all registers ...");
    fe->configureGlobal();
    //unsigned numRegs = 138;
    //for (unsigned addr=0; addr<numRegs; addr++) {
    //    fe->sendWrReg(fe->getChipId(), addr, m_cfg[addr]);

    //    // Special handling of preamp register
    //    if (addr == 13) // specifically wait after setting preamp bias
    //        while(!hw->isCmdEmpty()){;}
    //        std::this_thread::sleep_for(std::chrono::microseconds(100));

    //    if (addr % 20 == 0) // Wait every 20 regs to not overflow a buffer
    //        while(!hw->isCmdEmpty()){;}
    //}
    //while(!hw->isCmdEmpty());
}

void rd53b::helpers::configure_pixels(std::unique_ptr<SpecController>& hw, std::unique_ptr<Rd53b>& fe) {
    fe->configurePixels();
    //// Setup pixel programming
    //fe->writeRegister(&Rd53b::PixAutoRow, 1);
    //fe->writeRegister(&Rd53b::PixBroadcast, 0);
    //// Writing two columns and six rows at the same time
    //for (unsigned dc=0; dc<n_DC; dc++) {
    //    fe->writeRegister(&Rd53b::PixRegionCol, dc);
    //    fe->writeRegister(&Rd53b::PixRegionRow, 0);
    //    for (unsigned row=0; row<n_Row; row++) {
    //        fe->writeRegister(&Rd53b::PixPortal, pixRegs[dc][row]);
    //        if (row%32==0)
    //            while(!hw->isCmdEmpty()){;}
    //    }
    //    while(!hw->isCmdEmpty()){;}
    //}


}

void rd53b::helpers::rd53b_configure(std::unique_ptr<SpecController>& hw, std::unique_ptr<Rd53b>& fe) {

    rd53b::helpers::configure_init(hw, fe);
    rd53b::helpers::configure_global(hw, fe);
    rd53b::helpers::configure_pixels(hw, fe);

}

bool rd53b::helpers::rd53b_reset(std::unique_ptr<SpecController>& hw,
                                 std::unique_ptr<Rd53b>& fe) {
    std::cout << "Resetting RD53B..." << std::endl;
    auto fe_cfg = dynamic_cast<FrontEndCfg*>(fe.get());
    hw->setCmdEnable(fe_cfg->getTxChannel());

    for (unsigned int i = 0; i < 800; i++) {
        hw->writeFifo(0xffffffff);
        hw->writeFifo(0xffffffff);
        hw->writeFifo(0xffffffff);
        hw->writeFifo(0x00000000);
        hw->writeFifo(0x00000000);
        hw->writeFifo(0x00000000);
    }  // i
    hw->releaseFifo();
    while (!hw->isCmdEmpty()) {
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // sync cmd decoder
    for (unsigned int i = 0; i < 32; i++) {
        // hw->writeFifo(0x817eaaaa);
        hw->writeFifo(0x817e817e);
    }  // i
    hw->releaseFifo();
    while (!hw->isCmdEmpty()) {
    }

    return true;
}

bool rd53b::helpers::disable_pixels(std::unique_ptr<Rd53b>& fe) {
    std::cout << "Disabling all pixels..." << std::endl;
    for (unsigned col = 0; col < Rd53b::n_Col; col++) {
        for (unsigned row = 0; row < Rd53b::n_Row; row++) {
            fe->setEn(col, row, 0);
            fe->setInjEn(col, row, 0);
            fe->setHitbus(col, row, 0);
        }  // row
    }      // col
    fe->configurePixels();
}

void rd53b::helpers::set_core_columns(std::unique_ptr<Rd53b>& fe,
                                      std::array<uint16_t, 4> cores) {
    fe->writeRegister(&Rd53b::EnCoreCol0, cores[0]);
    fe->writeRegister(&Rd53b::EnCoreCol1, cores[1]);
    fe->writeRegister(&Rd53b::EnCoreCol2, cores[2]);
    fe->writeRegister(&Rd53b::EnCoreCol3, cores[3]);
    fe->writeRegister(&Rd53b::RstCoreCol0, cores[0]);
    fe->writeRegister(&Rd53b::RstCoreCol1, cores[1]);
    fe->writeRegister(&Rd53b::RstCoreCol2, cores[2]);
    fe->writeRegister(&Rd53b::RstCoreCol3, cores[3]);
    fe->writeRegister(&Rd53b::EnCoreColCal0, cores[0]);
    fe->writeRegister(&Rd53b::EnCoreColCal1, cores[1]);
    fe->writeRegister(&Rd53b::EnCoreColCal2, cores[2]);
    fe->writeRegister(&Rd53b::EnCoreColCal3, cores[3]);
    fe->writeRegister(&Rd53b::HitOrMask0,~cores[0]);
    fe->writeRegister(&Rd53b::HitOrMask1,~cores[1]);
    fe->writeRegister(&Rd53b::HitOrMask2,~cores[2]);
    fe->writeRegister(&Rd53b::HitOrMask3,~cores[3]);
}

bool rd53b::helpers::clear_tot_memories(std::unique_ptr<SpecController>& hw,
                                        std::unique_ptr<Rd53b>& fe,
                                        float pixel_fraction) {
    std::cout << "Clearing ToT memories..." << std::endl;
    auto cfg = dynamic_cast<FrontEndCfg*>(fe.get());
    hw->setCmdEnable(cfg->getTxChannel());
    hw->setTrigEnable(0x0);  // disable

    fe->configure();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    while (!hw->isCmdEmpty()) {
    }

    hw->flushBuffer();
    hw->setCmdEnable(cfg->getTxChannel());
    hw->setRxEnable(cfg->getRxChannel());

    hw->runMode();

    /////////////////////////////////
    // pre-scan
    /////////////////////////////////
    json pre_scan_cfg = {{"InjDigEn", 1}, {"Latency", 500}};
    hw->setCmdEnable(cfg->getTxChannel());
    for (auto j : pre_scan_cfg.items()) {
        fe->writeNamedRegister(j.key(), j.value());
    }
    while (!hw->isCmdEmpty()) {
    }

    // disable pixels
    rd53b::helpers::disable_pixels(fe);

    // mask loop
    std::vector<std::pair<unsigned, unsigned>> modPixels;
    auto apply_mask = [](unsigned column, unsigned row) {
        unsigned int core_row = row / 8;
        unsigned serial =
            (core_row * 64) + ((column + (core_row % 8)) % 8) * 8 + row % 8;
        int max = 1;
        if ((serial % max) == 0) return true;
        return false;
    };
    unsigned n_pix_enabled = 0;
    int total_n_pixels = Rd53b::n_Col * Rd53b::n_Row;
    for (unsigned col = 0; col < Rd53b::n_Col; col++) {
        for (unsigned row = 0; row < Rd53b::n_Row; row++) {
            // float frac_enabled = static_cast<float>(n_pix_enabled) /
            // static_cast<float>(total_n_pixels); frac_enabled *= 100.; int
            // enable = (frac_enabled >= pixel_fraction) ? 0 : 1; fe->setEn(col,
            // row, enable); fe->setInjEn(col, row, enable); fe->setHitbus(col,
            // row, enable); modPixels.push_back(std::make_pair(col, row));
            // if(enable == 1u
            //{
            //    n_pix_enabled++;
            //}
            if (fe->getInjEn(col, row) == 1) {
                fe->setEn(col, row, 0);
                fe->setInjEn(col, row, 0);
                fe->setHitbus(col, row, 0);
                modPixels.push_back(std::make_pair(col, row));
            }
            if (apply_mask(col, row)) {
                fe->setEn(col, row, 1);
                fe->setInjEn(col, row, 1);
                fe->setHitbus(col, row, 1);
                modPixels.push_back(std::make_pair(col, row));
                n_pix_enabled++;
            }
        }  // row
    }      // col
    ////logger(logDEBUG) << "Enabling " << n_pix_enabled
    //                 << " pixels in pixel mask loop"
    //                 << " (" << std::fixed << std::setprecision(2)
    //                 << 100 * (static_cast<float>(n_pix_enabled) /
    //                           static_cast<float>(total_n_pixels))
    //                 << " %)";
    fe->configurePixels();
    while (!hw->isCmdEmpty()) {
    }

    // core column loop
    std::array<uint16_t, 4> cores = {0x0, 0x0, 0x0, 0x0};
    set_core_columns(fe, cores);
    while (!hw->isCmdEmpty()) {
    }

    unsigned int m_minCore = 0;
    unsigned int m_maxCore = 50;
    unsigned int m_nSteps = 50;
    unsigned int coreStep = 1;
    const uint32_t one = 0x1;

    json trig_config = {{"trigMultiplier", 16},
                        {"count", 1000},
                        {"delay", 56},
                        {"extTrigger", false},
                        {"frequency", 800000},
                        {"noInject", false},
                        {"time", 0},
                        {"edgeMode", true},
                        {"edgeDuration", 2}};

    // begin scan
    for (unsigned int m_cur = m_minCore; m_cur < m_maxCore; m_cur += coreStep) {
        cores = {0x0, 0x0, 0x0, 0x0};
        for (unsigned int i = m_minCore; i < m_maxCore; i += coreStep) {
            if (i % m_nSteps == m_cur) {
                cores[i / 16] |= one << i % 16;
            }
        }  // i
        hw->setCmdEnable(cfg->getTxChannel());
        set_core_columns(fe, cores);
        while (!hw->isCmdEmpty()) {
        }

        spec_init_trigger(hw, trig_config);
        while (!hw->isCmdEmpty()) {
        }
        spec_trigger_loop(hw);
    }  // m_cur

    hw->disableCmd();
    hw->disableRx();

    return true;
}


