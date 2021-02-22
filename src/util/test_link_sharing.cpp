//std/stl
#include <iostream>
#include <experimental/filesystem>
#include <memory>  // unique_ptr
#include <string>
#include <vector>
#include <getopt.h>
#include <bitset>
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

const uint8_t PToT_maskStaging[4][4] = {
    {0, 1, 2, 3},
    {4, 5, 6, 7},
    {2, 3, 0, 1},
    {6, 7, 4, 5}};


#define LOGGER(x) spdlog::x

struct option longopts_t[] = {{"hw", required_argument, NULL, 'r'},
                              {"primary", required_argument, NULL, 'p'},
                              {"secondary", required_argument, NULL, 's'},
                              {"debug", no_argument, NULL, 'd'},
                              {"help", no_argument, NULL, 'h'},
                              {0, 0, 0, 0}};

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
    std::cout << "   -p|--primary    JSON configuration for PRIMARY chip" << std::endl;
    std::cout << "   -s|--secondary  JSON configuration for SECONDARY chip" << std::endl;
    std::cout << "   -d|--debug      turn on debug-level" << std::endl;
    std::cout << "   -h|--help       print this help message" << std::endl;
    std::cout << "=========================================================="
              << std::endl;
}

void wait(std::unique_ptr<SpecController>& hw) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    while(!hw->isCmdEmpty()) {}
}

struct Stream {
    Stream() {
        chip_id = 0;
        blocks.clear();
    }
    ~Stream() {
        chip_id = 0;
        blocks.clear();
    }
    uint8_t chip_id;
    std::vector<uint64_t> blocks;
};

struct Hit {

    Hit() {
        col =  0;
        row = 0;
        tot = 0;
        ptot = 0;
        ptoa = 0;
    }
    ~Hit() {
        col = 0;
        row = 0;
        tot = 0;
        ptot = 0;
        ptoa = 0;
    }
    unsigned col;
    unsigned row;
    unsigned tot;
    unsigned ptot;
    unsigned ptoa;
};

struct Event {
    Event() {
        tag = 0;
        hits.clear();
    }
    unsigned tag;
    std::vector<Hit> hits;
};

uint32_t retrieve(unsigned& start_pos, unsigned length, const std::vector<uint64_t>& data) {
    uint32_t value = 0;
    unsigned data_block_idx_start = (start_pos / 64);
    unsigned end_pos = start_pos + length;
    unsigned data_block_idx_end = (end_pos / 64);

    unsigned start_pos_rev = 64 - start_pos;
    unsigned end_pos_rev = 64 - end_pos;
    unsigned mask = (1 << length) - 1;

    if(data_block_idx_end != data_block_idx_start) {
        unsigned n_over = (end_pos % 64);
        unsigned length_first = length - n_over;
        unsigned mask_first = (1 << length_first) - 1;
        unsigned mask_second = (1 << n_over) - 1;

        uint32_t value0 = data[data_block_idx_start] & mask_first;
        uint32_t value1 = (data[data_block_idx_end] >> ((63-3) - (n_over-1))) & mask_second;
        value = (value0 << n_over) | value1;
    } else {
        value = (data[data_block_idx_start] >> end_pos_rev) & mask;
    }

    start_pos += length;
    return value;
}


//std::vector<Event> decode_stream(const std::vector<uint64_t>& data, bool drop_tot = false) {
std::vector<Event> decode_stream(Stream& stream, bool drop_tot = false, bool do_compressed_hitmap = false, bool use_ptot = false) {
    //std::vector<uint64_t> = data.blocks;
    std::vector<uint64_t> data = stream.blocks;
    unsigned pos = 0;
    uint8_t ns_bit = retrieve(pos, 1, data);
    uint8_t ch_id = retrieve(pos, 2, data);
    uint16_t tag = retrieve(pos, 8, data);
    //LOGGER(info)("Decode: NS = {}, ch_id = {}, tag = {}", ns_bit, ch_id, tag);

    // loop over all events in the stream
    std::vector<Event> events;
    Event current_event;
    current_event.tag = tag;
    unsigned n_hits = 0;
    while(true) {

        uint16_t ccol = retrieve(pos, 6, data);

        // if ccol is 0 this is the end of stream marker and nothing beyond it is valid data
        if(ccol == 0) {
            events.push_back(current_event);
            //LOGGER(warn)("HIT EOS MARKER!");
            break;
        }

        // valid ccol are < 56 (0b111000) and any ccol greater or equal to 56 indicates that
        // the next field is an internal tag, not a ccol!
        if(ccol >= 56) {
            events.push_back(current_event);
            tag = (ccol << 5) | retrieve(pos, 5, data); // get the rest of the 11-bit internal tag
            current_event.tag = tag;
            current_event.hits.clear();
            continue; // go back to the start of the hit loop
        }

        // loop over all hits in the event
        uint8_t qrow = 0;
        uint8_t is_last = 0;
        do {
            is_last = retrieve(pos, 1, data);
            uint8_t is_neighbor = retrieve(pos, 1, data);
            if(is_neighbor == 1) {
                qrow = qrow + 1;
            } else {
                qrow = retrieve(pos, 8, data);
            }


            uint32_t hitmap = retrieve(pos, 16, data);
            if(do_compressed_hitmap) {
                uint16_t hitmap_raw = (hitmap & 0xffff);
                hitmap = (RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFFFF);
                uint8_t hitmap_rollBack = ((RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFF000000) >> 24);
                if(hitmap_rollBack > 0) {
                    if(hitmap_rollBack != 0xff) {
                        pos = pos - hitmap_rollBack;
                    }
                    uint16_t rowMap = retrieve(pos, 14, data);
                    hitmap |= (RD53BDecoding::_LUT_BinaryTreeRowHMap[rowMap] << 8);
                    pos -= (RD53BDecoding::_LUT_BinaryTreeRowHMap[rowMap] & 0xFF00) >> 8;
                } else {
                    pos -= (RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFF0000) >> 16;
                }
                
            }

            if(qrow >= 196) {
                for(unsigned ibus = 0; ibus < 4; ibus++) {
                    uint8_t hitbus = (hitmap >> (ibus << 2)) & 0xf;
                    if(hitbus) {
                        uint16_t ptot_ptoa_buf = 0xffff;
                        for(unsigned iread = 0; iread < 4; iread++) {
                            if((hitbus >> iread) & 0x1) {
                                ptot_ptoa_buf &= ~((~retrieve(pos, 4, data) & 0xf) << (iread<<2));
                            }
                        } // iread
                        uint16_t ptot = ptot_ptoa_buf & 0x7ff;
                        uint16_t ptoa = ptot_ptoa_buf >> 11;
                        unsigned step = 0;
                        uint16_t pix_col = (ccol -1 ) * 8 + PToT_maskStaging[step % 4][ibus] + 1;
                        uint16_t pix_row = step / 2 + 1;
                        Hit hit;
                        hit.col = pix_col;
                        hit.row = pix_row;
                        hit.ptot = ptot;
                        hit.ptoa = ptoa;
                        n_hits++;
                        current_event.hits.push_back(hit);
                    } // if hitbus
                } // ibus
            } // qrow >= 196
            else if(!use_ptot){

                // not considering compressed hitmaps right now
                auto arr_size = RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] << 2;
                if(arr_size == 0) {
                    LOGGER(error)("Received fragment with no ToT! ({}, {})", ccol, qrow);
                    throw std::runtime_error("Decoding error");
                }
                uint64_t tot = drop_tot ? 0 : retrieve(pos, arr_size, data);
                for(unsigned ihit = 0; ihit < arr_size; ihit++) {
                    uint8_t pix_tot = (tot >> (ihit << 2)) & 0xf;
                    uint16_t pix_col = ((ccol - 1) * 8) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] >> 4) + 1;
                    uint16_t pix_row = ((qrow)*2) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] & 0xF) + 1;

                    // consider tot == 0 to be a "no hit"
                    if(pix_tot > 0) {
                        Hit hit;
                        hit.col = (pix_col-1);
                        hit.row = (pix_row-1);
                        hit.tot = pix_tot;
                        n_hits++;
                        current_event.hits.push_back(hit);
                    } // non-empty tot value
                } // ihit
            } 
        } while(!is_last);
        //events.push_back(current_event);
    } // event loop
    if(n_hits == 0) {
        events.clear();
    }
    return events;
}

int main(int argc, char* argv[]) {
	std::string defaultLogPattern = "[%T:%e]%^[%=8l]:%$ %v";
	spdlog::set_pattern(defaultLogPattern);

    std::string primary_config_filename = "";
    std::string secondary_config_filename = "";
    std::string hw_config_filename = "";
    bool use_ptot = false;
	bool verbose = false;
    int c;
    while ((c = getopt_long(argc, argv, "r:p:s:hd", longopts_t, NULL)) != -1) {
        switch (c) {
            case 'r':
                hw_config_filename = optarg;
                break;
            case 'p':
                primary_config_filename = optarg;
                break;
            case 's':
                secondary_config_filename = optarg;
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
    fs::path primary_config_path(primary_config_filename);
    fs::path secondary_config_path(secondary_config_filename);
    if (!fs::exists(hw_config_path)) {
		LOGGER(error)("Provided HW config file (=\"{}\") does not exist!", hw_config_filename);
        return 1;
    }
    if (!fs::exists(primary_config_path)) {
        LOGGER(error)("Provided config for PRIMARY (=\"{}\") does not exist!", primary_config_filename);
        return 1;
    }
    if (!fs::exists(secondary_config_path)) {
        LOGGER(error)("Provided config for SECONDARY (=\"{}\") does not exist!", secondary_config_filename);
        return 1;
    }

    namespace rh = rd53b::helpers;
    auto hw = rh::spec_init(hw_config_filename);
    auto fe_primary = rh::rd53b_init(hw, primary_config_filename);
    auto fe_secondary = rh::rd53b_init(hw, secondary_config_filename);

    // Sync CMD decoder
    hw->setCmdEnable(fe_primary->getTxChannel());
    hw->setRxEnable(fe_primary->getRxChannel());
    hw->runMode();
    unsigned temp_chip_id = fe_primary->getChipId();
    fe_primary->setChipId(16);
    fe_primary->configure();
    //rh::rd53b_configure(hw, fe_primary);
    fe_primary->writeNamedRegister("GpLvdsBias", 15);
    fe_primary->writeNamedRegister("GpLvdsEn", 15);
    fe_primary->writeNamedRegister("GpLvdsPad0", 0);
    fe_primary->writeNamedRegister("GpLvdsPad1", 0);
    fe_primary->writeNamedRegister("GpLvdsPad2", 0);
    fe_primary->writeNamedRegister("GpLvdsPad3", 0);
    rh::clear_tot_memories(hw, fe_primary);
    fe_primary->setChipId(temp_chip_id);
    //hw->setTrigEnable(0x0);
    //hw->runMode();
    //LOGGER(info)("Sending CMD syncs...");
    //for(unsigned int i=0; i<32; i++)
    //    hw->writeFifo(0x817E817E);
    //hw->releaseFifo();
    //wait(hw);

    //fe->setChipId(set_chip_id);

    //if(fe->getChipId() != set_chip_id) {
    //    LOGGER(error)("Chip-ID from chip JSON configuration (={}) does not equal the one specified by the user (={})!", fe->getChipId(), set_chip_id);
    //    throw std::runtime_error("Error in setting chip id!");
    //}

    auto cfg_primary = dynamic_cast<FrontEndCfg*>(fe_primary.get());
    auto cfg_secondary = dynamic_cast<FrontEndCfg*>(fe_secondary.get());

    // configure the primary
    LOGGER(info)("Configuring the PRIMARY");
    hw->setCmdEnable(cfg_primary->getTxChannel());
    //rh::rd53b_configure(hw, fe_primary);
    fe_primary->configureGlobal();
    rh::disable_pixels(fe_primary);
    wait(hw);

    // configure the secondary
    LOGGER(info)("Configuring the SECONDARY");
    hw->setCmdEnable(cfg_secondary->getTxChannel());
    //rh::rd53b_configure(hw, fe_secondary);
    fe_secondary->configureGlobal();
    wait(hw);


    // common scan-specific configuration
    LOGGER(info)("Configuring the pre-scan items");
    json pre_scan_cfg;
    pre_scan_cfg = {{"Latency", 60},
                        {"EnChipId", 1},
                        {"DataEnEos", 1},
                        {"NumOfEventsInStream", 1},
                        {"DataEnBinaryRo", 0}, // drop ToT
                        {"DataEnRaw", 0}, // drop hit map compression (always 16-bit hit maps)
                        {"InjVcalHigh", 2000},
                        {"InjVcalMed", 200}
    };
    write_config(pre_scan_cfg, fe_primary);
    fe_primary->writeNamedRegister("InjDigEn", 0);
    wait(hw);
    write_config(pre_scan_cfg, fe_secondary);
    fe_secondary->writeNamedRegister("InjDigEn", 1);
    wait(hw);

    // enable pixels for digital injection on SECONDARY
    LOGGER(info)("Enabling SECONDARY pixels for digital injection");
    std::vector<std::pair<unsigned, unsigned>> pixel_addresses {
        {0,0}
        ,{0,1}
    };
    for(auto pix_address : pixel_addresses) {
        auto col = std::get<0>(pix_address);
        auto row = std::get<1>(pix_address);
        LOGGER(warn)("Enabling pix (col,row) = ({},{})", col, row);
        fe_secondary->setEn(col, row, use_ptot ? 0 : 1);
        fe_secondary->setInjEn(col, row, 1);
        fe_secondary->setHitbus(col, row, use_ptot ? 1 : 0);
    } // pix_address
    fe_secondary->configurePixels();
    wait(hw);

    // configure the corresponding core columns
    std::array<uint16_t, 4> cores = {0x0, 0x0, 0x0, 0x0};
    set_cores(fe_secondary, cores, use_ptot);
    wait(hw);
    cores[0] = 0x1;
    set_cores(fe_secondary, cores, use_ptot);
    wait(hw);


    // setup the trigger
    hw->setTrigEnable(0x0);
    wait(hw);
    hw->flushBuffer();
    hw->setCmdEnable(cfg_primary->getTxChannel());
    hw->setRxEnable(cfg_primary->getRxChannel());
    hw->runMode();

    // enable triggers
    hw->setCmdEnable(cfg_primary->getTxChannel());
    json trigger_config =  {{"trigMultiplier", 16},
                            {"count", 50},
                            {"delay", 56},
                            {"extTrigger", false},
                            {"frequency", 5000},
                            {"noInject", false},
                            {"time", 0},
                            {"edgeMode", true},
                            {"edgeDuration", 20}};
    rh::spec_init_trigger(hw, trigger_config);
    wait(hw);

    // begin triggers
    hw->runMode();
    fe_primary->sendClear(fe_primary->getChipId());
    fe_secondary->sendClear(fe_secondary->getChipId());
    wait(hw);
    hw->flushBuffer();
    wait(hw);
    hw->setTrigEnable(0x1);

    if(hw->getTrigEnable() == 0) {
        LOGGER(error)("Trigger is not enabled!");
        throw std::runtime_error("Trigger is not enabled but waiting for triggers!");
    }

    // look for data only from the secondary
    unsigned set_chip_id = fe_secondary->getChipId();
    unsigned set_chip_id_ls = 0x3 & set_chip_id;

    uint32_t done = 0;
    RawData* data = nullptr;
    unsigned total_word_count_32 = 0;
    std::vector<uint32_t> data_vec;
    while(done == 0) {
        done = hw->isTrigDone();
        do {
            data = hw->readData();
            if(data != nullptr) {
                for(size_t i = 0; i < data->words; i++) {
                    data_vec.push_back(data->buf[i]);
                } // i
            }
        } while (data != nullptr);
    }
    std::this_thread::sleep_for(hw->getWaitTime());
    do {
        data = hw->readData();
        if(data != nullptr) {
            for(size_t i = 0; i < data->words; i++) {
                data_vec.push_back(data->buf[i]);
            } // i
        }
    } while (data != nullptr);

    // create streams of 64-bit words
    std::vector<uint64_t> blocks;
    for(size_t i =  0; i < data_vec.size(); i+=2) {
        uint64_t data0 = static_cast<uint64_t>(data_vec.at(i));
        uint64_t data1 = static_cast<uint64_t>(data_vec.at(i+1));
        uint64_t data = data1 | (data0 << 32);
        std::bitset<64> bits(data);
        //std::cout << "block: " << std::hex << bits.to_ulong() << std::endl; /// bits.to_string() << std::endl;
        std::cout << "block: " << bits.to_string() << std::endl;
        blocks.push_back(data);
    }

    std::map<unsigned, std::vector<Stream>> stream_map;
    std::map<unsigned, unsigned> stream_in_progress_status;
    std::map<unsigned, std::vector<uint64_t>> stream_in_progress;

    LOGGER(error)("Hard-coding the assumed LS-bits of Chip-Id to be equal to {}!", set_chip_id_ls);
    stream_in_progress[set_chip_id_ls];

    for(size_t block_num =  0; block_num < (data_vec.size()/2); block_num++) {
        auto data = blocks[block_num];
        uint8_t ns_bit = (data >> 63) & 0x1;
        uint8_t ch_id = (data >> 61) & 0x3;
        std::bitset<64> bits(data);
        if(ch_id == set_chip_id_ls) {
            LOGGER(info)("Data from CH ID {}: {}", set_chip_id_ls, bits.to_string());
            //LOGGER(info)("Data from CH ID {}: {:x}", set_chip_id_ls, bits.to_ulong());//to_string());
        }
        //LOGGER(warn)("Skipping data with CH.ID = {}", ch_id);
        if(ch_id != set_chip_id_ls) continue;
        if(ns_bit == 1) {
            if(stream_in_progress.at(ch_id).size() > 0) {
                Stream st;
                st.chip_id = ch_id;
                st.blocks = stream_in_progress.at(ch_id);
                stream_map[ch_id].push_back(st);
                stream_in_progress[ch_id].clear();
            }
        }
        stream_in_progress[ch_id].push_back(data);
    }

    LOGGER(error)("Hard-coding the assumed LS-bits of Chip-Id to be equal to {}!", set_chip_id_ls);
    uint8_t chip_id = set_chip_id_ls;
    std::vector<Event> events;
    unsigned n_hits_total = 0;
    for(size_t i = 0; i < stream_map[chip_id].size(); i++) {
        auto stream = stream_map[chip_id][i];
        events = decode_stream(stream, /*drop tot*/ false, /*do compressed hitmap*/ true, /*use_ptot*/ use_ptot);
        if(events.size()>0) {
            LOGGER(info)("-------------------------------------------------------------------");
            LOGGER(info)("Stream for Chip {} has {} events", stream.chip_id, events.size());
            for(auto event : events) {
                LOGGER(info)("   TAG: {}", event.tag);
                auto hits = event.hits;
                n_hits_total += hits.size();
                if(hits.size() == 0) {
                    LOGGER(info)("        EMPTY!");
                } else {
                    for(size_t ihit = 0; ihit < hits.size(); ihit++) {
                        LOGGER(info)("        Hit[{:02d}]: (col, row) = ({}, {}) -> ToT = {}, PToT = {}, PToA = {}", ihit, hits.at(ihit).col, hits.at(ihit).row, hits.at(ihit).tot, hits.at(ihit).ptot, hits.at(ihit).ptoa);
                    } // ihit
                }
            } // event
        } // non-empty event
    } // i
    LOGGER(info)("-------------------------------------------------------------------");
    LOGGER(warn)("Total number of hits seen for chip-id {}: {}", chip_id, n_hits_total);

    return 0;
}