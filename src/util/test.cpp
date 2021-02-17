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
    }
    ~Hit() {
        col = 0;
        row = 0;
        tot = 0;
    }
    unsigned col;
    unsigned row;
    unsigned tot;
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
        uint32_t value1 = (data[data_block_idx_end] >> (63 - (n_over-1))) & mask_second;
        value = (value0 << n_over) | value1;
    } else {
        value = (data[data_block_idx_start] >> end_pos_rev) & mask;
    }

    start_pos += length;
    return value;
}


//std::vector<Event> decode_stream(const std::vector<uint64_t>& data, bool drop_tot = false) {
std::vector<Event> decode_stream(Stream& stream, bool drop_tot = false, bool do_compressed_hitmap = false) {
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

            if(qrow >= 196) {
                LOGGER(error)("Cannot handle precision ToT data, whoops!");
                throw std::runtime_error("Handling of HitOR and precision timing data not allowed");
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
	//rh::rd53b_reset(hw, fe);
    auto cfg = dynamic_cast<FrontEndCfg*>(fe.get());

    // lets enable a few pixels for digital injection
    fe->configure();
    wait(hw);
    hw->flushBuffer();
    hw->setCmdEnable(cfg->getTxChannel());
    hw->setRxEnable(cfg->getRxChannel());

    // pre-scan
    json pre_scan_cfg = {{"InjDigEn", 1},
                        {"Latency", 60},
                        {"EnChipId", 1},
                        {"DataEnEos", 1},
                        {"NumOfEventsInStream", 1},
                        {"DataEnBinaryRo", 0}, // drop ToT
                        {"DataEnRaw", 0}, // drop hit map compression (always 16-bit hit maps)
                        {"InjVcalHigh", 2000},
                        {"InjVcalMed", 200}
    };
    for(auto j: pre_scan_cfg.items()) {
        fe->writeNamedRegister(j.key(), j.value());
    }
    wait(hw);

    // disable all pixels
    rh::disable_pixels(fe);

    // enable specific pixels
    std::vector<std::pair<unsigned, unsigned>> pixel_addresses { {1,1}, {8, 2}};
    for(unsigned col = 0; col < Rd53b::n_Col; col++) {
        for(unsigned row = 0; row < Rd53b::n_Row; row++) {
            fe->setEn(col, row, 0);
            fe->setInjEn(col, row, 0);
            fe->setHitbus(col, row, 0);
        }
    }
    for(auto pix_address : pixel_addresses) {
        auto col = std::get<0>(pix_address);
        auto row = std::get<1>(pix_address);
        LOGGER(warn)("Enabling pix (col,row) = ({},{})", col, row);
        fe->setEn(col, row, 1);
        fe->setInjEn(col, row, 1);
        fe->setHitbus(col, row, 0);
    }
    fe->configurePixels();
    wait(hw);

    // enable cores
    std::array<uint16_t, 4> cores = {0x0, 0x0, 0x0, 0x0};
    rh::set_core_columns(fe, cores);
    wait(hw);
    cores[0] = 0x1;//0x1 | 0x4 | 0x10;
    //cores = {0xffff, 0xffff, 0xffff, 0xffff};
    rh::set_core_columns(fe, cores);
    wait(hw);

    // enable triggers
    hw->setCmdEnable(cfg->getTxChannel());
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
    fe->sendClear(0xf);
    wait(hw);
    hw->flushBuffer();
    wait(hw);
    hw->setTrigEnable(0x1);

    if(hw->getTrigEnable() == 0) {
        LOGGER(error)("Trigger is not enabled!");
        throw std::runtime_error("Trigger is not enabled but waiting for triggers!");
    }


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
        blocks.push_back(data);
    }

    std::map<unsigned, std::vector<Stream>> stream_map;
    std::map<unsigned, unsigned> stream_in_progress_status;
    std::map<unsigned, std::vector<uint64_t>> stream_in_progress;

    LOGGER(error)("Hard-coding the assumed LS-bits of Chip-Id to be equal to 3 (0b11)!");
    stream_in_progress[3];
    
    for(size_t block_num =  0; block_num < (data_vec.size()/2); block_num++) {
        auto data = blocks[block_num];
        uint8_t ns_bit = (data >> 63) & 0x1;
        uint8_t ch_id = (data >> 61) & 0x3;
        if(ns_bit == 1) {
            if(stream_in_progress.at(ch_id).size() > 0) {
                Stream st;
                st.chip_id = ch_id;
                st.blocks = stream_in_progress.at(ch_id);
                stream_map[ch_id].push_back(st);
                stream_in_progress.at(ch_id).clear();
            }
        }
        stream_in_progress.at(ch_id).push_back(data);
    }

    LOGGER(error)("Hard-coding the assumed LS-bits of Chip-Id to be equal to 3 (0b11)!");
    uint8_t chip_id = 3;
    std::vector<Event> events;
    unsigned n_hits_total = 0;
    for(size_t i = 0; i < stream_map[chip_id].size(); i++) {
        auto stream = stream_map[chip_id][i];
        events = decode_stream(stream, /*drop tot*/ false, /*do compressed hitmap*/ true);
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
                        LOGGER(info)("        Hit[{:02d}]: (col, row) = ({}, {}) -> ToT = {}", ihit, hits.at(ihit).col, hits.at(ihit).row, hits.at(ihit).tot);
                    } // ihit
                }
            } // event
        } // non-empty event
    } // i
    LOGGER(info)("-------------------------------------------------------------------");
    LOGGER(warn)("Total number of hits seen for chip-id {}: {}", chip_id, n_hits_total);

    

    return 0;
}
