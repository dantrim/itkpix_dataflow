//std/stl
#include <iostream>
#include <experimental/filesystem>
#include <memory>  // unique_ptr
#include <string>
#include <vector>
#include <getopt.h>
#include <bitset>
#include <sstream>
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

const uint8_t PToT_maskStaging[4][4] = {
    {0, 1, 2, 3},
    {4, 5, 6, 7},
    {2, 3, 0, 1},
    {6, 7, 4, 5}};


#define LOGGER(x) spdlog::x

struct option longopts_t[] = {{"hw", required_argument, NULL, 'r'},
                              {"primary", required_argument, NULL, 'p'},
                              {"secondary", required_argument, NULL, 's'},
                              {"trigger", required_argument, NULL, 't'},
                              {"debug", no_argument, NULL, 'd'},
                              {"force", no_argument, NULL, 'f'},
                              {"no-decode", no_argument, NULL, 'x'},
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
    //wait(hw);
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
    std::cout << "   -p|--primary    JSON configuration for PRIMARY chip" << std::endl;
    std::cout << "   -s|--secondary  JSON configuration for SECONDARY chip" << std::endl;
    std::cout << "   -t|--trigger    JSON configuration for trigger [optional]" << std::endl;
    std::cout << "   -d|--debug      turn on debug-level" << std::endl;
    std::cout << "   -f|--force      do not configure the SerSelOut of any of the chips" << std::endl;
    std::cout << "   -h|--help       print this help message" << std::endl;
    std::cout << "=========================================================="
              << std::endl;
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

        uint64_t value0 = data[data_block_idx_start] & mask_first;
        uint64_t value1 = (data[data_block_idx_end] >> ((63-3) - (n_over-1))) & mask_second;

        uint64_t ns_bit = (value1 >> 63) & 0x1;
        uint64_t ch_id = (value1 >> 61) & 0x3;
        if(ns_bit == 1) {
            LOGGER(error)("Unexpected NS=1 seen for CH.ID = {}", ch_id);
            throw std::runtime_error("Unexpected NS bit seen");
        }
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
    if(ch_id == 2 && tag == 8) {
        LOGGER(error)("---------------------------------");
        LOGGER(error)("Decoding chip id 2 tag = {}", tag);
        unsigned idx = 0;
        for(auto db : data) {
            std::bitset<64> dbbits(db);
            LOGGER(error)("    [{}] {}", idx, dbbits.to_string());
            idx++;
        }
    }
    bool print_stuff = (tag == 8 && ch_id == 2);
    //LOGGER(error)("Decode: NS = {}, ch_id = {}, tag = {}", ns_bit, ch_id, tag);
    //size_t i = 0;
    //for(auto block : data) {
    //    std::bitset<64> d(block);
    //    LOGGER(error)("    [{}] {}", i, d.to_string()); 
    //    i += 1;
    //}

    // loop over all events in the stream
    std::vector<Event> events;
    Event current_event;
    current_event.tag = tag;
    unsigned n_hits = 0;
    while(true) {

        uint16_t ccol = retrieve(pos, 6, data);
        if(print_stuff) {
            std::bitset<6> ccol_bits(ccol);
            LOGGER(error)("CCOL = {}", ccol_bits.to_string());
        }

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

        uint8_t qrow = 0;
        uint8_t is_last = 0;
        uint8_t is_neighbor = 0;
        while(true) {
            is_last = retrieve(pos, 1, data);
            is_neighbor = retrieve(pos, 1, data);
            if(is_neighbor == 1) {
                qrow = qrow + 1;
            } else {
                qrow = retrieve(pos, 8, data);
            }

            uint32_t hitmap = retrieve(pos, 16, data);
            if(print_stuff) {
                std::bitset<8> qrow_bits(qrow);
                std::bitset<16> hitmap_bits(hitmap);
                LOGGER(error)("IS_LAST = {}", is_last);
                LOGGER(error)("IS_NEIGHBOR = {}", is_neighbor);
                LOGGER(error)("QROW = {}", qrow_bits.to_string());
                LOGGER(error)("HITMAP = {}", hitmap_bits.to_string());
            }
            if(qrow >= 196) {
                throw std::runtime_error("Invalid qrow observed");
            }

            if(RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] == 0) {
                LOGGER(error)("Received fragment with no ToT! ({}, {}) [chip id = {}, tag = {}], hitmap = {:x}, pos = {}", ccol, qrow, ch_id, tag, hitmap, pos);
                throw std::runtime_error("Decoding error");
            }

            uint64_t tot_field_width = RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] << 2;
            uint64_t n_tots = RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap];
            uint64_t tot_field = retrieve(pos, tot_field_width, data);
            if(print_stuff) {
                std::bitset<64> tot_bits(tot_field);
                LOGGER(error)("TOT_FIELD WIDTH = {}", tot_field_width);
                LOGGER(error)("TOT_FIELD = {}", tot_bits.to_string());
            }
            //uint64_t tot = drop_tot ? 0 : retrieve(pos, RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] << 2, data);
            if(drop_tot) { tot_field_width = 0; }
            for(unsigned ihit = 0; ihit < n_tots; ihit++) {
                uint8_t tot_val = (tot_field >> (ihit << 2)) & 0xf;
                uint16_t pix_col = ((ccol - 1) * 8) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] >> 4) + 1;
                uint16_t pix_row = ((qrow)*2) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] & 0xF) + 1;

                if(tot_val > 0) {
                    Hit hit;
                    hit.col = (pix_col - 1);
                    hit.row = (pix_row - 1);
                    hit.tot = tot_val;
                    n_hits++;
                    current_event.hits.push_back(hit);
                }
            } // ihit

            // that was the last block of hits in this ccol, so break out of this loop
            if(is_last == 1) {
                break;
            }
        } // end hit loop
    } // event loop


//////butts
//        // loop over all hits in the event
//        uint8_t qrow = 0;
//        uint8_t is_last = 0;
//        do {
//            is_last = retrieve(pos, 1, data);
//            uint8_t is_neighbor = retrieve(pos, 1, data);
//            if(is_neighbor == 1) {
//                qrow = qrow + 1;
//            } else {
//                qrow = retrieve(pos, 8, data);
//            }
//
//
//            //LOGGER(error)("Hitmap pos INITIAL = {}", pos);
//            uint32_t hitmap = retrieve(pos, 16, data);
//            LOGGER(error)("HITS FOR CH ID {} TAG {} CCOL {} QROW {} IS_LAST {} IS_NEIGHBOR {} HITMAP {:x}", ch_id, tag, ccol, qrow, is_last, is_neighbor, hitmap);
//            //std::bitset<16> hitmap_bits(hitmap);
//            //if(print_stuff) {
//            //    LOGGER(error)("   hitmap bits = {}", hitmap_bits.to_string());
//            //}
//            if(do_compressed_hitmap) {
//                uint16_t hitmap_raw = (hitmap & 0xffff);
//                hitmap = (RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFFFF);
//                uint8_t hitmap_rollBack = ((RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFF000000) >> 24);
//                if(hitmap_rollBack > 0) {
//                    if(hitmap_rollBack != 0xff) {
//                        pos = pos - hitmap_rollBack;
//                    }
//                    uint16_t rowMap = retrieve(pos, 14, data);
//                    hitmap |= (RD53BDecoding::_LUT_BinaryTreeRowHMap[rowMap] << 8);
//                    pos -= (RD53BDecoding::_LUT_BinaryTreeRowHMap[rowMap] & 0xFF00) >> 8;
//                } else {
//                    pos -= (RD53BDecoding::_LUT_BinaryTreeHitMap[hitmap_raw] & 0xFF0000) >> 16;
//                }
//                
//            }
//            //LOGGER(error)("Hitmap pos FINAL   = {}", pos);
//
//            if(qrow >= 196) {
//                for(unsigned ibus = 0; ibus < 4; ibus++) {
//                    uint8_t hitbus = (hitmap >> (ibus << 2)) & 0xf;
//                    if(hitbus) {
//                        uint16_t ptot_ptoa_buf = 0xffff;
//                        for(unsigned iread = 0; iread < 4; iread++) {
//                            if((hitbus >> iread) & 0x1) {
//                                ptot_ptoa_buf &= ~((~retrieve(pos, 4, data) & 0xf) << (iread<<2));
//                            }
//                        } // iread
//                        uint16_t ptot = ptot_ptoa_buf & 0x7ff;
//                        uint16_t ptoa = ptot_ptoa_buf >> 11;
//                        unsigned step = 0;
//                        uint16_t pix_col = (ccol -1 ) * 8 + PToT_maskStaging[step % 4][ibus] + 1;
//                        uint16_t pix_row = step / 2 + 1;
//                        Hit hit;
//                        hit.col = pix_col;
//                        hit.row = pix_row;
//                        hit.ptot = ptot;
//                        hit.ptoa = ptoa;
//                        n_hits++;
//                        current_event.hits.push_back(hit);
//                    } // if hitbus
//                } // ibus
//            } // qrow >= 196
//            else if(!use_ptot){
//
//                // not considering compressed hitmaps right now
//                //auto arr_size = RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] << 2;
//                if(RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] == 0) {
//                    LOGGER(error)("Received fragment with no ToT! ({}, {}) [chip id = {}, tag = {}]", ccol, qrow, ch_id, tag);
//                    throw std::runtime_error("Decoding error");
//                }
//
////47                         for (unsigned ihit = 0; ihit < _LUT_PlainHMap_To_ColRow_ArrSize[hitmap]; ++ihit)
////348                         {
////349                             const uint8_t pix_tot = (ToT >> (ihit << 2)) & 0xF;
////350                             // First pixel is 1,1, last pixel is 400,384
////351                             const uint16_t pix_col = ((ccol - 1) * 8) + (_LUT_PlainHMap_To_ColRow[hitmap][ihit] >> 4) + 1;
////352                             const uint16_t pix_row = ((qrow)*2) + (_LUT_PlainHMap_To_ColRow[hitmap][ihit] & 0xF) + 1;
////353
////354                             // For now fill in events without checking whether the addresses are valid
////355                             if (events[channel] == 0)
////356                             {
////357                                 logger->warn("[{}] No header in data fragment!", channel);
////358                                 curOut[channel]->newEvent(666, l1id[channel], bcid[channel]);
////359                                 events[channel]++;
////360                             }
////361
////362                             curOut[channel]->curEvent->addHit({pix_col,pix_row,pix_tot});
////363                             //logger->info("Hit: row({}) col({}) tot({}) ", pix_row, pix_col, pix_tot);
////364                             hits[channel]++;
////365                         }
////366                     }
//                //LOGGER(error)("ToT data start at POS = {}, size = {}", pos, arr_size);
//                uint64_t tot = drop_tot ? 0 : retrieve(pos, RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap] << 2, data);
//                //std::bitset<64> tot_bits(tot);
//                //uint64_t data_val = data[static_cast<unsigned>(pos / 64)];
//                //std::bitset<64> data_bits(data_val);
//                //if(print_stuff) {
//                //    LOGGER(error)("             ARR_SIZE = {}", arr_size);
//                //    LOGGER(error)("        current block = {}", data_bits.to_string());
//                //    LOGGER(error)("                  tot = {}", tot_bits.to_string());
//                //}
//                for(unsigned ihit = 0; ihit < RD53BDecoding::_LUT_PlainHMap_To_ColRow_ArrSize[hitmap]; ++ihit) {
//                    uint8_t pix_tot = (tot >> (ihit << 2)) & 0xf;
//                    uint16_t pix_col = ((ccol - 1) * 8) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] >> 4) + 1;
//                    uint16_t pix_row = ((qrow)*2) + (RD53BDecoding::_LUT_PlainHMap_To_ColRow[hitmap][ihit] & 0xF) + 1;
//
//                    // consider tot == 0 to be a "no hit"
//                    if(pix_tot > 0) {
//                        Hit hit;
//                        hit.col = (pix_col-1);
//                        hit.row = (pix_row-1);
//                        hit.tot = pix_tot;
//                        n_hits++;
//                        current_event.hits.push_back(hit);
//                    } // non-empty tot value
//                } // ihit
//            } 
//        } while(!is_last);
//        //events.push_back(current_event);
//    } // event loop
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
    std::string trigger_config_filename = "";
    bool use_ptot = false;
	bool verbose = false;
    bool force_ser = false;
    bool skip_decoding = false;
    int c;
    while ((c = getopt_long(argc, argv, "r:p:s:t:hdfx", longopts_t, NULL)) != -1) {
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
            case 't':
                trigger_config_filename = optarg;
                break;
            case 'd':
				verbose = true;
                break;
            case 'f':
                force_ser = true;
                break;
            case 'x':
                skip_decoding = true;
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
    if(trigger_config_filename != "") {
        fs::path trigger_config_path(trigger_config_filename);
        if(!fs::exists(trigger_config_filename)) {
            LOGGER(error)("Provided TRIGGER config file (=\"{}\") does not exist!", trigger_config_filename);
            return 1;
        }
        LOGGER(info)("Loading trigger config from: {}", trigger_config_filename);
    }

    namespace rh = rd53b::helpers;
    auto hw = rh::spec_init(hw_config_filename);
    auto fe_global = rh::rd53b_init(hw, primary_config_filename);
    fe_global->setChipId(16);

    auto fe_primary = rh::rd53b_init(hw, primary_config_filename);
    auto fe_secondary = rh::rd53b_init(hw, secondary_config_filename);
    auto cfg_primary = dynamic_cast<Rd53bCfg*>(fe_primary.get());
    auto cfg_secondary = dynamic_cast<Rd53bCfg*>(fe_secondary.get());


    // print out all registers and their current values
    //auto regMap = cfg_primary->regMap;
    //auto cfg_global = dynamic_cast<Rd53bGlobalCfg*>(cfg_primary);
    //for(auto it : regMap) {
    //    std::string name = it.first;
    //    LOGGER(info)("FOO {}: {}", name, (cfg_global->*it.second).read());
    //}
    fe_primary->sendClear(fe_primary->getChipId());
    fe_secondary->sendClear(fe_secondary->getChipId());

    // Sync CMD decoder
    hw->setCmdEnable(fe_global->getTxChannel());
    hw->setTrigEnable(0x0);
    wait(hw);
    hw->flushBuffer();
    hw->setRxEnable(fe_global->getRxChannel());
    hw->setRxEnable(fe_primary->getRxChannel());
    hw->setRxEnable(fe_secondary->getRxChannel());
    hw->runMode();

    rh::rd53b_configure(hw, fe_primary);
    rh::disable_pixels(fe_primary);
    wait(hw);
    hw->flushBuffer();
    wait(hw);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep to let SECONDARY catch up the CMD signals from the PRIMARY
    rh::rd53b_configure(hw, fe_secondary);
    rh::disable_pixels(fe_secondary);
    wait(hw);

    if(!force_ser) {
        LOGGER(info)("Setting SerSelOut to CLK/2");
        fe_secondary->writeNamedRegister("SerSelOut0", 0);
        fe_secondary->writeNamedRegister("SerSelOut1", 0);
        fe_secondary->writeNamedRegister("SerSelOut2", 0);
        fe_secondary->writeNamedRegister("SerSelOut3", 0);
        wait(hw);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //uint16_t reset_cmd = 0x90;
    uint16_t reset_cmd = 0xB9;
    send_reset(hw, fe_primary, reset_cmd);
//    send_reset(hw, fe_secondary, reset_cmd);

    // first configure the secondary to send clock signals

    // configure pixels that we want to inject triggers
    std::vector<std::pair<unsigned, unsigned>> pixel_addresses_primary {
        {0,0}
        ,{1,0}
    };
    std::vector<std::pair<unsigned, unsigned>> pixel_addresses_secondary {
        {7,0}
        ,{2,1}
    };
    std::array<uint16_t, 4> cores = {0x0, 0x0, 0x0, 0x0};

    set_cores(fe_primary, cores, use_ptot);
    wait(hw);
    if(fe_primary->InjDigEn.read() == 1) {
        LOGGER(info)("Enabling PRIMARY pixels for digital injection");
        set_pixels_enable(fe_primary, pixel_addresses_primary);
        wait(hw);
        // configure the corresponding core columns
        cores[0] = 0xf;
        set_cores(fe_primary, cores, use_ptot);
        wait(hw);
    }

    // enable pixels for digital injection on SECONDARY
    cores = {0x0, 0x0, 0x0, 0x0};
    set_cores(fe_secondary, cores, use_ptot);
    if(fe_secondary->InjDigEn.read() == 1) {
        LOGGER(info)("Enabling SECONDARY pixels for digital injection");
        set_pixels_enable(fe_secondary, pixel_addresses_secondary);
        wait(hw);
        cores[0] = 0xf;
        set_cores(fe_secondary, cores, use_ptot);
        wait(hw);
    }


    // configure and start the triggers
    hw->setCmdEnable(cfg_primary->getTxChannel());
    hw->setCmdEnable(cfg_secondary->getTxChannel());
    json trigger_config =  {{"trigMultiplier", 16},
                            {"count", 5},
                            {"delay", 56},
                            {"extTrigger", false},
                            {"frequency", 5000},
                            {"noInject", false},
                            {"time", 0},
                            {"edgeMode", true},
                            {"edgeDuration", 20}};
    if(trigger_config_filename != "") {
        auto jtrig = ScanHelper::openJsonFile(trigger_config_filename);
        trigger_config = jtrig["rd53b"]["trigger_config"];
    }
    int count = trigger_config["count"];
    LOGGER(info)("Trigger config count = {}", count);
    rh::spec_init_trigger(hw, trigger_config);
    wait(hw);

    // begin triggers
    hw->runMode();

    // let's send a global reset command to reset the PRIMARY's data merging path
    //reset_cmd = 0x90;
    //send_reset(hw, fe_primary, reset_cmd);
    //send_reset(hw, fe_secondary, reset_cmd);

    // now tell the secondary to send AURORA
    if(!force_ser) {
        LOGGER(info)("Setting SerSelOut to AURORA");
        fe_secondary->writeNamedRegister("SerSelOut0", 1);
        fe_secondary->writeNamedRegister("SerSelOut1", 1);
        fe_secondary->writeNamedRegister("SerSelOut2", 1);
        fe_secondary->writeNamedRegister("SerSelOut3", 1);
        wait(hw);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    send_reset(hw, fe_primary, reset_cmd);
    wait(hw);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
        if(blocks.size() > 500) {
            LOGGER(error)("More than 500 blocks, not considering any more!");
            break;
        }
        uint64_t data0 = static_cast<uint64_t>(data_vec.at(i));
        uint64_t data1 = static_cast<uint64_t>(data_vec.at(i+1));
        uint64_t data = data1 | (data0 << 32);
        std::bitset<64> bits(data);
        std::cout << "block[" << std::setw(4) << blocks.size() << "]: " << bits.to_string() << std::endl;
        blocks.push_back(data);
    }


    std::map<unsigned, std::vector<Stream>> stream_map;
    std::map<unsigned, unsigned> stream_in_progress_status;
    std::map<unsigned, std::vector<uint64_t>> stream_in_progress;

    stream_in_progress[0x3 & fe_primary->getChipId()];
    stream_in_progress[0x3 & fe_secondary->getChipId()];

    std::map<unsigned, unsigned> block_count;

    if(data_vec.size() % 2 != 0) {
        LOGGER(error)("Received non-even number of 32-bit words (={})", data_vec.size());
        return 1;
    }
    //for(size_t block_num =  0; block_num < (data_vec.size()/2); block_num++) {
    for(size_t block_num = 0; block_num < blocks.size(); block_num++) {
        auto data = blocks[block_num];
        uint8_t ns_bit = (data >> 63) & 0x1;
        uint8_t ch_id = (data >> 61) & 0x3;

        if(block_count.find(ch_id) == block_count.end()) {
            block_count[ch_id] = 0;
        }
        block_count.at(ch_id)++;

        std::bitset<64> bits(data);
        //if(ch_id == set_chip_id_ls) {
        //LOGGER(warn)("Skipping data with CH.ID = {}", ch_id);
        bool is_primary = (ch_id == 3);
        bool is_secondary = (ch_id == 2);
        bool is_expected_chip = (is_primary || is_secondary);
        if(!is_expected_chip) {
            LOGGER(error)("Data from unexpected chip id = {}", ch_id);
            continue;
        }
        // ignore empty blocks
        //uint64_t non_header = (data << 11);
        //if(non_header == 0 && ns_bit == 1) { 
        //    LOGGER(warn)("IGNORING DATA BLOCK FOR STREAM SINCE ITS EMPTY: {}", bits.to_string());
        //    continue; 
        //}
        if(ns_bit == 1) {
            //LOGGER(warn)("NS bit seen for ch_id = {}", ch_id);
            if(stream_in_progress.at(ch_id).size() > 0) {
                Stream st;
                st.chip_id = ch_id;
                st.blocks = stream_in_progress.at(ch_id);
                LOGGER(warn)("Pushing back stream for ch id {} that is {} 64-bit blocks long", ch_id, st.blocks.size());
                for(auto b : st.blocks) {
                    std::bitset<64> bbits(b);
                    LOGGER(warn)("    -> {}", bbits.to_string());
                }
                stream_map[ch_id].push_back(st);
                stream_in_progress[ch_id].clear();
                //LOGGER(warn)("Pushing back data blocks for stream with ch_id = {}", ch_id);
            }
        }
        //LOGGER(error)("PUSHING BACK DATA FOR STREAM_IN_PROGRESS[{}]", ch_id);

        if(ch_id == (0x3 & fe_primary->getChipId()) || ch_id == (0x3 & fe_secondary->getChipId())) {
            LOGGER(info)("Data from CH ID {}: {}", ch_id, bits.to_string());
            //LOGGER(info)("Data from CH ID {}: {:x}", set_chip_id_ls, bits.to_ulong());//to_string());
        }
        stream_in_progress[ch_id].push_back(data);
    }

    if(skip_decoding) {
        LOGGER(info)("Skipping data stream decoding...");
        return 0;
    }

    bool do_compressed_hitmap = fe_primary->DataEnRaw.read() == 1;
    if(fe_primary->DataEnRaw.read() != fe_secondary->DataEnRaw.read()) {
        LOGGER(error)("Primary and Secondary are both not set to have the same hitmap compression!");
        LOGGER(error)("Exiting!");
        return 1;
    }

    std::vector<unsigned> chip_ids {fe_primary->getChipId(), fe_secondary->getChipId()};
    for(auto chip_id_full : chip_ids) {
        LOGGER(info)("----------------------------------------------------------------");
        uint8_t chip_id = 0x3 & chip_id_full;
        std::vector<Event> events;
        unsigned n_hits_total = 0;
        for(size_t i = 0; i < stream_map[chip_id].size(); i++) {
            auto stream = stream_map[chip_id][i];
            //LOGGER(warn)("Calling decode_stream for stream with ch_id = {}", stream.chip_id);
            events = decode_stream(stream, /*drop tot*/ false, /*do compressed hitmap*/ do_compressed_hitmap, /*use_ptot*/ use_ptot);
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
    } // chip_id_full
    LOGGER(info)("-------------------------------------------------------------------");
    LOGGER(info)("Total blocks seen for each observed chip id (2 ls bits):");
    for(auto cnt: block_count) {
        LOGGER(info)("   CH ID LS[{}] = {} blocks seen", cnt.first, cnt.second);
    }


    return 0;
}
