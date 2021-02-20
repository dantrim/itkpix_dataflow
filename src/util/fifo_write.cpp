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

struct option longopts_t[] = {{"hw", required_argument, NULL, 'c'},
                              {"chip", required_argument, NULL, 'r'},
                              {"debug", no_argument, NULL, 'd'},
                              {"help", no_argument, NULL, 'h'},
                              {"chip-id", required_argument, NULL, 'i'},
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

void print_help() {
    std::cout << "=========================================================="
              << std::endl;
	std::cout << " ITkPix dataflow test" << std::endl;
    std::cout << std::endl;
    std::cout << " Usage: [CMD] [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << " Options:" << std::endl;
    std::cout << "   --hw         JSON configuration file for hw controller"
              << std::endl;
    std::cout << "   --chip       JSON connectivity configuration for RD53B"
              << std::endl;
    std::cout << "   -p           use PToT" << std::endl;
    std::cout << "   -i|--chip-id Chip ID (must be same as the ChipId field in the chip JSON config" << std::endl;
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

    unsigned set_chip_id = 0xf;
    unsigned set_chip_id_ls = 0x0;

    std::string chip_config_filename = "";
    std::string hw_config_filename = "";
	bool verbose = false;
    bool use_ptot = false;
    int c;
    while ((c = getopt_long(argc, argv, "c:dr:hpi:", longopts_t, NULL)) != -1) {
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
            case 'p':
                use_ptot = true;
                break;
            case 'i':
                set_chip_id = 0xffff & atoi(optarg);
                set_chip_id_ls = (set_chip_id & 0x3); // lower 2 bits
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

    if(fe->getChipId() != set_chip_id) {
        LOGGER(error)("Chip-ID from chip JSON configuration (={}) does not equal the one specified by the user (={})!", fe->getChipId(), set_chip_id);
        throw std::runtime_error("Error in setting chip id!");
    }

	//rh::clear_tot_memories(hw, fe);
	//rh::rd53b_reset(hw, fe);
    auto cfg = dynamic_cast<FrontEndCfg*>(fe.get());

    hw->setCmdEnable(cfg->getTxChannel());
    std::string foo;
    fe->configure();
    fe->writeNamedRegister("GpLvdsBias", 15);
    fe->writeNamedRegister("GpLvdsEn", 15);
    fe->writeNamedRegister("GpLvdsPad0", 0);
    fe->writeNamedRegister("GpLvdsPad1", 0);
    fe->writeNamedRegister("GpLvdsPad2", 0);
    fe->writeNamedRegister("GpLvdsPad3", 0);

    
    
    return 0;
}
