#include "deflate.h"
#include <iostream>

HuffmanTable load_code_lengths(Bitstream* bitstream, uint16_t hclen){
    std::vector<uint16_t> code_length_order = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    std::vector<uint16_t> code_lengths(19, 0);
    for (size_t i = 0; i < hclen; ++i) {
        if(bitstream->finished()){
            break;
        }
        code_lengths[code_length_order[i]] = bitstream->read_bits(3);
    }

    HuffmanTable code_length_table(code_lengths);
    return code_length_table;
}
HuffmanTable load_literal_lengths(Bitstream& bitstream, const HuffmanTable* code_length_table, uint16_t hlit){
    std::vector<uint16_t> literal_lengths;

    while (literal_lengths.size() < hlit) {
        if(bitstream.finished()){
            break;
        }
        uint16_t symbol = code_length_table->decode(bitstream);
        if (symbol < 16) {
            literal_lengths.push_back(symbol);
        } else if (symbol == 16 && literal_lengths.size() > 0) {
            uint16_t repeat = bitstream.read_bits(2) + 3;
            literal_lengths.insert(literal_lengths.end(), repeat, literal_lengths.back());
        } else if (symbol == 17) {
            uint16_t repeat = bitstream.read_bits(3) + 3;
            literal_lengths.insert(literal_lengths.end(), repeat, 0);
        } else if (symbol == 18) {
            uint16_t repeat = bitstream.read_bits(7) + 11;
            literal_lengths.insert(literal_lengths.end(), repeat, 0);
        }
    }
    HuffmanTable literal_table(literal_lengths);
    return literal_table;
}
HuffmanTable load_distance_lengths(Bitstream& bitstream, const HuffmanTable* code_length_table, uint16_t hdist){
    std::vector<uint16_t> distance_lengths;
    while (distance_lengths.size() < hdist) {
        if(bitstream.finished()){
            break;
        }
        uint16_t symbol = code_length_table->decode(bitstream);
        if (symbol < 16) {
            distance_lengths.push_back(symbol);
        } else if (symbol == 16 && distance_lengths.size() > 0) {
            uint16_t repeat = bitstream.read_bits(2) + 3;
            distance_lengths.insert(distance_lengths.end(), repeat, distance_lengths.back());
        } else if (symbol == 17) {
            uint16_t repeat = bitstream.read_bits(3) + 3;
            distance_lengths.insert(distance_lengths.end(), repeat, 0);
        } else if (symbol == 18) {
            uint16_t repeat = bitstream.read_bits(7) + 11;
            distance_lengths.insert(distance_lengths.end(), repeat, 0);
        }
    }
    HuffmanTable distance_table(distance_lengths);
    return distance_table;
}

std::vector<DeflatePacket> header_scan(const std::vector<uint8_t>& compressed_data){
        // Read and verify zlib header
    // Process compressed data blocks
    Bitstream bitstream(compressed_data);
    std::vector<DeflatePacket> packets;
    
    for(size_t i=0; i<compressed_data.size()*8; i++){
        bitstream.set_pos(i);
        //std::cout << (int) i << std::endl;
        DeflatePacket test_packet;
        test_packet.start_position = i;
        test_packet.last = 1 == bitstream.read_bits(1);
        test_packet.block_type = bitstream.read_bits(2);
        test_packet.header_start = bitstream.bit_pos;
        test_packet.safe = true;
        if (test_packet.block_type == 2) {
            // Dynamic Huffman block
            uint16_t hlit = bitstream.read_bits(5) + 257;
            uint16_t hdist = bitstream.read_bits(5) + 1;
            uint16_t hclen = bitstream.read_bits(4) + 4;
            HuffmanTable code_length_table = load_code_lengths(&bitstream, hclen);
            if(!verifyLengths(code_length_table.lengths)){
                //std::cout << "Bad lengths in code length!" << std::endl;
                continue;
            }
            HuffmanTable literal_table = load_literal_lengths(bitstream, &code_length_table, hlit);
            HuffmanTable distance_table = load_distance_lengths(bitstream,&code_length_table, hdist);
            /*
            Time to check if we have a valid huffman tree!
            */
            if(!verifyLengths(literal_table.lengths) || !verifyLengths(distance_table.lengths)){
                //std::cout << "Bad lengths!" << std::endl;
                continue;
            }
            test_packet.data_start = bitstream.bit_pos;

            /*
            Now read til the end...
            */
            bool safe_end = false;
            while (true) {
                if(bitstream.finished()){
                    break;
                }
                uint16_t symbol = literal_table.decode(bitstream);
                if (symbol < 256) {
                } else if (symbol == 256) {
                    safe_end = true;
                    break;
                } else {
                    uint16_t length = 0;
                    if (symbol <= 264) length = symbol - 257 + 3;
                    else if (symbol <= 284) {
                        uint32_t extra_bits = (symbol - 261) / 4;
                        if (symbol < 261){
                            extra_bits = 0;
                        }
                        bitstream.read_bits(extra_bits);
                    } else if (symbol == 285) length = 258;

                    uint16_t dist_symbol = distance_table.decode(bitstream);

                    if (dist_symbol <= 3) {
                    } else {
                        // Calculate the number of extra bits
                        uint32_t extra_bits = (dist_symbol - 2) / 2;
                        // Use base distance and extra bits to calculate the distance
                        bitstream.read_bits(extra_bits);
                    }
                }
            }
            test_packet.end_position = bitstream.bit_pos;
            test_packet.bitstream = 0;
            if(safe_end){
                packets.push_back(test_packet);
            }
        } else {
            continue;
        }
    }

    return packets;
}


std::vector<uint8_t> custom_inflate(const std::vector<uint8_t>& compressed_data) {

    // Read and verify zlib header
    if (compressed_data.size() < 2) {
        std::cerr << "Error: Data size is too small for zlib header" << std::endl;
        exit(1);
    }

    uint16_t header = (compressed_data[0] | (compressed_data[1] << 8));
    uint8_t cmf = header & 0xFF;
    uint8_t flg = (header >> 8) & 0xFF;

    if ((cmf * 256 + flg) % 31 != 0) {
        std::cerr << "Error: Incorrect header check value" << std::endl;
        exit(1);
    }

    if ((cmf & 0x0F) != 8) {
        std::cerr << "Error: Unsupported compression method" << std::endl;
        exit(1);
    }

    // Process compressed data blocks
    Bitstream bitstream(compressed_data);
    bitstream.read_bits(8); // Skip the CMF byte
    bitstream.read_bits(8); // Skip the FLG byte
    std::vector<uint8_t> decompressed_data;

    bool last_block = false;
    while (!last_block) {
        if(bitstream.finished()){
            break;
        }
        last_block = bitstream.read_bits(1);
        uint32_t block_type = bitstream.read_bits(2);

        #ifdef DEBUG_INFLATE
        std::cerr << "Block type " << block_type << std::endl;
        #endif
        #ifdef DEBUG_PACKET_LOCATIONS
        std::cout << "Packet: " << std::hex << bitstream.bit_pos - 3 << std::dec << ", type " << block_type << ", end " << last_block << std::endl;
        #endif
        if (block_type == 0) {
            // Uncompressed block (not implemented in this example)
            continue;
            throw std::runtime_error("Uncompressed blocks are not supported in this example");
        } else if (block_type == 1) {
            // Fixed Huffman block (not implemented in this example)
            continue;
            throw std::runtime_error("Fixed Huffman blocks are not supported in this example");
        }  else if (block_type == 2) {
            // Dynamic Huffman block
            uint16_t hlit = bitstream.read_bits(5) + 257;
            uint16_t hdist = bitstream.read_bits(5) + 1;
            uint16_t hclen = bitstream.read_bits(4) + 4;

            
            HuffmanTable code_length_table = load_code_lengths(&bitstream, hclen);
            if(!verifyLengths(code_length_table.lengths)){
                //std::cout << "Bad lengths in code length!" << std::endl;
                continue;
            }
            HuffmanTable literal_table = load_literal_lengths(bitstream, &code_length_table, hlit);
            HuffmanTable distance_table = load_distance_lengths(bitstream,&code_length_table, hdist);
            while (true) {
                if(bitstream.finished()){
                    break;
                }
                uint16_t symbol = literal_table.decode(bitstream);
                #ifdef DEBUG_INFLATE
                std::cout << "Symbol " << std::hex << (int) symbol << std::dec << std::endl;
                #endif
                if (symbol < 256) {
                    #ifdef DEBUG_INFLATE
                    std::cout << "Hit literal " << std::hex << (int) symbol << std::dec << std::endl;
                    #endif
                    decompressed_data.push_back(static_cast<uint8_t>(symbol));
                } else if (symbol == 256) {
                    #ifdef DEBUG_INFLATE
                    std::cout << "Hit end of block symbol!" << std::endl;
                    #endif
                    break;
                } else {
                    uint16_t length = 0;
                    if (symbol <= 264) length = symbol - 257 + 3;
                    else if (symbol <= 284) {
                        const uint32_t length_base[28] = {
                            3, 4, 5, 6, 7, 8, 9, 10,
                            11, 13, 15, 17, 19, 23, 27, 31,
                            35, 43, 51, 59, 67, 83, 99, 115,
                            131, 163, 195, 227
                        };
                        uint32_t extra_bits = (symbol - 261) / 4;
                        if (symbol < 261){
                            extra_bits = 0;
                        }
                        uint32_t base_length = length_base[symbol - 257];
                        length = base_length + bitstream.read_bits(extra_bits);
                        #ifdef DEBUG_INFLATE
                        std::cout << "Reading extra bits "  << (int) extra_bits << " with length base " << (int) base_length << std::endl;
                        #endif

                    } else if (symbol == 285) length = 258;

                    // Define base distances and extra bits length for distance symbols
                    const uint16_t distance_base[30] = {
                        1, 2, 3, 4, 5, 7, 9, 13,
                        17, 25, 33, 49, 65, 97, 129, 193,
                        257, 385, 513, 769, 1025, 1537, 2049, 3073, 
                        4097, 6145, 8193, 12289, 16385, 24577
                    };

                    uint16_t dist_symbol = distance_table.decode(bitstream);
                    #ifdef DEBUG_INFLATE
                    std::cout << "Distance symbol " << std::dec << (int) dist_symbol << std::endl;
                    #endif
                    uint16_t distance = 0;

                    if (dist_symbol <= 3) {
                        // Use base distance directly
                        distance = dist_symbol + 1;
                    } else {
                        // Calculate the number of extra bits
                        uint32_t extra_bits = (dist_symbol - 2) / 2;
                        // Use base distance and extra bits to calculate the distance
                        distance = distance_base[dist_symbol] + bitstream.read_bits(extra_bits);
                    }
                    #ifdef DEBUG_INFLATE
                    std::cout << "match " << std::dec << (int) length << " " << (int) distance << std::dec << std::endl;
                    #endif
                    size_t copy_pos = decompressed_data.size() - distance;
                    for (uint16_t i = 0; i < length; ++i) {
                        decompressed_data.push_back(decompressed_data[copy_pos++]);
                    }
                }
            }
        } else {
            continue;
            std::cerr << "Error: Invalid block type: " << block_type << std::endl;
            exit(1);
        }
    }

    return decompressed_data;
}

void print_packet_details(const std::vector<DeflatePacket> &packets) {
    for (const auto &packet : packets) {
        std::cout << "Packet details:\n" <<std::hex;
        std::cout << "  Start Position: " << packet.start_position << "\n";
        std::cout << "  Header Start: " << packet.header_start << "\n";
        std::cout << "  Data Start: " << packet.data_start << "\n";
        std::cout << "  End Position: " << packet.end_position << "\n";
        std::cout << "  Block Type: " << packet.block_type << "\n";
        std::cout << "  Last Block: " << (packet.last ? "Yes" : "No") << "\n";
        std::cout << "  Safe: " << (packet.safe ? "Yes" : "No") << "\n";
        std::cout << std::dec << std::endl;
    }
}
