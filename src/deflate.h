#ifndef DEFLATE_H
#define DEFLATE_H

#include <cstdint>
#include <vector>
#include <cstddef>
#include <unordered_map>
#include <map>
#include <stdexcept>


class Bitstream {
public:
    Bitstream(const std::vector<uint8_t>& data) : data(data), bit_pos(0) {}

    uint32_t read_bits(size_t count) {
        uint32_t value = 0;
        for (size_t i = 0; i < count; ++i) {
            if (bit_pos >= data.size() * 8) {
                //std::cerr << "Error: Attempt to read past end of bitstream" << std::endl;
            } else {
                value |= (get_bit(bit_pos) << i);
                bit_pos++;
            }
        }
        return value;
    }

    void set_pos(size_t pos){
        bit_pos = pos;
    }

    void byte_align() {
        bit_pos = (bit_pos + 7) & ~7;
    }

    const std::vector<uint8_t>& data;
    size_t bit_pos;

    uint8_t get_bit(size_t bit_pos) {
        //std::cout << "Reading from " << bit_pos << std::endl;
        //std::cout << "At byte " << std::hex << (int) data[bit_pos / 8] << std::dec << std::endl;
        return (data[bit_pos / 8] >> ((bit_pos % 8))) & 1;
    }
    bool finished() {
        //std::cout << "Reading from " << bit_pos << std::endl;
        //std::cout << "At byte " << std::hex << (int) data[bit_pos / 8] << std::dec << std::endl;
        return bit_pos >= data.size() * 8;
    }
};

bool verifyLengths(const std::vector<uint16_t>& lengths);

class HuffmanTable {
    public:
    std::vector<uint16_t> lengths;
    std::map<uint32_t, uint16_t> codes;

    HuffmanTable() = default;

    HuffmanTable(const std::vector<uint16_t>& lengths) {
        build(lengths);
    }

    void build(const std::vector<uint16_t>& lengths) {
        this->lengths = lengths;
        codes.clear();

        std::vector<uint32_t> bl_count(16, 0);
        for (auto len : lengths) {
            if (len > 0) {
                bl_count[len]++;
            }
        }

        std::vector<uint32_t> next_code(16, 0);
        uint32_t code = 0;
        for (size_t bits = 1; bits <= 15; ++bits) {
            code = (code + bl_count[bits - 1]) << 1;
            next_code[bits] = code;
        }

        for (size_t n = 0; n < lengths.size(); ++n) {
            uint16_t len = lengths[n];
            if (len != 0) {
                codes[next_code[len]] = n;
                next_code[len]++;
            }
        }
    }

    uint16_t decode(Bitstream& stream) const {
        uint32_t code = 0;
        for (uint16_t len = 1; len <= 15; ++len) {
            code |= stream.read_bits(1);
            auto it = codes.find(code);
            if (it != codes.end() && lengths[it->second] == len) {
                return it->second;
            }
            code <<= 1;
        }
        throw std::runtime_error("Invalid Huffman code");
    }
};

HuffmanTable load_code_lengths(Bitstream* bitstream, uint16_t hclen);
HuffmanTable load_literal_lengths(Bitstream& bitstream, const HuffmanTable* code_length_table, uint16_t hlit);
HuffmanTable load_distance_lengths(Bitstream& bitstream, const HuffmanTable* code_length_table, uint16_t hdist);

class DeflatePacket {
    public:
    size_t start_position;
    size_t header_start;
    size_t data_start;
    size_t end_position;
    size_t block_type;
    bool last;
    bool safe;
    Bitstream* bitstream;
    HuffmanTable literal_table;
    HuffmanTable code_length_table;
    HuffmanTable distance_table;
    std::unordered_map<uint16_t, size_t> literal_map;
    std::unordered_map<uint16_t, size_t> distance_map;
    
    DeflatePacket() = default;
};

class LZToken {
    public:
    uint8_t character;
    size_t distance;
    size_t length;
    LZToken(uint8_t val){
        this->character = val;
        this->distance = 0;
        this->length = 0;
    }
    LZToken(int distance, int length){
        this->character = 0;
        this->distance = distance;
        this->length = length;
    }
    LZToken(){
        this->character = 0;
        this->distance = 0;
        this->length = 0;
    }
};

std::vector<DeflatePacket> header_scan(const std::vector<uint8_t>& compressed_data);

std::vector<uint8_t> custom_inflate(const std::vector<uint8_t>& compressed_data);

std::vector<LZToken> symbolic_inflate(const std::vector<uint8_t>& compressed_data, size_t start_position);

void print_packet_details(const std::vector<DeflatePacket> &packets);

#endif
