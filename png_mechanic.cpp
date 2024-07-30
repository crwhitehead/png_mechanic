/*

g++ png_mechanic.cpp -o png_mechanic -O3

*/


#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <zlib.h>
#include <bitset>
#include <map>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <numeric>
#include <cmath>

#undef DEBUG_CRC_INPUT
#undef DEBUG_INFLATE
#undef DEBUG_FILTERS
#define DEBUG_PACKET_LOCATIONS 1


struct uncertainByte {
    uint8_t value;
    uint8_t* foundValue;
    int32_t reference;
    bool found;
    bool testing;
    uncertainByte() {
        this->value = 0;
        this->reference = 0;
        this->found = false;
        this->testing = false;
        this->foundValue = 0;
    }
    uncertainByte(uint8_t value, int32_t reference, bool found) {
        this->value = value;
        this->reference = reference;
        this->found = found;
        this->testing = false;
        this->foundValue = 0;
    }
    uint8_t getValue(){
        if(this->foundValue == 0){
            return *this->foundValue;
        } else {
            return this->value;
        }
    }
};

size_t MEDIAN_PASSES = 0;
size_t SMOOTH_PASSES = 0;
size_t SMOOTH_LOOPS = 0;

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
    
bool verifyLengths(const std::vector<uint16_t>& lengths){
    unsigned long total = 0;
    for (auto len : lengths) {
        if (len > 0) {
            total += 1 << (20-len);
        }
    }
    //printf("%p\n", total);
    return total == 0x100000;
}


struct HuffmanTable {
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

struct DeflatePacket {
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

struct PNGChunk {
    uint32_t length;
    char type[5];
    std::vector<uint8_t> data;
    uint32_t crc;
    uint32_t computed_crc;
};

struct PNGImage {
    uint32_t width;
    uint32_t height;
    uint8_t bit_depth;
    uint8_t color_type;
    uint8_t compression_method;
    uint8_t filter_method;
    uint8_t interlace_method;
    std::vector<PNGChunk> chunks;
    std::vector<DeflatePacket> packets;
    std::vector<uint8_t> image_data;
    std::vector<uint8_t> decompressed_data;
    std::vector<uint8_t> pixels;
};

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

// CRC computation table for verifying chunk CRC values
static unsigned long crc_table[256];
static bool crc_table_computed = false;

 /* Make the table for a fast CRC. */
void make_crc_table(void)
{
    unsigned long c;
    int n, k;
    for (n = 0; n < 256; n++) {
        c = (unsigned long) n;
        for (k = 0; k < 8; k++) {
            if (c & 1){ 
                c = 0xedb88320L ^ (c >> 1);
            } else {
                c = c >> 1;
            }
        }
        crc_table[n] = c;
    }
    crc_table_computed = 1;
}

/* Update a running CRC with the bytes buf[0..len-1]--the CRC
 should be initialized to all 1's, and the transmitted value
 is the 1's complement of the final running CRC (see the
 crc() routine below)). */

unsigned long update_crc(unsigned long crc, unsigned char *buf,
                       int len)
{
    unsigned long c = 0xffffffffL^crc;
    int n;

    if (!crc_table_computed){
        make_crc_table();
    }
    for (n = 0; n < len; n++) {
        #ifdef DEBUG_CRC_INPUT
        std::cout << std::hex << static_cast<int>(buf[n]) << " " << std::dec;
        #endif
        c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
    }
    #ifdef DEBUG_CRC_INPUT
    std::cout << std::endl;
    #endif
    return 0xffffffffL^c;
}

/* Return the CRC of the bytes buf[0..len-1]. */
unsigned long crc(unsigned char *buf, int len)
{
    return update_crc(0, buf, len);
}


std::vector<uint8_t> build_image_data(const std::vector<PNGChunk> &chunks) {
    std::vector<uint8_t> image_data;
    for (const auto &chunk : chunks) {
        if (strcmp(chunk.type, "IDAT") == 0) {
            image_data.insert(image_data.end(), chunk.data.begin(), chunk.data.end());
        }
    }
    return image_data;
}

PNGImage parse_png(const std::string &filename) {
    PNGImage image;

    bool ihdr_parsed = false;
std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        exit(1);
    }

    // Read the entire file into a buffer
    std::vector<uint8_t> file_data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto it = file_data.begin();

    // Skip PNG signature
    std::advance(it, 8);

    while (true) {
        if(it >= file_data.end()){
            break;
        }
        PNGChunk chunk;

        // Read chunk length
        uint32_t chunk_length = (it[0] << 24) | (it[1] << 16) | (it[2] << 8) | it[3];
        std::advance(it, 4);
        chunk.length = chunk_length;

        // Read chunk type
        std::copy(it, it + 4, chunk.type);
        chunk.type[4] = '\0';
        std::advance(it, 4);

        // Read chunk data
        chunk.data.resize(chunk_length);
        std::copy(it, it + chunk_length, chunk.data.begin());
        std::advance(it, chunk_length);

        // Read chunk CRC
        uint32_t chunk_crc = (it[0] << 24) | (it[1] << 16) | (it[2] << 8) | it[3];
        std::advance(it, 4);
        chunk.crc = chunk_crc;

        // Verify CRC
        uint32_t computed_crc = crc(reinterpret_cast<uint8_t *>(chunk.type), 4);
        computed_crc = update_crc(computed_crc, chunk.data.data(), chunk.length);
        chunk.computed_crc = computed_crc;
        if (computed_crc != chunk.crc) {
            std::cerr << "Error: CRC mismatch in chunk " << chunk.type << std::endl;
            // exit(1);
        }

        if (strcmp(chunk.type, "IHDR") == 0) {
            if (chunk.length != 13 || chunk.data.size() != 13) {
                std::cerr << "Error: Invalid IHDR chunk length." << std::endl;
                exit(1);
            }
            image.width = (chunk.data[0] << 24) | (chunk.data[1] << 16) | (chunk.data[2] << 8) | chunk.data[3];
            image.height = (chunk.data[4] << 24) | (chunk.data[5] << 16) | (chunk.data[6] << 8) | chunk.data[7];
            image.bit_depth = chunk.data[8];
            image.color_type = chunk.data[9];
            image.compression_method = chunk.data[10];
            image.filter_method = chunk.data[11];
            image.interlace_method = chunk.data[12];
            ihdr_parsed = true;
        }

        image.chunks.push_back(chunk);

        if (strcmp(chunk.type, "IEND") == 0)
            break;
    }
    if (!ihdr_parsed) {
        std::cerr << "Error: Missing IHDR chunk." << std::endl;
        exit(1);
    }
    image.image_data = build_image_data(image.chunks);
    return image;
}

void print_chunk_details(const std::vector<PNGChunk> &chunks) {
    for (const auto &chunk : chunks) {
        std::cout << "Chunk Type: " << chunk.type << std::endl;
        std::cout << "Chunk Length: " << chunk.length << std::endl;
        std::cout << "Chunk CRC: " << std::hex << chunk.crc << std::dec << std::endl;
        std::cout << "Computed CRC: " << std::hex << chunk.computed_crc << std::dec << std::endl;
        std::cout << "Chunk Data (first 20 bytes): ";
        for (size_t i = 0; i < std::min(chunk.data.size(), size_t(20)); ++i) {
            std::cout << std::hex << static_cast<int>(chunk.data[i]) << " ";
        }
        std::cout << std::dec << std::endl << std::endl;
    }
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


std::vector<uint8_t> decompress_idat_data(const std::vector<uint8_t> &compressed_data) {
    return custom_inflate(compressed_data);
}

void print_image_info(const PNGImage &image) {
    std::cout << "Image Width: " << image.width << std::endl;
    std::cout << "Image Height: " << image.height << std::endl;
    std::cout << "Bit Depth: " << static_cast<int>(image.bit_depth) << std::endl;
    std::cout << "Color Type: " << static_cast<int>(image.color_type) << std::endl;
    std::cout << "Compression Method: " << static_cast<int>(image.compression_method) << std::endl;
    std::cout << "Filter Method: " << static_cast<int>(image.filter_method) << std::endl;
    std::cout << "Interlace Method: " << static_cast<int>(image.interlace_method) << std::endl;
}

uint8_t paeth_predictor(uint8_t a, uint8_t b, uint8_t c) {
    int p = a + b - c;
    int pa = std::abs(p - a);
    int pb = std::abs(p - b);
    int pc = std::abs(p - c);

    if (pa <= pb && pa <= pc) return a;
    else if (pb <= pc) return b;
    else return c;
}

void unfilter_scanline(uint8_t filter_type, std::vector<uint8_t> &scanline, const std::vector<uint8_t> &previous_scanline, uint32_t bpp) {
    switch (filter_type) {
        case 0:
            // No filter
            break;
        case 1:
            // Sub filter
            for (uint32_t i = bpp; i < scanline.size(); ++i) {
                scanline[i] += scanline[i - bpp];
            }
            break;
        case 2:
            // Up filter
            for (uint32_t i = 0; i < scanline.size(); ++i) {
                scanline[i] += previous_scanline[i];
            }
            break;
        case 3:
            // Average filter
            for (uint32_t i = 0; i < scanline.size(); ++i) {
                uint8_t left = (i >= bpp) ? scanline[i - bpp] : 0;
                uint8_t above = previous_scanline[i];
                scanline[i] += (left + above) / 2;
            }
            break;
        case 4:
            // Paeth filter
            for (uint32_t i = 0; i < scanline.size(); ++i) {
                uint8_t left = (i >= bpp) ? scanline[i - bpp] : 0;
                uint8_t above = previous_scanline[i];
                uint8_t upper_left = (i >= bpp) ? previous_scanline[i - bpp] : 0;
                scanline[i] += paeth_predictor(left, above, upper_left);
            }
            break;
        default:
            std::cerr << "Error: Unknown filter type " << static_cast<int>(filter_type) << std::endl;
            //exit(1);
    }
}

void reconstruct_data(PNGImage &image, std::vector<uint8_t> data) {
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t stride = image.width * bpp;
    image.pixels.clear();

    std::vector<uint8_t> previous_scanline(stride, 0);
    std::vector<uint8_t> current_scanline(stride);

    size_t pos = 0;
    for (uint32_t y = 0; y < image.height; ++y) {
        uint8_t filter_type = data[pos++];
        std::copy(data.begin() + pos, data.begin() + pos + stride, current_scanline.begin());
        pos += stride;
        #ifdef DEBUG_FILTERS
        std::cout << "Filter: " << (int) filter_type << std::endl;
        #endif

        unfilter_scanline(filter_type, current_scanline, previous_scanline, bpp);

        image.pixels.insert(image.pixels.end(), current_scanline.begin(), current_scanline.end());
        std::swap(previous_scanline, current_scanline);
    }
}

void reconstruct_image(PNGImage &image) {
    reconstruct_data(image, image.decompressed_data);
}

void save_as_ppm(const PNGImage &image, const std::string &filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        exit(1);
    }

    file << "P6\n" << image.width << " " << image.height << "\n255\n";

    size_t pixel_size = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
            size_t index = (y * image.width + x) * pixel_size;
            if (image.color_type == 6) {
                // RGBA to RGB
                file.put(image.pixels[index]);
                file.put(image.pixels[index + 1]);
                file.put(image.pixels[index + 2]);
            } else if (image.color_type == 2) {
                // RGB
                file.put(image.pixels[index]);
                file.put(image.pixels[index + 1]);
                file.put(image.pixels[index + 2]);
            } else if (image.color_type == 0) {
                // Grayscale
                file.put(image.pixels[index]);
                file.put(image.pixels[index]);
                file.put(image.pixels[index]);
            }
            // Additional color types can be handled similarly
        }
    }
}

void write_raw_to_file(const std::vector<uint8_t>& idat_data, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        exit(1);
    }
    file.write(reinterpret_cast<const char*>(idat_data.data()), idat_data.size());
}






/*


BIG LOGIC HERE

*/

std::vector<uncertainByte> inflatePacket(DeflatePacket &packet, std::vector<uint8_t>& compressed_data) {

    // Process compressed data blocks
    Bitstream bitstream(compressed_data);
    bitstream.set_pos(packet.start_position);
    std::vector<uncertainByte> decompressed_data;
    bool last_block = bitstream.read_bits(1);
    uint32_t block_type = bitstream.read_bits(2);
    if (block_type != 2) {
        // Uncompressed block (not implemented in this example)
        throw std::runtime_error("Uncompressed blocks are not supported in this example");
    } 
    uint16_t hlit = bitstream.read_bits(5) + 257;
    uint16_t hdist = bitstream.read_bits(5) + 1;
    uint16_t hclen = bitstream.read_bits(4) + 4;

    
    packet.code_length_table = load_code_lengths(&bitstream, hclen);
    if(!verifyLengths(packet.code_length_table.lengths)){
        throw std::runtime_error("Bad lengths in code length!");
        
    }
    packet.literal_table = load_literal_lengths(bitstream, &packet.code_length_table, hlit);
    packet.distance_table = load_distance_lengths(bitstream,&packet.code_length_table, hdist);
    
    while (true) {
        if(bitstream.finished()){
            break;
        }
        uint16_t symbol = packet.literal_table.decode(bitstream);
        packet.literal_map[symbol]++;
        #ifdef DEBUG_INFLATE
        std::cout << "Symbol " << std::hex << (int) symbol << std::dec << std::endl;
        #endif
        if (symbol < 256) {
            #ifdef DEBUG_INFLATE
            std::cout << "Hit literal " << std::hex << (int) symbol << std::dec << std::endl;
            #endif
            decompressed_data.push_back(uncertainByte(symbol, 0, true));
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

            uint16_t dist_symbol = packet.distance_table.decode(bitstream);
            packet.distance_map[dist_symbol]++;
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
            for (uint16_t i = 0; i < length; ++i) {
                decompressed_data.push_back(uncertainByte(0, -distance, true));
            }
        }
    }
    return decompressed_data;
}

float scoreTable(HuffmanTable &table, std::unordered_map<uint16_t, size_t> &frequency){
    
    std::vector<std::pair<uint16_t, size_t>> len_freq_pairs;

    // Combine lengths and frequencies into a vector of pairs
    for (size_t n = 0; n < table.lengths.size(); ++n) {
        uint16_t len = table.lengths[n];
        size_t freq = frequency[n];
        if(len > 0 && freq > 0){
            len_freq_pairs.emplace_back(len, freq);
        }
    }

    // Sort the vector by ascending frequency
    std::sort(len_freq_pairs.begin(), len_freq_pairs.end(),
[](const std::pair<uint16_t, size_t> &a, const std::pair<uint16_t, size_t> &b) {
        if(a.second == b.second){
            return a.first > b.first;
        }
        return a.second < b.second;
    });
    float score = 0.0;
    for (size_t n = 0; n < len_freq_pairs.size()-1; n++) {
        //std::cout << std::dec << len_freq_pairs[n].first << " : " << len_freq_pairs[n].second << std::endl;
        if(len_freq_pairs[n].first >= len_freq_pairs[n+1].first){
            //std::cout << "scored! " << score << std::endl;
            score += 1.0;
        } else {
            //puts("MISS!");
        }
    }

    return score/(len_freq_pairs.size()-1.0);
    
}

float scorePacket(DeflatePacket &packet){
    float literal_score = scoreTable(packet.literal_table, packet.literal_map);
    float distance_score = scoreTable(packet.literal_table, packet.literal_map);
    //std::cout << "Literally: " << literal_score << std::endl;
    //std::cout << "Distance: " << distance_score << std::endl;
    return (literal_score + distance_score) / 2.0;
}

void mark_damaged_packets(PNGImage &image){
    uint32_t offset = 0;
    for(auto chunk: image.chunks){
        if(strcmp(chunk.type, "IDAT")==0){
            uint32_t end = offset + chunk.length;
            if(chunk.computed_crc != chunk.crc){
                for(auto &packet : image.packets){
                    if((packet.end_position / 8) > offset && (packet.start_position / 8) < end){
                        packet.safe = false;
                        std::cout << "Unsafe!" << std::endl;
                    }
                }
            }
            offset = end;
        }
    }
}

static inline int64_t abs(int64_t x){
    if(x < 0){
        return -x;
    } else {
        return x;
    }
}
static inline int64_t square(int64_t x){
    return x*x;
}
static inline int64_t diff(uint8_t a, uint8_t b){
    return square(((int64_t) a) - ((int64_t) b))/256;
}

std::vector<uint8_t> make_certain(std::vector<uncertainByte> data){
    std::vector<uint8_t> output;
    for(int n=0; n<data.size(); n++){
        if(data[n].reference != 0){
            data[n].value=data[n+data[n].reference].value;
            data[n].found=data[n+data[n].reference].found;
        }
    }
    for(auto byte: data){
        output.push_back(byte.value);
    }
    return output;
}

// Function to calculate Euclidean distance between two byte vectors
double euclidean_distance(const std::vector<uint8_t>& vec1, const std::vector<uint8_t>& vec2) {
    if (vec1.size() != vec2.size()) {
        std::cerr << "Error: Vectors must be of the same length to calculate Euclidean distance." << std::endl;
        exit(1);
    }

    double sum = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        double diff = static_cast<double>(vec1[i]) - static_cast<double>(vec2[i]);
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

static inline double scoreUncertainty(PNGImage &image, std::vector<uncertainByte> &uncertain_data){
    std::vector<uint8_t> data = make_certain(uncertain_data);
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t real_stride = 1+image.width * bpp;
    uint32_t total_size = real_stride*image.height;
    
    int64_t score = 0;
    //std::cout << image.height << " by " << real_stride  << std::endl;
    
    uint32_t stride = image.width * bpp;

    std::vector<uint8_t> old_scanline(stride, 0);
    std::vector<uint8_t> previous_scanline(stride, 0);
    std::vector<uint8_t> current_scanline(stride);
    std::vector<double> scores;
    double total_score;
    size_t pos = 0;
    for (uint32_t y = 0; y < image.height; ++y) {
        uint8_t filter_type = data[pos++];
        std::copy(data.begin() + pos, data.begin() + pos + stride, current_scanline.begin());
        pos += stride;

        unfilter_scanline(filter_type, current_scanline, previous_scanline, bpp);
        if(y>1){
            scores.push_back(euclidean_distance(previous_scanline, current_scanline));
            total_score += scores.back();
        }
        old_scanline = previous_scanline;
        previous_scanline = current_scanline;
    }
    double mean = total_score / scores.size();
    double real_score = 0;
    //std::cout << "START-----------------------" << std::endl;
    for(double val : scores){
        //std::cout << val << " - " << mean << std::endl;
        real_score += std::abs(val-mean);
        //std::cout << real_score << std::endl;
    }
    //std::cout << "END--------------" << std::endl;
    return real_score;
    return score;
}

void getGuessable(PNGImage &image, std::vector<uncertainByte> &data, std::vector<std::pair<size_t, size_t>> &guessable,     std::unordered_map<size_t, size_t> &counts, std::unordered_map<size_t, std::vector<size_t>> &links){
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t real_stride = 1+image.width * bpp;
    uint32_t total_size = real_stride*image.height;
    for(int y=image.height-1; y>=0; y--){
        //std::cout << y  << std::endl;
        for(int x=real_stride-1; x>0; x--){
            size_t byte_location = real_stride*y+x;
            if(!data[byte_location].found && data[byte_location].reference == 0){
                guessable.push_back(std::pair<size_t, size_t>(byte_location, counts[byte_location]));
                links[byte_location].push_back(byte_location);
            }
            if(!data[byte_location].found && data[byte_location].reference != 0){
                counts[byte_location+data[byte_location].reference]++;
                links[byte_location+data[byte_location].reference].push_back(byte_location);
            }
        }
    }
    std::sort(guessable.begin(), guessable.end(),
[](const std::pair<size_t, size_t> &a, const std::pair<size_t, size_t> &b) {
        if(a.second == b.second){
            return a.first > b.first;
        }
        return a.second > b.second;
    });
}
void medianUncertain(PNGImage &image, std::vector<uncertainByte> &data){
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t real_stride = 1+image.width * bpp;
    uint32_t total_size = real_stride*image.height;
    std::vector<std::pair<size_t, size_t>> guessable;
    std::unordered_map<size_t, size_t> counts;
    std::unordered_map<size_t, std::vector<size_t>> links;
    
    getGuessable(image, data, guessable, counts, links);
    std::cout << "Smoothing over medians for " << std::dec << guessable.size() << " different unknown bytes and " << counts.size() << " have references!" << std::endl;
    uint8_t scale = 128;
    for(int loops=0; loops<5; loops++){
        int medianned = 0;
        puts("LOOP!");
        for(auto index: guessable){
            medianned++;
            if(medianned > MEDIAN_PASSES){
                break;
            }
            //std::cout << medianned << "/" << guessable.size() << std::endl;
            std::vector<int8_t> neighbours;
            for(auto n : links[index.first]){
                int y = n / real_stride;
                int x = (n - (y*real_stride)-1);
                int pixel_base = y*real_stride + x + 1 - (x%bpp);
                neighbours.push_back(data[pixel_base + (0*bpp)/3].value);
                neighbours.push_back(data[pixel_base + (1*bpp)/3].value);
                neighbours.push_back(data[pixel_base + (2*bpp)/3].value);
                if(n%real_stride>bpp){
                    int loc = n - bpp;
                    if(data[loc].found || loops >= 4){
                        neighbours.push_back(data[loc+data[loc].reference].value);
                    }
                }
                if(n%real_stride<real_stride-bpp){
                    int loc = n + bpp;
                    if(data[loc].found || loops >= 2){
                        neighbours.push_back(data[loc+data[loc].reference].value);
                    }
                }
                if(n > real_stride){
                    int loc = n-real_stride;
                    if(data[loc].found || loops >= 3){
                        neighbours.push_back(data[loc+data[loc].reference].value);
                    }
                }
                if(n < total_size- real_stride-1){
                    int loc = n+real_stride;
                    if(data[loc].found || loops >= 1){
                        neighbours.push_back(data[loc+data[loc].reference].value);
                    }
                }
            }
            if(neighbours.size() == 0){
                continue;
            }
            const auto middle = neighbours.begin() + neighbours.size() / 2;
            std::nth_element(neighbours.begin(), middle, neighbours.end());
            uint8_t median = *middle;
            int32_t sum = 0.0;
            for(auto val : neighbours){
                sum += val;
            }
            int32_t meaned = sum / neighbours.size();
            //std::cout << sum << " : " << meaned << std::endl;
            uint8_t mean = mean;
            //std::cout << "loop # " << loops << " on " << index.first << "# from " << (int) data[index.first].value << " : " << (int) median << " with " << neighbours.size() << " borders! @" << medianned << "/" << guessable.size()  << std::endl;
            data[index.first].value = median;
        }
    }
}


void smoothUncertainty(PNGImage &image, std::vector<uncertainByte> &data){
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t real_stride = 1+image.width * bpp;
    uint32_t total_size = real_stride*image.height;
    std::vector<std::pair<size_t, size_t>> guessable;
    std::unordered_map<size_t, size_t> counts;
    std::unordered_map<size_t, std::vector<size_t>> links;
    getGuessable(image, data, guessable, counts, links);
    std::cout << "Smoothing over " << std::dec << guessable.size() << " different unknown bytes and " << counts.size() << " have references!" << std::endl;
    
    reconstruct_data(image, make_certain(data));
    save_as_ppm(image, "pre_median.ppm");
    
    medianUncertain(image, data);
    reconstruct_data(image, make_certain(data));
    save_as_ppm(image, "post_median.ppm");
    int smooth_ppms = 0;
    for(int loop=0; loop<SMOOTH_LOOPS; loop++){
        auto current = scoreUncertainty(image, data);
        int processed = 0;
        for(auto index: guessable){
            uint8_t scale = 128;
            while(scale > 0){
                std::cout << "Value: "<< current << std::endl;
                std::cout << processed << "/" << guessable.size() << std::endl;
                for(int attempt=-scale; attempt <= scale; attempt+=scale*2){
                    uint8_t original = data[index.first].value;
                    data[index.first].value += attempt;
                    auto result = scoreUncertainty(image, data);
                    if(result < current || current != current){
                        std::cout << result << " < " << current << std::endl;
                        current = result;
                        std::cout << "Improved! " << current << " at scale " << (int) scale << " loop # " << loop << std::endl;
                        reconstruct_data(image, make_certain(data));
                        save_as_ppm(image, "post_smooth_" + std::to_string(++smooth_ppms) + ".ppm");
                    } else {
                        data[index.first].value = original;
                    }
                }
                scale /= 2;
            }
            processed++;
            if(processed > SMOOTH_PASSES){
                break;
            }
        }
    }
    
}

std::vector<uncertainByte> get_uncertain_bytes(PNGImage &image){
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t real_stride = 1+image.width * bpp;
    uint32_t total_size = real_stride*image.height;
    std::vector<uncertainByte> data;
    for(int n=0; n<total_size; n++){
        data.push_back(uncertainByte(0, 0, false));
    }
    int up_offset = 0;
    for(int n=0; n<image.packets.size(); n++){
        if(!image.packets[n].safe){
            /*std::vector<uncertainByte> loaded = inflatePacket(image.packets[n], image.image_data);
            int good = 0;
            for(; good<loaded.size(); good++){
                if((good+up_offset) % real_stride == 0){
                    if(loaded[good].reference < -good){
                        loaded[good].value = data[up_offset+good+loaded[good].reference].value;
                    } else {
                        loaded[good].value = loaded[good+loaded[good].reference].value;
                    }
                    if(loaded[good].value != data[up_offset-(up_offset%real_stride)].value && loaded[good].value != data[up_offset-(up_offset%real_stride)-real_stride].value){
                        std::cout << "BAD!" << std::endl;
                        break;
                    }
                }
            }
            std::cout << "good?" << std::endl;
            std::cout << good << std::endl;
            int safety = 3 * real_stride;
            for(int i=0; i<good-safety; i++){
                data[up_offset] = loaded[i];
                up_offset++;
            }*/
            break;
        } else {
            std::vector<uncertainByte> loaded = inflatePacket(image.packets[n], image.image_data);
            for(int i=0; i<loaded.size(); i++){
                data[up_offset] = loaded[i];
                data[up_offset].value = data[up_offset+data[up_offset].reference].value;
                up_offset++;
            }
        }
    }
    int down_offset = total_size-1;
    for(int n=image.packets.size()-1; n>0; n--){
        if(!image.packets[n].safe){
            /*
            std::vector<uncertainByte> loaded = inflatePacket(image.packets[n], image.image_data);
            
            int good = loaded.size()-1;
            for(; good>=0; good--){
                if((total_size-good+down_offset) % real_stride == 0){
                    if(loaded[good].reference < -good){
                        loaded[good].value = data[total_size-good+down_offset+loaded[good].reference].value;
                    } else {
                        loaded[good].value = loaded[good+loaded[good].reference].value;
                    }
                    if(loaded[good].value != data[up_offset-(up_offset%real_stride)].value && loaded[good].value != data[up_offset-(up_offset%real_stride)-real_stride].value){
                        std::cout << "BAD!" << std::endl;
                        break;
                    }
                }
            }
            std::cout << "good?" << std::endl;
            std::cout << good << std::endl;
            int safety = 3 * real_stride;
            for(int i=1; i<=good-safety; i++){
                if(data[down_offset-i].found){
                    std::cout << "Stopping backfill" << std::endl;
                    break;
                }
                data[down_offset-i] = loaded[loaded.size()-i];
                data[down_offset-i].found = false;
                data[down_offset-i].testing = true;
            }*/
            break;
        }
        std::vector<uncertainByte> loaded = inflatePacket(image.packets[n], image.image_data);
        for(int i=0; i<loaded.size(); i++){
            data[down_offset] = loaded[loaded.size()-1-i];
            down_offset--;
        }
    }
    for(int n=0; n<total_size; n++){
        if(data[n].reference != 0 && !data[n].testing){
            data[n].value=data[n+data[n].reference].value;
            data[n].found=data[n+data[n].reference].found;
            if(data[n+data[n].reference].reference != 0){
                if(data[n+data[n].reference].reference + data[n].reference < -n){
                    throw std::runtime_error("Bounds problem!");
                }
                data[n].reference += data[n+data[n].reference].reference;
            }
        }
    }
    
    for(int n=0; n<total_size; n+=real_stride){
        if(!data[n].found){
            data[n].value=4;
            data[n].found=true;
            data[n].reference=0;
        }
        std::cout << n << " : " << (int) data[n].value << " found " << (int) data[n].found << std::endl;
    }
    std::cout << "Scoring!" << std::endl;
    std::cout << "Current score: " << std::dec << scoreUncertainty(image, data) << std::endl;
    smoothUncertainty(image, data);
    std::cout << "End score: " << std::dec << scoreUncertainty(image, data) << std::endl;
    return data;
}
void recover(PNGImage &image){
    if(image.chunks.size() <= 0){
        throw std::runtime_error("No chunks to recover from!");
    }
    if(strcmp(image.chunks.back().type, "IEND") != 0){
        throw std::runtime_error("End is cut off, no guaranteed last chunk");
    }
    image.image_data = build_image_data(image.chunks);
    
    image.packets = header_scan(image.image_data);
    mark_damaged_packets(image);
    std::cout << "Recovered " << image.packets.size() << " packets!" << std::endl;
    print_packet_details(image.packets);
    std::vector<uncertainByte> data = get_uncertain_bytes(image);
    reconstruct_data(image, make_certain(data));
    /*
    for (auto &packet : packets) {
        std::vector<uncertainByte> result = inflatePacket(packet, image.image_data);
        std::cout << "Packet Scores:\n" <<std::hex;
        std::cout << "  Start Position: " << packet.start_position << "\n";
        std::cout << "  Frequency Score: " << scorePacket(packet) << std::endl;
    }*/
    
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image.png> [options]\n";
        std::cerr << "Options:\n";
        std::cerr << "  -o <filename>     Output PPM file\n";
        std::cerr << "  -p                Perfect(ish) recovery\n";
        std::cerr << "  -s                Generate zlib statistics\n";
        std::cerr << "  -c <filename>     Output compressed data to file\n";
        std::cerr << "  -r <filename>     Output recovered image to PPM file\n";
        return 1;
    }

    std::string filename = argv[1];
    std::string output_ppm = "out.ppm";
    std::string compressed_output = "compressed.bin";
    std::string repair_output = "fixed.ppm";
    bool generate_statistics = false;
    bool output_ppm_flag = false;
    bool repair_flag = false;
    bool output_compressed_flag = false;

    MEDIAN_PASSES = 10000000;
    SMOOTH_PASSES = 10;
    SMOOTH_LOOPS = 1;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            output_ppm = argv[++i];
            output_ppm_flag = true;
        } else if (arg == "-s") {
            generate_statistics = true;
        } else if (arg == "-p") {
            MEDIAN_PASSES = 10000000;
            SMOOTH_PASSES = 50;
            SMOOTH_LOOPS = 2;
        } else if (arg == "-c" && i + 1 < argc) {
            compressed_output = argv[++i];
            output_compressed_flag = true;
        }  else if (arg == "-r" && i + 1 < argc) {
            repair_output = argv[++i];
            repair_flag = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    PNGImage image = parse_png(filename);

    // Print PNG chunk details
    print_chunk_details(image.chunks);

    // Print image info
    print_image_info(image);

    // Write the raw IDAT data to a file if requested
    if (output_compressed_flag) {
        write_raw_to_file(image.image_data, compressed_output);
    }


    // Generate statistics if requested
    if (generate_statistics) {
        std::vector<DeflatePacket> packets = header_scan(image.image_data);
        std::cout << "Recovered " << packets.size() << " packets!" << std::endl;
        print_packet_details(packets);
    }
    // Generate statistics if requested
    if (repair_flag) {
        puts("Starting repair");
        std::cout << repair_output << std::endl;
        recover(image);
        save_as_ppm(image, repair_output);
    }

    // Reconstruct image and save as PPM if requested
    if (output_ppm_flag) {
        image.decompressed_data = custom_inflate(image.image_data);
        reconstruct_image(image);
        save_as_ppm(image, output_ppm);
    }

    return 0;
}


/*
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image.png>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    PNGImage image = parse_png(filename);
    //print_chunk_details(image.chunks);
    print_image_info(image);
    write_raw_to_file(image.image_data, "compressed.bin");

    image.decompressed_data = decompress_idat_data(image.image_data);
    puts("Decompressed!");
    write_raw_to_file(image.decompressed_data, "uncompressed.bin");

    reconstruct_image(image);
    
    save_as_ppm(image, "out.ppm");

    return 0;
}*/
