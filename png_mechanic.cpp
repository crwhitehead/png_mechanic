/*

g++ png_mechanic.cpp -o png_mechanic

*/


#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <zlib.h>
#include <bitset>
#include <map>

#undef DEBUG_CRC_INPUT
#undef DEBUG_INFLATE
#undef DEBUG_FILTERS
#define DEBUG_PACKET_LOCATIONS 1


struct extractedByte {
    uint8_t value;
    int32_t reference;
    bool found;
    bool testing;
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
    std::vector<uint8_t> image_data;
    std::vector<uint8_t> decompressed_data;
    std::vector<uint8_t> pixels;
};

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


struct DeflatePacket {
    size_t start_position;
    size_t header_start;
    size_t data_start;
    size_t end_position;
    size_t block_type;
    bool last;
    Bitstream* bitstream;
    
    DeflatePacket() = default;
};

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

HuffmanTable load_code_lengths(Bitstream* bitstream, uint16_t hclen){
    std::vector<uint16_t> code_length_order = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    std::vector<uint16_t> code_lengths(19, 0);
    for (size_t i = 0; i < hclen; ++i) {
        code_lengths[code_length_order[i]] = bitstream->read_bits(3);
    }

    HuffmanTable code_length_table(code_lengths);
    return code_length_table;
}
HuffmanTable load_literal_lengths(Bitstream& bitstream, const HuffmanTable* code_length_table, uint16_t hlit){
    std::vector<uint16_t> literal_lengths;

    while (literal_lengths.size() < hlit) {
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
        DeflatePacket test_packet;
        test_packet.start_position = i;
        test_packet.last = 1 == bitstream.read_bits(1);
        test_packet.block_type = bitstream.read_bits(2);
        test_packet.header_start = bitstream.bit_pos;
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
            throw std::runtime_error("Uncompressed blocks are not supported in this example");
        } else if (block_type == 1) {
            // Fixed Huffman block (not implemented in this example)
            throw std::runtime_error("Fixed Huffman blocks are not supported in this example");
        }  else if (block_type == 2) {
            // Dynamic Huffman block
            uint16_t hlit = bitstream.read_bits(5) + 257;
            uint16_t hdist = bitstream.read_bits(5) + 1;
            uint16_t hclen = bitstream.read_bits(4) + 4;

            std::vector<uint16_t> code_length_order = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
            std::vector<uint16_t> code_lengths(19, 0);
            for (size_t i = 0; i < hclen; ++i) {
                code_lengths[code_length_order[i]] = bitstream.read_bits(3);
            }

            HuffmanTable code_length_table(code_lengths);
            std::vector<uint16_t> literal_lengths;
            std::vector<uint16_t> distance_lengths;
            
            while (literal_lengths.size() < hlit) {
                uint16_t symbol = code_length_table.decode(bitstream);
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

            while (distance_lengths.size() < hdist) {
                uint16_t symbol = code_length_table.decode(bitstream);
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


            //verifyLengths(code_lengths);
            //verifyLengths(literal_lengths);
            //verifyLengths(distance_lengths);
            HuffmanTable literal_table(literal_lengths);
            HuffmanTable distance_table(distance_lengths);
            while (true) {
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
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        exit(1);
    }

    // Read and validate PNG signature
    uint8_t png_signature[8];
    file.read(reinterpret_cast<char *>(png_signature), 8);
    if (memcmp(png_signature, "\x89PNG\r\n\x1a\n", 8) != 0) {
        std::cerr << "Error: Not a valid PNG file." << std::endl;
        exit(1);
    }

    PNGImage image;
    bool ihdr_parsed = false;

    while (true) {
        PNGChunk chunk;

        // Read chunk length
        uint8_t length_bytes[4];
        file.read(reinterpret_cast<char *>(length_bytes), 4);
        chunk.length = (length_bytes[0] << 24) | (length_bytes[1] << 16) | (length_bytes[2] << 8) | length_bytes[3];

        // Read chunk type
        file.read(chunk.type, 4);
        chunk.type[4] = '\0';

        // Read chunk data
        chunk.data.resize(chunk.length);
        file.read(reinterpret_cast<char *>(chunk.data.data()), chunk.length);

        // Read chunk CRC
        uint8_t crc_bytes[4];
        file.read(reinterpret_cast<char *>(crc_bytes), 4);
        chunk.crc = (crc_bytes[0] << 24) | (crc_bytes[1] << 16) | (crc_bytes[2] << 8) | crc_bytes[3];

        // Verify CRC
        uint32_t computed_crc = crc(reinterpret_cast<uint8_t *>(chunk.type), 4);
        computed_crc = update_crc(computed_crc, chunk.data.data(), chunk.length);
        if (computed_crc != chunk.crc) {
            std::cerr << "Error: CRC mismatch in chunk " << chunk.type << std::endl;
            //exit(1);
        }

        if (strcmp(chunk.type, "IHDR") == 0) {
            if (chunk.length != 13) {
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

void reconstruct_image(PNGImage &image) {
    uint32_t bpp = (image.bit_depth / 8) * (image.color_type == 2 ? 3 : (image.color_type == 6 ? 4 : 1));
    uint32_t stride = image.width * bpp;

    std::vector<uint8_t> previous_scanline(stride, 0);
    std::vector<uint8_t> current_scanline(stride);

    size_t pos = 0;
    for (uint32_t y = 0; y < image.height; ++y) {
        uint8_t filter_type = image.decompressed_data[pos++];
        std::copy(image.decompressed_data.begin() + pos, image.decompressed_data.begin() + pos + stride, current_scanline.begin());
        pos += stride;
        #ifdef DEBUG_FILTERS
        std::cout << "Filter: " << (int) filter_type << std::endl;
        #endif

        unfilter_scanline(filter_type, current_scanline, previous_scanline, bpp);

        image.pixels.insert(image.pixels.end(), current_scanline.begin(), current_scanline.end());
        std::swap(previous_scanline, current_scanline);
    }
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
    
    std::vector<DeflatePacket> packets = header_scan(image.image_data);
    std::cout << "Recovered " << packets.size() << " packets!" << std::endl;
    print_packet_details(packets);
    save_as_ppm(image, "out.ppm");

    return 0;
}
