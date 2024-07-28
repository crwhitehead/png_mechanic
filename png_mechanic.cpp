/*

g++ png_mechanic.cpp -o png_mechanic -lz

*/


#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <zlib.h>

#undef DEBUG_CRC_INPUT

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

        if (strcmp(chunk.type, "IDAT") == 0) {
            image.image_data.insert(image.image_data.end(), chunk.data.begin(), chunk.data.end());
        }

        image.chunks.push_back(chunk);

        if (strcmp(chunk.type, "IEND") == 0)
            break;
    }

    if (!ihdr_parsed) {
        std::cerr << "Error: Missing IHDR chunk." << std::endl;
        exit(1);
    }

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

std::vector<uint8_t> decompress_idat_data(const std::vector<uint8_t> &compressed_data) {
    std::vector<uint8_t> decompressed_data;

    z_stream stream;
    memset(&stream, 0, sizeof(stream));
    inflateInit(&stream);

    stream.next_in = const_cast<Bytef *>(compressed_data.data());
    stream.avail_in = compressed_data.size();

    uint8_t buffer[4096];
    do {
        stream.next_out = buffer;
        stream.avail_out = sizeof(buffer);

        int res = inflate(&stream, Z_NO_FLUSH);
        if (res == Z_STREAM_ERROR || res == Z_DATA_ERROR || res == Z_MEM_ERROR) {
            std::cerr << "Error: Failed to decompress IDAT data." << std::endl;
            inflateEnd(&stream);
            //exit(1);
        }

        decompressed_data.insert(decompressed_data.end(), buffer, buffer + (sizeof(buffer) - stream.avail_out));
    } while (stream.avail_out == 0);

    inflateEnd(&stream);

    return decompressed_data;
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
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image.png>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    PNGImage image = parse_png(filename);
    print_chunk_details(image.chunks);
    print_image_info(image);

    image.decompressed_data = decompress_idat_data(image.image_data);

    reconstruct_image(image);

    save_as_ppm(image, "out.ppm");

    return 0;
}
