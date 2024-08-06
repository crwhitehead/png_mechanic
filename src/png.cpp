#include "png.h"

#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <cstring>
#include <fstream>
#include "utility.h"
#include "deflate.h"

size_t MEDIAN_PASSES = 0;
size_t SMOOTH_PASSES = 0;
size_t SMOOTH_LOOPS = 0;
int smooth_ppms = 0;

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

