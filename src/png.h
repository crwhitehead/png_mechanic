#ifndef PNG_H
#define PNG_H

#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include "utility.h"
#include "deflate.h"

extern size_t MEDIAN_PASSES;
extern size_t SMOOTH_PASSES;
extern size_t SMOOTH_LOOPS;
extern int smooth_ppms;


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

std::vector<uint8_t> build_image_data(const std::vector<PNGChunk> &chunks);

PNGImage parse_png(const std::string &filename);

void print_chunk_details(const std::vector<PNGChunk> &chunks);

void print_image_info(const PNGImage &image);

uint8_t paeth_predictor(uint8_t a, uint8_t b, uint8_t c);

void unfilter_scanline(uint8_t filter_type, std::vector<uint8_t> &scanline, const std::vector<uint8_t> &previous_scanline, uint32_t bpp);

void reconstruct_data(PNGImage &image, std::vector<uint8_t> data);

void reconstruct_image(PNGImage &image);

void save_as_ppm(const PNGImage &image, const std::string &filename);

#endif
