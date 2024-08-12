#include "recovery.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <fstream>

RecoveryByte::RecoveryByte() : value(0), variant(unknown_byte), referenced(nullptr) {}
RecoveryByte::RecoveryByte(uint8_t val) : value(val), variant(fixed_byte), referenced(nullptr) {}

Pixel::Pixel(uint16_t r, uint16_t g, uint16_t b, uint16_t a)
    : r(r), g(g), b(b), a(a) {}

// Define methods for hue, saturation, luminosity, and alpha as needed
uint16_t Pixel::hue() { /* Implementation */ return 0; }
uint16_t Pixel::saturation() { /* Implementation */ return 0; }
uint16_t Pixel::luminosity() { /* Implementation */ return 0; }
uint16_t Pixel::alpha() { /* Implementation */ return 0; }

EuclideanRowDistanceHeuristic::EuclideanRowDistanceHeuristic() : mean_distance(0.0) {}

void EuclideanRowDistanceHeuristic::calculate_initial_distances(const RecoveryImage& img) {
    row_distances.resize(img.height - 1);
    double total_distance = 0.0;

    for (uint32_t y = 0; y < img.height - 1; ++y) {
        row_distances[y] = calculate_row_distance(img, y, y + 1);
        total_distance += row_distances[y];
    }

    mean_distance = total_distance / (img.height - 1);
}

void EuclideanRowDistanceHeuristic::update_pixel(RecoveryImage& img, uint32_t x, uint32_t y) {
    if (y > 0) {
        double old_distance = row_distances[y - 1];
        double new_distance = calculate_row_distance(img, y - 1, y);
        mean_distance += (new_distance - old_distance) / (img.height - 1);
        row_distances[y - 1] = new_distance;
    }

    if (y < img.height - 1) {
        double old_distance = row_distances[y];
        double new_distance = calculate_row_distance(img, y, y + 1);
        mean_distance += (new_distance - old_distance) / (img.height - 1);
        row_distances[y] = new_distance;
    }
}

uint64_t EuclideanRowDistanceHeuristic::score(const RecoveryImage& img) const {
    double total_deviation = 0.0;

    for (const auto& distance : row_distances) {
        total_deviation += std::abs(distance - mean_distance);
    }

    return static_cast<uint64_t>(total_deviation);
}

double EuclideanRowDistanceHeuristic::calculate_row_distance(const RecoveryImage& img, uint32_t y1, uint32_t y2) const {
    double distance = 0.0;

    for (uint32_t x = 0; x < img.width; ++x) {
        const Pixel& p1 = img.pixels[y1 * img.width + x];
        const Pixel& p2 = img.pixels[y2 * img.width + x];
        distance += std::sqrt(
            std::pow(static_cast<double>(p1.r) - p2.r, 2.0) +
            std::pow(static_cast<double>(p1.g) - p2.g, 2.0) +
            std::pow(static_cast<double>(p1.b) - p2.b, 2.0)
        );
    }

    return distance;
}

RecoveryImage::RecoveryImage(const std::string& filename) {
    this->base = parse_png(filename);
    this->width = this->base.width;
    this->height = this->base.height;
    this->bit_depth = this->base.bit_depth;
    this->color_type = this->base.color_type;
    this->compression_method = this->base.compression_method;
    this->filter_method = this->base.filter_method;
    this->interlace_method = this->base.interlace_method;
    this->chunks = this->base.chunks;
    this->packets = this->base.packets;
    this->bpp = (this->bit_depth / 8) * (this->color_type == 2 ? 3 : (this->color_type == 6 ? 4 : 1));
    this->stride = this->width * bpp;
    this->raw_data = new RecoveryByte[(this->stride + 1) * this->height];
    this->scan_data = new uint8_t[(this->stride) * this->height];
    this->pixels = new Pixel[this->width * this->height];
}

RecoveryImage::~RecoveryImage() {
    delete[] this->raw_data;
    delete[] this->scan_data;
    delete[] this->pixels;
}

void RecoveryImage::initialize_heuristics() {
    heuristics.push_back(std::make_unique<EuclideanRowDistanceHeuristic>());
    dynamic_cast<EuclideanRowDistanceHeuristic*>(heuristics.back().get())->calculate_initial_distances(*this);
}

void RecoveryImage::recalculate_heuristics() {
    // Implement any scoring or heuristic recalculations based on updated data
}

void RecoveryImage::modify_pixel(uint32_t offset, Pixel val) {
    uint32_t y = offset / this->width;
    uint32_t x = offset % this->width;
    pixels[offset] = val;

    for (auto& heuristic : heuristics) {
        heuristic->update_pixel(*this, x, y);
    }
}

uint64_t RecoveryImage::total_score() const {
    uint64_t total = 0;
    for (const auto& heuristic : heuristics) {
        total += heuristic->score(*this);
    }
    return total;
}

void RecoveryImage::simplify_backreferences() {
    for (uint32_t i = 0; i < this->width * this->height; ++i) {
        if (this->raw_data[i].variant == reference_byte) {
            RecoveryByte* ref = this->raw_data[i].referenced;
            while (ref && ref->variant == reference_byte) {
                ref = ref->referenced;
            }
            if (ref) {
                this->raw_data[i].referenced = ref;
                ref->reliant_bytes.push_back(&this->raw_data[i]);
            }
        }
    }
}

ScanlineType RecoveryImage::get_scanline_type(uint32_t location) {
    uint32_t scanline_index = location / (this->stride + 1);
    uint8_t filter_type = this->raw_data[scanline_index * (this->stride + 1)].value;
    switch (filter_type) {
        case 0: return ScanlineNone;
        case 1: return ScanlineSub;
        case 2: return ScanlineUp;
        case 3: return ScanlineAvg;
        case 4: return ScanlinePaeth;
        default: return ScanlineError;
    }
}

void RecoveryImage::modify_raw(uint32_t offset, uint8_t val) {
    this->raw_data[offset].value = val;
}

void RecoveryImage::flag_modified_raw(uint32_t offset) {
    this->scan_updates.insert(offset);
}

void RecoveryImage::modify_scanline(uint32_t offset, uint8_t val) {
    this->scan_data[offset] = val;
}

void RecoveryImage::flag_modified_scanline(uint32_t offset) {
    this->pixel_updates.insert(offset);
}

void RecoveryImage::update_scanlines() {
    for (const auto& offset : scan_updates) {
        uint32_t scanline_index = offset / (this->stride + 1);
        uint32_t byte_offset = offset % (this->stride + 1);
        switch (get_scanline_type(offset)) {
            case ScanlineNone:
                apply_filter_none(scanline_index, byte_offset);
                break;
            case ScanlineSub:
                apply_filter_sub(scanline_index, byte_offset);
                break;
            case ScanlineUp:
                apply_filter_up(scanline_index, byte_offset);
                break;
            case ScanlineAvg:
                apply_filter_avg(scanline_index, byte_offset);
                break;
            case ScanlinePaeth:
                apply_filter_paeth(scanline_index, byte_offset);
                break;
            default:
                throw std::runtime_error("Problem!");
                apply_filter_none(scanline_index, byte_offset);
                break;
        }
        //printf("Updating scanline %d:%d for %d from %d!\n", scanline_index, byte_offset, (int) this->scan_data[scanline_index * this->stride + byte_offset], (int) this->raw_data[offset].value);
    }

    scan_updates.clear();
}

void RecoveryImage::update_pixels() {
    for (auto pixel_index : pixel_updates) {
        uint32_t y = pixel_index / this->width;
        uint32_t x = pixel_index % this->width;
        this->pixels[pixel_index] = extract_pixel(x, y);
        //printf("Extracted!\n");
    }
    pixel_updates.clear();
}
    
void RecoveryImage::apply_filter_none(uint32_t scanline_index, uint32_t byte_offset) {
    // No filter to apply, copy raw data to scanline data
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].value;
}

void RecoveryImage::apply_filter_sub(uint32_t scanline_index, uint32_t byte_offset) {
    uint32_t prior_offset = (byte_offset >= this->bpp) ? byte_offset - this->bpp : 0;
    uint8_t prior_byte = (byte_offset >= this->bpp) ? scan_data[scanline_index * this->stride + prior_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].value + prior_byte;
}

void RecoveryImage::apply_filter_up(uint32_t scanline_index, uint32_t byte_offset) {
    uint8_t prior_scanline_byte = (scanline_index > 0) ? scan_data[(scanline_index - 1) * this->stride + byte_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].value + prior_scanline_byte;
}

void RecoveryImage::apply_filter_avg(uint32_t scanline_index, uint32_t byte_offset) {
    uint32_t prior_offset = (byte_offset >= this->bpp) ? byte_offset - this->bpp : 0;
    uint8_t prior_byte = (byte_offset >= this->bpp) ? scan_data[scanline_index * this->stride + prior_offset] : 0;
    uint8_t prior_scanline_byte = (scanline_index > 0) ? scan_data[(scanline_index - 1) * this->stride + byte_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].value + (prior_byte + prior_scanline_byte) / 2;
}

uint8_t paeth_predict(uint8_t a, uint8_t b, uint8_t c) {
    int p = a + b - c;
    int pa = std::abs(p - a);
    int pb = std::abs(p - b);
    int pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    else if (pb <= pc) return b;
    else return c;
}
    

void RecoveryImage::apply_filter_paeth(uint32_t scanline_index, uint32_t byte_offset) {
    uint8_t prior_byte = (byte_offset >= this->bpp) ? scan_data[scanline_index * this->stride + byte_offset - this->bpp] : 0;
    uint8_t prior_scanline_byte = (scanline_index > 0) ? scan_data[(scanline_index - 1) * this->stride + byte_offset] : 0;
    uint8_t prior_diag_byte = (scanline_index > 0 && byte_offset >= this->bpp) ? scan_data[(scanline_index - 1) * this->stride + byte_offset - this->bpp] : 0;
    uint8_t paeth_predictor = paeth_predict(prior_byte, prior_scanline_byte, prior_diag_byte);
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].value + paeth_predictor;
}

uint8_t RecoveryImage::paeth_predict(uint8_t a, uint8_t b, uint8_t c) {
    int p = a + b - c;
    int pa = std::abs(p - a);
    int pb = std::abs(p - b);
    int pc = std::abs(p - c);

    if (pa <= pb && pa <= pc) return a;
    else if (pb <= pc) return b;
    else return c;
}

Pixel RecoveryImage::extract_pixel(uint32_t x, uint32_t y) {
    uint32_t offset = y * this->stride + x * this->bpp;
    Pixel pixel;
    if (this->color_type == 2) {  // RGB
        pixel.r = scan_data[offset];
        pixel.g = scan_data[offset + 1];
        pixel.b = scan_data[offset + 2];
    } else if (this->color_type == 6) {  // RGBA
        pixel.r = scan_data[offset];
        pixel.g = scan_data[offset + 1];
        pixel.b = scan_data[offset + 2];
        pixel.a = scan_data[offset + 3];
    } else {
        // Add support for other color types if needed
        throw std::runtime_error("Unsupported color type");
    }
    return pixel;
}

void RecoveryImage::initialize_raw() {
    // This function initializes the `raw_data` array from the Deflate packets.
    // Assuming that `packets` is a vector of `RecoveryPacket` objects,
    // and each `RecoveryPacket` contains a vector of bytes representing decompressed data.
    
    uint32_t offset = 0;
    //for (const auto& packet : packets) {
        for (const auto& byte : custom_inflate(this->base.image_data)) {
            if(offset >= this->stride * this->height){
                break;
            }
            raw_data[offset].value = byte;
            raw_data[offset].variant = fixed_byte;
            ++offset;
        }
    //}
    this->flag_all_for_update();
    this->process_updates();
}

void RecoveryImage::save_to_ppm(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing");
    }

    // Write PPM header
    file << "P6\n" << width << " " << height << "\n255\n";

    // Write pixel data
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            //printf("x %d y %d rgb %d,%d,%d\n", x, y, pixel.r, pixel.g, pixel.b);
            const Pixel& pixel = pixels[y * width + x];
            uint8_t r = static_cast<uint8_t>(pixel.r);
            uint8_t g = static_cast<uint8_t>(pixel.g);
            uint8_t b = static_cast<uint8_t>(pixel.b);
            file.write(reinterpret_cast<char*>(&r), 1);
            file.write(reinterpret_cast<char*>(&g), 1);
            file.write(reinterpret_cast<char*>(&b), 1);
        }
    }

    file.close();
}

void RecoveryImage::flag_all_for_update() {
    // Flag all scanlines for updating
    for (uint32_t scanline = 0; scanline < height; ++scanline) {
        for (uint32_t byte_offset = 0; byte_offset < stride + 1; ++byte_offset) {
            uint32_t offset = scanline * (stride + 1) + byte_offset;
            scan_updates.insert(offset);
        }
    }

    // Flag all pixels for updating
    for (uint32_t pixel = 0; pixel < width * height; ++pixel) {
        pixel_updates.insert(pixel);
    }
}

void RecoveryImage::process_updates() {
    // Process all scanline updates
    update_scanlines();

    // Process all pixel updates
    update_pixels();
}

void RecoveryImage::print_filter_statistics() const {
    // Count occurrences of each filter type
    std::vector<uint32_t> filter_counts(5, 0); // Index 0: None, 1: Sub, 2: Up, 3: Avg, 4: Paeth

    // Iterate through raw data to count filter usage
    for (uint32_t i = 0; i < height; ++i) {
        uint8_t filter_type = raw_data[i * (stride + 1)].value;
        if (filter_type >= 0 && filter_type <= 4) {
            filter_counts[filter_type]++;
        }
    }

    // Print the filter statistics
    std::cout << "PNG Filter Statistics:\n";
    std::cout << "Filter None: " << filter_counts[ScanlineNone] << "\n";
    std::cout << "Filter Sub: " << filter_counts[ScanlineSub] << "\n";
    std::cout << "Filter Up: " << filter_counts[ScanlineUp] << "\n";
    std::cout << "Filter Avg: " << filter_counts[ScanlineAvg] << "\n";
    std::cout << "Filter Paeth: " << filter_counts[ScanlinePaeth] << "\n";
}

