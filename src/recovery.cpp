#include "recovery.h"
#include "utility.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <iomanip>  // For formatted output
#include <fstream>
#include <cstring>

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
    std::vector<uint8_t> data = read_file_to_vector(filename);
    //TODO: Use this!
    this->chunks = scan_png_headers(data);
    print_recovery_chunks(this->chunks);
    this->load_packets();
    this->base = parse_png(filename);
    this->width = this->base.width;
    this->height = this->base.height;
    this->bit_depth = this->base.bit_depth;
    this->color_type = this->base.color_type;
    this->compression_method = this->base.compression_method;
    this->filter_method = this->base.filter_method;
    this->interlace_method = this->base.interlace_method;
    //this->chunks = this->base.chunks;
    this->bpp = (this->bit_depth / 8) * (this->color_type == 2 ? 3 : (this->color_type == 6 ? 4 : 1));
    this->stride = this->width * bpp;
    this->raw_data = new RecoveryByte[this->raw_data_size()];
    for(int i=0; i<this->raw_data_size();i++){
        this->raw_data[i].offset = i;
    }
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
    for (uint32_t i = 0; i < this->raw_data_size(); ++i) {
        if (this->raw_data[i].variant == reference_byte) {
            RecoveryByte* ref = this->raw_data[i].referenced;
            while (ref && ref->variant == reference_byte) {
                ref = ref->referenced;
            }
            if (ref) {
                this->raw_data[i].referenced = ref;
                ref->reliant_bytes.push_back(&this->raw_data[i]);
            } else {
                throw std::runtime_error("Reached a null ref!");
            }
            if(ref->variant == reference_byte){
                throw std::runtime_error("Reached a bad reference!");
            }
            if(this->raw_data[i].referenced->variant == reference_byte){
                throw std::runtime_error("Reached a 2nd degree bad reference!");
            }
        }
    }
}

ScanlineType RecoveryImage::get_scanline_type(uint32_t location) {
    uint32_t scanline_index = location / (this->stride + 1);
    uint8_t filter_type = this->raw_data[scanline_index * (this->stride + 1)].getValue();
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
    this->raw_data[offset].setValue(val);
    this->flag_modified_raw(offset);
    for(auto x: this->raw_data[offset].reliant_bytes){
        this->flag_modified_raw(x->offset);
    }
    
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
        if(byte_offset == 0){
            for(int i=0; i<this->stride+1; i++){
                scan_updates.insert(offset+i);
            }
            continue;
        }
    }
    for (const auto& offset : scan_updates) {
        uint32_t scanline_index = offset / (this->stride + 1);
        uint32_t byte_offset = offset % (this->stride + 1);
        if(byte_offset == 0){
            continue;
        }
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
                //throw std::runtime_error("Problem!");
                
                //std::cerr << "Invalid scanline type, defaulting to none!" << std::endl;
                apply_filter_none(scanline_index, byte_offset);
                break;
        }
        //printf("Updating scanline %d:%d for %d from %d!\n", scanline_index, byte_offset, (int) this->scan_data[scanline_index * this->stride + byte_offset], (int) this->raw_data[offset].value);
    }
    //TODO: Handle case for scanline type being changed
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
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].getValue();
}

void RecoveryImage::apply_filter_sub(uint32_t scanline_index, uint32_t byte_offset) {
    uint32_t prior_offset = (byte_offset >= this->bpp) ? byte_offset - this->bpp : 0;
    uint8_t prior_byte = (byte_offset >= this->bpp) ? scan_data[scanline_index * this->stride + prior_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].getValue() + prior_byte;
}

void RecoveryImage::apply_filter_up(uint32_t scanline_index, uint32_t byte_offset) {
    uint8_t prior_scanline_byte = (scanline_index > 0) ? scan_data[(scanline_index - 1) * this->stride + byte_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].getValue() + prior_scanline_byte;
}

void RecoveryImage::apply_filter_avg(uint32_t scanline_index, uint32_t byte_offset) {
    uint32_t prior_offset = (byte_offset >= this->bpp) ? byte_offset - this->bpp : 0;
    uint8_t prior_byte = (byte_offset >= this->bpp) ? scan_data[scanline_index * this->stride + prior_offset] : 0;
    uint8_t prior_scanline_byte = (scanline_index > 0) ? scan_data[(scanline_index - 1) * this->stride + byte_offset] : 0;
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].getValue() + (prior_byte + prior_scanline_byte) / 2;
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
    scan_data[scanline_index * this->stride + byte_offset] = raw_data[scanline_index * (this->stride + 1) + 1 + byte_offset].getValue() + paeth_predictor;
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


void RecoveryImage::load_packets() {
    std::cout << "Loading packets" << std::endl;
    std::vector<uint8_t> image_data;
    for (const auto &chunk : this->chunks) {
        if (chunk.chunk_type == "IDAT") {
            image_data.insert(image_data.end(), chunk.chunk_data.begin(), chunk.chunk_data.end());
        }
    }
    std::cout << "Loaded image data of " << image_data.size() << std::endl;
    this->packets = header_scan(image_data);
    for(auto &packet: packets){
        packet.symbolic_inflate();
    }
    uint32_t offset = 0;
    for (const auto &chunk : this->chunks) {
        if (chunk.chunk_type == "IDAT") {
            uint32_t end = offset + chunk.chunk_data.size();
            if(chunk.crc_corrupted){
                for(auto &packet : this->packets){
                    if((packet.end_position / 8) > offset && (packet.start_position / 8) < end){
                        packet.safe = false;
                    }
                }
            }
            offset = end;
        }
    }
    print_packet_details(this->packets);
    
}

size_t RecoveryImage::raw_data_size(){
    return (this->stride + 1) * this->height;
}

void RecoveryImage::initialize_smart(){
    // This function initializes the `raw_data` array from the Deflate packets.
    // Assuming that `packets` is a vector of `RecoveryPacket` objects,
    // and each `RecoveryPacket` contains a vector of bytes representing decompressed data.
    
    uint32_t positive_offset = 0;
    for (auto& packet: this->packets) {
        if(packet.safe){
            std::vector<LZToken> data = packet.symbolic_inflate();
            for(LZToken& token : data){
                //std::cout << token.to_string() << std::endl;
                if(token.distance == 0){
                    raw_data[positive_offset].value = token.character;
                    raw_data[positive_offset].variant = fixed_byte;
                    ++positive_offset;
                } else {
                    for(int i=0; i<token.length; i++){
                        raw_data[positive_offset].referenced = &raw_data[positive_offset-token.distance];
                        raw_data[positive_offset].variant = reference_byte;
                        ++positive_offset;
                    }
                }
            }
        } else {
            break;
        }
        if(positive_offset >= this->stride * this->height){
            std::cout << "Too far!" << std::endl;
            break;
        }
    }
    this->simplify_backreferences();
    this->flag_all_for_update();
    this->process_updates();
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
    for (uint32_t offset = 0; offset < this->raw_data_size(); offset++) {
        scan_updates.insert(offset);
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
        uint8_t filter_type = raw_data[i * (stride + 1)].getValue();
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

bool is_valid_chunk_type(const std::vector<uint8_t>& type) {
    static const std::set<std::string> valid_chunk_types = {
        "IHDR",  // Image header
        "PLTE",  // Palette table
        "IDAT",  // Image data
        "IEND",  // Image trailer
        "tRNS",  // Transparency information
        "gAMA",  // Gamma correction
        "cHRM",  // Primary chromaticities and white point
        "sRGB",  // Standard RGB color space
        "iCCP",  // Embedded ICC profile
        "tEXt",  // Textual data
        "zTXt",  // Compressed textual data
        "iTXt",  // International textual data
        "bKGD",  // Background color
        "pHYs",  // Physical pixel dimensions
        "sBIT",  // Significant bits
        "sPLT",  // Suggested palette
        "hIST",  // Image histogram
        "tIME",  // Image last-modification time
        "eXIf",  // Exchangeable image file format (Exif) data
        "oFFs"   // Image offset
    };

    std::string chunk_type(type.begin(), type.end());
    return valid_chunk_types.find(chunk_type) != valid_chunk_types.end();
}


// Function to convert bytes to a 32-bit unsigned integer (big-endian)
uint32_t bytes_to_uint32(const std::vector<uint8_t>& data, size_t start) {
    return (data[start] << 24) | (data[start + 1] << 16) | (data[start + 2] << 8) | data[start + 3];
}

std::vector<RecoveryChunk> scan_png_headers(std::vector<uint8_t>& data) {
    std::vector<RecoveryChunk> recovery_chunks;
    std::set<uint32_t> used_offsets;
    uint32_t offset = 8; // Skip the PNG header (8 bytes)

    while (offset < data.size()) {
        if (offset + 8 > data.size()) break; // Not enough data for a valid chunk header

        // Read the chunk length (4 bytes, big-endian)
        uint32_t chunk_length = (data[offset] << 24) |
                                (data[offset + 1] << 16) |
                                (data[offset + 2] << 8) |
                                (data[offset + 3]);

        // Read the chunk type (4 bytes)
        std::string chunk_type(reinterpret_cast<const char*>(&data[offset + 4]), 4);

        // Calculate chunk's end location
        uint32_t chunk_end = offset + 12 + chunk_length;

        // Check if the chunk length goes outside the bounds of the data
        bool length_out_of_bounds = (chunk_end > data.size());

        // Extract the chunk data and CRC
        std::vector<uint8_t> chunk_data(data.begin() + offset + 8, data.begin() + offset + 8 + chunk_length);
        uint32_t expected_crc = (data[chunk_end - 4] << 24) |
                                (data[chunk_end - 3] << 16) |
                                (data[chunk_end - 2] << 8) |
                                (data[chunk_end - 1]);

        // Calculate actual CRC using provided crc function
        unsigned long actual_crc = crc(reinterpret_cast<unsigned char*>(&data[offset + 4]), chunk_length + 4);

        // Check if the CRC is corrupted
        bool crc_corrupted = (expected_crc != actual_crc);

        // Determine if the next chunk is valid
        bool invalid_chunk_end = true;
        if (chunk_type == "IEND") {
            invalid_chunk_end = false;
        } else if (chunk_end < data.size()) {
            std::vector<uint8_t> next_chunk_type(data.begin() + chunk_end + 4, data.begin() + chunk_end + 4 + 4);
            invalid_chunk_end = !is_valid_chunk_type(next_chunk_type);
        }

        // Check for overlap with other chunks
        bool overlaps_other_chunks = false;
        for (uint32_t i = offset; i < chunk_end; ++i) {
            if (!used_offsets.insert(i).second) {
                overlaps_other_chunks = true;
                break;
            }
        }

        // Create a RecoveryChunk instance
        RecoveryChunk chunk{
            chunk_type,
            chunk_data,
            offset,
            crc_corrupted,
            invalid_chunk_end,
            overlaps_other_chunks,
            length_out_of_bounds
        };

        // Add the chunk to the vector
        recovery_chunks.push_back(chunk);

        // Move to the next chunk
        offset = chunk_end;
    }

    return recovery_chunks;
}


// Function to print details of a vector of RecoveryChunks
void print_recovery_chunks(const std::vector<RecoveryChunk>& chunks) {
    std::cout << std::setw(10) << "Type" 
              << std::setw(10) << "Start"
              << std::setw(10) << "Length"
              << std::setw(10) << "Corrupt" 
              << std::setw(10) << "Bad End"
              << std::setw(10) << "Overlap"
              << std::setw(10) << "Bad length"
              << std::endl;

    std::cout << std::string(70, '-') << std::endl;

    for (const auto& chunk : chunks) {
        std::cout << std::setw(10) << chunk.chunk_type
                  << std::setw(10) << chunk.start_location
                  << std::setw(10) << chunk.chunk_data.size()
                  << std::setw(10) << (chunk.crc_corrupted ? "Yes" : "No")
                  << std::setw(10) << (chunk.invalid_chunk_end ? "Yes" : "No")
                  << std::setw(10) << (chunk.overlaps_other_chunks ? "Yes" : "No")
                  << std::setw(10) << (chunk.length_out_of_bounds ? "Yes" : "No")
                  << std::endl;
    }
}
