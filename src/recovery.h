#ifndef RECOVERY_H
#define RECOVERY_H

#include <vector>
#include <cstdint>
#include <set>
#include <memory>
#include "png.h"

enum RecoveryByteType { fixed_byte, unknown_byte, restricted_byte, reference_byte };

#define RecoveryPacket DeflatePacket

class RecoveryByte {
public:
    uint8_t value;
    RecoveryByteType variant;
    std::vector<uint8_t> possible;
    std::vector<RecoveryByte*> reliant_bytes;
    RecoveryByte* referenced;

    RecoveryByte();
    RecoveryByte(uint8_t val);
};

class Pixel {
public:
    uint16_t r, g, b, a;
    Pixel(uint16_t r = 0, uint16_t g = 0, uint16_t b = 0, uint16_t a = 0);

    uint16_t hue();    
    uint16_t saturation();    
    uint16_t luminosity();    
    uint16_t alpha();
};

enum ScanlineType { ScanlineNone = 0, ScanlineSub = 1, ScanlineUp = 2, ScanlineAvg = 3, ScanlinePaeth = 4, ScanlineError = 5 };

class RecoveryImage;  // Forward declaration

class Heuristic {
public:
    virtual ~Heuristic() = default;
    virtual void update_pixel(RecoveryImage& img, uint32_t x, uint32_t y) = 0;
    virtual uint64_t score(const RecoveryImage& img) const = 0;
};

class EuclideanRowDistanceHeuristic : public Heuristic {
public:
    std::vector<double> row_distances;
    double mean_distance;

    EuclideanRowDistanceHeuristic();
    void calculate_initial_distances(const RecoveryImage& img);
    void update_pixel(RecoveryImage& img, uint32_t x, uint32_t y) override;
    uint64_t score(const RecoveryImage& img) const override;

private:
    double calculate_row_distance(const RecoveryImage& img, uint32_t y1, uint32_t y2) const;
};

class RecoveryImage {
public:
    uint32_t width, height;
    uint8_t bit_depth, color_type, compression_method, filter_method, interlace_method;
    uint64_t score;
    uint32_t bpp, stride;
    PNGImage base;
    std::vector<PNGChunk> chunks;
    std::vector<RecoveryPacket> packets;
    RecoveryByte* raw_data;
    uint8_t* scan_data;
    Pixel* pixels;
    std::vector<std::unique_ptr<Heuristic>> heuristics;
    std::set<uint32_t> scan_updates;
    std::set<uint32_t> pixel_updates;

    RecoveryImage(const std::string& filename);
    ~RecoveryImage();
    void initialize_raw(); // Function to initialize raw values from DeflatePackets
    void save_to_ppm(const std::string& filename); // Function to save the image to a PPM file
    void initialize_heuristics();
    void recalculate_heuristics();
    void modify_pixel(uint32_t offset, Pixel val);
    uint64_t total_score() const;
    void simplify_backreferences();
    ScanlineType get_scanline_type(uint32_t location);
    void modify_raw(uint32_t offset, uint8_t val);
    void flag_modified_raw(uint32_t offset);
    void modify_scanline(uint32_t offset, uint8_t val);
    void flag_modified_scanline(uint32_t offset);
    void update_scanlines();
    void update_pixels();
    void flag_all_for_update(); // Flag all scanlines and pixels for updating
    void process_updates();     // Process all flagged scanline and pixel updates
    void print_filter_statistics() const; // Print statistics on PNG filters used


private:
    void apply_filter_none(uint32_t scanline_index, uint32_t byte_offset);
    void apply_filter_sub(uint32_t scanline_index, uint32_t byte_offset);
    void apply_filter_up(uint32_t scanline_index, uint32_t byte_offset);
    void apply_filter_avg(uint32_t scanline_index, uint32_t byte_offset);
    void apply_filter_paeth(uint32_t scanline_index, uint32_t byte_offset);
    uint8_t paeth_predict(uint8_t a, uint8_t b, uint8_t c);
    Pixel extract_pixel(uint32_t x, uint32_t y);
};

#endif // RECOVERY_H

