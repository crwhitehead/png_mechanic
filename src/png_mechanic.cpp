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


#include "png_mechanic.h"


#include "utility.h"
#include "png.h"








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
    uint32_t old_certainty = 0;
    uint32_t previous_certainty = 0;
    uint32_t current_certainty = 0;
    std::vector<std::pair<double, uint32_t>> scores;
    double total_score = 0;
    double total_weight = 0;
    std::unordered_map<uint64_t, double> color_freq_goal;
    double total_goal;
    std::unordered_map<uint64_t, double> color_freq_actual;
    double total_actual;
    
    size_t pos = 0;
    bool end_of_safe = false;
    for (uint32_t y = 0; y < image.height; ++y) {
        uint8_t filter_type = data[pos++];
        std::copy(data.begin() + pos, data.begin() + pos + stride, current_scanline.begin());
        pos += stride;

        unfilter_scanline(filter_type, current_scanline, previous_scanline, bpp);
        current_certainty = 0;
        for(int n=y*real_stride; n<(y+1)*real_stride; n++){
            current_certainty += uncertain_data[n].found;
        }
        for(int n=y*real_stride+1; n<(y+1)*real_stride; n+=bpp){
            uint64_t color = 0;
            size_t certainty = 0;
            for(int q=0; q<bpp; q++){
                color += (data[n+q]/8) << (q*8);
                certainty += uncertain_data[n+q].found;
            }
            if(certainty == bpp && !end_of_safe){
                color_freq_goal[color] += 1.0;
                total_goal += 1.0;
            } else {
                end_of_safe = true;
            }
            color_freq_actual[color] += 1.0;
            total_actual += 1.0;
            
        }
        if(end_of_safe){
            current_certainty /= 5.0;
        }
        if(y>1){
            scores.push_back(std::pair<double, uint32_t>(euclidean_distance(previous_scanline, current_scanline), previous_certainty+current_certainty));
            total_score += scores.back().first * ((float) scores.back().second);
            total_weight += scores.back().second;
            std::vector<uint8_t> left(current_scanline.end() - current_scanline.size()+bpp, current_scanline.end());
            std::vector<uint8_t> right(current_scanline.end() - current_scanline.size(), current_scanline.end()-bpp);
            scores.push_back(std::pair<double, uint32_t>(euclidean_distance(left, right), current_certainty));
            total_score += scores.back().first * ((float) scores.back().second);
            total_weight += scores.back().second;
        }
        old_scanline = previous_scanline;
        previous_scanline = current_scanline;
        old_certainty = previous_certainty;
        current_certainty = current_certainty;
    }
    double mean = total_score / total_weight;
    double real_score = 0;
    //std::cout << "START-----------------------" << std::endl;
    for(auto val : scores){
        //std::cout << val << " - " << mean << std::endl;
        real_score += std::abs(val.first-mean)*(real_stride-val.second);
    }
    double color_score = 1.0;
    for(auto& val : color_freq_actual){
        //std::cout << val << std::endl;
        double occurence = color_freq_actual[val.first] / total_actual;
        double expected = color_freq_goal[val.first] / total_goal;
        double rating = std::abs(occurence - expected) / (occurence + expected);
        color_score += rating;
    }
    real_score *= color_score+10.0;
    //std::cout << "COLOR SCORE " << color_score << " : " << real_score << std::endl;
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

void attempt_modification(PNGImage &image, double* goal_score, std::vector<size_t> &modified, std::vector<uncertainByte> &data, uint8_t max_scale){
    uint8_t scale = max_scale;
    while(scale > 0){
        //std::cout << "Value: "<< current << std::endl;
        for(int attempt=-scale; attempt <= scale; attempt+=scale*2){
            for(auto val: modified){
                data[val].value += attempt;
            }
            auto result = scoreUncertainty(image, data);
            if(result < *goal_score || *goal_score != *goal_score){
                std::cout << result << " < " << *goal_score << std::endl;
                *goal_score = result;
                std::cout << "Improved! " << *goal_score << " at scale " << (int) scale << std::endl;
                reconstruct_data(image, make_certain(data));
                if(smooth_ppms >= 0){
                    save_as_ppm(image, "post_smooth_" + std::to_string(++smooth_ppms) + ".ppm");
                }
                smooth_ppms++;
            } else {
                for(auto val: modified){
                    data[val].value -= attempt;
                }
            }
        }
        scale /= 2;
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
    for(int loop=0; loop<SMOOTH_LOOPS; loop++){
        auto current = scoreUncertainty(image, data);
        int processed = 0;
        for(int y=0;y<image.height; y++){
            const int segment_size = 64;
            for(int x_segment = 1; x_segment<real_stride; x_segment+=segment_size){
                std::vector<size_t> modified;
                for(int x=x_segment;x<real_stride && x < x_segment+segment_size;x++){
                    size_t loc = x+y*real_stride;
                    if(!data[loc].found && data[loc].reference == 0){
                        modified.push_back(loc);
                    }
                }
                if(modified.size() > 0){
                    attempt_modification(image, &current, modified, data, 128);
                }
            }
            std::cout << "Done with line " << y << " of " << image.height << " on loop " << loop << std::endl;
        }
        for(auto &index: guessable){
            std::cout << processed << "/" << guessable.size() << std::endl;
            std::vector<size_t> modified;
            modified.push_back(index.first);
            attempt_modification(image, &current, modified, data, 128);
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
            /*
            std::vector<uncertainByte> loaded = inflatePacket(image.packets[n], image.image_data);
            int good = 0;
            int full_row = (up_offset / real_stride) - 1;
            if (row < 0){
                break;
            }
            int original_up = up_offset;
            uint8_t so_far = make_certain(data);*/
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
            SMOOTH_PASSES = 1000;
            SMOOTH_LOOPS = 5;
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
