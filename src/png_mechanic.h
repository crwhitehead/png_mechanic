#ifndef PNG_MECHANIC_H
#define PNG_MECHANIC_H

#undef DEBUG_CRC_INPUT
#undef DEBUG_INFLATE
#undef DEBUG_FILTERS
#define DEBUG_PACKET_LOCATIONS 1

#include <iostream>
#include <vector>
#include <bitset>
#include <map>
#include <unordered_map>
#include <utility>
#include "deflate.h"

extern size_t MEDIAN_PASSES;
extern size_t SMOOTH_PASSES;
extern size_t SMOOTH_LOOPS;
extern int smooth_ppms;

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
    };
    uncertainByte(uint8_t value, int32_t reference, bool found) {
        this->value = value;
        this->reference = reference;
        this->found = found;
        this->testing = false;
        this->foundValue = 0;
    };
    uint8_t getValue(){
        if(this->foundValue == 0){
            return *this->foundValue;
        } else {
            return this->value;
        }
    };
};


#endif
