#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <cstdint>

bool verifyLengths(const std::vector<uint16_t>& lengths);

void write_raw_to_file(const std::vector<uint8_t>& idat_data, const std::string& filename);

// CRC computation table for verifying chunk CRC values
static unsigned long crc_table[256];
static bool crc_table_computed = false;

void make_crc_table(void);

unsigned long update_crc(unsigned long crc, unsigned char *buf,
                       int len);
                       
unsigned long crc(unsigned char *buf, int len);


#endif
