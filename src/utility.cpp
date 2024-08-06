#include <vector>
#include "utility.h"

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


void write_raw_to_file(const std::vector<uint8_t>& idat_data, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        exit(1);
    }
    file.write(reinterpret_cast<const char*>(idat_data.data()), idat_data.size());
}
