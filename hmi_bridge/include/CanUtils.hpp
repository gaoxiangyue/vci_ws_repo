#pragma once

#include <can_msgs/Frame.h>

namespace bosch_mrr {

template <typename T>
T extract_bits(const can_msgs::Frame& can_frame, 
               unsigned int start_bit,
               unsigned int bit_len,
               T factor,
               T offset,
               bool big_endian = false,
               bool is_signed = false)
{
    /* start_bit: MSB for big-endian; LSB for little-endian */
    int64_t buf = 0;

    if (big_endian) {
        for (int i = 0; i < 8; i++) {
            buf <<= 8;
            buf += can_frame.data[i];
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            buf <<= 8;
            buf += can_frame.data[i];
        }
    }

    uint64_t mask = 0;
    assert(bit_len <= 64);
    if (bit_len == 64) {
        mask = ~0ULL;
    } else {
        mask = (1ULL << bit_len) - 1;
    }

    int lsb = start_bit;
    if (big_endian) {
        int msb = start_bit % 8 + (7 - (start_bit / 8)) * 8;
        lsb = msb - (bit_len - 1);
        //lsb=((63-start_bit)/8)*8+start_bit%8;//for motorola msgstyle
    }

    T result;
    int64_t raw_result = (buf >> lsb) & mask;
    if (is_signed) {
        if (raw_result & (1ULL << (bit_len - 1))) {
            raw_result -= (1ULL << bit_len);
        }
    }
    result = raw_result * factor + offset;
   
#ifdef DEBUG
    std::cerr << "ID: " << can_frame.id << " parsing: " << raw_result << " as: " << result << std::endl; 
#endif

    return result;
}


template <typename T>
void pack_bits(can_msgs::Frame& can_frame, 
               unsigned int start_bit,
               unsigned int bit_len,
               T factor,
               T offset,
               T data,
               bool big_endian = false,
               bool is_signed = false)
{
    uint64_t buf = 0;

    if (big_endian) {
        for (int i = 0; i < 8; i++) {
            buf <<= 8;
            buf += can_frame.data[i];
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            buf <<= 8;
            buf += can_frame.data[i];
        }
    }

    uint64_t mask = 0;
    assert(bit_len <= 64);
    if (bit_len == 64) {
        mask = ~0ULL;
    } else {
        mask = (1ULL << bit_len) - 1;
    }

    int64_t data_raw = (int64_t)(round((data - offset) / factor));
    if (data_raw < 0) {
        data_raw = (1ULL << bit_len) + data_raw;
    }
    data_raw &= mask;

    int lsb = start_bit;
    if (big_endian) {
        int msb = start_bit % 8 + (7 - (start_bit / 8)) * 8;
        lsb = msb - (bit_len - 1);
    }

    buf += data_raw << lsb;

#ifdef DEBUG
    std::cerr << "ID: " << can_frame.id << " parsing: " << data << " as: " << data_raw << std::endl; 
#endif

    if (big_endian) {
        for (int i = 7; i >= 0; i--) {
            can_frame.data[i] = buf & 0xff;
            buf >>= 8;
        }
    } else {
        for (int i = 0; i < 8; i++) {
            can_frame.data[i] = buf & 0xff;
            buf >>= 8;
        }
    }
}

} // namespace bosch_mrr

