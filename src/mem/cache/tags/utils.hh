#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iostream>

using namespace std;

#define UINT32      unsigned int
#define UINT64      unsigned long long
#define INT32       int
#define ADDRINT     unsigned long long
#define COUNTER     unsigned long long
#define BITVECTOR   unsigned long long
#define Addr_t      unsigned long long

// Works by finding position of MSB set.
static inline INT32 CRC_FloorLog2(UINT32 n)
{
    INT32 p = 0;

    if (n == 0) return -1;

    if (n & 0xffff0000) { p += 16; n >>= 16; }
    if (n & 0x0000ff00) { p +=  8; n >>=  8; }
    if (n & 0x000000f0) { p +=  4; n >>=  4; }
    if (n & 0x0000000c) { p +=  2; n >>=  2; }
    if (n & 0x00000002) { p +=  1; }

    return p;
}

// Works by finding position of MSB set.
// @returns -1 if n == 0.
static inline INT32 CRC_CeilLog2(UINT32 n)
{
    return CRC_FloorLog2(n - 1) + 1;
}

#endif