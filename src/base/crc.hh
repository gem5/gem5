/*
 * Copyright (c) 2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BASE_CRC_HH__
#define __BASE_CRC_HH__

#include "base/bitfield.hh"

/**
 * Evaluate the CRC32 of the first size bytes of a data buffer,
 * using a specific polynomium and an initial value.
 * The crc is accomplished by reversing the input, the initial value
 * and the output (remainder).
 *
 * @param data: Input data buffer pointer
 * @param crc:  Initial value of the checksum
 * @param size: Number of bytes
 *
 * @return 32-bit remainder of the checksum
 */
template <uint32_t Poly>
uint32_t
crc32(const uint8_t* data, uint32_t crc, std::size_t size)
{
    uint32_t byte = 0;

    crc = reverseBits(crc);
    for (auto i = 0; i < size; i++) {
        byte = data[i];

        // 32-bit reverse
        byte = reverseBits(byte);
        for (auto j = 0; j <= 7; j++) {
            if ((int)(crc ^ byte) < 0) {
                crc = (crc << 1) ^ Poly;
            } else {
                crc = crc << 1;
            }
            byte = byte << 1;
        }
    }
    return reverseBits(crc);
}

#endif // __BASE_CRC_HH__
