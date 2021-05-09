/*
 * Copyright (c) 2015 ARM Limited
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

#include "base/pixel.hh"

#include <cassert>

#include "base/bitfield.hh"

namespace gem5
{

const PixelConverter PixelConverter::rgba8888_le(4, 0, 8, 16, 8, 8, 8);
const PixelConverter PixelConverter::rgba8888_be(4, 0, 8, 16, 8, 8, 8,
                                                 ByteOrder::big);
const PixelConverter PixelConverter::rgb565_le(2,  0, 5, 11, 5, 6, 5);
const PixelConverter PixelConverter::rgb565_be(2,  0, 5, 11, 5, 6, 5,
                                               ByteOrder::big);

PixelConverter::PixelConverter(unsigned _length,
                               unsigned ro, unsigned go, unsigned bo,
                               unsigned rw, unsigned gw, unsigned bw,
                               ByteOrder _byte_order)
    : length(_length),
      depth(rw + gw + bw),
      byte_order(_byte_order),
      ch_r(ro, rw),
      ch_g(go, gw),
      ch_b(bo, bw)
{
    assert(length > 1);
}

PixelConverter::Channel::Channel(unsigned _offset, unsigned width)
    : offset(_offset),
      mask(gem5::mask(width)),
      factor(255.0 / mask)
{
}

uint32_t
PixelConverter::readWord(const uint8_t *p) const
{
    uint32_t word(0);

    if (byte_order == ByteOrder::little) {
        for (int i = 0; i < length; ++i)
            word |= p[i] << (8 * i);
    } else {
        for (int i = 0; i < length; ++i)
            word |= p[i] << (8 * (length - i - 1));
    }

    return word;
}

void
PixelConverter::writeWord(uint8_t *p, uint32_t word) const
{
    if (byte_order == ByteOrder::little) {
        for (int i = 0; i < length; ++i)
            p[i] = (word >> (8 * i)) & 0xFF;
    } else {
        for (int i = 0; i < length; ++i)
            p[i] = (word >> (8 * (length - i - 1))) & 0xFF;
    }
}

} // namespace gem5
