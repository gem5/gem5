/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          William Wang
 */

#include <cassert>

#include "base/vnc/convert.hh"
#include "base/misc.hh"

/** @file
 * This file provides conversion functions for a variety of video modes
 */

VideoConvert::VideoConvert(Mode input_mode, Mode output_mode, int _width,
        int _height)
    : inputMode(input_mode), outputMode(output_mode), width(_width),
    height(_height)
{
    if (inputMode != bgr565 && inputMode != rgb565 &&
        inputMode != bgr8888 && inputMode != bgr888)
        fatal("Only support converting from bgr565, rdb565, "
              "bgr8888 and bgr888\n");

    if (outputMode != rgb8888)
        fatal("Only support converting to rgb8888\n");

    assert(0 < height && height < 4000);
    assert(0 < width && width < 4000);
}

VideoConvert::~VideoConvert()
{
}

uint8_t*
VideoConvert::convert(const uint8_t *fb) const
{
    switch (inputMode) {
      case bgr565:
        return m565rgb8888(fb, true);
      case rgb565:
        return m565rgb8888(fb, false);
      case bgr8888:
        return bgr8888rgb8888(fb);
      case bgr888:
        return bgr888rgb8888(fb);
      default:
        panic("Unimplemented Mode\n");
    }
}

uint8_t*
VideoConvert::m565rgb8888(const uint8_t *fb, bool bgr) const
{
    uint8_t *out = new uint8_t[area() * sizeof(uint32_t)];
    uint32_t *out32 = (uint32_t*)out;

    uint16_t *in16 = (uint16_t*)fb;

    for (int x = 0; x < area(); x++) {
        Bgr565 inpx;
        Rgb8888 outpx = 0;

        inpx = in16[x];

        if (bgr) {
            outpx.red = inpx.blue << 3;
            outpx.green = inpx.green << 2;
            outpx.blue = inpx.red << 3;
        } else {
            outpx.blue = inpx.blue << 3;
            outpx.green = inpx.green << 2;
            outpx.red = inpx.red << 3;
        }

        out32[x] = outpx;
    }

    return out;
}


uint8_t*
VideoConvert::bgr8888rgb8888(const uint8_t *fb) const
{
    uint8_t *out = new uint8_t[area() * sizeof(uint32_t)];
    uint32_t *out32 = (uint32_t*)out;

    uint32_t *in32 = (uint32_t*)fb;

    for (int x = 0; x < area(); x++) {
        Rgb8888 outpx = 0;
        Bgr8888 inpx;


        inpx = in32[x];

        outpx.red = inpx.blue;
        outpx.green = inpx.green;
        outpx.blue = inpx.red;

        out32[x] = outpx;
    }

    return out;
}

uint8_t*
VideoConvert::bgr888rgb8888(const uint8_t *fb) const
{
    uint8_t *out = new uint8_t[area() * sizeof(uint32_t)];
    uint32_t *out32 = (uint32_t*)out;

    typedef uint8_t In24[3];
    const In24 *in24 = (In24 *)fb;
    for (int x = 0; x < area(); x++) {
        Rgb8888 outpx = 0;

        outpx.blue = in24[x][0];
        outpx.green = in24[x][1];
        outpx.red = in24[x][2];
        outpx.alpha = 0xFF;

        out32[x] = outpx;
    }

    return out;
}

/*
uint64_t
VideoConvert::getHash(const uint8_t *fb) const
{
    const uint8_t *fb_e = fb + area();

    uint64_t hash = 1;
    while (fb < fb_e - 8) {
        hash += *((const uint64_t*)fb);
        fb += 8;
    }

    while (fb < fb_e) {
        hash += *(fb++);
    }

    return hash;
}*/
