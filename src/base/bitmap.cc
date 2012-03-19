/*
 * Copyright (c) 2010 ARM Limited
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
 * Authors: William Wang
 *          Ali Saidi
 *          Chris Emmons
 */

#include <cassert>

#include "base/bitmap.hh"
#include "base/misc.hh"

const size_t Bitmap::sizeofHeaderBuffer = sizeof(Magic) + sizeof(Header) +
                                        sizeof(Info);

// bitmap class ctor
Bitmap::Bitmap(VideoConvert::Mode _mode, uint16_t w, uint16_t h, uint8_t *d)
    : mode(_mode), height(h), width(w), data(d),
    vc(mode, VideoConvert::rgb8888, width, height), headerBuffer(0)
{
}

Bitmap::~Bitmap() {
    if (headerBuffer)
        delete [] headerBuffer;
}

void
Bitmap::write(std::ostream *bmp) const
{
    assert(data);

    // header is always the same for a bitmap object; compute the info once per
    //   bitmap object
    if (!headerBuffer) {
        // For further information see:
        //   http://en.wikipedia.org/wiki/BMP_file_format
        Magic magic = {{'B','M'}};
        Header header = {
            static_cast<uint32_t>(sizeof(VideoConvert::Rgb8888)) *
            width * height, 0, 0, 54};
        Info info = {static_cast<uint32_t>(sizeof(Info)), width, height, 1,
                     static_cast<uint32_t>(sizeof(VideoConvert::Rgb8888)) * 8,
                     0, static_cast<uint32_t>(sizeof(VideoConvert::Rgb8888)) *
                     width * height, 1, 1, 0, 0};

        char *p = headerBuffer = new char[sizeofHeaderBuffer];
        memcpy(p, &magic, sizeof(Magic));
        p += sizeof(Magic);
        memcpy(p, &header, sizeof(Header));
        p += sizeof(Header);
        memcpy(p, &info,   sizeof(Info));
    }

    // 1.  write the header
    bmp->write(headerBuffer, sizeofHeaderBuffer);

    // 2.  write the bitmap data
    uint8_t *tmp = vc.convert(data);
    uint32_t *tmp32 = (uint32_t*)tmp;

    // BMP start store data left to right starting with the bottom row
    // so we need to do some creative flipping
    for (int i = height - 1; i >= 0; i--)
        for (int j = 0; j < width; j++)
            bmp->write((char*)&tmp32[i * width + j], sizeof(uint32_t));

    bmp->flush();

    delete [] tmp;
}

