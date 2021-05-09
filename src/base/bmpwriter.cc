/*
 * Copyright (c) 2010, 2015, 2017 ARM Limited
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

#include "base/bmpwriter.hh"

#include <cassert>

#include "base/logging.hh"

namespace gem5
{

const char* BmpWriter::_imgExtension = "bmp";

// bitmap class ctor
BmpWriter::BmpWriter(const FrameBuffer *_fb)
    : ImgWriter(_fb)
{
}

const BmpWriter::CompleteV1Header
BmpWriter::getCompleteHeader() const
{
    const uint32_t pixel_array_size(sizeof(PixelType) * fb.area());
    const uint32_t file_size(sizeof(CompleteV1Header) + pixel_array_size);

    const CompleteV1Header header = {
        // File header
        {
            {'B','M'}, /* Magic */
            file_size,
            0, 0, /* Reserved */
            sizeof(CompleteV1Header) /* Offset to pixel array */
        },
        // Info/DIB header
        {
            sizeof(InfoHeaderV1),
            fb.width(),
            fb.height(),
            1, /* Color planes */
            32, /* Bits per pixel */
            0, /* No compression */
            pixel_array_size, /* Image size in bytes */
            2835, /* x pixels per meter (assume 72 DPI) */
            2835, /* y pixels per meter (assume 72 DPI) */
            0, /* Colors in color table */
            0 /* Important color count (0 == all are important) */
        }
    };

    return header;
}

void
BmpWriter::write(std::ostream &bmp) const
{
    const CompleteV1Header header(getCompleteHeader());

    // 1.  write the header
    bmp.write(reinterpret_cast<const char *>(&header), sizeof(header));

    // 2.  write the bitmap data
    // BMP start store data left to right starting with the bottom row
    // so we need to do some creative flipping
    std::vector<PixelType> line_buffer(fb.width());
    for (int y = 0; y < fb.height(); ++y) {
        for (unsigned x = 0; x < fb.width(); ++x)
            line_buffer[x] = fb.pixel(x, fb.height() - y - 1);

        bmp.write(reinterpret_cast<const char *>(line_buffer.data()),
                  line_buffer.size() * sizeof(line_buffer[0]));
    }

    bmp.flush();
}

} // namespace gem5
