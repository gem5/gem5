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

#include "base/framebuffer.hh"

#include <zlib.h>

#include "base/bitfield.hh"

const FrameBuffer FrameBuffer::dummy(320, 240);

FrameBuffer::FrameBuffer(unsigned width, unsigned height)
    : pixels(width * height),
      _width(width), _height(height)
{
    clear();
}

FrameBuffer::FrameBuffer()
    : _width(0), _height(0)
{
}

FrameBuffer::~FrameBuffer()
{
}


void
FrameBuffer::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_width);
    SERIALIZE_SCALAR(_height);
    SERIALIZE_CONTAINER(pixels);
}

void
FrameBuffer::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_width);
    UNSERIALIZE_SCALAR(_height);
    UNSERIALIZE_CONTAINER(pixels);
}

void
FrameBuffer::resize(unsigned width, unsigned height)
{
    _width = width;
    _height = height;

    pixels.resize(width * height);
}

void
FrameBuffer::fill(const Pixel &pixel)
{
    for (auto &p : pixels)
        p = pixel;
}

void
FrameBuffer::clear()
{
    static const Pixel black(0, 0, 0);

    fill(black);
}

void
FrameBuffer::copyIn(const uint8_t *fb, const PixelConverter &conv)
{
    for (auto &p : pixels) {
        p = conv.toPixel(fb);
        fb += conv.length;
    }
}

void
FrameBuffer::copyOut(uint8_t *fb, const PixelConverter &conv) const
{
    for (auto &p : pixels) {
        conv.fromPixel(fb, p);
        fb += conv.length;
    }
}

uint64_t
FrameBuffer::getHash() const
{
    return adler32(0UL,
                   reinterpret_cast<const Bytef *>(pixels.data()),
                   area() * sizeof(Pixel));
}
