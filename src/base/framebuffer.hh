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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __BASE_FRAMEBUFFER_HH__
#define __BASE_FRAMEBUFFER_HH__

#include <cmath>
#include <cstdint>

#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/pixel.hh"
#include "base/str.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

/**
 * Internal gem5 representation of a frame buffer
 *
 * Display controllers and other devices producing images are expected
 * to use this class to represent the final image.
 *
 * Pixels are indexed relative to the upper left corner of the
 * image. That is, the pixel at position (0, 0) is the upper left
 * corner. The backing store is a linear vector of Pixels ordered left
 * to right starting in the upper left corner.
 */
class FrameBuffer : public Serializable
{
  public:
    /**
     * Create a frame buffer of a given size.
     *
     * @param width Width in pixels
     * @param height Height in pixels
     */
    FrameBuffer(unsigned width, unsigned height);
    /** Create an empty (0x0) frame buffer */
    FrameBuffer();

    virtual ~FrameBuffer();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Resize the frame buffer.
     *
     * This method resizes frame buffer including the backing
     * store. The contents of the backing store are undefined after
     * this operation.
     *
     * @param with Width in pixels.
     * @param height Height in pixels.
     */
    void resize(unsigned width, unsigned height);

    /** Frame buffer width in pixels */
    unsigned width() const { return _width; }
    /** Frame buffer height in pixels */
    unsigned height() const { return _height; }
    /** Total number of pixels in frame buffer */
    unsigned area() const { return _width * _height; }

    /**
     * Fill the frame buffer with a single pixel value
     *
     * @param pixel Pixel value to fill with.
     */
    void fill(const Pixel &pixel);
    /**
     * Fill the frame buffer with black pixels
     */
    void clear();

    /**
     * Fill the frame buffer with pixel data from an external buffer
     * of the same width and height as this frame buffer.
     *
     * @param fb External frame buffer
     * @param conv Pixel conversion helper
     */
    void copyIn(const uint8_t *fb, const PixelConverter &conv);
    /**
     * Fill the frame buffer with pixel data from an external buffer
     * of the same width and height as this frame buffer.
     *
     * @param fb External frame buffer
     * @param conv Pixel conversion helper
     */
    void copyIn(const std::vector<uint8_t> &fb, const PixelConverter &conv) {
        copyIn(fb.data(), conv);
    }

    /**
     * Store the contents of this frame buffer in an external buffer
     * of the same width and height as this frame buffer.
     *
     * @param fb External frame buffer
     * @param conv Pixel conversion helper
     */
    void copyOut(uint8_t *fb, const PixelConverter &conv) const;
    /**
     * Store the contents of this frame buffer in an external buffer
     * of the same width and height as this frame buffer.
     *
     * @param fb External frame buffer
     * @param conv Pixel conversion helper
     */
    void copyOut(std::vector<uint8_t> &fb, const PixelConverter &conv) const {
        copyOut(fb.data(), conv);
    }

    /**
     * Get a pixel from an (x, y) coordinate
     *
     * @param x Distance from the left margin.
     * @param y Distance from the top of the frame.
     */
    const Pixel &pixel(unsigned x, unsigned y) const {
        assert(x < _width);
        assert(y < _height);

        return pixels[y * _width + x];
    }

    /**
     * Get a pixel from an (x, y) coordinate
     *
     * @param x Distance from the left margin.
     * @param y Distance from the top of the frame.
     */
    Pixel &pixel(unsigned x, unsigned y) {
        assert(x < _width);
        assert(y < _height);

        return pixels[y * _width + x];
    }

    /**
     * Create a hash of the image that can be used for quick
     * comparisons.
     */
    uint64_t getHash() const;

    /**
     * Static "dummy" frame buffer.
     *
     * This is a dummy frame buffer that can be used as a place holder
     * for devices that always expect a frame buffer to be present.
     */
    static const FrameBuffer dummy;

    /** Frame buffer backing store */
    std::vector<Pixel> pixels;

  protected:
    /** Width in pixels */
    unsigned _width;
    /** Height in pixels */
    unsigned _height;
};

#endif // __BASE_FRAMEBUFFER_HH__
