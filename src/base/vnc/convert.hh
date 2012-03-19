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
 */

/** @file
 * This file provides conversion functions for a variety of video modes
 */

#ifndef __BASE_VNC_CONVERT_HH__
#define __BASE_VNC_CONVERT_HH__

#include <zlib.h>
#include "base/bitunion.hh"

class VideoConvert
{
  public:
    enum Mode {
        UnknownMode,
        bgr565,
        rgb565,
        bgr8888,
        rgb8888,
        rgb888,
        bgr888,
        bgr444,
        bgr4444,
        rgb444,
        rgb4444
    };

    // supports bpp32 RGB (bmp) and bpp16 5:6:5 mode BGR (linux)
    BitUnion32(Rgb8888)
        Bitfield<7,0> blue;
        Bitfield<15,8> green;
        Bitfield<23,16> red;
        Bitfield<31,24> alpha;
    EndBitUnion(Rgb8888)

    BitUnion32(Bgr8888)
        Bitfield<7,0> red;
        Bitfield<15,8> green;
        Bitfield<23,16> blue;
        Bitfield<31,24> alpha;
    EndBitUnion(Bgr8888)

    BitUnion16(Bgr565)
        Bitfield<4,0> red;
        Bitfield<10,5> green;
        Bitfield<15,11> blue;
    EndBitUnion(Bgr565)

    BitUnion16(Rgb565)
        Bitfield<4,0> red;
        Bitfield<10,5> green;
        Bitfield<15,11> blue;
    EndBitUnion(Rgb565)

    /** Setup the converter with the given parameters
     * @param input_mode type of data that will be provided
     * @param output_mode type of data that should be output
     * @param _width width of the frame buffer
     * @param _height height of the frame buffer
     */
    VideoConvert(Mode input_mode, Mode output_mode, int _width, int _height);

    /** Destructor
     */
    ~VideoConvert();

    /** Convert the provided frame buffer data into the format specified in the
     * constructor.
     * @param fb the frame buffer to convert
     * @return the converted data (user must free)
     */
    uint8_t* convert(const uint8_t *fb) const;

    /** Return the number of pixels that this buffer specifies
     * @return number of pixels
     */
    int area() const { return width * height; }

    /**
     * Returns a hash on the raw data.
     *
     * @return hash of the buffer
     */
    inline uint64_t getHash(const uint8_t *fb) const {
        return adler32(0UL, fb, width * height);
    }

  private:

    /**
     * Convert a bgr8888 input to rgb8888.
     * @param fb the data to convert
     * @return converted data
     */
    uint8_t* bgr8888rgb8888(const uint8_t *fb) const;

    /**
     * Convert a bgr565 or rgb565 input to rgb8888.
     * @param fb the data to convert
     * @param bgr true if the input data is bgr565
     * @return converted data
     */
    uint8_t* m565rgb8888(const uint8_t *fb, bool bgr) const;

    Mode inputMode;
    Mode outputMode;
    int width;
    int height;
};

#endif // __BASE_VNC_CONVERT_HH__

