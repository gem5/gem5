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
#ifndef __BASE_BITMAP_HH__
#define __BASE_BITMAP_HH__

#include <fstream>

#include "base/vnc/convert.hh"

/**
 * @file Declaration of a class that writes a frame buffer to a bitmap
 */


// write frame buffer into a bitmap picture
class  Bitmap
{
  public:
    /** Create a Bitmap creator that takes data in the given mode & size
     * and outputs to an fstream
     * @param mode the type of data that is being provided
     * @param h the hight of the image
     * @param w the width of the image
     * @param d the data for the image in mode
     */
    Bitmap(VideoConvert::Mode mode, uint16_t w, uint16_t h, uint8_t *d);

    /** Destructor */
    ~Bitmap();

    /** Provide the converter with the data that should be output. It will be
     * converted into rgb8888 and write out when write() is called.
     * @param d the data
     */
    void rawData(uint8_t* d) { data = d; }

    /** Write the provided data into the fstream provided
     * @param bmp stream to write to
     */
    void write(std::ostream *bmp) const;

    /** Gets a hash over the bitmap for quick comparisons to other bitmaps.
     * @return hash of the bitmap
     */
    uint64_t getHash() const { return vc.getHash(data); }


  private:
    VideoConvert::Mode mode;
    uint16_t height;
    uint16_t width;
    uint8_t *data;

    VideoConvert vc;

    mutable char *headerBuffer;
    static const size_t sizeofHeaderBuffer;

    struct Magic
    {
        unsigned char magic_number[2];
    };

    struct Header
    {
        uint32_t size;
        uint16_t reserved1;
        uint16_t reserved2;
        uint32_t offset;
    };

    struct Info
    {
        uint32_t Size;
        uint32_t Width;
        uint32_t Height;
        uint16_t Planes;
        uint16_t BitCount;
        uint32_t Compression;
        uint32_t SizeImage;
        uint32_t XPelsPerMeter;
        uint32_t YPelsPerMeter;
        uint32_t ClrUsed;
        uint32_t ClrImportant;
    };
};

#endif // __BASE_BITMAP_HH__

