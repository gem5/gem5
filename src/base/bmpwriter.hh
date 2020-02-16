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
#ifndef __BASE_BITMAP_HH__
#define __BASE_BITMAP_HH__

#include <ostream>

#include "base/compiler.hh"
#include "base/framebuffer.hh"
#include "base/imgwriter.hh"

/**
 * @file Declaration of a class that writes a frame buffer to a bitmap
 */

// write frame buffer into a bitmap picture
class  BmpWriter : public ImgWriter
{
  public:
    /**
     * Create a bitmap that takes data in a given mode & size and
     * outputs to an ostream.
     */
    BmpWriter(const FrameBuffer *fb);

    ~BmpWriter() {};

    /*
     * Return Image format as a string
     *
     * @return img extension (e.g. bmp for Bitmap)
     */
    const char* getImgExtension() const override
    { return _imgExtension; }

    /**
     * Write the frame buffer data into the provided ostream
     *
     * @param bmp stream to write to
     */
    void write(std::ostream &bmp) const override;

  private:
    struct FileHeader {
        unsigned char magic_number[2];
        uint32_t size;
        uint16_t reserved1;
        uint16_t reserved2;
        uint32_t offset;
    } M5_ATTR_PACKED;

    struct InfoHeaderV1 { /* Aka DIB header */
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
    } M5_ATTR_PACKED;

    struct CompleteV1Header {
        FileHeader file;
        InfoHeaderV1 info;
    } M5_ATTR_PACKED;

    struct BmpPixel32 {
        BmpPixel32 &operator=(const Pixel &rhs) {
            red = rhs.red;
            green = rhs.green;
            blue = rhs.blue;
            padding = 0;

            return *this;
        }
        uint8_t blue;
        uint8_t green;
        uint8_t red;
        uint8_t padding;
    } M5_ATTR_PACKED;

    typedef BmpPixel32 PixelType;

    static const char* _imgExtension;

    const CompleteV1Header getCompleteHeader() const;
};


#endif // __BASE_BITMAP_HH__

