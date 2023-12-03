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

/**
 * @file Declaration of a class that writes a frame buffer to a png
 */

#ifndef __BASE_PNG_HH__
#define __BASE_PNG_HH__

#include "base/compiler.hh"
#include "base/framebuffer.hh"
#include "base/imgwriter.hh"

namespace gem5
{

/** Image writer implementing support for PNG */
class PngWriter : public ImgWriter
{
  public:
    /**
     * Create a png that takes data in a given mode & size and
     * outputs to an ostream.
     */
    PngWriter(const FrameBuffer *_fb) : ImgWriter(_fb) {}

    ~PngWriter(){};

    /**
     * Return Image format as a string
     *
     * @return img extension (e.g. .png for Png)
     */
    const char *
    getImgExtension() const override
    {
        return _imgExtension;
    }

    /**
     * Write the frame buffer data into the provided ostream
     *
     * @param png stream to write to
     */
    void write(std::ostream &png) const override;

  private:
    /** Png Pixel type: not containing padding */
    struct GEM5_PACKED PngPixel24
    {
        PngPixel24 &
        operator=(const Pixel &rhs)
        {
            red = rhs.red;
            green = rhs.green;
            blue = rhs.blue;

            return *this;
        }

        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    /**
     * Handle to resources used by libpng:
     *   - png_struct: Structure holding write informations
     *   - png_info  : Structure holding image informations
     *
     * The class is automatically taking care of struct
     * allocation/deallocation
     */
    struct PngStructHandle;

    typedef PngPixel24 PixelType;

    static const char *_imgExtension;
};

} // namespace gem5

#endif // __BASE_PNG_HH__
