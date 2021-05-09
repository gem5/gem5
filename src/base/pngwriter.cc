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
 * @file Definition of a class that writes a frame buffer to a png
 */

#include "base/pngwriter.hh"

extern "C"
{
#include <png.h>
}

#include <cstdio>
#include <cstdlib>

#include "base/logging.hh"

namespace gem5
{

const char* PngWriter::_imgExtension = "png";

/**
 * Write callback to use with libpng APIs
 *
 * @param pngPtr  pointer to the png_struct structure
 * @param data    pointer to the data being written
 * @param length  number of bytes being written
 */
static void
writePng(png_structp pngPtr, png_bytep data, png_size_t length)
{
    // Here we get our IO pointer back from the write struct
    // and we cast it into a ostream* type.
    std::ostream* strmPtr = reinterpret_cast<std::ostream*>(
        png_get_io_ptr(pngPtr)
    );

    // Write length bytes to data
    strmPtr->write(reinterpret_cast<const char *>(data), length);
}

struct PngWriter::PngStructHandle
{
  private:
    // Make PngStructHandle uncopyable
    PngStructHandle(const PngStructHandle&) = delete;
    PngStructHandle& operator=(const PngStructHandle&) = delete;
  public:

    PngStructHandle() :
        pngWriteP(NULL), pngInfoP(NULL)
    {
        // Creating write structure
        pngWriteP = png_create_write_struct(
            PNG_LIBPNG_VER_STRING, NULL, NULL, NULL
        );

        if (pngWriteP) {
            // Creating info structure
            pngInfoP = png_create_info_struct(pngWriteP);
        }
    }

    ~PngStructHandle()
    {
        if (pngWriteP) {
            png_destroy_write_struct(&pngWriteP, &pngInfoP);
        }
    }

    /** Pointer to PNG Write struct */
    png_structp pngWriteP;

    /** Pointer to PNG Info struct */
    png_infop pngInfoP;
};

void
PngWriter::write(std::ostream &png) const
{

    // Height of the frame buffer
    unsigned height = fb.height();
    unsigned width  = fb.width();

    // Do not write if frame buffer is empty
    if (!fb.area()) {
        png.flush();
        return;
    }

    // Initialize Png structures
    PngStructHandle handle;

    // Png info/write pointers.
    png_structp pngPtr  = handle.pngWriteP;
    png_infop   infoPtr = handle.pngInfoP;

    if (!pngPtr) {
        warn("Frame buffer dump aborted: Unable to create"
             "Png Write Struct\n");
        return;
    }

    if (!infoPtr) {
        warn("Frame buffer dump aborted: Unable to create"
             "Png Info Struct\n");
        return;
    }

    // We cannot use default libpng write function since it requires
    // a file pointer (FILE*), whereas we want to use the ostream.
    // The following function replaces the write function with a custom
    // one provided by us (writePng)
    png_set_write_fn(pngPtr, (png_voidp)&png, writePng, NULL);

    png_set_IHDR(pngPtr, infoPtr, width, height, 8,
                 PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(pngPtr, infoPtr);

    // libpng requires an array of pointers to the frame buffer's rows.
    std::vector<PixelType> rowPacked(width);
    for (unsigned y=0; y < height; ++y) {
        for (unsigned x=0; x < width; ++x) {
            rowPacked[x] = fb.pixel(x, y);
        }

        png_write_row(pngPtr,
            reinterpret_cast<png_bytep>(rowPacked.data())
        );
    }

    // End of write
    png_write_end(pngPtr, NULL);
}

} // namespace gem5
