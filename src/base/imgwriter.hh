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
 *
 * Authors: Giacomo Travaglini
 */
#ifndef __BASE_IMGWRITER_HH__
#define __BASE_IMGWRITER_HH__

#include <ostream>

#include "base/compiler.hh"
#include "base/framebuffer.hh"

#include "enums/ImageFormat.hh"

// write frame buffer to an image
class ImgWriter
{
  public:
    ImgWriter(const FrameBuffer *_fb)
      : fb(*_fb)
    {}

    virtual ~ImgWriter() {};
    /**
     * Write the frame buffer data into the provided ostream
     *
     * @param out output stream to write to
     */
    virtual void write(std::ostream &out) const = 0;
    /*
     * Return Image format as a string
     *
     * @return img extension (e.g. bmp for Bitmap)
     */
    virtual const char* getImgExtension() const = 0;

  protected:
    const FrameBuffer &fb;
};

/**
 * Factory Function which allocates a ImgWriter object and returns
 * a smart pointer to it. The dynamic type of the object being pointed
 * depends upon the enum type passed as a first parameter.
 * If the enum contains an invalid value, the function will produce a warning
 * and will default to Bitamp.
 *
 * @param type Image writer type (e.g. Bitamp, Png)
 * @param fb Pointer to a FrameBuffer object
 *           This contains the raw data which will be stored as an image
 *           when calling the appropriate object method
 * @return smart pointer to the allocated Image Writer
 */
std::unique_ptr<ImgWriter>
createImgWriter(Enums::ImageFormat type, const FrameBuffer *fb);

#endif //__BASE_IMGWRITER_HH__
