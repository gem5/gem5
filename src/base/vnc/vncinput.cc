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

/** @file
 * Implementiation of a VNC input
 */

#include "base/vnc/vncinput.hh"

#include <sys/types.h>

#include "base/logging.hh"
#include "base/output.hh"

#include "base/trace.hh"
#include "debug/VNC.hh"

namespace gem5
{

VncInput::VncInput(const Params &p)
    : SimObject(p), keyboard(NULL), mouse(NULL),
      fb(&FrameBuffer::dummy),
      _videoWidth(fb->width()), _videoHeight(fb->height()),
      captureEnabled(p.frame_capture),
      captureCurrentFrame(0), captureLastHash(0),
      imgFormat(p.img_format)
{
    if (captureEnabled) {
        // remove existing frame output directory if it exists, then create a
        //   clean empty directory
        const std::string FRAME_OUTPUT_SUBDIR = "frames_" + name();
        simout.remove(FRAME_OUTPUT_SUBDIR, true);
        captureOutputDirectory = simout.createSubdirectory(
                                FRAME_OUTPUT_SUBDIR);
    }
}

void
VncInput::setFrameBuffer(const FrameBuffer *rfb)
{
    if (!rfb)
        panic("Trying to VNC frame buffer to NULL!");

    fb = rfb;

    // Create the Image Writer object in charge of dumping
    // the frame buffer raw data into a file in a specific format.
    if (captureEnabled) {
        captureImage = createImgWriter(imgFormat, rfb);
    }

    // Setting a new frame buffer means that we need to send an update
    // to the client. Mark the internal buffers as dirty to do so.
    setDirty();
}

void
VncInput::setDirty()
{
    const unsigned width(fb->width());
    const unsigned height(fb->height());

    if (_videoWidth != width || _videoHeight != height) {
        DPRINTF(VNC, "Updating video params: width: %d height: %d\n",
                width, height);

        _videoWidth = width;
        _videoHeight = height;

        frameBufferResized();
    }

     if (captureEnabled)
        captureFrameBuffer();
}

void
VncInput::captureFrameBuffer()
{
    assert(captureImage);

    // skip identical frames
    uint64_t new_hash = fb->getHash();
    if (captureLastHash == new_hash)
        return;
    captureLastHash = new_hash;

    // get the filename for the current frame
    char frameFilenameBuffer[64];
    snprintf(frameFilenameBuffer, 64, "fb.%06d.%lld.%s.gz",
            captureCurrentFrame, static_cast<long long int>(curTick()),
            captureImage->getImgExtension());
    const std::string frameFilename(frameFilenameBuffer);

    // create the compressed framebuffer file
    OutputStream *fb_out(captureOutputDirectory->create(frameFilename, true));
    captureImage->write(*fb_out->stream());
    captureOutputDirectory->close(fb_out);

    ++captureCurrentFrame;
}

} // namespace gem5
