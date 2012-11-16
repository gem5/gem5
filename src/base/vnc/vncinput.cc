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
 * Authors: Ali Saidi
 *          William Wang
 */

/** @file
 * Implementiation of a VNC input
 */

#include <sys/types.h>

#include "base/vnc/vncinput.hh"
#include "base/output.hh" //simout
#include "base/trace.hh"
#include "debug/VNC.hh"

using namespace std;

VncInput::VncInput(const Params *p)
    : SimObject(p), keyboard(NULL), mouse(NULL),
      vc(NULL), fbPtr(NULL), videoMode(VideoConvert::UnknownMode),
      _videoWidth(1), _videoHeight(1), captureEnabled(p->frame_capture),
      captureCurrentFrame(0), captureLastHash(0), captureBitmap(0)
{
    if (captureEnabled) {
        // remove existing frame output directory if it exists, then create a
        //   clean empty directory
        const string FRAME_OUTPUT_SUBDIR = "frames_" + name();
        simout.remove(FRAME_OUTPUT_SUBDIR, true);
        captureOutputDirectory = simout.createSubdirectory(
                                FRAME_OUTPUT_SUBDIR);
    }
}

void
VncInput::setFrameBufferParams(VideoConvert::Mode mode, uint16_t width,
    uint16_t height)
{
    DPRINTF(VNC, "Updating video params: mode: %d width: %d height: %d\n", mode,
            width, height);

    if (mode != videoMode || width != videoWidth() || height != videoHeight()) {
        videoMode = mode;
        _videoWidth = width;
        _videoHeight = height;

        if (vc)
            delete vc;

        vc = new VideoConvert(mode, VideoConvert::rgb8888, videoWidth(),
                videoHeight());

        if (captureEnabled) {
            // create bitmap of the frame with new attributes
            if (captureBitmap)
                delete captureBitmap;

            assert(fbPtr);
            captureBitmap = new Bitmap(videoMode, width, height, fbPtr);
            assert(captureBitmap);
        }
    }
}

void
VncInput::captureFrameBuffer()
{
    assert(captureBitmap);

    // skip identical frames
    uint64_t new_hash = captureBitmap->getHash();
    if (captureLastHash == new_hash)
        return;
    captureLastHash = new_hash;

    // get the filename for the current frame
    char frameFilenameBuffer[64];
    snprintf(frameFilenameBuffer, 64, "fb.%06d.%lld.bmp.gz",
            captureCurrentFrame, static_cast<long long int>(curTick()));
    const string frameFilename(frameFilenameBuffer);

    // create the compressed framebuffer file
    ostream *fb_out = simout.create(captureOutputDirectory + frameFilename,
                    true);
    captureBitmap->write(fb_out);
    simout.close(fb_out);

    ++captureCurrentFrame;
}

// create the VNC Replayer object
VncInput *
VncInputParams::create()
{
    return new VncInput(this);
}
