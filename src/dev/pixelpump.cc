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

#include "dev/pixelpump.hh"

const DisplayTimings DisplayTimings::vga(
    640, 480,
    48, 96, 16,
    33, 2, 10);


DisplayTimings::DisplayTimings(unsigned _width, unsigned _height,
                               unsigned hbp, unsigned h_sync, unsigned hfp,
                               unsigned vbp, unsigned v_sync, unsigned vfp)
    : width(_width), height(_height),
      hBackPorch(hbp), hFrontPorch(hfp), hSync(h_sync),
      vBackPorch(vbp), vFrontPorch(vfp), vSync(v_sync)
{
}

void
DisplayTimings::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(width);
    SERIALIZE_SCALAR(height);

    SERIALIZE_SCALAR(hBackPorch);
    SERIALIZE_SCALAR(hFrontPorch);
    SERIALIZE_SCALAR(hSync);

    SERIALIZE_SCALAR(vBackPorch);
    SERIALIZE_SCALAR(vFrontPorch);
    SERIALIZE_SCALAR(vSync);
}

void
DisplayTimings::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(width);
    UNSERIALIZE_SCALAR(height);

    UNSERIALIZE_SCALAR(hBackPorch);
    UNSERIALIZE_SCALAR(hFrontPorch);
    UNSERIALIZE_SCALAR(hSync);

    UNSERIALIZE_SCALAR(vBackPorch);
    UNSERIALIZE_SCALAR(vFrontPorch);
    UNSERIALIZE_SCALAR(vSync);
}


BasePixelPump::BasePixelPump(EventManager &em, ClockDomain &pxl_clk,
                             unsigned pixel_chunk)
    : EventManager(em), Clocked(pxl_clk), Serializable(),
      pixelChunk(pixel_chunk),
      pixelEvents(),
      evVSyncBegin("evVSyncBegin", this, &BasePixelPump::onVSyncBegin),
      evVSyncEnd("evVSyncEnd", this, &BasePixelPump::onVSyncEnd),
      evHSyncBegin("evHSyncBegin", this, &BasePixelPump::onHSyncBegin),
      evHSyncEnd("evHSyncEnd", this, &BasePixelPump::onHSyncEnd),
      evBeginLine("evBeginLine", this, &BasePixelPump::beginLine),
      evRenderPixels("evRenderPixels", this, &BasePixelPump::renderPixels),
      _timings(DisplayTimings::vga),
      line(0), _posX(0), _underrun(false)
{
}

BasePixelPump::~BasePixelPump()
{
}

void
BasePixelPump::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(line);
    SERIALIZE_SCALAR(_posX);
    SERIALIZE_SCALAR(_underrun);

    SERIALIZE_OBJ(_timings);
    SERIALIZE_OBJ(fb);

    for (PixelEvent *event : pixelEvents)
        event->serializeSection(cp, event->name());
}

void
BasePixelPump::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(line);
    UNSERIALIZE_SCALAR(_posX);
    UNSERIALIZE_SCALAR(_underrun);

    UNSERIALIZE_OBJ(_timings);
    UNSERIALIZE_OBJ(fb);

    // We don't need to reschedule the event here since the event was
    // suspended by PixelEvent::drain() and will be rescheduled by
    // PixelEvent::drainResume().
    for (PixelEvent *event : pixelEvents)
        event->unserializeSection(cp, event->name());
}


void
BasePixelPump::start(const DisplayTimings &timings)
{
    _timings = timings;

    // Resize the frame buffer if needed
    if (_timings.width != fb.width() || _timings.height != fb.height())
        fb.resize(timings.width, timings.height);

    // Set the current line past the last line in the frame. This
    // triggers the new frame logic in beginLine().
    line = _timings.linesPerFrame();
    schedule(evBeginLine, clockEdge());
}

void
BasePixelPump::stop()
{
    if (evVSyncEnd.scheduled())
        deschedule(evVSyncEnd);

    if (evHSyncBegin.scheduled())
        deschedule(evHSyncBegin);

    if (evHSyncEnd.scheduled())
        deschedule(evHSyncEnd);

    if (evBeginLine.scheduled())
        deschedule(evBeginLine);

    if (evRenderPixels.scheduled())
        deschedule(evRenderPixels);
}

void
BasePixelPump::beginLine()
{
    _posX = 0;
    line++;
    if (line >= _timings.linesPerFrame()) {
        _underrun = false;
        line = 0;
    }

    if (line == _timings.lineVSyncStart()) {
        onVSyncBegin();
    } else if (line == _timings.lineVBackPorchStart()) {
        onVSyncEnd();
    }

    const Cycles h_sync_begin(0);
    schedule(evHSyncBegin, clockEdge(h_sync_begin));

    const Cycles h_sync_end(h_sync_begin + _timings.hSync);
    schedule(evHSyncEnd, clockEdge(h_sync_end));

    // Visible area
    if (line >= _timings.lineFirstVisible() &&
        line < _timings.lineFrontPorchStart()) {

        const Cycles h_first_visible(h_sync_end + _timings.hBackPorch);
        schedule(evRenderPixels, clockEdge(h_first_visible));
    }

    schedule(evBeginLine, clockEdge(_timings.cyclesPerLine()));
}

void
BasePixelPump::renderPixels()
{
    // Try to handle multiple pixels at a time; doing so reduces the
    // accuracy of the underrun detection but lowers simulation
    // overhead
    const unsigned x_end(std::min(_posX + pixelChunk, _timings.width));
    const unsigned pxl_count(x_end - _posX);
    const unsigned pos_y(posY());

    Pixel pixel(0, 0, 0);
    const Pixel underrun_pixel(0, 0, 0);
    for (; _posX < x_end && !_underrun; ++_posX) {
        if (!nextPixel(pixel)) {
            warn("Input buffer underrun in BasePixelPump (%u, %u)\n",
                 _posX, pos_y);
            _underrun = true;
            onUnderrun(_posX, pos_y);
            pixel = underrun_pixel;
        }
        fb.pixel(_posX, pos_y) = pixel;
    }

    // Fill remaining pixels with a dummy pixel value if we ran out of
    // data
    for (; _posX < x_end; ++_posX)
        fb.pixel(_posX, pos_y) = underrun_pixel;

    // Schedule a new event to handle the next block of pixels
    if (_posX < _timings.width) {
        schedule(evRenderPixels, clockEdge(Cycles(pxl_count)));
    } else {
        if (pos_y == _timings.height - 1)
            onFrameDone();
    }
}

BasePixelPump::PixelEvent::PixelEvent(
    const char *name, BasePixelPump *_parent, CallbackType _func)
    : Event(), Drainable(),
      _name(name), parent(*_parent), func(_func),
      suspended(false),
      relativeTick(0)
{
    parent.pixelEvents.push_back(this);
}

DrainState
BasePixelPump::PixelEvent::drain()
{
    if (scheduled())
        suspend();
    return DrainState::Drained;
}

void
BasePixelPump::PixelEvent::drainResume()
{
    if (suspended)
        resume();
}

void
BasePixelPump::PixelEvent::serialize(CheckpointOut &cp) const
{
    assert(!scheduled());
    Event::serialize(cp);
    SERIALIZE_SCALAR(suspended);
    SERIALIZE_SCALAR(relativeTick);
}

void
BasePixelPump::PixelEvent::unserialize(CheckpointIn &cp)
{
    Event::unserialize(cp);
    UNSERIALIZE_SCALAR(suspended);
    UNSERIALIZE_SCALAR(relativeTick);
    assert(!scheduled());
}

void
BasePixelPump::PixelEvent::suspend()
{
    assert(scheduled());
    assert(!suspended);

    suspended = true;
    relativeTick = when() - curTick();
    parent.deschedule(this);
}

void
BasePixelPump::PixelEvent::resume()
{
    assert(!scheduled());
    assert(suspended);
    parent.schedule(this, relativeTick + curTick());
    suspended = false;
    relativeTick = 0;
}
