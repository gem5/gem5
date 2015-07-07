/*
 * Copyright (c) 2010-2013, 2015 ARM Limited
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
 * Authors: Chris Emmons
 */

#include "dev/arm/hdlcd.hh"

#include "base/vnc/vncinput.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "debug/HDLcd.hh"
#include "debug/Uart.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

using std::vector;


// initialize hdlcd registers
HDLcd::HDLcd(const Params *p)
    : AmbaDmaDevice(p), version(VERSION_RESETV),
      int_rawstat(0), int_clear(0), int_mask(0), int_status(0),
      fb_base(0), fb_line_length(0), fb_line_count(0), fb_line_pitch(0),
      bus_options(BUS_OPTIONS_RESETV),
      v_sync(0), v_back_porch(0), v_data(0), v_front_porch(0),
      h_sync(0), h_back_porch(0), h_data(0), h_front_porch(0),
      polarities(0), command(0), pixel_format(0),
      red_select(0), green_select(0), blue_select(0),
      pixelClock(p->pixel_clock),
      fb(0, 0), vnc(p->vnc), bmp(&fb), pic(NULL),
      frameReadStartTime(0),
      dmaStartAddr(0), dmaCurAddr(0), dmaMaxAddr(0), dmaPendingNum(0),
      frameUnderrun(false), pixelBufferSize(0),
      pixelIndex(0), doUpdateParams(false), frameUnderway(false),
      dmaBytesInFlight(0),
      startFrameEvent(this), endFrameEvent(this), renderPixelEvent(this),
      fillPixelBufferEvent(this), intEvent(this),
      dmaDoneEventAll(MAX_OUTSTANDING_DMA_REQ_CAPACITY, this),
      dmaDoneEventFree(MAX_OUTSTANDING_DMA_REQ_CAPACITY),
      enableCapture(p->enable_capture),
      workaround_swap_rb(p->workaround_swap_rb)
{
    pioSize = 0xFFFF;

    for (int i = 0; i < MAX_OUTSTANDING_DMA_REQ_CAPACITY; ++i)
        dmaDoneEventFree[i] = &dmaDoneEventAll[i];

    if (vnc)
        vnc->setFrameBuffer(&fb);
}

HDLcd::~HDLcd()
{
}

// read registers and frame buffer
Tick
HDLcd::read(PacketPtr pkt)
{
    uint32_t data = 0;
    const Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(HDLcd, "read register BASE+0x%04x size=%d\n", daddr,
            pkt->getSize());

    assert(pkt->getAddr() >= pioAddr &&
            pkt->getAddr() < pioAddr + pioSize &&
            pkt->getSize() == 4);

    switch (daddr) {
      case Version:
        data = version;
        break;
      case Int_RawStat:
        data = int_rawstat;
        break;
      case Int_Clear:
        panic("HDLCD INT_CLEAR register is Write-Only\n");
        break;
      case Int_Mask:
        data = int_mask;
        break;
      case Int_Status:
        data = int_status;
        break;
      case Fb_Base:
        data = fb_base;
        break;
      case Fb_Line_Length:
        data = fb_line_length;
        break;
      case Fb_Line_Count:
        data = fb_line_count;
        break;
      case Fb_Line_Pitch:
        data = fb_line_pitch;
        break;
      case Bus_Options:
        data = bus_options;
        break;
      case V_Sync:
        data = v_sync;
        break;
      case V_Back_Porch:
        data = v_back_porch;
        break;
      case V_Data:
        data = v_data;
        break;
      case V_Front_Porch:
        data = v_front_porch;
        break;
      case H_Sync:
        data = h_sync;
        break;
      case H_Back_Porch:
        data = h_back_porch;
        break;
      case H_Data:
        data = h_data;
        break;
      case H_Front_Porch:
        data = h_front_porch;
        break;
      case Polarities:
        data = polarities;
        break;
      case Command:
        data = command;
        break;
      case Pixel_Format:
        data = pixel_format;
        break;
      case Red_Select:
        data = red_select;
        break;
      case Green_Select:
        data = green_select;
        break;
      case Blue_Select:
        data = blue_select;
        break;
      default:
        panic("Tried to read HDLCD register that doesn't  exist\n", daddr);
        break;
    }

    pkt->set<uint32_t>(data);
    pkt->makeAtomicResponse();
    return pioDelay;
}

// write registers and frame buffer
Tick
HDLcd::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr &&
           pkt->getAddr() < pioAddr + pioSize &&
           pkt->getSize() == 4);

    const uint32_t data = pkt->get<uint32_t>();
    const Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(HDLcd, "write register BASE+%0x04x <= 0x%08x\n", daddr,
            pkt->get<uint32_t>());

    switch (daddr) {
      case Version:
        panic("HDLCD VERSION register is read-Only\n");
        break;
      case Int_RawStat:
        int_rawstat = data;
        break;
      case Int_Clear:
        int_clear = data;
        break;
      case Int_Mask:
        int_mask = data;
        break;
      case Int_Status:
        panic("HDLCD INT_STATUS register is read-Only\n");
        break;
      case Fb_Base:
        fb_base = data;
        DPRINTF(HDLcd, "HDLCD Frame Buffer located at addr 0x%08x\n", fb_base);
        break;
      case Fb_Line_Length:
        fb_line_length = data;
        DPRINTF(HDLcd, "HDLCD res = %d x %d\n", width(), height());
        break;
      case Fb_Line_Count:
        fb_line_count = data;
        DPRINTF(HDLcd, "HDLCD res = %d x %d\n", width(), height());
        break;
      case Fb_Line_Pitch:
        fb_line_pitch = data;
        break;
      case Bus_Options: {
        BusOptsReg old_bus_options;
        old_bus_options = bus_options;
        bus_options = data;
        if (bus_options.max_outstanding != old_bus_options.max_outstanding)
            DPRINTF(HDLcd,
                "Changing HDLcd outstanding dma transactions from %d to %d\n",
                old_bus_options.max_outstanding, bus_options.max_outstanding);
        if (bus_options.burst_len != old_bus_options.burst_len)
            DPRINTF(HDLcd,
                "Changing HDLcd dma burst length from %d bytes to %d bytes\n",
                old_bus_options.burst_len, bus_options.burst_len); }
        break;
      case V_Sync:
        v_sync = data;
        break;
      case V_Back_Porch:
        v_back_porch = data;
        break;
      case V_Data:
        v_data = data;
        break;
      case V_Front_Porch:
        v_front_porch = data;
        break;
      case H_Sync:
        h_sync = data;
        break;
      case H_Back_Porch:
        h_back_porch = data;
        break;
      case H_Data:
        h_data = data;
        break;
      case H_Front_Porch:
        h_front_porch = data;
        break;
      case Polarities:
        polarities = data;
        break;
      case Command: {
        CommandReg new_command;
        new_command = data;
        if (new_command.enable != command.enable) {
            DPRINTF(HDLcd, "HDLCD switched %s\n",
                    new_command.enable==0 ? "off" : "on");
            if (new_command.enable) {
                doUpdateParams = true;
                if (!frameUnderway) {
                    schedule(startFrameEvent, clockEdge());
                }
            }
        }
        command = new_command; }
        break;
      case Pixel_Format:
        pixel_format = data;
        DPRINTF(HDLcd, "HDLCD res = %d x %d\n", width(), height());
        DPRINTF(HDLcd, "HDLCD bytes per pixel = %d\n", bytesPerPixel());
        DPRINTF(HDLcd, "HDLCD endianness = %s\n",
                pixel_format.big_endian ? "big" : "little");
        break;
      case Red_Select:
        red_select = data;
        break;
      case Green_Select:
        green_select = data;
        break;
      case Blue_Select:
        blue_select = data;
        break;
      default:
        panic("Tried to write HDLCD register that doesn't exist\n", daddr);
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
HDLcd::updateVideoParams(bool unserializing = false)
{
    const uint16_t bpp M5_VAR_USED = bytesPerPixel() << 3;

    // Workaround configuration bugs where multiple display
    // controllers are attached to the same VNC server by reattaching
    // enabled devices. This isn't ideal, but works as long as only
    // one display controller is active at a time.
    if (command.enable && vnc)
        vnc->setFrameBuffer(&fb);

    // updating these parameters while LCD is enabled is not supported
    if (frameUnderway && !unserializing)
        panic("Attempting to change some HDLCD parameters while the controller"
                " is active is not allowed");

    // resize the virtualDisplayBuffer unless we are unserializing - it may
    //   have changed size
    // there must be no outstanding DMA transactions for this to work
    if (!unserializing) {
        assert(dmaPendingNum == 0);

        virtualDisplayBuffer.resize(bytesPerPixel() * area());
        fb.resize(width(), height());
        fb.clear();

        std::fill(virtualDisplayBuffer.begin(), virtualDisplayBuffer.end(),
                  0);
    }

    DPRINTF(HDLcd, "bpp = %d\n", bpp);
    DPRINTF(HDLcd, "display size = %d x %d\n", width(), height());
#if TRACING_ON
    const size_t totalLinesPerFrame = v_back_porch.val + 1 +
                                      v_data.val + 1 +
                                      v_front_porch.val + 1 +
                                      v_sync.val + 1;
    const double fps = (double)SimClock::Frequency /
            (double)(PClksPerLine() * totalLinesPerFrame * pixelClock);
#endif
    DPRINTF(HDLcd, "simulated refresh rate ~ %.1ffps generating ~ %.1fMB/s "
            "traffic ([%.1fMHz, T=%d sim clocks] pclk, %d bpp => %.1fMB/s peak requirement)\n",
            fps,
            fps * virtualDisplayBuffer.size() / 1024 / 1024,
            (double)SimClock::Frequency / pixelClock / 1000000.0,
            pixelClock,
            bpp,
            (double)(SimClock::Frequency / pixelClock * (bpp / 8)) / 1024 / 1024);
}

void
HDLcd::startFrame()
{
    // 0. Check that we are in the appropriate state
    assert(!frameUnderway);
    if (!command.enable)
        return;
    DPRINTF(HDLcd, "Frame read started\n");
    if (doUpdateParams) {
        updateVideoParams();
        doUpdateParams = false;
    }
    frameUnderway = true;
    assert(!virtualDisplayBuffer.empty());
    assert(pixelBufferSize == 0);
    assert(dmaBytesInFlight == 0);
    assert(dmaPendingNum == 0);
    assert(dmaDoneEventFree.size() == dmaDoneEventAll.size());
    assert(!renderPixelEvent.scheduled());
    // currently only support positive line pitches equal to the line length
    assert(width() * bytesPerPixel() == fb_line_pitch);

    // 1. Start DMA'ing the frame; subsequent transactions created as we go
    dmaCurAddr = dmaStartAddr = fb_base;
    dmaMaxAddr = static_cast<Addr>(width() * height() * bytesPerPixel()) +
                    dmaCurAddr;
    frameReadStartTime = curTick();
    pixelIndex = 0;
    frameUnderrun = false;
    fillPixelBuffer();

    // 2. Schedule first pixelclock read; subsequent reads generated as we go
    Tick firstPixelReadTick = curTick() + pixelClock * (
                                  PClksPerLine() * (v_sync.val + 1 +
                                                    v_back_porch.val + 1) +
                                  h_sync.val + 1 +
                                  h_back_porch.val + 1);
    schedule(renderPixelEvent, firstPixelReadTick);
}

void
HDLcd::fillPixelBuffer()
{
    // - am I under the LCD dma transaction total?
    // - do I have more data to transfer?
    // - have I not yet underrun for this frame?
    // - is there room to put the data in the pixel buffer including any
    //   outstanding dma transfers in flight?
    while ((dmaPendingNum < maxOutstandingDma()) &&
           (dmaMaxAddr > dmaCurAddr) &&
           !frameUnderrun &&
           bytesFreeInPixelBuffer() > dmaBurstLength() * AXI_PORT_WIDTH) {
        // try largest transaction size allowed first but switch to smaller
        // sizes for trailing bytes
        size_t transaction_size = dmaBurstLength() * AXI_PORT_WIDTH;
        while (transaction_size > (dmaMaxAddr - dmaCurAddr))
            transaction_size >>= 1;
        assert(transaction_size > 0);

        // concurrent dma reads need different dma done events
        // due to assertion in scheduling state
        ++dmaPendingNum;

        assert(!dmaDoneEventFree.empty());
        DmaDoneEvent *event(dmaDoneEventFree.back());
        dmaDoneEventFree.pop_back();
        assert(event);
        assert(!event->scheduled());

        // We use a uncachable request here because the requests from the CPU
        // will be uncacheable as well. If we have uncacheable and cacheable
        // requests in the memory system for the same address it won't be
        // pleased
        uint8_t *const dma_dst(
            virtualDisplayBuffer.data() + dmaCurAddr - dmaStartAddr);
        event->setTransactionSize(transaction_size);
        dmaPort.dmaAction(MemCmd::ReadReq, dmaCurAddr, transaction_size, event,
                          dma_dst, 0, Request::UNCACHEABLE);
        dmaCurAddr += transaction_size;
        dmaBytesInFlight += transaction_size;
    }
}

void
HDLcd::renderPixel()
{
    // try to handle multiple pixels at a time; doing so reduces the accuracy
    //   of the underrun detection but lowers simulation overhead
    const size_t count = 32;
    assert(width() % count == 0); // not set up to handle trailing pixels

    // have we underrun on this frame anytime before?
    if (frameUnderrun) {
        // the LCD controller gives up on a frame if an underrun occurs and
        //   resumes regular operation on the next frame
        pixelBufferSize = 0;
    } else {
        // did we underrun on this set of pixels?
        if (pixelBufferSize < bytesPerPixel() * count) {
            warn("HDLcd controller buffer underrun\n");
            frameUnderrun = true;
            int_rawstat.underrun = 1;
            if (!intEvent.scheduled())
                schedule(intEvent, clockEdge());
        } else {
            // emulate the pixel read from the internal buffer
            pixelBufferSize -= bytesPerPixel() * count;
        }
    }

    // the DMA may have previously stalled due to the buffer being full;
    //   give it a kick; it knows not to fill if at end of frame, underrun, etc
    if (!fillPixelBufferEvent.scheduled())
        schedule(fillPixelBufferEvent, clockEdge());

    // schedule the next pixel read according to where it is in the frame
    pixelIndex += count;
    assert(pixelIndex <= width() * height());
    size_t x = pixelIndex % width();
    Tick nextEventTick = curTick();
    if (x == 0) {
        // start of new line
        nextEventTick += pixelClock * ((h_front_porch.val + 1) +
                                       (h_back_porch.val + 1) +
                                       (h_sync.val + 1));
        if (pixelIndex == width() * height()) {
            // end of frame
            nextEventTick += PClksPerLine() * (v_front_porch.val + 1) *
                             pixelClock;
            schedule(endFrameEvent, nextEventTick);
            return;
        }
    } else {
        nextEventTick += pixelClock * count;
    }

    schedule(renderPixelEvent, nextEventTick);
}

PixelConverter
HDLcd::pixelConverter() const
{
    ByteOrder byte_order(
        pixel_format.big_endian ? BigEndianByteOrder : LittleEndianByteOrder);

    /* Some Linux kernels have a broken driver that swaps the red and
     * blue color select registers. */
    if (!workaround_swap_rb) {
        return PixelConverter(
            bytesPerPixel(),
            red_select.offset, green_select.offset, blue_select.offset,
            red_select.size, green_select.size, blue_select.size,
            byte_order);
    } else {
        return PixelConverter(
            bytesPerPixel(),
            blue_select.offset, green_select.offset, red_select.offset,
            blue_select.size, green_select.size, red_select.size,
            byte_order);
    }
}

void
HDLcd::endFrame() {
    assert(pixelBufferSize == 0);
    assert(dmaPendingNum == 0);
    assert(dmaBytesInFlight == 0);
    assert(dmaDoneEventFree.size() == dmaDoneEventAll.size());

    fb.copyIn(virtualDisplayBuffer, pixelConverter());

    if (vnc)
        vnc->setDirty();

    if (enableCapture) {
        if (!pic)
            pic = simout.create(csprintf("%s.framebuffer.bmp", sys->name()), true);

        assert(pic);
        pic->seekp(0);
        bmp.write(*pic);
    }

    // start the next frame
    frameUnderway = false;
    startFrame();
}

void
HDLcd::dmaDone(DmaDoneEvent *event)
{
    const size_t transactionLength = event->getTransactionSize();
    assert(pixelBufferSize + transactionLength < PIXEL_BUFFER_CAPACITY);
    assert(dmaCurAddr <= dmaMaxAddr);

    dmaDoneEventFree.push_back(event);
    --dmaPendingNum;
    assert(MAX_OUTSTANDING_DMA_REQ_CAPACITY - dmaDoneEventFree.size() ==
            dmaPendingNum);

    // add the data to the pixel buffer
    dmaBytesInFlight -= transactionLength;
    pixelBufferSize += transactionLength;

    // schedule another dma transaction if:
    // - we're not done reading the frame
    // - there is sufficient room in the pixel buffer for another transaction
    // - another fillPixelBufferEvent is not already scheduled
    const size_t targetTransSize = dmaBurstLength() * AXI_PORT_WIDTH;
    if ((dmaCurAddr < dmaMaxAddr) &&
        (bytesFreeInPixelBuffer() + targetTransSize < PIXEL_BUFFER_CAPACITY) &&
        !fillPixelBufferEvent.scheduled()) {
        schedule(fillPixelBufferEvent, clockEdge());
    }
}

void
HDLcd::serialize(CheckpointOut &cp) const
{
    DPRINTF(HDLcd, "Serializing ARM HDLCD\n");

    const uint32_t version_serial = version;
    SERIALIZE_SCALAR(version_serial);
    const uint32_t int_rawstat_serial = int_rawstat;
    SERIALIZE_SCALAR(int_rawstat_serial);
    const uint32_t int_clear_serial = int_clear;
    SERIALIZE_SCALAR(int_clear_serial);
    const uint32_t int_mask_serial = int_mask;
    SERIALIZE_SCALAR(int_mask_serial);
    const uint32_t int_status_serial = int_status;
    SERIALIZE_SCALAR(int_status_serial);

    SERIALIZE_SCALAR(fb_base);
    SERIALIZE_SCALAR(fb_line_length);

    const uint32_t fb_line_count_serial = fb_line_count;
    SERIALIZE_SCALAR(fb_line_count_serial);

    SERIALIZE_SCALAR(fb_line_pitch);

    const uint32_t bus_options_serial = bus_options;
    SERIALIZE_SCALAR(bus_options_serial);
    const uint32_t v_sync_serial = v_sync;
    SERIALIZE_SCALAR(v_sync_serial);
    const uint32_t v_back_porch_serial = v_back_porch;
    SERIALIZE_SCALAR(v_back_porch_serial);
    const uint32_t v_data_serial = v_data;
    SERIALIZE_SCALAR(v_data_serial);
    const uint32_t v_front_porch_serial = v_front_porch;
    SERIALIZE_SCALAR(v_front_porch_serial);
    const uint32_t h_sync_serial = h_sync;
    SERIALIZE_SCALAR(h_sync_serial);
    const uint32_t h_back_porch_serial = h_back_porch;
    SERIALIZE_SCALAR(h_back_porch_serial);
    const uint32_t h_data_serial = h_data;
    SERIALIZE_SCALAR(h_data_serial);
    const uint32_t h_front_porch_serial = h_front_porch;
    SERIALIZE_SCALAR(h_front_porch_serial);
    const uint32_t polarities_serial = polarities;
    SERIALIZE_SCALAR(polarities_serial);
    const uint32_t command_serial = command;
    SERIALIZE_SCALAR(command_serial);
    const uint32_t pixel_format_serial = pixel_format;
    SERIALIZE_SCALAR(pixel_format_serial);
    const uint32_t red_select_serial = red_select;
    SERIALIZE_SCALAR(red_select_serial);
    const uint32_t green_select_serial = green_select;
    SERIALIZE_SCALAR(green_select_serial);
    const uint32_t blue_select_serial = blue_select;
    SERIALIZE_SCALAR(blue_select_serial);

    SERIALIZE_SCALAR(frameReadStartTime);
    SERIALIZE_SCALAR(dmaStartAddr);
    SERIALIZE_SCALAR(dmaCurAddr);
    SERIALIZE_SCALAR(dmaMaxAddr);
    SERIALIZE_SCALAR(dmaPendingNum);
    SERIALIZE_SCALAR(frameUnderrun);

    arrayParamOut(cp, "virtualDisplayBuffer", virtualDisplayBuffer);

    SERIALIZE_SCALAR(pixelBufferSize);
    SERIALIZE_SCALAR(pixelIndex);
    SERIALIZE_SCALAR(doUpdateParams);
    SERIALIZE_SCALAR(frameUnderway);
    SERIALIZE_SCALAR(dmaBytesInFlight);

    Tick start_event_time = 0;
    Tick end_event_time = 0;
    Tick render_pixel_event_time = 0;
    Tick fill_pixel_buffer_event_time = 0;
    Tick int_event_time = 0;
    if (startFrameEvent.scheduled())
        start_event_time = startFrameEvent.when();
    if (endFrameEvent.scheduled())
        end_event_time = endFrameEvent.when();
    if (renderPixelEvent.scheduled())
        render_pixel_event_time = renderPixelEvent.when();
    if (fillPixelBufferEvent.scheduled())
        fill_pixel_buffer_event_time = fillPixelBufferEvent.when();
    if (intEvent.scheduled())
        int_event_time = intEvent.when();
    SERIALIZE_SCALAR(start_event_time);
    SERIALIZE_SCALAR(end_event_time);
    SERIALIZE_SCALAR(render_pixel_event_time);
    SERIALIZE_SCALAR(fill_pixel_buffer_event_time);
    SERIALIZE_SCALAR(int_event_time);

    vector<Tick> dma_done_event_tick(MAX_OUTSTANDING_DMA_REQ_CAPACITY);
    vector<size_t> dma_done_event_burst_len(MAX_OUTSTANDING_DMA_REQ_CAPACITY);
    for (int x = 0; x < MAX_OUTSTANDING_DMA_REQ_CAPACITY; ++x) {
        dma_done_event_tick[x] = dmaDoneEventAll[x].scheduled() ?
            dmaDoneEventAll[x].when() : 0;
        dma_done_event_burst_len[x] = dmaDoneEventAll[x].scheduled() ?
            dmaDoneEventAll[x].getTransactionSize() : 0;
    }
    arrayParamOut(cp, "dma_done_event_tick", dma_done_event_tick);
    arrayParamOut(cp, "dma_done_event_burst_length", dma_done_event_burst_len);
}

void
HDLcd::unserialize(CheckpointIn &cp)
{
    uint32_t version_serial, int_rawstat_serial, int_clear_serial,
            int_mask_serial, int_status_serial, fb_line_count_serial,
            bus_options_serial, v_sync_serial, v_back_porch_serial,
            v_data_serial, v_front_porch_serial, h_sync_serial,
            h_back_porch_serial, h_data_serial, h_front_porch_serial,
            polarities_serial, command_serial, pixel_format_serial,
            red_select_serial, green_select_serial, blue_select_serial;

    DPRINTF(HDLcd, "Unserializing ARM HDLCD\n");

    UNSERIALIZE_SCALAR(version_serial);
    version = version_serial;
    UNSERIALIZE_SCALAR(int_rawstat_serial);
    int_rawstat = int_rawstat_serial;
    UNSERIALIZE_SCALAR(int_clear_serial);
    int_clear = int_clear_serial;
    UNSERIALIZE_SCALAR(int_mask_serial);
    int_mask = int_mask_serial;
    UNSERIALIZE_SCALAR(int_status_serial);
    int_status = int_status_serial;

    UNSERIALIZE_SCALAR(fb_base);
    UNSERIALIZE_SCALAR(fb_line_length);

    UNSERIALIZE_SCALAR(fb_line_count_serial);
    fb_line_count = fb_line_count_serial;

    UNSERIALIZE_SCALAR(fb_line_pitch);

    UNSERIALIZE_SCALAR(bus_options_serial);
    bus_options = bus_options_serial;
    UNSERIALIZE_SCALAR(v_sync_serial);
    v_sync = v_sync_serial;
    UNSERIALIZE_SCALAR(v_back_porch_serial);
    v_back_porch = v_back_porch_serial;
    UNSERIALIZE_SCALAR(v_data_serial);
    v_data = v_data_serial;
    UNSERIALIZE_SCALAR(v_front_porch_serial);
    v_front_porch = v_front_porch_serial;
    UNSERIALIZE_SCALAR(h_sync_serial);
    h_sync = h_sync_serial;
    UNSERIALIZE_SCALAR(h_back_porch_serial);
    h_back_porch = h_back_porch_serial;
    UNSERIALIZE_SCALAR(h_data_serial);
    h_data = h_data_serial;
    UNSERIALIZE_SCALAR(h_front_porch_serial);
    h_front_porch = h_front_porch_serial;
    UNSERIALIZE_SCALAR(polarities_serial);
    polarities = polarities_serial;
    UNSERIALIZE_SCALAR(command_serial);
    command = command_serial;
    UNSERIALIZE_SCALAR(pixel_format_serial);
    pixel_format = pixel_format_serial;
    UNSERIALIZE_SCALAR(red_select_serial);
    red_select = red_select_serial;
    UNSERIALIZE_SCALAR(green_select_serial);
    green_select = green_select_serial;
    UNSERIALIZE_SCALAR(blue_select_serial);
    blue_select = blue_select_serial;

    UNSERIALIZE_SCALAR(frameReadStartTime);
    UNSERIALIZE_SCALAR(dmaStartAddr);
    UNSERIALIZE_SCALAR(dmaCurAddr);
    UNSERIALIZE_SCALAR(dmaMaxAddr);
    UNSERIALIZE_SCALAR(dmaPendingNum);
    UNSERIALIZE_SCALAR(frameUnderrun);
    UNSERIALIZE_SCALAR(dmaBytesInFlight);

    arrayParamIn(cp, "virtualDisplayBuffer", virtualDisplayBuffer);

    UNSERIALIZE_SCALAR(pixelBufferSize);
    UNSERIALIZE_SCALAR(pixelIndex);
    UNSERIALIZE_SCALAR(doUpdateParams);
    UNSERIALIZE_SCALAR(frameUnderway);

    Tick start_event_time = 0;
    Tick end_event_time = 0;
    Tick render_pixel_event_time = 0;
    Tick fill_pixel_buffer_event_time = 0;
    Tick int_event_time = 0;
    UNSERIALIZE_SCALAR(start_event_time);
    UNSERIALIZE_SCALAR(end_event_time);
    UNSERIALIZE_SCALAR(render_pixel_event_time);
    UNSERIALIZE_SCALAR(fill_pixel_buffer_event_time);
    UNSERIALIZE_SCALAR(int_event_time);
    if (start_event_time)
        schedule(startFrameEvent, start_event_time);
    if (end_event_time)
        schedule(endFrameEvent, end_event_time);
    if (render_pixel_event_time)
        schedule(renderPixelEvent, render_pixel_event_time);
    if (fill_pixel_buffer_event_time)
        schedule(fillPixelBufferEvent, fill_pixel_buffer_event_time);
    if (int_event_time)
        schedule(intEvent, int_event_time);

    vector<Tick> dma_done_event_tick(MAX_OUTSTANDING_DMA_REQ_CAPACITY);
    vector<Tick> dma_done_event_burst_len(MAX_OUTSTANDING_DMA_REQ_CAPACITY);
    arrayParamIn(cp, "dma_done_event_tick", dma_done_event_tick);
    arrayParamIn(cp, "dma_done_event_burst_length", dma_done_event_burst_len);
    dmaDoneEventFree.clear();
    for (int x = 0; x < MAX_OUTSTANDING_DMA_REQ_CAPACITY; ++x) {
        if (dma_done_event_tick[x]) {
            dmaDoneEventAll[x].setTransactionSize(dma_done_event_burst_len[x]);
            schedule(dmaDoneEventAll[x], dma_done_event_tick[x]);
        } else
            dmaDoneEventFree.push_back(&dmaDoneEventAll[x]);
    }
    assert(MAX_OUTSTANDING_DMA_REQ_CAPACITY - dmaDoneEventFree.size() == dmaPendingNum);

    if (frameUnderway) {
        updateVideoParams(true);
        fb.resize(width(), height());
        fb.copyIn(virtualDisplayBuffer, pixelConverter());
        if (vnc)
            vnc->setDirty();
    }
}

void
HDLcd::generateInterrupt()
{
    int_status = int_rawstat & int_mask;
    DPRINTF(HDLcd, "Generate Interrupt: int_rawstat=0x%08x int_mask=0x%08x "
            "int_status=0x%08x\n",
            (uint32_t)int_rawstat, (uint32_t)int_mask, (uint32_t)int_status);

    if (int_status != 0) {
        gic->sendInt(intNum);
        DPRINTF(HDLcd, " -- Generated\n");
    }
}

AddrRangeList
HDLcd::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

HDLcd *
HDLcdParams::create()
{
    return new HDLcd(this);
}
