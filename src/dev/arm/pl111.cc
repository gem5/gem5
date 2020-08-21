/*
 * Copyright (c) 2010-2012, 2015 ARM Limited
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

#include "dev/arm/pl111.hh"

#include "base/output.hh"
#include "base/trace.hh"
#include "base/vnc/vncinput.hh"
#include "debug/PL111.hh"
#include "debug/Uart.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::vector;

// initialize clcd registers
Pl111::Pl111(const Params *p)
    : AmbaDmaDevice(p), lcdTiming0(0), lcdTiming1(0), lcdTiming2(0),
      lcdTiming3(0), lcdUpbase(0), lcdLpbase(0), lcdControl(0), lcdImsc(0),
      lcdRis(0), lcdMis(0),
      clcdCrsrCtrl(0), clcdCrsrConfig(0), clcdCrsrPalette0(0),
      clcdCrsrPalette1(0), clcdCrsrXY(0), clcdCrsrClip(0), clcdCrsrImsc(0),
      clcdCrsrIcr(0), clcdCrsrRis(0), clcdCrsrMis(0),
      pixelClock(p->pixel_clock),
      converter(PixelConverter::rgba8888_le), fb(LcdMaxWidth, LcdMaxHeight),
      vnc(p->vnc), bmp(&fb), pic(NULL),
      width(LcdMaxWidth), height(LcdMaxHeight),
      bytesPerPixel(4), startTime(0), startAddr(0), maxAddr(0), curAddr(0),
      waterMark(0), dmaPendingNum(0),
      readEvent([this]{ readFramebuffer(); }, name()),
      fillFifoEvent([this]{ fillFifo(); }, name()),
      dmaDoneEventAll(maxOutstandingDma, this),
      dmaDoneEventFree(maxOutstandingDma),
      intEvent([this]{ generateInterrupt(); }, name()),
      enableCapture(p->enable_capture)
{
    pioSize = 0xFFFF;

    dmaBuffer = new uint8_t[buffer_size];

    memset(lcdPalette, 0, sizeof(lcdPalette));
    memset(cursorImage, 0, sizeof(cursorImage));
    memset(dmaBuffer, 0, buffer_size);

    for (int i = 0; i < maxOutstandingDma; ++i)
        dmaDoneEventFree[i] = &dmaDoneEventAll[i];

    if (vnc)
        vnc->setFrameBuffer(&fb);
}

Pl111::~Pl111()
{
    delete[] dmaBuffer;
}

// read registers and frame buffer
Tick
Pl111::read(PacketPtr pkt)
{
    // use a temporary data since the LCD registers are read/written with
    // different size operations

    uint32_t data = 0;

    assert(pkt->getAddr() >= pioAddr &&
           pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(PL111, " read register %#x size=%d\n", daddr, pkt->getSize());

    switch (daddr) {
      case LcdTiming0:
        data = lcdTiming0;
        break;
      case LcdTiming1:
        data = lcdTiming1;
        break;
      case LcdTiming2:
        data = lcdTiming2;
        break;
      case LcdTiming3:
        data = lcdTiming3;
        break;
      case LcdUpBase:
        data = lcdUpbase;
        break;
      case LcdLpBase:
        data = lcdLpbase;
        break;
      case LcdControl:
        data = lcdControl;
        break;
      case LcdImsc:
        data = lcdImsc;
        break;
      case LcdRis:
        data = lcdRis;
        break;
      case LcdMis:
        data = lcdMis;
        break;
      case LcdIcr:
        panic("LCD register at offset %#x is Write-Only\n", daddr);
        break;
      case LcdUpCurr:
        data = curAddr;
        break;
      case LcdLpCurr:
        data = curAddr;
        break;
      case ClcdCrsrCtrl:
        data = clcdCrsrCtrl;
        break;
      case ClcdCrsrConfig:
        data = clcdCrsrConfig;
        break;
      case ClcdCrsrPalette0:
        data = clcdCrsrPalette0;
        break;
      case ClcdCrsrPalette1:
        data = clcdCrsrPalette1;
        break;
      case ClcdCrsrXY:
        data = clcdCrsrXY;
        break;
      case ClcdCrsrClip:
        data = clcdCrsrClip;
        break;
      case ClcdCrsrImsc:
        data = clcdCrsrImsc;
        break;
      case ClcdCrsrIcr:
        panic("CLCD register at offset %#x is Write-Only\n", daddr);
        break;
      case ClcdCrsrRis:
        data = clcdCrsrRis;
        break;
      case ClcdCrsrMis:
        data = clcdCrsrMis;
        break;
      default:
        if (readId(pkt, AMBA_ID, pioAddr)) {
            // Hack for variable size accesses
            data = pkt->getUintX(ByteOrder::little);
            break;
        } else if (daddr >= CrsrImage && daddr <= 0xBFC) {
            // CURSOR IMAGE
            int index;
            index = (daddr - CrsrImage) >> 2;
            data= cursorImage[index];
            break;
        } else if (daddr >= LcdPalette && daddr <= 0x3FC) {
            // LCD Palette
            int index;
            index = (daddr - LcdPalette) >> 2;
            data = lcdPalette[index];
            break;
        } else {
            panic("Tried to read CLCD register at offset %#x that "
                       "doesn't exist\n", daddr);
            break;
        }
    }

    pkt->setUintX(data, ByteOrder::little);
    pkt->makeAtomicResponse();
    return pioDelay;
}

// write registers and frame buffer
Tick
Pl111::write(PacketPtr pkt)
{
    // use a temporary data since the LCD registers are read/written with
    // different size operations
    //
    const uint32_t data = pkt->getUintX(ByteOrder::little);

    assert(pkt->getAddr() >= pioAddr &&
           pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(PL111, " write register %#x value %#x size=%d\n", daddr,
            pkt->getLE<uint8_t>(), pkt->getSize());

    switch (daddr) {
      case LcdTiming0:
        lcdTiming0 = data;
        // width = 16 * (PPL+1)
        width = (lcdTiming0.ppl + 1) << 4;
        break;
      case LcdTiming1:
        lcdTiming1 = data;
        // height = LPP + 1
        height = (lcdTiming1.lpp) + 1;
        break;
      case LcdTiming2:
        lcdTiming2 = data;
        break;
      case LcdTiming3:
        lcdTiming3 = data;
        break;
      case LcdUpBase:
        lcdUpbase = data;
        DPRINTF(PL111, "####### Upper panel base set to: %#x #######\n", lcdUpbase);
        break;
      case LcdLpBase:
        warn_once("LCD dual screen mode not supported\n");
        lcdLpbase = data;
        DPRINTF(PL111, "###### Lower panel base set to: %#x #######\n", lcdLpbase);
        break;
      case LcdControl:
        int old_lcdpwr;
        old_lcdpwr = lcdControl.lcdpwr;
        lcdControl = data;

        DPRINTF(PL111, "LCD power is:%d\n", lcdControl.lcdpwr);

        // LCD power enable
        if (lcdControl.lcdpwr && !old_lcdpwr) {
            updateVideoParams();
            DPRINTF(PL111, " lcd size: height %d width %d\n", height, width);
            waterMark = lcdControl.watermark ? 8 : 4;
            startDma();
        }
        break;
      case LcdImsc:
        lcdImsc = data;
        if (lcdImsc.vcomp)
            panic("Interrupting on vcomp not supported\n");

        lcdMis = lcdImsc & lcdRis;

        if (!lcdMis)
            interrupt->clear();

         break;
      case LcdRis:
        panic("LCD register at offset %#x is Read-Only\n", daddr);
        break;
      case LcdMis:
        panic("LCD register at offset %#x is Read-Only\n", daddr);
        break;
      case LcdIcr:
        lcdRis = lcdRis & ~data;
        lcdMis = lcdImsc & lcdRis;

        if (!lcdMis)
            interrupt->clear();

        break;
      case LcdUpCurr:
        panic("LCD register at offset %#x is Read-Only\n", daddr);
        break;
      case LcdLpCurr:
        panic("LCD register at offset %#x is Read-Only\n", daddr);
        break;
      case ClcdCrsrCtrl:
        clcdCrsrCtrl = data;
        break;
      case ClcdCrsrConfig:
        clcdCrsrConfig = data;
        break;
      case ClcdCrsrPalette0:
        clcdCrsrPalette0 = data;
        break;
      case ClcdCrsrPalette1:
        clcdCrsrPalette1 = data;
        break;
      case ClcdCrsrXY:
        clcdCrsrXY = data;
        break;
      case ClcdCrsrClip:
        clcdCrsrClip = data;
        break;
      case ClcdCrsrImsc:
        clcdCrsrImsc = data;
        break;
      case ClcdCrsrIcr:
        clcdCrsrIcr = data;
        break;
      case ClcdCrsrRis:
        panic("CLCD register at offset %#x is Read-Only\n", daddr);
        break;
      case ClcdCrsrMis:
        panic("CLCD register at offset %#x is Read-Only\n", daddr);
        break;
      default:
        if (daddr >= CrsrImage && daddr <= 0xBFC) {
            // CURSOR IMAGE
            int index;
            index = (daddr - CrsrImage) >> 2;
            cursorImage[index] = data;
            break;
        } else if (daddr >= LcdPalette && daddr <= 0x3FC) {
            // LCD Palette
            int index;
            index = (daddr - LcdPalette) >> 2;
            lcdPalette[index] = data;
            break;
        } else {
            panic("Tried to write PL111 register at offset %#x that "
                       "doesn't exist\n", daddr);
            break;
        }
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

PixelConverter
Pl111::pixelConverter() const
{
    unsigned rw, gw, bw;
    unsigned offsets[3];

    switch (lcdControl.lcdbpp) {
      case bpp24:
        rw = gw = bw = 8;
        offsets[0] = 0;
        offsets[1] = 8;
        offsets[2] = 16;
        break;

      case bpp16m565:
        rw = 5;
        gw = 6;
        bw = 5;
        offsets[0] = 0;
        offsets[1] = 5;
        offsets[2] = 11;
        break;

      default:
        panic("Unimplemented video mode\n");
    }

    if (lcdControl.bgr) {
        return PixelConverter(
            bytesPerPixel,
            offsets[2], offsets[1], offsets[0],
            rw, gw, bw,
            ByteOrder::little);
    } else {
        return PixelConverter(
            bytesPerPixel,
            offsets[0], offsets[1], offsets[2],
            rw, gw, bw,
            ByteOrder::little);
    }
}

void
Pl111::updateVideoParams()
{
    if (lcdControl.lcdbpp == bpp24) {
        bytesPerPixel = 4;
    } else if (lcdControl.lcdbpp == bpp16m565) {
        bytesPerPixel = 2;
    }

    fb.resize(width, height);
    converter = pixelConverter();

    // Workaround configuration bugs where multiple display
    // controllers are attached to the same VNC server by reattaching
    // enabled devices. This isn't ideal, but works as long as only
    // one display controller is active at a time.
    if (lcdControl.lcdpwr && vnc)
        vnc->setFrameBuffer(&fb);
}

void
Pl111::startDma()
{
    if (dmaPendingNum != 0 || readEvent.scheduled())
        return;
    readFramebuffer();
}

void
Pl111::readFramebuffer()
{
    // initialization for dma read from frame buffer to dma buffer
    uint32_t length = height * width;
    if (startAddr != lcdUpbase)
        startAddr = lcdUpbase;

    // Updating base address, interrupt if we're supposed to
    lcdRis.baseaddr = 1;
    if (!intEvent.scheduled())
        schedule(intEvent, clockEdge());

    curAddr = 0;
    startTime = curTick();

    maxAddr = static_cast<Addr>(length * bytesPerPixel);

    DPRINTF(PL111, " lcd frame buffer size of %d bytes \n", maxAddr);

    fillFifo();
}

void
Pl111::fillFifo()
{
    while ((dmaPendingNum < maxOutstandingDma) && (maxAddr >= curAddr + dmaSize )) {
        // concurrent dma reads need different dma done events
        // due to assertion in scheduling state
        ++dmaPendingNum;

        assert(!dmaDoneEventFree.empty());
        DmaDoneEvent *event(dmaDoneEventFree.back());
        dmaDoneEventFree.pop_back();
        assert(!event->scheduled());

        // We use a uncachable request here because the requests from the CPU
        // will be uncacheable as well. If we have uncacheable and cacheable
        // requests in the memory system for the same address it won't be
        // pleased
        dmaPort.dmaAction(MemCmd::ReadReq, curAddr + startAddr, dmaSize,
                          event, curAddr + dmaBuffer,
                          0, Request::UNCACHEABLE);
        curAddr += dmaSize;
    }
}

void
Pl111::dmaDone()
{
    DPRINTF(PL111, "DMA Done\n");

    Tick maxFrameTime = lcdTiming2.cpl * height * pixelClock;

    --dmaPendingNum;

    if (maxAddr == curAddr && !dmaPendingNum) {
        if ((curTick() - startTime) > maxFrameTime) {
            warn("CLCD controller buffer underrun, took %d ticks when should"
                 " have taken %d\n", curTick() - startTime, maxFrameTime);
            lcdRis.underflow = 1;
            if (!intEvent.scheduled())
                schedule(intEvent, clockEdge());
        }

        assert(!readEvent.scheduled());
        fb.copyIn(dmaBuffer, converter);
        if (vnc)
            vnc->setDirty();

        if (enableCapture) {
            DPRINTF(PL111, "-- write out frame buffer into bmp\n");

            if (!pic)
                pic = simout.create(csprintf("%s.framebuffer.bmp", sys->name()),
                                    true);

            assert(pic);
            pic->stream()->seekp(0);
            bmp.write(*pic->stream());
        }

        // schedule the next read based on when the last frame started
        // and the desired fps (i.e. maxFrameTime), we turn the
        // argument into a relative number of cycles in the future
        if (lcdControl.lcden)
            schedule(readEvent, clockEdge(ticksToCycles(startTime -
                                                        curTick() +
                                                        maxFrameTime)));
    }

    if (dmaPendingNum > (maxOutstandingDma - waterMark))
        return;

    if (!fillFifoEvent.scheduled())
        schedule(fillFifoEvent, clockEdge());
}

void
Pl111::serialize(CheckpointOut &cp) const
{
    DPRINTF(PL111, "Serializing ARM PL111\n");

    uint32_t lcdTiming0_serial = lcdTiming0;
    SERIALIZE_SCALAR(lcdTiming0_serial);

    uint32_t lcdTiming1_serial = lcdTiming1;
    SERIALIZE_SCALAR(lcdTiming1_serial);

    uint32_t lcdTiming2_serial = lcdTiming2;
    SERIALIZE_SCALAR(lcdTiming2_serial);

    uint32_t lcdTiming3_serial = lcdTiming3;
    SERIALIZE_SCALAR(lcdTiming3_serial);

    SERIALIZE_SCALAR(lcdUpbase);
    SERIALIZE_SCALAR(lcdLpbase);

    uint32_t lcdControl_serial = lcdControl;
    SERIALIZE_SCALAR(lcdControl_serial);

    uint8_t lcdImsc_serial = lcdImsc;
    SERIALIZE_SCALAR(lcdImsc_serial);

    uint8_t lcdRis_serial = lcdRis;
    SERIALIZE_SCALAR(lcdRis_serial);

    uint8_t lcdMis_serial = lcdMis;
    SERIALIZE_SCALAR(lcdMis_serial);

    SERIALIZE_ARRAY(lcdPalette, LcdPaletteSize);
    SERIALIZE_ARRAY(cursorImage, CrsrImageSize);

    SERIALIZE_SCALAR(clcdCrsrCtrl);
    SERIALIZE_SCALAR(clcdCrsrConfig);
    SERIALIZE_SCALAR(clcdCrsrPalette0);
    SERIALIZE_SCALAR(clcdCrsrPalette1);
    SERIALIZE_SCALAR(clcdCrsrXY);
    SERIALIZE_SCALAR(clcdCrsrClip);

    uint8_t clcdCrsrImsc_serial = clcdCrsrImsc;
    SERIALIZE_SCALAR(clcdCrsrImsc_serial);

    uint8_t clcdCrsrIcr_serial = clcdCrsrIcr;
    SERIALIZE_SCALAR(clcdCrsrIcr_serial);

    uint8_t clcdCrsrRis_serial = clcdCrsrRis;
    SERIALIZE_SCALAR(clcdCrsrRis_serial);

    uint8_t clcdCrsrMis_serial = clcdCrsrMis;
    SERIALIZE_SCALAR(clcdCrsrMis_serial);

    SERIALIZE_SCALAR(height);
    SERIALIZE_SCALAR(width);
    SERIALIZE_SCALAR(bytesPerPixel);

    SERIALIZE_ARRAY(dmaBuffer, buffer_size);
    SERIALIZE_SCALAR(startTime);
    SERIALIZE_SCALAR(startAddr);
    SERIALIZE_SCALAR(maxAddr);
    SERIALIZE_SCALAR(curAddr);
    SERIALIZE_SCALAR(waterMark);
    SERIALIZE_SCALAR(dmaPendingNum);

    Tick int_event_time = 0;
    Tick read_event_time = 0;
    Tick fill_fifo_event_time = 0;

    if (readEvent.scheduled())
        read_event_time = readEvent.when();
    if (fillFifoEvent.scheduled())
        fill_fifo_event_time = fillFifoEvent.when();
    if (intEvent.scheduled())
        int_event_time = intEvent.when();

    SERIALIZE_SCALAR(read_event_time);
    SERIALIZE_SCALAR(fill_fifo_event_time);
    SERIALIZE_SCALAR(int_event_time);

    vector<Tick> dma_done_event_tick;
    dma_done_event_tick.resize(maxOutstandingDma);
    for (int x = 0; x < maxOutstandingDma; x++) {
        dma_done_event_tick[x] = dmaDoneEventAll[x].scheduled() ?
            dmaDoneEventAll[x].when() : 0;
    }
    SERIALIZE_CONTAINER(dma_done_event_tick);
}

void
Pl111::unserialize(CheckpointIn &cp)
{
    DPRINTF(PL111, "Unserializing ARM PL111\n");

    uint32_t lcdTiming0_serial;
    UNSERIALIZE_SCALAR(lcdTiming0_serial);
    lcdTiming0 = lcdTiming0_serial;

    uint32_t lcdTiming1_serial;
    UNSERIALIZE_SCALAR(lcdTiming1_serial);
    lcdTiming1 = lcdTiming1_serial;

    uint32_t lcdTiming2_serial;
    UNSERIALIZE_SCALAR(lcdTiming2_serial);
    lcdTiming2 = lcdTiming2_serial;

    uint32_t lcdTiming3_serial;
    UNSERIALIZE_SCALAR(lcdTiming3_serial);
    lcdTiming3 = lcdTiming3_serial;

    UNSERIALIZE_SCALAR(lcdUpbase);
    UNSERIALIZE_SCALAR(lcdLpbase);

    uint32_t lcdControl_serial;
    UNSERIALIZE_SCALAR(lcdControl_serial);
    lcdControl = lcdControl_serial;

    uint8_t lcdImsc_serial;
    UNSERIALIZE_SCALAR(lcdImsc_serial);
    lcdImsc = lcdImsc_serial;

    uint8_t lcdRis_serial;
    UNSERIALIZE_SCALAR(lcdRis_serial);
    lcdRis = lcdRis_serial;

    uint8_t lcdMis_serial;
    UNSERIALIZE_SCALAR(lcdMis_serial);
    lcdMis = lcdMis_serial;

    UNSERIALIZE_ARRAY(lcdPalette, LcdPaletteSize);
    UNSERIALIZE_ARRAY(cursorImage, CrsrImageSize);

    UNSERIALIZE_SCALAR(clcdCrsrCtrl);
    UNSERIALIZE_SCALAR(clcdCrsrConfig);
    UNSERIALIZE_SCALAR(clcdCrsrPalette0);
    UNSERIALIZE_SCALAR(clcdCrsrPalette1);
    UNSERIALIZE_SCALAR(clcdCrsrXY);
    UNSERIALIZE_SCALAR(clcdCrsrClip);

    uint8_t clcdCrsrImsc_serial;
    UNSERIALIZE_SCALAR(clcdCrsrImsc_serial);
    clcdCrsrImsc = clcdCrsrImsc_serial;

    uint8_t clcdCrsrIcr_serial;
    UNSERIALIZE_SCALAR(clcdCrsrIcr_serial);
    clcdCrsrIcr = clcdCrsrIcr_serial;

    uint8_t clcdCrsrRis_serial;
    UNSERIALIZE_SCALAR(clcdCrsrRis_serial);
    clcdCrsrRis = clcdCrsrRis_serial;

    uint8_t clcdCrsrMis_serial;
    UNSERIALIZE_SCALAR(clcdCrsrMis_serial);
    clcdCrsrMis = clcdCrsrMis_serial;

    UNSERIALIZE_SCALAR(height);
    UNSERIALIZE_SCALAR(width);
    UNSERIALIZE_SCALAR(bytesPerPixel);

    UNSERIALIZE_ARRAY(dmaBuffer, buffer_size);
    UNSERIALIZE_SCALAR(startTime);
    UNSERIALIZE_SCALAR(startAddr);
    UNSERIALIZE_SCALAR(maxAddr);
    UNSERIALIZE_SCALAR(curAddr);
    UNSERIALIZE_SCALAR(waterMark);
    UNSERIALIZE_SCALAR(dmaPendingNum);

    Tick int_event_time = 0;
    Tick read_event_time = 0;
    Tick fill_fifo_event_time = 0;

    UNSERIALIZE_SCALAR(read_event_time);
    UNSERIALIZE_SCALAR(fill_fifo_event_time);
    UNSERIALIZE_SCALAR(int_event_time);

    if (int_event_time)
        schedule(intEvent, int_event_time);
    if (read_event_time)
        schedule(readEvent, read_event_time);
    if (fill_fifo_event_time)
        schedule(fillFifoEvent, fill_fifo_event_time);

    vector<Tick> dma_done_event_tick;
    dma_done_event_tick.resize(maxOutstandingDma);
    UNSERIALIZE_CONTAINER(dma_done_event_tick);
    dmaDoneEventFree.clear();
    for (int x = 0; x < maxOutstandingDma; x++) {
        if (dma_done_event_tick[x])
            schedule(dmaDoneEventAll[x], dma_done_event_tick[x]);
        else
            dmaDoneEventFree.push_back(&dmaDoneEventAll[x]);
    }
    assert(maxOutstandingDma - dmaDoneEventFree.size() == dmaPendingNum);

    if (lcdControl.lcdpwr) {
        updateVideoParams();
        fb.copyIn(dmaBuffer, converter);
        if (vnc)
            vnc->setDirty();
    }
}

void
Pl111::generateInterrupt()
{
    DPRINTF(PL111, "Generate Interrupt: lcdImsc=0x%x lcdRis=0x%x lcdMis=0x%x\n",
            (uint32_t)lcdImsc, (uint32_t)lcdRis, (uint32_t)lcdMis);
    lcdMis = lcdImsc & lcdRis;

    if (lcdMis.underflow || lcdMis.baseaddr || lcdMis.vcomp || lcdMis.ahbmaster) {
        interrupt->raise();
        DPRINTF(PL111, " -- Generated\n");
    }
}

AddrRangeList
Pl111::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

Pl111 *
Pl111Params::create()
{
    return new Pl111(this);
}


