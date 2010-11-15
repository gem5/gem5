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
 */

#include "base/trace.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/gic.hh"
#include "dev/arm/pl111.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

using namespace AmbaDev;

// initialize clcd registers
Pl111::Pl111(const Params *p)
    : AmbaDmaDevice(p), lcdTiming0(0), lcdTiming1(0), lcdTiming2(0),
      lcdTiming3(0), lcdUpbase(0), lcdLpbase(0), lcdControl(0), lcdImsc(0),
      lcdRis(0), lcdMis(0), lcdIcr(0), lcdUpcurr(0), lcdLpcurr(0),
      clcdCrsrCtrl(0), clcdCrsrConfig(0), clcdCrsrPalette0(0),
      clcdCrsrPalette1(0), clcdCrsrXY(0), clcdCrsrClip(0), clcdCrsrImsc(0),
      clcdCrsrIcr(0), clcdCrsrRis(0), clcdCrsrMis(0), clock(p->clock),
      height(0), width(0), startTime(0), startAddr(0), maxAddr(0), curAddr(0),
      waterMark(0), dmaPendingNum(0), readEvent(this), fillFifoEvent(this),
      dmaDoneEvent(maxOutstandingDma, this), intEvent(this)
{
    pioSize = 0xFFFF;

    memset(lcdPalette, 0, sizeof(lcdPalette));
    memset(cursorImage, 0, sizeof(cursorImage));
    memset(dmaBuffer, 0, sizeof(dmaBuffer));
    memset(frameBuffer, 0, sizeof(frameBuffer));
}

// read registers and frame buffer
Tick
Pl111::read(PacketPtr pkt)
{
    // use a temporary data since the LCD registers are read/written with
    // different size operations

    uint32_t data = 0;

    if ((pkt->getAddr()& 0xffff0000) == pioAddr) {

        assert(pkt->getAddr() >= pioAddr &&
               pkt->getAddr() < pioAddr + pioSize);

        Addr daddr = pkt->getAddr()&0xFFFF;
        pkt->allocate();

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
            warn("LCD interrupt set/clear function not supported\n");
            data = lcdImsc;
            break;
          case LcdRis:
            warn("LCD Raw interrupt status function not supported\n");
            data = lcdRis;
            break;
          case LcdMis:
            warn("LCD Masked interrupt status function not supported\n");
            data = lcdMis;
            break;
          case LcdIcr:
            panic("LCD register at offset %#x is Write-Only\n", daddr);
            break;
          case LcdUpCurr:
            data = lcdUpcurr;
            break;
          case LcdLpCurr:
            data = lcdLpcurr;
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
            if (AmbaDev::readId(pkt, AMBA_ID, pioAddr)) {
                // Hack for variable size accesses
                data = pkt->get<uint32_t>();
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
                panic("Tried to read CLCD register at offset %#x that \
                       doesn't exist\n", daddr);
                break;
            }
        }
    }

    switch(pkt->getSize()) {
      case 1:
        pkt->set<uint8_t>(data);
        break;
      case 2:
        pkt->set<uint16_t>(data);
        break;
      case 4:
        pkt->set<uint32_t>(data);
        break;
      default:
        panic("CLCD controller read size too big?\n");
        break;
    }

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
    uint32_t data = 0;

    switch(pkt->getSize()) {
      case 1:
        data = pkt->get<uint8_t>();
        break;
      case 2:
        data = pkt->get<uint16_t>();
        break;
      case 4:
        data = pkt->get<uint32_t>();
        break;
      default:
        panic("PL111 CLCD controller write size too big?\n");
        break;
    }

    if ((pkt->getAddr()& 0xffff0000) == pioAddr) {

        assert(pkt->getAddr() >= pioAddr &&
               pkt->getAddr() < pioAddr + pioSize);

        Addr daddr = pkt->getAddr() - pioAddr;

        DPRINTF(PL111, " write register %#x value %#x size=%d\n", daddr,
                pkt->get<uint8_t>(), pkt->getSize());

        switch (daddr) {
          case LcdTiming0:
            lcdTiming0 = data;
            // width = 16 * (PPL+1)
            width = (lcdTiming0.ppl + 1) << 4;
            break;
          case LcdTiming1:
            lcdTiming1 = data;
            // height = LPP + 1
            height  = (lcdTiming1.lpp) + 1;
            break;
          case LcdTiming2:
            lcdTiming2 = data;
            break;
          case LcdTiming3:
            lcdTiming3 = data;
            break;
          case LcdUpBase:
            lcdUpbase  = data;
            break;
          case LcdLpBase:
            warn("LCD dual screen mode not supported\n");
            lcdLpbase  = data;
            break;
          case LcdControl:
            int old_lcdpwr;
            old_lcdpwr = lcdControl.lcdpwr;
            lcdControl = data;
            // LCD power enable
            if (lcdControl.lcdpwr&&!old_lcdpwr) {
                DPRINTF(PL111, " lcd size: height %d width %d\n", height, width);
                waterMark = lcdControl.watermark ? 8 : 4;
                readFramebuffer();
            }
            break;
          case LcdImsc:
            warn("LCD interrupt mask set/clear not supported\n");
            lcdImsc    = data;
            break;
          case LcdRis:
            warn("LCD register at offset %#x is Read-Only\n", daddr);
            break;
          case LcdMis:
            warn("LCD register at offset %#x is Read-Only\n", daddr);
            break;
          case LcdIcr:
            warn("LCD interrupt clear not supported\n");
            lcdIcr     = data;
            break;
          case LcdUpCurr:
            warn("LCD register at offset %#x is Read-Only\n", daddr);
            break;
          case LcdLpCurr:
            warn("LCD register at offset %#x is Read-Only\n", daddr);
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
            warn("CLCD register at offset %#x is Read-Only\n", daddr);
            break;
          case ClcdCrsrMis:
            warn("CLCD register at offset %#x is Read-Only\n", daddr);
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
                panic("Tried to write PL111 register at offset %#x that \
                       doesn't exist\n", daddr);
                break;
            }
        }
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Pl111::readFramebuffer()
{
    // initialization for dma read from frame buffer to dma buffer
    uint32_t length  = height*width;
    if (startAddr != lcdUpbase) {
        startAddr = lcdUpbase;
    }
    curAddr = 0;
    startTime = curTick;
    maxAddr = static_cast<Addr>(length*sizeof(uint32_t));
    dmaPendingNum =0 ;

    fillFifo();
}

void
Pl111::fillFifo()
{
    while ((dmaPendingNum < maxOutstandingDma) && (maxAddr >= curAddr + dmaSize )) {
        // concurrent dma reads need different dma done events
        // due to assertion in scheduling state
        ++dmaPendingNum;
        DPRINTF(PL111, " ++ DMA pending number %d read addr %#x\n",
                dmaPendingNum, curAddr);
        assert(!dmaDoneEvent[dmaPendingNum-1].scheduled());
        dmaRead(curAddr + startAddr, dmaSize, &dmaDoneEvent[dmaPendingNum-1],
                curAddr + dmaBuffer);
        curAddr += dmaSize;
    }
}

void
Pl111::dmaDone()
{
    Tick maxFrameTime = lcdTiming2.cpl*height*clock;

    --dmaPendingNum;

    DPRINTF(PL111, " -- DMA pending number %d\n", dmaPendingNum);

    if (maxAddr == curAddr && !dmaPendingNum) {
        if ((curTick - startTime) > maxFrameTime)
            warn("CLCD controller buffer underrun, took %d cycles when should"
                 " have taken %d\n", curTick - startTime, maxFrameTime);

        // double buffering so the vnc server doesn't see a tear in the screen
        memcpy(frameBuffer, dmaBuffer, maxAddr);
        assert(!readEvent.scheduled());

        DPRINTF(PL111, "-- write out frame buffer into bmp\n");
        writeBMP(frameBuffer);

        DPRINTF(PL111, "-- schedule next dma read event at %d tick \n",
                maxFrameTime + curTick);
        schedule(readEvent, nextCycle(startTime + maxFrameTime));
    }

    if (dmaPendingNum > (maxOutstandingDma - waterMark))
        return;

    if (!fillFifoEvent.scheduled())
        schedule(fillFifoEvent, nextCycle());

}

Tick
Pl111::nextCycle()
{
    Tick nextTick = curTick + clock - 1;
    nextTick -= nextTick%clock;
    return nextTick;
}

Tick
Pl111::nextCycle(Tick beginTick)
{
    Tick nextTick = beginTick;
    if (nextTick%clock!=0)
        nextTick = nextTick - (nextTick%clock) + clock;

    assert(nextTick >= curTick);
    return nextTick;
}

// write out the frame buffer into a bitmap file
void
Pl111::writeBMP(uint32_t* frameBuffer)
{
    fstream pic;

    // write out bmp head
    std::string filename = "./m5out/frameBuffer.bmp";
    pic.open(filename.c_str(), ios::out|ios::binary);
    Bitmap bm(pic, height, width);

    DPRINTF(PL111, "-- write out data into bmp\n");

    // write out frame buffer data
    for (int i = height -1; i >= 0; --i) {
        for (int j = 0; j< width; ++j) {
            uint32_t pixel = frameBuffer[i*width + j];
            pic.write(reinterpret_cast<char*>(&pixel),
                      sizeof(uint32_t));
            DPRINTF(PL111, " write pixel data  %#x at addr %#x\n",
                    pixel, i*width + j);
        }
    }

    pic.close();
}

void
Pl111::serialize(std::ostream &os)
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

    uint8_t lcdIcr_serial = lcdIcr;
    SERIALIZE_SCALAR(lcdIcr_serial);

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

    SERIALIZE_SCALAR(clock);
    SERIALIZE_SCALAR(height);
    SERIALIZE_SCALAR(width);

    SERIALIZE_ARRAY(dmaBuffer, height*width);
    SERIALIZE_ARRAY(frameBuffer, height*width);
    SERIALIZE_SCALAR(startTime);
    SERIALIZE_SCALAR(startAddr);
    SERIALIZE_SCALAR(maxAddr);
    SERIALIZE_SCALAR(curAddr);
    SERIALIZE_SCALAR(waterMark);
    SERIALIZE_SCALAR(dmaPendingNum);
}

void
Pl111::unserialize(Checkpoint *cp, const std::string &section)
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

    uint8_t lcdIcr_serial;
    UNSERIALIZE_SCALAR(lcdIcr_serial);
    lcdIcr = lcdIcr_serial;

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

    UNSERIALIZE_SCALAR(clock);
    UNSERIALIZE_SCALAR(height);
    UNSERIALIZE_SCALAR(width);

    UNSERIALIZE_ARRAY(dmaBuffer, height*width);
    UNSERIALIZE_ARRAY(frameBuffer, height*width);
    UNSERIALIZE_SCALAR(startTime);
    UNSERIALIZE_SCALAR(startAddr);
    UNSERIALIZE_SCALAR(maxAddr);
    UNSERIALIZE_SCALAR(curAddr);
    UNSERIALIZE_SCALAR(waterMark);
    UNSERIALIZE_SCALAR(dmaPendingNum);
}

void
Pl111::generateInterrupt()
{
    DPRINTF(PL111, "Generate Interrupt: lcdImsc=0x%x lcdRis=0x%x lcdMis=0x%x\n",
            lcdImsc, lcdRis, lcdMis);
    lcdMis = lcdImsc & lcdRis;

    if (lcdMis.ffufie || lcdMis.nbupie || lcdMis.vtcpie || lcdMis.ahmeie) {
        gic->sendInt(intNum);
        DPRINTF(PL111, " -- Generated\n");
    }
}

void
Pl111::addressRanges(AddrRangeList& range_list)
{
    range_list.clear();
    range_list.push_back(RangeSize(pioAddr, pioSize));
}

Pl111 *
Pl111Params::create()
{
    return new Pl111(this);
}

// bitmap class ctor
Bitmap::Bitmap(std::fstream& bmp, uint16_t h, uint16_t w)
{
    Magic  magic  = {{'B','M'}};
    Header header = {sizeof(Color)*w*h , 0, 0, 54};
    Info   info   = {sizeof(Info), w, h, 1, sizeof(Color)*8, 0,
                     ( sizeof(Color) *(w*h) ), 1, 1, 0, 0};

    bmp.write(reinterpret_cast<char*>(&magic),  sizeof(magic));
    bmp.write(reinterpret_cast<char*>(&header), sizeof(header));
    bmp.write(reinterpret_cast<char*>(&info),   sizeof(info));
}
