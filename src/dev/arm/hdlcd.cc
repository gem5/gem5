/*
 * Copyright (c) 2010-2013, 2015, 2017 ARM Limited
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
 *          Andreas Sandberg
 */

#include "dev/arm/hdlcd.hh"

#include "base/output.hh"
#include "base/trace.hh"
#include "base/vnc/vncinput.hh"
#include "debug/Checkpoint.hh"
#include "debug/HDLcd.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/base_gic.hh"
#include "enums/ImageFormat.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/HDLcd.hh"
#include "sim/system.hh"

using std::vector;


// initialize hdlcd registers
HDLcd::HDLcd(const HDLcdParams *p)
    : AmbaDmaDevice(p, 0xFFFF),
      // Parameters
      vnc(p->vnc),
      workaroundSwapRB(p->workaround_swap_rb),
      workaroundDmaLineCount(p->workaround_dma_line_count),
      addrRanges{RangeSize(pioAddr, pioSize)},
      enableCapture(p->enable_capture),
      pixelBufferSize(p->pixel_buffer_size),
      virtRefreshRate(p->virt_refresh_rate),

      // Registers
      version(VERSION_RESETV),
      int_rawstat(0), int_mask(0),

      fb_base(0), fb_line_length(0), fb_line_count(0), fb_line_pitch(0),
      bus_options(BUS_OPTIONS_RESETV),

      v_sync(0), v_back_porch(0), v_data(0), v_front_porch(0),
      h_sync(0), h_back_porch(0), h_data(0), h_front_porch(0),
      polarities(0),

      command(0),

      pixel_format(0),
      red_select(0), green_select(0), blue_select(0),

      virtRefreshEvent([this]{ virtRefresh(); }, name()),
      // Other
      imgFormat(p->frame_format), pic(NULL), conv(PixelConverter::rgba8888_le),
      pixelPump(*this, *p->pxl_clk, p->pixel_chunk)
{
    if (vnc)
        vnc->setFrameBuffer(&pixelPump.fb);

    imgWriter = createImgWriter(imgFormat, &pixelPump.fb);
}

HDLcd::~HDLcd()
{
}

void
HDLcd::regStats()
{
    AmbaDmaDevice::regStats();

    using namespace Stats;

    stats.underruns
        .name(name() + ".underruns")
        .desc("number of buffer underruns")
        .flags(nozero)
        ;
}

void
HDLcd::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing ARM HDLCD\n");

    SERIALIZE_SCALAR(int_rawstat);
    SERIALIZE_SCALAR(int_mask);

    SERIALIZE_SCALAR(fb_base);
    SERIALIZE_SCALAR(fb_line_length);
    SERIALIZE_SCALAR(fb_line_count);
    SERIALIZE_SCALAR(fb_line_pitch);
    SERIALIZE_SCALAR(bus_options);

    SERIALIZE_SCALAR(v_sync);
    SERIALIZE_SCALAR(v_back_porch);
    SERIALIZE_SCALAR(v_data);
    SERIALIZE_SCALAR(v_front_porch);

    SERIALIZE_SCALAR(h_sync);
    SERIALIZE_SCALAR(h_back_porch);
    SERIALIZE_SCALAR(h_data);
    SERIALIZE_SCALAR(h_front_porch);

    SERIALIZE_SCALAR(polarities);

    SERIALIZE_SCALAR(command);
    SERIALIZE_SCALAR(pixel_format);
    SERIALIZE_SCALAR(red_select);
    SERIALIZE_SCALAR(green_select);
    SERIALIZE_SCALAR(blue_select);

    SERIALIZE_OBJ(pixelPump);
    if (enabled())
        dmaEngine->serializeSection(cp, "dmaEngine");
}

void
HDLcd::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing ARM HDLCD\n");

    UNSERIALIZE_SCALAR(int_rawstat);
    UNSERIALIZE_SCALAR(int_mask);

    UNSERIALIZE_SCALAR(fb_base);
    UNSERIALIZE_SCALAR(fb_line_length);
    UNSERIALIZE_SCALAR(fb_line_count);
    UNSERIALIZE_SCALAR(fb_line_pitch);
    UNSERIALIZE_SCALAR(bus_options);

    UNSERIALIZE_SCALAR(v_sync);
    UNSERIALIZE_SCALAR(v_back_porch);
    UNSERIALIZE_SCALAR(v_data);
    UNSERIALIZE_SCALAR(v_front_porch);

    UNSERIALIZE_SCALAR(h_sync);
    UNSERIALIZE_SCALAR(h_back_porch);
    UNSERIALIZE_SCALAR(h_data);
    UNSERIALIZE_SCALAR(h_front_porch);

    UNSERIALIZE_SCALAR(polarities);

    UNSERIALIZE_SCALAR(command);
    UNSERIALIZE_SCALAR(pixel_format);
    UNSERIALIZE_SCALAR(red_select);
    UNSERIALIZE_SCALAR(green_select);
    UNSERIALIZE_SCALAR(blue_select);

    {
        // Try to unserialize the pixel pump. It might not exist if
        // we're unserializing an old checkpoint.
        ScopedCheckpointSection sec(cp, "pixelPump");
        if (cp.sectionExists(Serializable::currentSection()))
            pixelPump.unserialize(cp);
    }

    if (enabled()) {
        // Create the DMA engine and read its state from the
        // checkpoint. We don't need to worry about the pixel pump as
        // it is a proper SimObject.
        createDmaEngine();
        dmaEngine->unserializeSection(cp, "dmaEngine");

        conv = pixelConverter();
    }
}

void
HDLcd::drainResume()
{
    AmbaDmaDevice::drainResume();

    if (enabled()) {
        if (sys->bypassCaches()) {
            // We restart the HDLCD if we are in KVM mode. This
            // ensures that we always use the fast refresh logic if we
            // resume in KVM mode.
            cmdDisable();
            cmdEnable();
        } else if (!pixelPump.active()) {
            // We restored from an old checkpoint without a pixel
            // pump, start an new refresh. This typically happens when
            // restoring from old checkpoints.
            cmdEnable();
        }
    }

    // We restored from a checkpoint and need to update the VNC server
    if (pixelPump.active() && vnc)
        vnc->setDirty();
}

void
HDLcd::virtRefresh()
{
    pixelPump.renderFrame();
    schedule(virtRefreshEvent, (curTick() + virtRefreshRate));
}

// read registers and frame buffer
Tick
HDLcd::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr &&
           pkt->getAddr() < pioAddr + pioSize);

    const Addr daddr(pkt->getAddr() - pioAddr);
    panic_if(pkt->getSize() != 4,
             "Unhandled read size (address: 0x.4x, size: %u)",
             daddr, pkt->getSize());

    const uint32_t data(readReg(daddr));
    DPRINTF(HDLcd, "read register 0x%04x: 0x%x\n", daddr, data);

    pkt->set<uint32_t>(data);
    pkt->makeAtomicResponse();
    return pioDelay;
}

// write registers and frame buffer
Tick
HDLcd::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr &&
           pkt->getAddr() < pioAddr + pioSize);

    const Addr daddr(pkt->getAddr() - pioAddr);
    panic_if(pkt->getSize() != 4,
             "Unhandled read size (address: 0x.4x, size: %u)",
             daddr, pkt->getSize());
    const uint32_t data(pkt->get<uint32_t>());
    DPRINTF(HDLcd, "write register 0x%04x: 0x%x\n", daddr, data);

    writeReg(daddr, data);

    pkt->makeAtomicResponse();
    return pioDelay;
}

uint32_t
HDLcd::readReg(Addr offset)
{
    switch (offset) {
      case Version: return version;

      case Int_RawStat: return int_rawstat;
      case Int_Clear:
        panic("HDLCD INT_CLEAR register is Write-Only\n");
      case Int_Mask: return int_mask;
      case Int_Status: return intStatus();

      case Fb_Base: return fb_base;
      case Fb_Line_Length: return fb_line_length;
      case Fb_Line_Count: return fb_line_count;
      case Fb_Line_Pitch: return fb_line_pitch;
      case Bus_Options: return bus_options;

      case V_Sync: return v_sync;
      case V_Back_Porch: return v_back_porch;
      case V_Data: return v_data;
      case V_Front_Porch: return v_front_porch;
      case H_Sync: return h_sync;
      case H_Back_Porch: return h_back_porch;
      case H_Data: return h_data;
      case H_Front_Porch: return h_front_porch;
      case Polarities: return polarities;

      case Command: return command;
      case Pixel_Format: return pixel_format;
      case Red_Select: return red_select;
      case Green_Select: return green_select;
      case Blue_Select: return blue_select;

      default:
        panic("Tried to read HDLCD register that doesn't  exist\n", offset);
    }
}

void
HDLcd::writeReg(Addr offset, uint32_t value)
{
    switch (offset) {
      case Version:
        panic("HDLCD VERSION register is read-Only\n");

      case Int_RawStat:
        intRaise(value);
        return;
      case Int_Clear:
        intClear(value);
        return;
      case Int_Mask:
        intMask(value);
        return;
      case Int_Status:
        panic("HDLCD INT_STATUS register is read-Only\n");
        break;

      case Fb_Base:
        fb_base = value;
        return;

      case Fb_Line_Length:
        fb_line_length = value;
        return;

      case Fb_Line_Count:
        fb_line_count = value;
        return;

      case Fb_Line_Pitch:
        fb_line_pitch = value;
        return;

      case Bus_Options: {
          const BusOptsReg old_bus_options(bus_options);
          bus_options = value;

          if (bus_options.max_outstanding != old_bus_options.max_outstanding) {
              DPRINTF(HDLcd,
                      "Changing HDLcd outstanding DMA transactions: %d -> %d\n",
                      old_bus_options.max_outstanding,
                      bus_options.max_outstanding);

          }

          if (bus_options.burst_len != old_bus_options.burst_len) {
              DPRINTF(HDLcd,
                      "Changing HDLcd DMA burst flags: 0x%x -> 0x%x\n",
                      old_bus_options.burst_len, bus_options.burst_len);
          }
      } return;

      case V_Sync:
        v_sync = value;
        return;
      case V_Back_Porch:
        v_back_porch = value;
        return;
      case V_Data:
        v_data = value;
        return;
      case V_Front_Porch:
        v_front_porch = value;
        return;

      case H_Sync:
        h_sync = value;
        return;
      case H_Back_Porch:
        h_back_porch = value;
        return;
      case H_Data:
        h_data = value;
        return;
      case H_Front_Porch:
        h_front_porch = value;
        return;

      case Polarities:
        polarities = value;
        return;

      case Command: {
          const CommandReg new_command(value);

          if (new_command.enable != command.enable) {
              DPRINTF(HDLcd, "HDLCD switched %s\n",
                      new_command.enable ? "on" : "off");

              if (new_command.enable) {
                  cmdEnable();
              } else {
                  cmdDisable();
              }
          }
          command = new_command;
      } return;

      case Pixel_Format:
        pixel_format = value;
        return;

      case Red_Select:
        red_select = value;
        return;
      case Green_Select:
        green_select = value;
        return;
      case Blue_Select:
        blue_select = value;
        return;

      default:
        panic("Tried to write HDLCD register that doesn't exist\n", offset);
        return;
    }
}

PixelConverter
HDLcd::pixelConverter() const
{
    ByteOrder byte_order(
        pixel_format.big_endian ? BigEndianByteOrder : LittleEndianByteOrder);

    /* Some Linux kernels have a broken driver that swaps the red and
     * blue color select registers. */
    if (!workaroundSwapRB) {
        return PixelConverter(
            pixel_format.bytes_per_pixel + 1,
            red_select.offset, green_select.offset, blue_select.offset,
            red_select.size, green_select.size, blue_select.size,
            byte_order);
    } else {
        return PixelConverter(
            pixel_format.bytes_per_pixel + 1,
            blue_select.offset, green_select.offset, red_select.offset,
            blue_select.size, green_select.size, red_select.size,
            byte_order);
    }
}

DisplayTimings
HDLcd::displayTimings() const
{
    return DisplayTimings(
        h_data.val + 1, v_data.val + 1,
        h_back_porch.val + 1, h_sync.val + 1, h_front_porch.val + 1,
        v_back_porch.val + 1, v_sync.val + 1, v_front_porch.val + 1);
}

void
HDLcd::createDmaEngine()
{
    if (bus_options.max_outstanding == 0) {
        warn("Maximum number of outstanding DMA transfers set to 0.");
        return;
    }

    const uint32_t dma_burst_flags(bus_options.burst_len);
    const uint32_t dma_burst_len(
        dma_burst_flags ?
        (1UL << (findMsbSet(dma_burst_flags) - 1)) :
        MAX_BURST_LEN);
    // Some drivers seem to set the DMA line count incorrectly. This
    // could either be a driver bug or a specification bug. Unlike for
    // timings, the specification does not require 1 to be added to
    // the DMA engine's line count.
    const uint32_t dma_lines(
        fb_line_count + (workaroundDmaLineCount ? 1 : 0));

    dmaEngine.reset(new DmaEngine(
                        *this, pixelBufferSize,
                        AXI_PORT_WIDTH * dma_burst_len,
                        bus_options.max_outstanding,
                        fb_line_length, fb_line_pitch, dma_lines));
}

void
HDLcd::cmdEnable()
{
    createDmaEngine();
    conv = pixelConverter();

    // Update timing parameter before rendering frames
    pixelPump.updateTimings(displayTimings());

    if (sys->bypassCaches()) {
        schedule(virtRefreshEvent, clockEdge());
    } else {
        pixelPump.start();
    }
}

void
HDLcd::cmdDisable()
{
    pixelPump.stop();
    // Disable the virtual refresh event
    if (virtRefreshEvent.scheduled()) {
        assert(sys->bypassCaches());
        deschedule(virtRefreshEvent);
    }
    dmaEngine->abortFrame();
}

bool
HDLcd::pxlNext(Pixel &p)
{
    uint8_t pixel_data[MAX_PIXEL_SIZE];
    assert(conv.length <= sizeof(pixel_data));
    if (dmaEngine->tryGet(pixel_data, conv.length)) {
        p = conv.toPixel(pixel_data);
        return true;
    } else {
        return false;
    }
}

void
HDLcd::pxlVSyncBegin()
{
    DPRINTF(HDLcd, "Raising VSYNC interrupt.\n");
    intRaise(INT_VSYNC);
}

void
HDLcd::pxlVSyncEnd()
{
    DPRINTF(HDLcd, "End of VSYNC, starting DMA engine\n");
    dmaEngine->startFrame(fb_base);
}

void
HDLcd::pxlUnderrun()
{
    DPRINTF(HDLcd, "Buffer underrun, stopping DMA fill.\n");
    ++stats.underruns;
    intRaise(INT_UNDERRUN);
    dmaEngine->abortFrame();
}

void
HDLcd::pxlFrameDone()
{
    DPRINTF(HDLcd, "Reached end of last visible line.\n");

    if (dmaEngine->size()) {
        warn("HDLCD %u bytes still in FIFO after frame: Ensure that DMA "
             "and PixelPump configuration is consistent\n",
             dmaEngine->size());
        dmaEngine->dumpSettings();
        pixelPump.dumpSettings();
    }

    if (vnc)
        vnc->setDirty();

    if (enableCapture) {
        if (!pic) {
            pic = simout.create(
                csprintf("%s.framebuffer.%s",
                         sys->name(), imgWriter->getImgExtension()),
                true);
        }

        assert(pic);
        pic->stream()->seekp(0);
        imgWriter->write(*pic->stream());
    }
}

void
HDLcd::setInterrupts(uint32_t ints, uint32_t mask)
{
    const bool old_ints(intStatus());

    int_mask = mask;
    int_rawstat = ints;

    if (!old_ints && intStatus()) {
        gic->sendInt(intNum);
    } else if (old_ints && !intStatus()) {
        gic->clearInt(intNum);
    }
}

HDLcd::DmaEngine::DmaEngine(HDLcd &_parent, size_t size,
          unsigned request_size, unsigned max_pending,
          size_t line_size, ssize_t line_pitch, unsigned num_lines)
    : DmaReadFifo(
        _parent.dmaPort, size, request_size, max_pending,
        Request::UNCACHEABLE),
      parent(_parent),
      lineSize(line_size), linePitch(line_pitch), numLines(num_lines),
      nextLineAddr(0)
{
}

void
HDLcd::DmaEngine::serialize(CheckpointOut &cp) const
{
    DmaReadFifo::serialize(cp);

    SERIALIZE_SCALAR(nextLineAddr);
    SERIALIZE_SCALAR(frameEnd);
}

void
HDLcd::DmaEngine::unserialize(CheckpointIn &cp)
{
    DmaReadFifo::unserialize(cp);

    UNSERIALIZE_SCALAR(nextLineAddr);
    UNSERIALIZE_SCALAR(frameEnd);
}

void
HDLcd::DmaEngine::startFrame(Addr fb_base)
{
    nextLineAddr = fb_base;
    frameEnd = fb_base + numLines * linePitch;

    startFill(nextLineAddr, lineSize);
}

void
HDLcd::DmaEngine::abortFrame()
{
    nextLineAddr = frameEnd;
    stopFill();
    flush();
}


void
HDLcd::DmaEngine::dumpSettings()
{
    inform("DMA line size: %u bytes", lineSize);
    inform("DMA line pitch: %i bytes", linePitch);
    inform("DMA num lines: %u", numLines);
}

void
HDLcd::DmaEngine::onEndOfBlock()
{
    if (nextLineAddr == frameEnd)
        // We're done with this frame. Ignore calls to this method
        // until the next frame has been started.
        return;

    nextLineAddr += linePitch;
    if (nextLineAddr != frameEnd)
        startFill(nextLineAddr, lineSize);
}

void
HDLcd::DmaEngine::onIdle()
{
    parent.intRaise(INT_DMA_END);
}

void
HDLcd::PixelPump::dumpSettings()
{
    const DisplayTimings &t(timings());

    inform("PixelPump width: %u", t.width);
    inform("PixelPump height: %u", t.height);

    inform("PixelPump horizontal back porch: %u", t.hBackPorch);
    inform("PixelPump horizontal fron porch: %u", t.hFrontPorch);
    inform("PixelPump horizontal fron porch: %u", t.hSync);

    inform("PixelPump vertical back porch: %u", t.vBackPorch);
    inform("PixelPump vertical fron porch: %u", t.vFrontPorch);
    inform("PixelPump vertical fron porch: %u", t.vSync);
}


HDLcd *
HDLcdParams::create()
{
    return new HDLcd(this);
}
