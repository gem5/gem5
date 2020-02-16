/*
 * Copyright (c) 2013 ARM Limited
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
 * Implementiation of a GICv2m MSI shim.
 *
 * This shim adds MSI support to GICv2.
 *
 * This should be instantiated with the appropriate number of frames,
 * and SPI numbers thereof, to the system being modelled.
 *
 * For example, in RealView.py (or whichever board setup is used), instantiate:
 *
 * gicv2m = Gicv2m(frames=[
 *              Gicv2mFrame(addr=0x12340000, spi_base=320, spi_len=64),
 *              Gicv2mFrame(addr=0x12350000, spi_base=100, spi_len=32),
 *              Gicv2mFrame(addr=0x12360000, spi_base=150, spi_len=16),
 *              Gicv2mFrame(addr=0x12370000, spi_base=190, spi_len=8),
 *              ])
 *
 */

#include "dev/arm/gic_v2m.hh"

#include "base/bitunion.hh"
#include "base/intmath.hh"
#include "debug/Checkpoint.hh"
#include "debug/GICV2M.hh"
#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Gicv2m *
Gicv2mParams::create()
{
    return new Gicv2m(this);
}

Gicv2mFrame *
Gicv2mFrameParams::create()
{
    return new Gicv2mFrame(this);
}

Gicv2m::Gicv2m(const Params *p)
    : PioDevice(p), pioDelay(p->pio_delay), frames(p->frames), gic(p->gic)
{
    // Assert SPI ranges start at 32
    for (int i = 0; i < frames.size(); i++) {
        if (frames[i]->spi_base < 32)
            fatal("Gicv2m: Frame %d's SPI base (%d) is not in SPI space\n",
                  i, frames[i]->spi_base);
    }
    unsigned int x = frames.size();
    fatal_if(!isPowerOf2(x), "Gicv2m: The v2m shim must be configured with "
              "a power-of-two number of frames\n");
    log2framenum = floorLog2(x);
}

AddrRangeList
Gicv2m::getAddrRanges() const
{
    AddrRangeList ranges;
    for (int i = 0; i < frames.size(); i++) {
        ranges.push_back(RangeSize(frames[i]->addr, FRAME_SIZE));
    }
    return ranges;
}

Tick
Gicv2m::read(PacketPtr pkt)
{
    int frame = frameFromAddr(pkt->getAddr());

    assert(frame >= 0);

    Addr offset = pkt->getAddr() - frames[frame]->addr;

    switch (offset) {
      case MSI_TYPER:
        pkt->setLE<uint32_t>((frames[frame]->spi_base << 16) |
                           frames[frame]->spi_len);
        break;

      case PER_ID4:
        pkt->setLE<uint32_t>(0x4 | ((4+log2framenum) << 4));
        // Nr of 4KB blocks used by component.  This is messy as frames are 64K
        // (16, ie 2^4) and we should assert we're given a Po2 number of frames.
        break;
      default:
        DPRINTF(GICV2M, "GICv2m: Read of unk reg %#x\n", offset);
        pkt->setLE<uint32_t>(0);
    };

    pkt->makeAtomicResponse();

    return pioDelay;
}

Tick
Gicv2m::write(PacketPtr pkt)
{
    int frame = frameFromAddr(pkt->getAddr());

    assert(frame >= 0);

    Addr offset = pkt->getAddr() - frames[frame]->addr;

    if (offset == MSI_SETSPI_NSR) {
        /* Is payload SPI number within range? */
        uint32_t m = pkt->getLE<uint32_t>();
        if (m >= frames[frame]->spi_base &&
            m < (frames[frame]->spi_base + frames[frame]->spi_len)) {
            DPRINTF(GICV2M, "GICv2m: Frame %d raising MSI %d\n", frame, m);
            gic->sendInt(m);
        }
    } else {
        DPRINTF(GICV2M, "GICv2m: Write of unk reg %#x\n", offset);
    }

    pkt->makeAtomicResponse();

    return pioDelay;
}

int
Gicv2m::frameFromAddr(Addr a) const
{
    for (int i = 0; i < frames.size(); i++) {
        if (a >= frames[i]->addr && a < (frames[i]->addr + FRAME_SIZE))
            return i;
    }
    return -1;
}
