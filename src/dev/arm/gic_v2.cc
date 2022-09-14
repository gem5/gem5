/*
 * Copyright (c) 2010, 2013, 2015-2018, 2020-2022 Arm Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
 * All rights reserved.
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

#include "dev/arm/gic_v2.hh"

#include "base/compiler.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/GIC.hh"
#include "debug/IPI.hh"
#include "debug/Interrupt.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5
{

const AddrRange GicV2::GICD_IGROUPR   (0x080, 0x100);
const AddrRange GicV2::GICD_ISENABLER (0x100, 0x180);
const AddrRange GicV2::GICD_ICENABLER (0x180, 0x200);
const AddrRange GicV2::GICD_ISPENDR   (0x200, 0x280);
const AddrRange GicV2::GICD_ICPENDR   (0x280, 0x300);
const AddrRange GicV2::GICD_ISACTIVER (0x300, 0x380);
const AddrRange GicV2::GICD_ICACTIVER (0x380, 0x400);
const AddrRange GicV2::GICD_IPRIORITYR(0x400, 0x800);
const AddrRange GicV2::GICD_ITARGETSR (0x800, 0xc00);
const AddrRange GicV2::GICD_ICFGR     (0xc00, 0xd00);

void
GicV2Registers::copyDistRegister(GicV2Registers* from,
                                 GicV2Registers* to,
                                 ContextID ctx, Addr daddr)
{
    auto val = from->readDistributor(ctx, daddr);
    DPRINTF(GIC, "copy dist 0x%x 0x%08x\n", daddr, val);
    to->writeDistributor(ctx, daddr, val);
}

void
GicV2Registers::copyCpuRegister(GicV2Registers* from,
                                GicV2Registers* to,
                                ContextID ctx, Addr daddr)
{
    auto val = from->readCpu(ctx, daddr);
    DPRINTF(GIC, "copy cpu  0x%x 0x%08x\n", daddr, val);
    to->writeCpu(ctx, daddr, val);
}

void
GicV2Registers::copyBankedDistRange(System *sys, GicV2Registers* from,
                                    GicV2Registers* to,
                                    Addr daddr, size_t size)
{
    for (int ctx = 0; ctx < sys->threads.size(); ++ctx)
        for (auto a = daddr; a < daddr + size; a += 4)
            copyDistRegister(from, to, ctx, a);
}

void
GicV2Registers::clearBankedDistRange(System *sys, GicV2Registers* to,
                                     Addr daddr, size_t size)
{
    for (int ctx = 0; ctx < sys->threads.size(); ++ctx)
        for (auto a = daddr; a < daddr + size; a += 4)
            to->writeDistributor(ctx, a, 0xFFFFFFFF);
}

void
GicV2Registers::copyDistRange(GicV2Registers* from,
                              GicV2Registers* to,
                              Addr daddr, size_t size)
{
    for (auto a = daddr; a < daddr + size; a += 4)
        copyDistRegister(from, to, 0, a);
}

void
GicV2Registers::clearDistRange(GicV2Registers* to, Addr daddr, size_t size)
{
    for (auto a = daddr; a < daddr + size; a += 4)
        to->writeDistributor(0, a, 0xFFFFFFFF);
}

GicV2::GicV2(const Params &p)
    : BaseGic(p),
      gicdPIDR(p.gicd_pidr),
      gicdIIDR(p.gicd_iidr),
      giccIIDR(p.gicc_iidr),
      distRange(RangeSize(p.dist_addr, DIST_SIZE)),
      cpuRange(RangeSize(p.cpu_addr, p.cpu_size)),
      addrRanges{distRange, cpuRange},
      distPioDelay(p.dist_pio_delay),
      cpuPioDelay(p.cpu_pio_delay), intLatency(p.int_latency),
      enabled(false), haveGem5Extensions(p.gem5_extensions),
      itLines(p.it_lines),
      intEnabled {}, pendingInt {}, activeInt {}, intGroup {},
      intPriority {}, intConfig {}, cpuTarget {},
      cpuSgiPending {}, cpuSgiActive {},
      cpuSgiPendingExt {}, cpuSgiActiveExt {},
      cpuPpiPending {}, cpuPpiActive {},
      pendingDelayedInterrupts(0)
{
    for (int x = 0; x < CPU_MAX; x++) {
        iccrpr[x] = 0xff;
        cpuControl[x] = 0;
        cpuPriority[x] = 0xff;
        cpuBpr[x] = GICC_BPR_MINIMUM;
        // Initialize cpu highest int
        cpuHighestInt[x] = SPURIOUS_INT;
        postIntEvent[x] =
            new EventFunctionWrapper([this, x]{ postDelayedInt(x); },
                                     "Post Interrupt to CPU");
        postFiqEvent[x] =
            new EventFunctionWrapper([this, x]{ postDelayedFiq(x); },
                                     "Post FIQ to CPU");
    }
    DPRINTF(Interrupt, "cpuEnabled[0]=%d cpuEnabled[1]=%d\n", cpuEnabled(0),
            cpuEnabled(1));

    gem5ExtensionsEnabled = false;
}

GicV2::~GicV2()
{
    for (int x = 0; x < CPU_MAX; x++) {
        delete postIntEvent[x];
        delete postFiqEvent[x];
    }
}

Tick
GicV2::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();

    if (distRange.contains(addr))
        return readDistributor(pkt);
    else if (cpuRange.contains(addr))
        return readCpu(pkt);
    else
        panic("Read to unknown address %#x\n", pkt->getAddr());
}


Tick
GicV2::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();

    if (distRange.contains(addr))
        return writeDistributor(pkt);
    else if (cpuRange.contains(addr))
        return writeCpu(pkt);
    else
        panic("Write to unknown address %#x\n", pkt->getAddr());
}

Tick
GicV2::readDistributor(PacketPtr pkt)
{
    const Addr daddr = pkt->getAddr() - distRange.start();
    const ContextID ctx = pkt->req->contextId();

    const uint32_t resp = readDistributor(ctx, daddr, pkt->getSize());

    switch (pkt->getSize()) {
      case 1:
        pkt->setLE<uint8_t>(resp);
        break;
      case 2:
        pkt->setLE<uint16_t>(resp);
        break;
      case 4:
        pkt->setLE<uint32_t>(resp);
        break;
      default:
        panic("Invalid size while reading Distributor regs in GIC: %d\n",
               pkt->getSize());
    }

    pkt->makeAtomicResponse();
    return distPioDelay;
}

uint32_t
GicV2::readDistributor(ContextID ctx, Addr daddr, size_t resp_sz)
{
    if (GICD_IGROUPR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_IGROUPR.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_IGROUPR%d (%#x)\n", ix, daddr);
        return getIntGroup(ctx, ix);
    }

    if (GICD_ISENABLER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISENABLER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ISENABLER%d (%#x)\n",
                ix, daddr);
        return getIntEnabled(ctx, ix);
    }

    if (GICD_ICENABLER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICENABLER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ICENABLER%d (%#x)\n",
                ix, daddr);
        return getIntEnabled(ctx, ix);
    }

    if (GICD_ISPENDR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISPENDR.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ISPENDR%d (%#x)\n", ix, daddr);
        return getPendingInt(ctx, ix);
    }

    if (GICD_ICPENDR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICPENDR.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ICPENDR%d (%#x)\n", ix, daddr);
        return getPendingInt(ctx, ix);
    }

    if (GICD_ISACTIVER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISACTIVER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ISACTIVER%d (%#x)\n",
                ix, daddr);
        return getActiveInt(ctx, ix);
    }

    if (GICD_ICACTIVER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICACTIVER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor read GICD_ICACTIVER%d (%#x)\n",
                ix, daddr);
        return getActiveInt(ctx, ix);
    }

    if (GICD_IPRIORITYR.contains(daddr)) {
        Addr int_num = daddr - GICD_IPRIORITYR.start();
        assert(int_num < INT_LINES_MAX);
        DPRINTF(GIC, "gic distributor read GICD_IPRIORITYR%d (%#x)\n",
                int_num, daddr);

        switch (resp_sz) {
          default: // will panic() after return to caller anyway
          case 1:
            return getIntPriority(ctx, int_num);
          case 2:
            assert((int_num + 1) < INT_LINES_MAX);
            return (getIntPriority(ctx, int_num) |
                    getIntPriority(ctx, int_num+1) << 8);
          case 4:
            assert((int_num + 3) < INT_LINES_MAX);
            return (getIntPriority(ctx, int_num) |
                    getIntPriority(ctx, int_num+1) << 8 |
                    getIntPriority(ctx, int_num+2) << 16 |
                    getIntPriority(ctx, int_num+3) << 24);
        }
    }

    if (GICD_ITARGETSR.contains(daddr)) {
        Addr int_num = daddr - GICD_ITARGETSR.start();
        DPRINTF(GIC, "gic distributor read GICD_ITARGETSR%d (%#x)\n",
                int_num, daddr);
        assert(int_num < INT_LINES_MAX);

        if (resp_sz == 1) {
            return getCpuTarget(ctx, int_num);
        } else {
            assert(resp_sz == 4);
            int_num = mbits(int_num, 31, 2);
            return (getCpuTarget(ctx, int_num) |
                    getCpuTarget(ctx, int_num+1) << 8 |
                    getCpuTarget(ctx, int_num+2) << 16 |
                    getCpuTarget(ctx, int_num+3) << 24) ;
        }
    }

    if (GICD_ICFGR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICFGR.start()) >> 2;
        DPRINTF(GIC, "gic distributor read GICD_ICFGR%d (%#x)\n", ix, daddr);
        return getIntConfig(ctx, ix);
    }

    switch(daddr) {
      case GICD_CTLR:
        DPRINTF(GIC, "gic distributor read GICD_CTLR (%#x)\n", daddr);
        return enabled;
      case GICD_TYPER:
        /* The 0x100 is a made-up flag to show that gem5 extensions
         * are available,
         * write 0x200 to this register to enable it.  */
        DPRINTF(GIC, "gic distributor read GICD_TYPER (%#x)\n", daddr);
        return (((sys->threads.numRunning() - 1) << 5) |
                (itLines/INT_BITS_MAX -1) |
                (haveGem5Extensions ? 0x100 : 0x0));
      case GICD_PIDR0:
        //ARM defined DevID
        DPRINTF(GIC, "gic distributor read GICD_PIDR0 (%#x)\n", daddr);
        return (gicdPIDR & 0xFF);
      case GICD_PIDR1:
        DPRINTF(GIC, "gic distributor read GICD_PIDR1 (%#x)\n", daddr);
        return ((gicdPIDR >> 8) & 0xFF);
      case GICD_PIDR2:
        DPRINTF(GIC, "gic distributor read GICD_PIDR2 (%#x)\n", daddr);
        return ((gicdPIDR >> 16) & 0xFF);
      case GICD_PIDR3:
        DPRINTF(GIC, "gic distributor read GICD_PIDR3 (%#x)\n", daddr);
        return ((gicdPIDR >> 24) & 0xFF);
      case GICD_IIDR:
         /* revision id is resorted to 1 and variant to 0*/
        DPRINTF(GIC, "gic distributor read GICD_IIDR (%#x)\n", daddr);
        return gicdIIDR;
      default:
        panic("Tried to read Gic distributor at offset %#x\n", daddr);
        break;
    }
}

Tick
GicV2::readCpu(PacketPtr pkt)
{
    const Addr daddr = pkt->getAddr() - cpuRange.start();

    assert(pkt->req->hasContextId());
    const ContextID ctx = pkt->req->contextId();
    assert(ctx < sys->threads.numRunning());

    pkt->setLE<uint32_t>(readCpu(ctx, daddr));

    pkt->makeAtomicResponse();
    return cpuPioDelay;
}

uint32_t
GicV2::readCpu(ContextID ctx, Addr daddr)
{
    switch(daddr) {
      case GICC_IIDR:
        DPRINTF(GIC, "gic cpu read GICC_IIDR (%#x) cpu context: %d\n",
                daddr, ctx);
        return giccIIDR;
      case GICC_CTLR:
        DPRINTF(GIC, "gic cpu read GICC_CTLR (%#x) cpu context: %d\n",
                daddr, ctx);
        return cpuControl[ctx];
      case GICC_PMR:
        DPRINTF(GIC, "gic cpu read GICC_PMR (%#x) cpu context: %d\n",
                daddr, ctx);
        return cpuPriority[ctx];
      case GICC_BPR:
        DPRINTF(GIC, "gic cpu read GICC_BPR (%#x) cpu context: %d\n",
                daddr, ctx);
        return cpuBpr[ctx];
      case GICC_IAR:
        DPRINTF(GIC, "gic cpu read GICC_IAR (%#x) cpu context: %d\n",
                daddr, ctx);
        if (enabled && cpuEnabled(ctx)) {
            int active_int = cpuHighestInt[ctx];
            IAR iar = 0;
            iar.ack_id = active_int;
            iar.cpu_id = 0;
            if (active_int < SGI_MAX) {
                // this is a software interrupt from another CPU
                if (!gem5ExtensionsEnabled) {
                    panic_if(!cpuSgiPending[active_int],
                            "Interrupt %d active but no CPU generated it?\n",
                            active_int);
                    for (int x = 0; x < sys->threads.numRunning(); x++) {
                        // See which CPU generated the interrupt
                        uint8_t cpugen =
                            bits(cpuSgiPending[active_int], 7 + 8 * x, 8 * x);
                        if (cpugen & (1 << ctx)) {
                            iar.cpu_id = x;
                            break;
                        }
                    }
                    uint64_t sgi_num = 1ULL << (ctx + 8 * iar.cpu_id);
                    cpuSgiActive[iar.ack_id] |= sgi_num;
                    cpuSgiPending[iar.ack_id] &= ~sgi_num;
                } else {
                    uint64_t sgi_num = 1ULL << iar.ack_id;
                    cpuSgiActiveExt[ctx] |= sgi_num;
                    cpuSgiPendingExt[ctx] &= ~sgi_num;
                }
            } else if (active_int < (SGI_MAX + PPI_MAX) ) {
                uint32_t int_num = 1 << (cpuHighestInt[ctx] - SGI_MAX);
                cpuPpiActive[ctx] |= int_num;
                updateRunPri();
                if (!isLevelSensitive(ctx, active_int)) {
                    cpuPpiPending[ctx] &= ~int_num;
                }

            } else {
                uint32_t int_num = 1 << intNumToBit(cpuHighestInt[ctx]);
                getActiveInt(ctx, intNumToWord(cpuHighestInt[ctx])) |= int_num;
                updateRunPri();
                if (!isLevelSensitive(ctx, active_int)) {
                    getPendingInt(ctx, intNumToWord(cpuHighestInt[ctx]))
                      &= ~int_num;
                }
            }

            DPRINTF(Interrupt,
                    "CPU %d reading IAR.id=%d IAR.cpu=%d, iar=0x%x\n",
                    ctx, iar.ack_id, iar.cpu_id, iar);
            cpuHighestInt[ctx] = SPURIOUS_INT;
            updateIntState(-1);
            clearInt(ctx, active_int);
            return iar;
        } else {
             return SPURIOUS_INT;
        }

        break;
      case GICC_RPR:
        DPRINTF(GIC, "gic cpu read GICC_RPR (%#x) cpu context: %d\n",
                daddr, ctx);
        return iccrpr[0];
      case GICC_HPPIR:
        panic("Need to implement HPIR");
        break;
      default:
        panic("Tried to read Gic cpu at offset %#x\n", daddr);
        break;
    }
}

Tick
GicV2::writeDistributor(PacketPtr pkt)
{
    const Addr daddr = pkt->getAddr() - distRange.start();

    assert(pkt->req->hasContextId());
    const ContextID ctx = pkt->req->contextId();
    const size_t data_sz = pkt->getSize();

    [[maybe_unused]] uint32_t pkt_data;
    switch (data_sz)
    {
      case 1:
        pkt_data = pkt->getLE<uint8_t>();
        break;
      case 2:
        pkt_data = pkt->getLE<uint16_t>();
        break;
      case 4:
        pkt_data = pkt->getLE<uint32_t>();
        break;
      default:
        panic("Invalid size when writing to priority regs in Gic: %d\n",
              data_sz);
    }

    writeDistributor(ctx, daddr, pkt_data, data_sz);

    pkt->makeAtomicResponse();
    return distPioDelay;
}

void
GicV2::writeDistributor(ContextID ctx, Addr daddr, uint32_t data,
                        size_t data_sz)
{
    if (GICD_IGROUPR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_IGROUPR.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC,
            "gic distributor write GICD_IGROUPR%d (%#x) size %#x value %#x \n",
            ix, daddr, data_sz, data);
        getIntGroup(ctx, ix) |= data;
        return;
    }

    if (GICD_ISENABLER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISENABLER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor write GICD_ISENABLER%d (%#x) "
                "size %#x value %#x \n",
                ix, daddr, data_sz, data);
        getIntEnabled(ctx, ix) |= data;
        return;
    }

    if (GICD_ICENABLER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICENABLER.start()) >> 2;
        assert(ix < 32);
        DPRINTF(GIC, "gic distributor write GICD_ICENABLER%d (%#x) "
                "size %#x value %#x \n",
                ix, daddr, data_sz, data);
        getIntEnabled(ctx, ix) &= ~data;
        return;
    }

    if (GICD_ISPENDR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISPENDR.start()) >> 2;
        DPRINTF(GIC,
            "gic distributor write GICD_ISPENDR%d (%#x) size %#x value %#x \n",
            ix, daddr, data_sz, data);
        auto mask = data;
        if (ix == 0) mask &= SGI_MASK; // Don't allow SGIs to be changed
        getPendingInt(ctx, ix) |= mask;
        updateIntState(ix);
        return;
    }

    if (GICD_ICPENDR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICPENDR.start()) >> 2;
        DPRINTF(GIC, "gic distributor write GICD_ICPENDR%d (%#x) "
                "size %#x value %#x \n",
                ix, daddr, data_sz, data);
        auto mask = data;
        if (ix == 0) mask &= SGI_MASK; // Don't allow SGIs to be changed
        getPendingInt(ctx, ix) &= ~mask;
        updateIntState(ix);
        return;
    }

    if (GICD_ISACTIVER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ISACTIVER.start()) >> 2;
        DPRINTF(GIC, "gic distributor write GICD_ISACTIVER%d (%#x) "
                "size %#x value %#x \n",
                ix, daddr, data_sz, data);
        getActiveInt(ctx, ix) |= data;
        return;
    }

    if (GICD_ICACTIVER.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICACTIVER.start()) >> 2;
        DPRINTF(GIC, "gic distributor write GICD_ICACTIVER%d (%#x) "
                "size %#x value %#x \n",
                ix, daddr, data_sz, data);
        getActiveInt(ctx, ix) &= ~data;
        return;
    }

    if (GICD_IPRIORITYR.contains(daddr)) {
        Addr int_num = daddr - GICD_IPRIORITYR.start();
        DPRINTF(GIC, "gic distributor write GICD_IPRIORITYR%d (%#x) "
                "size %#x value %#x\n",
                (int_num >> 2), daddr, data_sz, data);
        switch(data_sz) {
          case 1:
            getIntPriority(ctx, int_num) = data;
            break;
          case 2: {
            getIntPriority(ctx, int_num) = bits(data, 7, 0);
            getIntPriority(ctx, int_num + 1) = bits(data, 15, 8);
            break;
          }
          case 4: {
            getIntPriority(ctx, int_num) = bits(data, 7, 0);
            getIntPriority(ctx, int_num + 1) = bits(data, 15, 8);
            getIntPriority(ctx, int_num + 2) = bits(data, 23, 16);
            getIntPriority(ctx, int_num + 3) = bits(data, 31, 24);
            break;
          }
          default:
            panic("Invalid size when writing to priority regs in Gic: %d\n",
                   data_sz);
        }

        updateIntState(-1);
        updateRunPri();
        return;
    }

    if (GICD_ITARGETSR.contains(daddr)) {
        Addr int_num = daddr - GICD_ITARGETSR.start();
        // Interrupts 0-31 are read only
        unsigned offset = SGI_MAX + PPI_MAX;
        if (int_num >= offset) {
            unsigned ix = int_num - offset; // index into cpuTarget array
            DPRINTF(GIC, "gic distributor write GICD_ITARGETSR%d (%#x) "
                    "size %#x value %#x\n",
                    ix, daddr, data_sz, data);
            if (data_sz == 1) {
                cpuTarget[ix] = data & 0xff;
            } else {
                assert (data_sz == 4);
                cpuTarget[ix]   = bits(data, 7, 0);
                cpuTarget[ix+1] = bits(data, 15, 8);
                cpuTarget[ix+2] = bits(data, 23, 16);
                cpuTarget[ix+3] = bits(data, 31, 24);
            }
            updateIntState(int_num >> 2);
        }
        return;
    }

    if (GICD_ICFGR.contains(daddr)) {
        uint32_t ix = (daddr - GICD_ICFGR.start()) >> 2;
        // Since the GICD_ICFGR0 is RO (WI), we are discarding the write
        // if ix = 0
        if (ix != 0) {
            DPRINTF(GIC, "gic distributor write GICD_ICFGR%d (%#x) "
                    "size %#x value %#x\n", ix, daddr, data_sz, data);
            getIntConfig(ctx, ix) = data;
        }
        if (data & NN_CONFIG_MASK) {
            warn("GIC N:N mode selected and not supported at this time\n");
        }
        return;
    }

    switch(daddr) {
      case GICD_CTLR:
        enabled = data;
        DPRINTF(GIC, "gic distributor write GICD_CTLR (%#x) ",
                daddr, data_sz, data);
        DPRINTF(Interrupt, "Distributor enable flag set to = %d\n", enabled);
        break;
      case GICD_TYPER:
        /* 0x200 is a made-up flag to enable gem5 extension functionality.
         * This reg is not normally written.
         */
        gem5ExtensionsEnabled = (data & 0x200) && haveGem5Extensions;
        DPRINTF(GIC, "gem5 extensions %s\n",
                gem5ExtensionsEnabled ? "enabled" : "disabled");
        break;
      case GICD_SGIR:
        DPRINTF(GIC, "gic distributor write GICD_SGIR (%#x) ",
                daddr, data_sz, data);
        softInt(ctx, data);
        break;
      default:
        panic("Tried to write Gic distributor at offset %#x\n", daddr);
        break;
    }
}

Tick
GicV2::writeCpu(PacketPtr pkt)
{
    const Addr daddr = pkt->getAddr() - cpuRange.start();

    assert(pkt->req->hasContextId());
    const ContextID ctx = pkt->req->contextId();
    const uint32_t data = pkt->getLE<uint32_t>();

    writeCpu(ctx, daddr, data);

    pkt->makeAtomicResponse();
    return cpuPioDelay;
}

void
GicV2::writeCpu(ContextID ctx, Addr daddr, uint32_t data)
{
    switch(daddr) {
      case GICC_CTLR:
        DPRINTF(GIC, "gic cpu write GICC_CTLR cpu:%d %#x val: %#x\n",
                ctx, daddr, data);
        cpuControl[ctx] = data;
        break;
      case GICC_PMR:
        DPRINTF(GIC, "gic cpu write GICC_PMR cpu:%d %#x val: %#x\n",
                ctx, daddr, data);
        cpuPriority[ctx] = data;
        break;
      case GICC_BPR: {
        auto bpr = data & 0x7;
        if (bpr < GICC_BPR_MINIMUM)
            bpr = GICC_BPR_MINIMUM;
        DPRINTF(GIC, "gic cpu write GICC_BPR cpu:%d %#x val: %#x\n",
                ctx, daddr, data);
        cpuBpr[ctx] = bpr;
        break;
      }
      case GICC_EOIR: {
        DPRINTF(GIC, "gic cpu write GICC_EOIR cpu:%d %#x val: %#x\n",
                ctx, daddr, data);
        const IAR iar = data;
        if (iar.ack_id < SGI_MAX) {
            // Clear out the bit that corresponds to the cleared int
            uint64_t clr_int = 1ULL << (ctx + 8 * iar.cpu_id);
            if (!(cpuSgiActive[iar.ack_id] & clr_int) &&
                !(cpuSgiActiveExt[ctx] & (1 << iar.ack_id)))
                panic("Done handling a SGI that isn't active?\n");
            if (gem5ExtensionsEnabled)
                cpuSgiActiveExt[ctx] &= ~(1 << iar.ack_id);
            else
                cpuSgiActive[iar.ack_id] &= ~clr_int;
        } else if (iar.ack_id < (SGI_MAX + PPI_MAX) ) {
            uint32_t int_num = 1 << (iar.ack_id - SGI_MAX);
            if (!(cpuPpiActive[ctx] & int_num))
                warn("CPU %d Done handling a PPI interrupt "
                      "that isn't active?\n", ctx);
            cpuPpiActive[ctx] &= ~int_num;
        } else {
            uint32_t int_num = 1 << intNumToBit(iar.ack_id);
            if (!(getActiveInt(ctx, intNumToWord(iar.ack_id)) & int_num))
                warn("Done handling interrupt that isn't active: %d\n",
                      intNumToBit(iar.ack_id));
            getActiveInt(ctx, intNumToWord(iar.ack_id)) &= ~int_num;
        }
        updateRunPri();
        DPRINTF(Interrupt, "CPU %d done handling intr IAR = %d from cpu %d\n",
                ctx, iar.ack_id, iar.cpu_id);
        break;
      }
      case GICC_APR0:
      case GICC_APR1:
      case GICC_APR2:
      case GICC_APR3:
        warn("GIC APRn write ignored because not implemented: %#x\n", daddr);
        break;
      case GICC_DIR:
        warn("GIC DIR write ignored because not implemented: %#x\n", daddr);
        break;
      default:
        panic("Tried to write Gic cpu at offset %#x\n", daddr);
        break;
    }
    if (cpuEnabled(ctx)) updateIntState(-1);
}

GicV2::BankedRegs&
GicV2::getBankedRegs(ContextID ctx) {
    if (bankedRegs.size() <= ctx)
        bankedRegs.resize(ctx + 1);

    if (!bankedRegs[ctx])
        bankedRegs[ctx] = new BankedRegs;
    return *bankedRegs[ctx];
}

void
GicV2::softInt(ContextID ctx, SWI swi)
{
    if (gem5ExtensionsEnabled) {
        switch (swi.list_type) {
          case 0: {
             // interrupt cpus specified
             int dest = swi.cpu_list;
             DPRINTF(IPI, "Generating softIRQ from CPU %d for CPU %d\n",
                    ctx, dest);
             if (cpuEnabled(dest)) {
                 cpuSgiPendingExt[dest] |= (1 << swi.sgi_id);
                 DPRINTF(IPI, "SGI[%d]=%#x\n", dest,
                         cpuSgiPendingExt[dest]);
             }
          } break;
          case 1: {
             // interrupt all
             for (int i = 0; i < sys->threads.size(); i++) {
                 DPRINTF(IPI, "Processing CPU %d\n", i);
                 if (!cpuEnabled(i))
                     continue;
                 cpuSgiPendingExt[i] |= 1 << swi.sgi_id;
                 DPRINTF(IPI, "SGI[%d]=%#x\n", swi.sgi_id,
                         cpuSgiPendingExt[i]);
              }
          } break;
          case 2: {
            // Interrupt requesting cpu only
            DPRINTF(IPI, "Generating softIRQ from CPU %d for CPU %d\n",
                    ctx, ctx);
            if (cpuEnabled(ctx)) {
                cpuSgiPendingExt[ctx] |= (1 << swi.sgi_id);
                DPRINTF(IPI, "SGI[%d]=%#x\n", ctx,
                        cpuSgiPendingExt[ctx]);
            }
          } break;
        }
    } else {
        switch (swi.list_type) {
          case 1:
            // interrupt all
            uint8_t cpu_list;
            cpu_list = 0;
            for (int x = 0; x < sys->threads.size(); x++)
                cpu_list |= cpuEnabled(x) ? 1 << x : 0;
            swi.cpu_list = cpu_list;
            break;
          case 2:
            // interrupt requesting cpu only
            swi.cpu_list = 1 << ctx;
            break;
            // else interrupt cpus specified
        }

        DPRINTF(IPI, "Generating softIRQ from CPU %d for %#x\n", ctx,
                swi.cpu_list);
        for (int i = 0; i < sys->threads.size(); i++) {
            DPRINTF(IPI, "Processing CPU %d\n", i);
            if (!cpuEnabled(i))
                continue;
            if (swi.cpu_list & (1 << i))
                cpuSgiPending[swi.sgi_id] |= (1 << i) << (8 * ctx);
            DPRINTF(IPI, "SGI[%d]=%#x\n", swi.sgi_id,
                    cpuSgiPending[swi.sgi_id]);
        }
    }
    updateIntState(-1);
}

uint64_t
GicV2::genSwiMask(int cpu)
{
    panic_if(cpu > sys->threads.size(), "Invalid CPU ID.");
    return 0x0101010101010101ULL << cpu;
}

uint8_t
GicV2::getCpuPriority(unsigned cpu)
{
    // see Table 3-2 in IHI0048B.b (GICv2)
    // mask some low-order priority bits per BPR value
    // NB: the GIC prioritization scheme is upside down:
    // lower values are higher priority; masking off bits
    // actually creates a higher priority, not lower.
    return cpuPriority[cpu] & (0xff00 >> (7 - cpuBpr[cpu]));
}

void
GicV2::updateIntState(int hint)
{
    if (blockIntUpdate())
        return;

    for (int cpu = 0; cpu < sys->threads.size(); cpu++) {
        if (!cpuEnabled(cpu))
            continue;

        /*@todo use hint to do less work. */
        int highest_int = SPURIOUS_INT;
        // Priorities below that set in GICC_PMR can be ignored
        uint8_t highest_pri = getCpuPriority(cpu);

        // Check SGIs
        for (int swi = 0; swi < SGI_MAX; swi++) {
            if (!cpuSgiPending[swi] && !cpuSgiPendingExt[cpu])
                continue;
            if ((cpuSgiPending[swi] & genSwiMask(cpu)) ||
                (cpuSgiPendingExt[cpu] & (1 << swi)))
                if (highest_pri > getIntPriority(cpu, swi)) {
                    highest_pri = getIntPriority(cpu, swi);
                    highest_int = swi;
                }
        }

        // Check PPIs
        if (cpuPpiPending[cpu]) {
            for (int ppi_idx = 0, int_num = SGI_MAX;
                 int_num < PPI_MAX + SGI_MAX;
                 ppi_idx++, int_num++) {

                const bool ppi_pending = bits(cpuPpiPending[cpu], ppi_idx);
                const bool ppi_enabled = bits(getIntEnabled(cpu, 0), int_num);
                const bool higher_priority =
                    highest_pri > getIntPriority(cpu, int_num);

                if (ppi_pending && ppi_enabled && higher_priority) {
                    highest_pri = getIntPriority(cpu, int_num);
                    highest_int = int_num;
                }
            }
        }

        bool mp_sys = sys->threads.numRunning() > 1;
        // Check other ints
        for (int x = 0; x < (itLines/INT_BITS_MAX); x++) {
            if (getIntEnabled(cpu, x) & getPendingInt(cpu, x)) {
                for (int y = 0; y < INT_BITS_MAX; y++) {
                   uint32_t int_nm = x * INT_BITS_MAX + y;
                   DPRINTF(GIC, "Checking for interrupt# %d \n",int_nm);
                    /* Set current pending int as highest int for current cpu
                       if the interrupt's priority higher than current priority
                       and if current cpu is the target (for mp configs only)
                     */
                    if ((bits(getIntEnabled(cpu, x), y)
                        &bits(getPendingInt(cpu, x), y)) &&
                        (getIntPriority(cpu, int_nm) < highest_pri))
                        if ((!mp_sys) ||
                            (gem5ExtensionsEnabled
                               ? (getCpuTarget(cpu, int_nm) == cpu)
                               : (getCpuTarget(cpu, int_nm) & (1 << cpu)))) {
                            highest_pri = getIntPriority(cpu, int_nm);
                            highest_int = int_nm;
                        }
                }
            }
        }

        uint32_t prev_highest = cpuHighestInt[cpu];
        cpuHighestInt[cpu] = highest_int;

        if (highest_int == SPURIOUS_INT) {
            if (isLevelSensitive(cpu, prev_highest)) {

                DPRINTF(Interrupt, "Clear IRQ for cpu%d\n", cpu);
                clearInt(cpu, prev_highest);
            }
            continue;
        }

        /* @todo make this work for more than one cpu, need to handle 1:N, N:N
         * models */
        if (enabled && cpuEnabled(cpu) &&
            (highest_pri < getCpuPriority(cpu)) &&
            !(getActiveInt(cpu, intNumToWord(highest_int))
              & (1 << intNumToBit(highest_int)))) {

            DPRINTF(Interrupt, "Posting interrupt %d to cpu%d\n", highest_int,
                    cpu);

            if (isFiq(cpu, highest_int)) {
                postFiq(cpu, curTick() + intLatency);
            } else {
                postInt(cpu, curTick() + intLatency);
            }
        }
    }
}

void
GicV2::updateRunPri()
{
    for (int cpu = 0; cpu < sys->threads.size(); cpu++) {
        if (!cpuEnabled(cpu))
            continue;
        uint8_t maxPriority = 0xff;
        for (int i = 0; i < itLines; i++) {
            if (i < SGI_MAX) {
                if (((cpuSgiActive[i] & genSwiMask(cpu)) ||
                     (cpuSgiActiveExt[cpu] & (1 << i))) &&
                        (getIntPriority(cpu, i) < maxPriority))
                    maxPriority = getIntPriority(cpu, i);
            } else if (i < (SGI_MAX + PPI_MAX)) {
                if ((cpuPpiActive[cpu] & ( 1 << (i - SGI_MAX))) &&
                        (getIntPriority(cpu, i) < maxPriority))
                    maxPriority = getIntPriority(cpu, i);

            } else {
                if (getActiveInt(cpu, intNumToWord(i))
                    & (1 << intNumToBit(i)))
                    if (getIntPriority(cpu, i) < maxPriority)
                        maxPriority = getIntPriority(cpu, i);
            }
        }
        iccrpr[cpu] = maxPriority;
    }
}

void
GicV2::sendInt(uint32_t num)
{
    uint8_t target = getCpuTarget(0, num);
    DPRINTF(Interrupt, "Received Interrupt number %d,  cpuTarget %#x: \n",
            num, target);
    if ((target & (target - 1)) && !gem5ExtensionsEnabled)
        panic("Multiple targets for peripheral interrupts is not supported\n");
    panic_if(num < SGI_MAX + PPI_MAX,
             "sentInt() must only be used for interrupts 32 and higher");
    getPendingInt(target, intNumToWord(num)) |= 1 << intNumToBit(num);
    updateIntState(intNumToWord(num));
}

void
GicV2::sendPPInt(uint32_t num, uint32_t cpu)
{
    DPRINTF(Interrupt, "Received PPI %d, cpuTarget %#x: \n",
            num, cpu);
    cpuPpiPending[cpu] |= 1 << (num - SGI_MAX);
    updateIntState(intNumToWord(num));
}

void
GicV2::clearInt(uint32_t num)
{
    if (isLevelSensitive(0, num)) {
        uint8_t target = getCpuTarget(0, num);

        DPRINTF(Interrupt,
                "Received Clear interrupt number %d, cpuTarget %#x:\n",
                num, target);

        getPendingInt(target, intNumToWord(num)) &= ~(1 << intNumToBit(num));
        updateIntState(intNumToWord(num));
    } else {
        /* Nothing to do :
         * Edge-triggered interrupt remain pending until software
         * writes GICD_ICPENDR or reads GICC_IAR */
    }
}

void
GicV2::clearPPInt(uint32_t num, uint32_t cpu)
{
    if (isLevelSensitive(cpu, num)) {
        DPRINTF(Interrupt, "Clearing PPI %d, cpuTarget %#x: \n",
                num, cpu);
        cpuPpiPending[cpu] &= ~(1 << (num - SGI_MAX));
        updateIntState(intNumToWord(num));
    } else {
        /* Nothing to do :
         * Edge-triggered interrupt remain pending until software
         * writes GICD_ICPENDR or reads GICC_IAR */
    }
}

void
GicV2::clearInt(ContextID ctx, uint32_t int_num)
{
    auto tc = sys->threads[ctx];
    if (isFiq(ctx, int_num)) {
        tc->getCpuPtr()->clearInterrupt(tc->threadId(), ArmISA::INT_FIQ, 0);
    } else {
        tc->getCpuPtr()->clearInterrupt(tc->threadId(), ArmISA::INT_IRQ, 0);
    }
}

void
GicV2::postInt(uint32_t cpu, Tick when)
{
    if (!(postIntEvent[cpu]->scheduled())) {
        ++pendingDelayedInterrupts;
        eventq->schedule(postIntEvent[cpu], when);
    }
}

void
GicV2::postDelayedInt(uint32_t cpu)
{
    auto tc = sys->threads[cpu];
    tc->getCpuPtr()->postInterrupt(tc->threadId(), ArmISA::INT_IRQ, 0);
    --pendingDelayedInterrupts;
    assert(pendingDelayedInterrupts >= 0);
    if (pendingDelayedInterrupts == 0)
        signalDrainDone();
}

void
GicV2::postFiq(uint32_t cpu, Tick when)
{
    if (!(postFiqEvent[cpu]->scheduled())) {
        ++pendingDelayedInterrupts;
        eventq->schedule(postFiqEvent[cpu], when);
    }
}

bool
GicV2::supportsVersion(GicVersion version)
{
    return version == GicVersion::GIC_V2;
}

void
GicV2::postDelayedFiq(uint32_t cpu)
{
    auto tc = sys->threads[cpu];
    tc->getCpuPtr()->postInterrupt(tc->threadId(), ArmISA::INT_FIQ, 0);
    --pendingDelayedInterrupts;
    assert(pendingDelayedInterrupts >= 0);
    if (pendingDelayedInterrupts == 0)
        signalDrainDone();
}

DrainState
GicV2::drain()
{
    if (pendingDelayedInterrupts == 0) {
        return DrainState::Drained;
    } else {
        return DrainState::Draining;
    }
}


void
GicV2::drainResume()
{
    // There may be pending interrupts if checkpointed from Kvm; post them.
    updateIntState(-1);
}

void
GicV2::copyGicState(GicV2Registers* from, GicV2Registers* to)
{
    Addr set, clear;
    size_t size;

    /// CPU state (GICC_*)
    // Copy CPU Interface Control Register (CTLR),
    //      Interrupt Priority Mask Register (PMR), and
    //      Binary Point Register (BPR)
    for (int ctx = 0; ctx < sys->threads.size(); ++ctx) {
        copyCpuRegister(from, to, ctx, GICC_CTLR);
        copyCpuRegister(from, to, ctx, GICC_PMR);
        copyCpuRegister(from, to, ctx, GICC_BPR);
    }

    /// Distributor state (GICD_*)
    // Copy Distributor Control Register (CTLR)
    copyDistRegister(from, to, 0, GICD_CTLR);

    // Copy interrupt-enabled statuses (I[CS]ENABLERn; R0 is per-CPU banked)
    set   = GICD_ISENABLER.start();
    clear = GICD_ICENABLER.start();
    size  = itLines / 8;
    clearBankedDistRange(sys, to, clear, 4);
    copyBankedDistRange(sys, from, to, set, 4);

    set += 4, clear += 4, size -= 4;
    clearDistRange(to, clear, size);
    copyDistRange(from, to, set, size);

    // Copy pending interrupts (I[CS]PENDRn; R0 is per-CPU banked)
    set   = GICD_ISPENDR.start();
    clear = GICD_ICPENDR.start();
    size  = itLines / 8;
    clearBankedDistRange(sys, to, clear, 4);
    copyBankedDistRange(sys, from, to, set, 4);

    set += 4, clear += 4, size -= 4;
    clearDistRange(to, clear, size);
    copyDistRange(from, to, set, size);

    // Copy active interrupts (I[CS]ACTIVERn; R0 is per-CPU banked)
    set   = GICD_ISACTIVER.start();
    clear = GICD_ICACTIVER.start();
    size  = itLines / 8;
    clearBankedDistRange(sys, to, clear, 4);
    copyBankedDistRange(sys, from, to, set, 4);

    set += 4, clear += 4, size -= 4;
    clearDistRange(to, clear, size);
    copyDistRange(from, to, set, size);

    // Copy interrupt priorities (IPRIORITYRn; R0-7 are per-CPU banked)
    set   = GICD_IPRIORITYR.start();
    copyBankedDistRange(sys, from, to, set, 32);

    set += 32;
    size = itLines - 32;
    copyDistRange(from, to, set, size);

    // Copy interrupt processor target regs (ITARGETRn; R0-7 are read-only)
    set = GICD_ITARGETSR.start() + 32;
    size = itLines - 32;
    copyDistRange(from, to, set, size);

    // Copy interrupt configuration registers (ICFGRn)
    set = GICD_ICFGR.start();
    size = itLines / 4;
    copyDistRange(from, to, set, size);
}

void
GicV2::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm GIC\n");

    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(itLines);
    SERIALIZE_ARRAY(intEnabled, INT_BITS_MAX-1);
    SERIALIZE_ARRAY(pendingInt, INT_BITS_MAX-1);
    SERIALIZE_ARRAY(activeInt, INT_BITS_MAX-1);
    SERIALIZE_ARRAY(intGroup, INT_BITS_MAX-1);
    SERIALIZE_ARRAY(iccrpr, CPU_MAX);
    SERIALIZE_ARRAY(intPriority, GLOBAL_INT_LINES);
    SERIALIZE_ARRAY(cpuTarget, GLOBAL_INT_LINES);
    SERIALIZE_ARRAY(intConfig, INT_BITS_MAX * 2 - 2);
    SERIALIZE_ARRAY(cpuControl, CPU_MAX);
    SERIALIZE_ARRAY(cpuPriority, CPU_MAX);
    SERIALIZE_ARRAY(cpuBpr, CPU_MAX);
    SERIALIZE_ARRAY(cpuHighestInt, CPU_MAX);
    SERIALIZE_ARRAY(cpuSgiActive, SGI_MAX);
    SERIALIZE_ARRAY(cpuSgiPending, SGI_MAX);
    SERIALIZE_ARRAY(cpuSgiActiveExt, CPU_MAX);
    SERIALIZE_ARRAY(cpuSgiPendingExt, CPU_MAX);
    SERIALIZE_ARRAY(cpuPpiActive, CPU_MAX);
    SERIALIZE_ARRAY(cpuPpiPending, CPU_MAX);
    SERIALIZE_SCALAR(gem5ExtensionsEnabled);

    for (uint32_t i=0; i < bankedRegs.size(); ++i) {
        if (!bankedRegs[i])
            continue;
        bankedRegs[i]->serializeSection(cp, csprintf("bankedRegs%i", i));
    }
}

void
GicV2::BankedRegs::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(intEnabled);
    SERIALIZE_SCALAR(pendingInt);
    SERIALIZE_SCALAR(activeInt);
    SERIALIZE_SCALAR(intGroup);
    SERIALIZE_ARRAY(intConfig, 2);
    SERIALIZE_ARRAY(intPriority, SGI_MAX + PPI_MAX);
}

void
GicV2::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm GIC\n");

    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(itLines);
    UNSERIALIZE_ARRAY(intEnabled, INT_BITS_MAX-1);
    UNSERIALIZE_ARRAY(pendingInt, INT_BITS_MAX-1);
    UNSERIALIZE_ARRAY(activeInt, INT_BITS_MAX-1);
    UNSERIALIZE_ARRAY(intGroup, INT_BITS_MAX-1);
    UNSERIALIZE_ARRAY(iccrpr, CPU_MAX);
    UNSERIALIZE_ARRAY(intPriority, GLOBAL_INT_LINES);
    UNSERIALIZE_ARRAY(cpuTarget, GLOBAL_INT_LINES);
    UNSERIALIZE_ARRAY(intConfig, INT_BITS_MAX * 2 - 2);
    UNSERIALIZE_ARRAY(cpuControl, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuPriority, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuBpr, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuHighestInt, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuSgiActive, SGI_MAX);
    UNSERIALIZE_ARRAY(cpuSgiPending, SGI_MAX);
    UNSERIALIZE_ARRAY(cpuSgiActiveExt, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuSgiPendingExt, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuPpiActive, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuPpiPending, CPU_MAX);

    // Handle checkpoints from before we drained the GIC to prevent
    // in-flight interrupts.
    if (cp.entryExists(Serializable::currentSection(), "interrupt_time")) {
        Tick interrupt_time[CPU_MAX];
        UNSERIALIZE_ARRAY(interrupt_time, CPU_MAX);

        for (uint32_t cpu = 0; cpu < CPU_MAX; cpu++) {
            if (interrupt_time[cpu])
                schedule(postIntEvent[cpu], interrupt_time[cpu]);
        }
    }

    if (!UNSERIALIZE_OPT_SCALAR(gem5ExtensionsEnabled))
        gem5ExtensionsEnabled = false;

    for (uint32_t i=0; i < CPU_MAX; ++i) {
        ScopedCheckpointSection sec(cp, csprintf("bankedRegs%i", i));
        if (cp.sectionExists(Serializable::currentSection())) {
            getBankedRegs(i).unserialize(cp);
        }
    }
}

void
GicV2::BankedRegs::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(intEnabled);
    UNSERIALIZE_SCALAR(pendingInt);
    UNSERIALIZE_SCALAR(activeInt);
    UNSERIALIZE_SCALAR(intGroup);
    UNSERIALIZE_ARRAY(intConfig, 2);
    UNSERIALIZE_ARRAY(intPriority, SGI_MAX + PPI_MAX);
}

} // namespace gem5
