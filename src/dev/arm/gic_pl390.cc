/*
 * Copyright (c) 2010, 2013 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          Prakash Ramrakhyani
 */

#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/GIC.hh"
#include "debug/IPI.hh"
#include "debug/Interrupt.hh"
#include "dev/arm/gic_pl390.hh"
#include "dev/arm/realview.hh"
#include "dev/terminal.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Pl390::Pl390(const Params *p)
    : BaseGic(p), distAddr(p->dist_addr),
      cpuAddr(p->cpu_addr), distPioDelay(p->dist_pio_delay),
      cpuPioDelay(p->cpu_pio_delay), intLatency(p->int_latency),
      enabled(false), itLines(p->it_lines), irqEnable(false),
      msixRegAddr(p->msix_addr),
      msixReg(0x0)
{
    itLinesLog2 = ceilLog2(itLines);

    for (int x = 0; x < CPU_MAX; x++) {
        cpuEnabled[x] = false;
        cpuPriority[x] = 0xff;
        cpuBpr[x] = 0;
        // Initialize cpu highest int
        cpuHighestInt[x] = SPURIOUS_INT;
        postIntEvent[x] = new PostIntEvent(x, p->platform);
    }
    DPRINTF(Interrupt, "cpuEnabled[0]=%d cpuEnabled[1]=%d\n", cpuEnabled[0],
            cpuEnabled[1]);

    for (int x = 0; x < INT_BITS_MAX; x++) {
        intEnabled[x] = 0;
        pendingInt[x] = 0;
        activeInt[x] = 0;
    }

    for (int x = 0; x < INT_LINES_MAX; x++) {
        intPriority[x] = 0;
        cpuTarget[x] = 0;
    }

    for (int x = 0; x < INT_BITS_MAX*2; x++) {
        intConfig[x] = 0;
    }

    for (int x = 0; x < SGI_MAX; x++) {
        cpuSgiActive[x] = 0;
        cpuSgiPending[x] = 0;
    }
    for (int x = 0; x < CPU_MAX; x++) {
        cpuPpiActive[x] = 0;
        cpuPpiPending[x] = 0;
    }

    for (int i = 0; i < CPU_MAX; i++) {
        for (int j = 0; j < (SGI_MAX + PPI_MAX); j++) {
            bankedIntPriority[i][j] = 0;
        }
    }

    RealView *rv = dynamic_cast<RealView*>(p->platform);
    assert(rv);
    rv->setGic(this);

}

Tick
Pl390::read(PacketPtr pkt)
{

    Addr addr = pkt->getAddr();

    if (addr >= distAddr && addr < distAddr + DIST_SIZE)
        return readDistributor(pkt);
    else if (addr >= cpuAddr && addr < cpuAddr + CPU_SIZE)
        return readCpu(pkt);
    else if (msixRegAddr != 0x0 &&
             addr >= msixRegAddr &&
             addr < msixRegAddr + MSIX_SIZE)
        return readMsix(pkt);
    else
        panic("Read to unknown address %#x\n", pkt->getAddr());
}


Tick
Pl390::write(PacketPtr pkt)
{

    Addr addr = pkt->getAddr();

    if (addr >= distAddr && addr < distAddr + DIST_SIZE)
        return writeDistributor(pkt);
    else if (addr >= cpuAddr && addr < cpuAddr + CPU_SIZE)
        return writeCpu(pkt);
    else if (msixRegAddr != 0x0 &&
             addr >= msixRegAddr &&
             addr < msixRegAddr + MSIX_SIZE)
        return writeMsix(pkt);
    else
        panic("Write to unknown address %#x\n", pkt->getAddr());
}

Tick
Pl390::readDistributor(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - distAddr;
    pkt->allocate();

    int ctx_id = pkt->req->contextId();

    DPRINTF(GIC, "gic distributor read register %#x\n", daddr);

    if (daddr >= ICDISER_ST && daddr < ICDISER_ED + 4) {
        assert((daddr-ICDISER_ST) >> 2 < 32);
        pkt->set<uint32_t>(intEnabled[(daddr-ICDISER_ST)>>2]);
        goto done;
    }

    if (daddr >= ICDICER_ST && daddr < ICDICER_ED + 4) {
        assert((daddr-ICDICER_ST) >> 2 < 32);
        pkt->set<uint32_t>(intEnabled[(daddr-ICDICER_ST)>>2]);
        goto done;
    }

    if (daddr >= ICDISPR_ST && daddr < ICDISPR_ED + 4) {
        assert((daddr-ICDISPR_ST) >> 2 < 32);
        pkt->set<uint32_t>(pendingInt[(daddr-ICDISPR_ST)>>2]);
        goto done;
    }

    if (daddr >= ICDICPR_ST && daddr < ICDICPR_ED + 4) {
        assert((daddr-ICDICPR_ST) >> 2 < 32);
        pkt->set<uint32_t>(pendingInt[(daddr-ICDICPR_ST)>>2]);
        goto done;
    }

    if (daddr >= ICDABR_ST && daddr < ICDABR_ED + 4) {
        assert((daddr-ICDABR_ST) >> 2 < 32);
        pkt->set<uint32_t>(activeInt[(daddr-ICDABR_ST)>>2]);
        goto done;
    }

    if (daddr >= ICDIPR_ST && daddr < ICDIPR_ED + 4) {
        Addr int_num;
        int_num = daddr - ICDIPR_ST;
        assert(int_num < INT_LINES_MAX);
        DPRINTF(Interrupt, "Reading interrupt priority at int# %#x \n",int_num);

        uint8_t* int_p;
        if (int_num < (SGI_MAX + PPI_MAX))
            int_p = bankedIntPriority[ctx_id];
        else
            int_p = intPriority;

        switch (pkt->getSize()) {
          case 1:
            pkt->set<uint8_t>(int_p[int_num]);
            break;
          case 2:
            assert((int_num + 1) < INT_LINES_MAX);
            pkt->set<uint16_t>(int_p[int_num] |
                               int_p[int_num+1] << 8);
            break;
          case 4:
            assert((int_num + 3) < INT_LINES_MAX);
            pkt->set<uint32_t>(int_p[int_num] |
                               int_p[int_num+1] << 8 |
                               int_p[int_num+2] << 16 |
                               int_p[int_num+3] << 24);
            break;
          default:
            panic("Invalid size while reading priority regs in GIC: %d\n",
                   pkt->getSize());
        }
        goto done;
    }

    if (daddr >= ICDIPTR_ST && daddr < ICDIPTR_ED + 4) {
        Addr int_num;
        int_num = (daddr-ICDIPTR_ST) ;
        DPRINTF(GIC, "Reading processor target register for int# %#x \n",
                 int_num);
        assert(int_num < INT_LINES_MAX);

        // First 31 interrupts only target single processor (SGI)
        if (int_num > 31) {
            if (pkt->getSize() == 1) {
                pkt->set<uint8_t>(cpuTarget[int_num]);
            } else {
                assert(pkt->getSize() == 4);
                int_num = mbits(int_num, 31, 2);
                pkt->set<uint32_t>(cpuTarget[int_num] |
                                   cpuTarget[int_num+1] << 8 |
                                   cpuTarget[int_num+2] << 16 |
                                   cpuTarget[int_num+3] << 24) ;
            }
        } else {
            assert(ctx_id < sys->numRunningContexts());
            // convert the CPU id number into a bit mask
            uint32_t ctx_mask = power(2, ctx_id);
            // replicate the 8-bit mask 4 times in a 32-bit word
            ctx_mask |= ctx_mask << 8;
            ctx_mask |= ctx_mask << 16;
            pkt->set<uint32_t>(ctx_mask);
        }
        goto done;
    }

    if (daddr >= ICDICFR_ST && daddr < ICDICFR_ED + 4) {
        assert((daddr-ICDICFR_ST) >> 2 < 64);
        /** @todo software generated interrutps and PPIs
         * can't be configured in some ways
         */
        pkt->set<uint32_t>(intConfig[(daddr-ICDICFR_ST)>>2]);
        goto done;
    }

    switch(daddr) {
      case ICDDCR:
        pkt->set<uint32_t>(enabled);
        break;
      case ICDICTR:
        uint32_t tmp;
        tmp = ((sys->numRunningContexts() - 1) << 5) |
              (itLines/INT_BITS_MAX -1);
        pkt->set<uint32_t>(tmp);
        break;
      default:
        panic("Tried to read Gic distributor at offset %#x\n", daddr);
        break;
    }
done:
    pkt->makeAtomicResponse();
    return distPioDelay;
}

Tick
Pl390::readCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - cpuAddr;
    pkt->allocate();

    assert(pkt->req->hasContextId());
    int ctx_id = pkt->req->contextId();
    assert(ctx_id < sys->numRunningContexts());

    DPRINTF(GIC, "gic cpu read register %#x cpu context: %d\n", daddr,
            ctx_id);

    switch(daddr) {
      case ICCICR:
        pkt->set<uint32_t>(cpuEnabled[ctx_id]);
        break;
      case ICCPMR:
        pkt->set<uint32_t>(cpuPriority[ctx_id]);
        break;
      case ICCBPR:
        pkt->set<uint32_t>(cpuBpr[ctx_id]);
        break;
      case ICCIAR:
        if (enabled && cpuEnabled[ctx_id]) {
            int active_int = cpuHighestInt[ctx_id];
            IAR iar = 0;
            iar.ack_id = active_int;
            iar.cpu_id = 0;
            if (active_int < SGI_MAX) {
                // this is a software interrupt from another CPU
                if (!cpuSgiPending[active_int])
                    panic("Interrupt %d active but no CPU generated it?\n",
                            active_int);
                for (int x = 0; x < CPU_MAX; x++) {
                    // See which CPU generated the interrupt
                    uint8_t cpugen =
                        bits(cpuSgiPending[active_int], 7 + 8 * x, 8 * x);
                    if (cpugen & (1 << ctx_id)) {
                        iar.cpu_id = x;
                        break;
                    }
                }
                uint64_t sgi_num = ULL(1) << (ctx_id + 8 * iar.cpu_id);
                cpuSgiActive[iar.ack_id] |= sgi_num;
                cpuSgiPending[iar.ack_id] &= ~sgi_num;
            } else if (active_int < (SGI_MAX + PPI_MAX) ) {
                uint32_t int_num = 1 << (cpuHighestInt[ctx_id] - SGI_MAX);
                cpuPpiActive[ctx_id] |= int_num;
                updateRunPri();
                cpuPpiPending[ctx_id] &= ~int_num;

            } else {
                uint32_t int_num = 1 << intNumToBit(cpuHighestInt[ctx_id]);
                activeInt[intNumToWord(cpuHighestInt[ctx_id])] |= int_num;
                updateRunPri();
                pendingInt[intNumToWord(cpuHighestInt[ctx_id])] &= ~int_num;
            }

            DPRINTF(Interrupt,"CPU %d reading IAR.id=%d IAR.cpu=%d, iar=0x%x\n",
                    ctx_id, iar.ack_id, iar.cpu_id, iar);
            cpuHighestInt[ctx_id] = SPURIOUS_INT;
            updateIntState(-1);
            pkt->set<uint32_t>(iar);
            platform->intrctrl->clear(ctx_id, ArmISA::INT_IRQ, 0);
        } else {
             pkt->set<uint32_t>(SPURIOUS_INT);
        }

        break;
      case ICCRPR:
        pkt->set<uint32_t>(iccrpr[0]);
        break;
      case ICCHPIR:
        pkt->set<uint32_t>(0);
        panic("Need to implement HPIR");
        break;
      default:
        panic("Tried to read Gic cpu at offset %#x\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return cpuPioDelay;
}

Tick
Pl390::readMsix(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - msixRegAddr;
    pkt->allocate();

    DPRINTF(GIC, "Gic MSIX read register %#x\n", daddr);

    switch (daddr) {
        case MSIX_SR:
            pkt->set<uint32_t>(msixReg);
            break;
        default:
            panic("Tried to read Gic MSIX register at offset %#x\n", daddr);
            break;
    }

    pkt->makeAtomicResponse();
    return distPioDelay;
}

Tick
Pl390::writeDistributor(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - distAddr;
    pkt->allocate();

    assert(pkt->req->hasContextId());
    int ctx_id = pkt->req->contextId();

    DPRINTF(GIC, "gic distributor write register %#x size %#x value %#x \n",
            daddr, pkt->getSize(), pkt->get<uint32_t>());

    if (daddr >= ICDISER_ST && daddr < ICDISER_ED + 4) {
        assert((daddr-ICDISER_ST) >> 2 < 32);
        intEnabled[(daddr-ICDISER_ST) >> 2] |= pkt->get<uint32_t>();
        goto done;
    }

    if (daddr >= ICDICER_ST && daddr < ICDICER_ED + 4) {
        assert((daddr-ICDICER_ST) >> 2 < 32);
        intEnabled[(daddr-ICDICER_ST) >> 2] &= ~pkt->get<uint32_t>();
        goto done;
    }

    if (daddr >= ICDISPR_ST && daddr < ICDISPR_ED + 4) {
        assert((daddr-ICDISPR_ST) >> 2 < 32);
        pendingInt[(daddr-ICDISPR_ST) >> 2] |= pkt->get<uint32_t>();
        pendingInt[0] &= SGI_MASK; // Don't allow SGIs to be changed
        updateIntState((daddr-ICDISPR_ST) >> 2);
        goto done;
    }

    if (daddr >= ICDICPR_ST && daddr < ICDICPR_ED + 4) {
        assert((daddr-ICDICPR_ST) >> 2 < 32);
        pendingInt[(daddr-ICDICPR_ST) >> 2] &= ~pkt->get<uint32_t>();
        pendingInt[0] &= SGI_MASK; // Don't allow SGIs to be changed
        updateIntState((daddr-ICDICPR_ST) >> 2);
        goto done;
    }

    if (daddr >= ICDIPR_ST && daddr < ICDIPR_ED + 4) {
        Addr int_num = daddr - ICDIPR_ST;
        assert(int_num < INT_LINES_MAX);
        uint8_t* int_p;
        if (int_num < (SGI_MAX + PPI_MAX))
            int_p = bankedIntPriority[ctx_id];
        else
            int_p = intPriority;
        uint32_t tmp;
        switch(pkt->getSize()) {
          case 1:
            tmp = pkt->get<uint8_t>();
            int_p[int_num] = bits(tmp, 7, 0);
            break;
          case 2:
            tmp = pkt->get<uint16_t>();
            int_p[int_num] = bits(tmp, 7, 0);
            int_p[int_num + 1] = bits(tmp, 15, 8);
            break;
          case 4:
            tmp = pkt->get<uint32_t>();
            int_p[int_num] = bits(tmp, 7, 0);
            int_p[int_num + 1] = bits(tmp, 15, 8);
            int_p[int_num + 2] = bits(tmp, 23, 16);
            int_p[int_num + 3] = bits(tmp, 31, 24);
            break;
          default:
            panic("Invalid size when writing to priority regs in Gic: %d\n",
                   pkt->getSize());
        }

        updateIntState(-1);
        updateRunPri();
        goto done;
    }

    if (daddr >= ICDIPTR_ST && daddr < ICDIPTR_ED + 4) {
        Addr int_num = (daddr-ICDIPTR_ST) ;
        assert(int_num < INT_LINES_MAX);
        // First 31 interrupts only target single processor
        if (int_num >= SGI_MAX) {
            if (pkt->getSize() == 1) {
                uint8_t tmp = pkt->get<uint8_t>();
                cpuTarget[int_num] = tmp & 0xff;
            } else {
                assert (pkt->getSize() == 4);
                int_num = mbits(int_num, 31, 2);
                uint32_t tmp = pkt->get<uint32_t>();
                cpuTarget[int_num]   = bits(tmp, 7, 0);
                cpuTarget[int_num+1] = bits(tmp, 15, 8);
                cpuTarget[int_num+2] = bits(tmp, 23, 16);
                cpuTarget[int_num+3] = bits(tmp, 31, 24);
            }
            updateIntState((daddr-ICDIPTR_ST)>>2);
        }
        goto done;
    }

    if (daddr >= ICDICFR_ST && daddr < ICDICFR_ED + 4) {
        assert((daddr-ICDICFR_ST) >> 2 < 64);
        intConfig[(daddr-ICDICFR_ST)>>2] = pkt->get<uint32_t>();
        if (pkt->get<uint32_t>() & NN_CONFIG_MASK)
            warn("GIC N:N mode selected and not supported at this time\n");
        goto done;
    }

    switch(daddr) {
      case ICDDCR:
        enabled = pkt->get<uint32_t>();
        DPRINTF(Interrupt, "Distributor enable flag set to = %d\n", enabled);
        break;
      case ICDSGIR:
        softInt(ctx_id, pkt->get<uint32_t>());
        break;
      default:
        panic("Tried to write Gic distributor at offset %#x\n", daddr);
        break;
    }

done:
    pkt->makeAtomicResponse();
    return distPioDelay;
}

Tick
Pl390::writeCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - cpuAddr;
    pkt->allocate();

    assert(pkt->req->hasContextId());
    int ctx_id = pkt->req->contextId();
    IAR iar;

    DPRINTF(GIC, "gic cpu write register cpu:%d %#x val: %#x\n",
            ctx_id, daddr, pkt->get<uint32_t>());

    switch(daddr) {
      case ICCICR:
        cpuEnabled[ctx_id] = pkt->get<uint32_t>();
        break;
      case ICCPMR:
        cpuPriority[ctx_id] = pkt->get<uint32_t>();
        break;
      case ICCBPR:
        cpuBpr[ctx_id] = pkt->get<uint32_t>();
        break;
      case ICCEOIR:
        iar = pkt->get<uint32_t>();
        if (iar.ack_id < SGI_MAX) {
            // Clear out the bit that corrseponds to the cleared int
            uint64_t clr_int = ULL(1) << (ctx_id + 8 * iar.cpu_id);
            if (!(cpuSgiActive[iar.ack_id] & clr_int))
                panic("Done handling a SGI that isn't active?\n");
            cpuSgiActive[iar.ack_id] &= ~clr_int;
        } else if (iar.ack_id < (SGI_MAX + PPI_MAX) ) {
            uint32_t int_num = 1 << (iar.ack_id - SGI_MAX);
            if (!(cpuPpiActive[ctx_id] & int_num))
                panic("CPU %d Done handling a PPI interrupt that isn't active?\n", ctx_id);
            cpuPpiActive[ctx_id] &= ~int_num;
        } else {
            uint32_t int_num = 1 << intNumToBit(iar.ack_id);
            if (!(activeInt[intNumToWord(iar.ack_id)] & int_num))
                panic("Done handling interrupt that isn't active?\n");
            activeInt[intNumToWord(iar.ack_id)] &= ~int_num;
        }
        updateRunPri();
        DPRINTF(Interrupt, "CPU %d done handling intr IAR = %d from cpu %d\n",
                ctx_id, iar.ack_id, iar.cpu_id);
        break;
      default:
        panic("Tried to write Gic cpu at offset %#x\n", daddr);
        break;
    }
    if (cpuEnabled[ctx_id]) updateIntState(-1);
    pkt->makeAtomicResponse();
    return cpuPioDelay;
}

Tick
Pl390::writeMsix(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - msixRegAddr;
    pkt->allocate();

    DPRINTF(GIC, "Gic MSI-X write register %#x data %d\n",
                 daddr, pkt->get<uint32_t>());

    switch (daddr) {
        case MSIX_SR:
            // This value is little endian, just like the ARM guest
            msixReg = pkt->get<uint32_t>();
            pendingInt[intNumToWord(letoh(msixReg))] |= 1UL << intNumToBit(letoh(msixReg));
            updateIntState(-1);
            break;
        default:
            panic("Tried to write Gic MSI-X register at offset %#x\n", daddr);
            break;
    }

    pkt->makeAtomicResponse();
    return distPioDelay;
}

void
Pl390::softInt(int ctx_id, SWI swi)
{
    switch (swi.list_type) {
      case 1:
        // interrupt all
        uint8_t cpu_list;
        cpu_list = 0;
        for (int x = 0; x < CPU_MAX; x++)
            cpu_list |= cpuEnabled[x] ? 1 << x : 0;
        swi.cpu_list = cpu_list;
        break;
      case 2:
        // interrupt requesting cpu only
        swi.cpu_list = 1 << ctx_id;
        break;
        // else interrupt cpus specified
    }

    DPRINTF(IPI, "Generating softIRQ from CPU %d for %#x\n", ctx_id,
            swi.cpu_list);
    for (int i = 0; i < CPU_MAX; i++) {
        DPRINTF(IPI, "Processing CPU %d\n", i);
        if (!cpuEnabled[i])
            continue;
        if (swi.cpu_list & (1 << i))
            cpuSgiPending[swi.sgi_id] |= (1 << i) << (8 * ctx_id);
        DPRINTF(IPI, "SGI[%d]=%#x\n", swi.sgi_id, cpuSgiPending[swi.sgi_id]);
    }
    updateIntState(-1);
}

uint64_t
Pl390::genSwiMask(int cpu)
{
    if (cpu > 7)
        panic("Invalid CPU ID\n");
    return ULL(0x0101010101010101) << cpu;
}

void
Pl390::updateIntState(int hint)
{
    for (int cpu = 0; cpu < CPU_MAX; cpu++) {
        if (!cpuEnabled[cpu])
            continue;
        if (cpu >= sys->numContexts())
            break;

        /*@todo use hint to do less work. */
        int highest_int = SPURIOUS_INT;
        // Priorities below that set in ICCPMR can be ignored
        uint8_t highest_pri = cpuPriority[cpu];

        // Check SGIs
        for (int swi = 0; swi < SGI_MAX; swi++) {
            if (!cpuSgiPending[swi])
                continue;
            if (cpuSgiPending[swi] & genSwiMask(cpu))
                if (highest_pri > bankedIntPriority[cpu][swi]) {
                    highest_pri = bankedIntPriority[cpu][swi];
                    highest_int = swi;
                }
        }

        // Check PPIs
        if (cpuPpiPending[cpu]) {
        for (int ppi = 0; ppi < PPI_MAX; ppi++) {
            if (cpuPpiPending[cpu] & (1 << ppi))
                if (highest_pri > bankedIntPriority[cpu][SGI_MAX + ppi]) {
                    highest_pri = bankedIntPriority[cpu][SGI_MAX + ppi];
                    highest_int = SGI_MAX + ppi;
                }
            }
        }

        bool mp_sys = sys->numRunningContexts() > 1;
        // Check other ints
        for (int x = 0; x < (itLines/INT_BITS_MAX); x++) {
            if (intEnabled[x] & pendingInt[x]) {
                for (int y = 0; y < INT_BITS_MAX; y++) {
                   uint32_t int_nm = x * INT_BITS_MAX + y;
                   DPRINTF(GIC, "Checking for interrupt# %d \n",int_nm);
                    /* Set current pending int as highest int for current cpu
                       if the interrupt's priority higher than current prioirty
                       and if currrent cpu is the target (for mp configs only)
                     */
                    if ((bits(intEnabled[x], y) & bits(pendingInt[x], y)) &&
                        (intPriority[int_nm] < highest_pri))
                        if ( (!mp_sys) || (cpuTarget[int_nm] & (1 << cpu))) {
                            highest_pri = intPriority[int_nm];
                            highest_int = int_nm;
                        }
                }
            }
        }

        cpuHighestInt[cpu] = highest_int;

        if (highest_int == SPURIOUS_INT)
            continue;

        /* @todo make this work for more than one cpu, need to handle 1:N, N:N
         * models */
        if (enabled && cpuEnabled[cpu] && (highest_pri < cpuPriority[cpu]) &&
            !(activeInt[intNumToWord(highest_int)]
            & (1 << intNumToBit(highest_int)))) {

            DPRINTF(Interrupt, "Posting interrupt %d to cpu%d\n", highest_int,
                    cpu);
            postInt(cpu, curTick() + intLatency);
        }
    }
}

void
Pl390::updateRunPri()
{
    for (int cpu = 0; cpu < CPU_MAX; cpu++) {
        if (!cpuEnabled[cpu])
            continue;
        uint8_t maxPriority = 0xff;
        for (int i = 0; i < itLines; i++){
            if (i < SGI_MAX) {
                if ((cpuSgiActive[i] & genSwiMask(cpu)) &&
                        (bankedIntPriority[cpu][i] < maxPriority))
                    maxPriority = bankedIntPriority[cpu][i];
            } else if (i < (SGI_MAX + PPI_MAX)) {
                if ((cpuPpiActive[cpu] & ( 1 << (i - SGI_MAX))) &&
                        (bankedIntPriority[cpu][i] < maxPriority))
                    maxPriority = bankedIntPriority[cpu][i];

            } else {
                if (activeInt[intNumToWord(i)] & (1 << intNumToBit(i)))
                    if (intPriority[i] < maxPriority)
                        maxPriority = intPriority[i];
            }
        }
        iccrpr[cpu] = maxPriority;
    }
}

void
Pl390::sendInt(uint32_t num)
{
    DPRINTF(Interrupt, "Received Interupt number %d,  cpuTarget %#x: \n",
            num, cpuTarget[num]);
    if (cpuTarget[num] & (cpuTarget[num] - 1))
        panic("Multiple targets for peripheral interrupts is not supported\n");
    pendingInt[intNumToWord(num)] |= 1 << intNumToBit(num);
    updateIntState(intNumToWord(num));

}

void
Pl390::sendPPInt(uint32_t num, uint32_t cpu)
{
    DPRINTF(Interrupt, "Received PPI %d, cpuTarget %#x: \n",
            num, cpu);
    cpuPpiPending[cpu] |= 1 << (num - SGI_MAX);
    updateIntState(intNumToWord(num));
}

void
Pl390::clearInt(uint32_t number)
{
    /* @todo assume edge triggered only at the moment. Nothing to do. */
}

void
Pl390::clearPPInt(uint32_t num, uint32_t cpu)
{
    DPRINTF(Interrupt, "Clearing PPI %d, cpuTarget %#x: \n",
            num, cpu);
    cpuPpiPending[cpu] &= ~(1 << (num - SGI_MAX));
    updateIntState(intNumToWord(num));
}

void
Pl390::postInt(uint32_t cpu, Tick when)
{
    if (!(postIntEvent[cpu]->scheduled()))
        eventq->schedule(postIntEvent[cpu], when);
}

AddrRangeList
Pl390::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(distAddr, DIST_SIZE));
    ranges.push_back(RangeSize(cpuAddr, CPU_SIZE));
    if (msixRegAddr != 0) {
        ranges.push_back(RangeSize(msixRegAddr, MSIX_SIZE));
    }
    return ranges;
}


void
Pl390::serialize(std::ostream &os)
{
    DPRINTF(Checkpoint, "Serializing Arm GIC\n");

    SERIALIZE_SCALAR(distAddr);
    SERIALIZE_SCALAR(cpuAddr);
    SERIALIZE_SCALAR(distPioDelay);
    SERIALIZE_SCALAR(cpuPioDelay);
    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(itLines);
    SERIALIZE_SCALAR(itLinesLog2);
    SERIALIZE_SCALAR(msixRegAddr);
    SERIALIZE_SCALAR(msixReg);
    SERIALIZE_ARRAY(intEnabled, INT_BITS_MAX);
    SERIALIZE_ARRAY(pendingInt, INT_BITS_MAX);
    SERIALIZE_ARRAY(activeInt, INT_BITS_MAX);
    SERIALIZE_ARRAY(iccrpr, CPU_MAX);
    SERIALIZE_ARRAY(intPriority, INT_LINES_MAX);
    SERIALIZE_ARRAY(cpuTarget, INT_LINES_MAX);
    SERIALIZE_ARRAY(intConfig, INT_BITS_MAX * 2);
    SERIALIZE_ARRAY(cpuEnabled, CPU_MAX);
    SERIALIZE_ARRAY(cpuPriority, CPU_MAX);
    SERIALIZE_ARRAY(cpuBpr, CPU_MAX);
    SERIALIZE_ARRAY(cpuHighestInt, CPU_MAX);
    SERIALIZE_ARRAY(cpuSgiActive, SGI_MAX);
    SERIALIZE_ARRAY(cpuSgiPending, SGI_MAX);
    SERIALIZE_ARRAY(cpuPpiActive, CPU_MAX);
    SERIALIZE_ARRAY(cpuPpiPending, CPU_MAX);
    SERIALIZE_ARRAY(*bankedIntPriority, CPU_MAX * (SGI_MAX + PPI_MAX));
    SERIALIZE_SCALAR(irqEnable);
    Tick interrupt_time[CPU_MAX];
    for (uint32_t cpu = 0; cpu < CPU_MAX; cpu++) {
        interrupt_time[cpu] = 0;
        if (postIntEvent[cpu]->scheduled()) {
            interrupt_time[cpu] = postIntEvent[cpu]->when();
        }
    }
    SERIALIZE_ARRAY(interrupt_time, CPU_MAX);

}

void
Pl390::unserialize(Checkpoint *cp, const std::string &section)
{
    DPRINTF(Checkpoint, "Unserializing Arm GIC\n");

    UNSERIALIZE_SCALAR(distAddr);
    UNSERIALIZE_SCALAR(cpuAddr);
    UNSERIALIZE_SCALAR(distPioDelay);
    UNSERIALIZE_SCALAR(cpuPioDelay);
    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(itLines);
    UNSERIALIZE_SCALAR(itLinesLog2);
    UNSERIALIZE_SCALAR(msixRegAddr);
    UNSERIALIZE_SCALAR(msixReg);
    UNSERIALIZE_ARRAY(intEnabled, INT_BITS_MAX);
    UNSERIALIZE_ARRAY(pendingInt, INT_BITS_MAX);
    UNSERIALIZE_ARRAY(activeInt, INT_BITS_MAX);
    UNSERIALIZE_ARRAY(iccrpr, CPU_MAX);
    UNSERIALIZE_ARRAY(intPriority, INT_LINES_MAX);
    UNSERIALIZE_ARRAY(cpuTarget, INT_LINES_MAX);
    UNSERIALIZE_ARRAY(intConfig, INT_BITS_MAX * 2);
    UNSERIALIZE_ARRAY(cpuEnabled, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuPriority, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuBpr, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuHighestInt, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuSgiActive, SGI_MAX);
    UNSERIALIZE_ARRAY(cpuSgiPending, SGI_MAX);
    UNSERIALIZE_ARRAY(cpuPpiActive, CPU_MAX);
    UNSERIALIZE_ARRAY(cpuPpiPending, CPU_MAX);
    UNSERIALIZE_ARRAY(*bankedIntPriority, CPU_MAX * (SGI_MAX + PPI_MAX));
    UNSERIALIZE_SCALAR(irqEnable);

    Tick interrupt_time[CPU_MAX];
    UNSERIALIZE_ARRAY(interrupt_time, CPU_MAX);

    for (uint32_t cpu = 0; cpu < CPU_MAX; cpu++) {
        if (interrupt_time[cpu])
            schedule(postIntEvent[cpu], interrupt_time[cpu]);
    }

}

Pl390 *
Pl390Params::create()
{
    return new Pl390(this);
}

/* Functions for debugging and testing */
void
Pl390::driveSPI(unsigned int spiVect)
{
    DPRINTF(GIC, "Received SPI Vector:%x Enable: %d\n", spiVect, irqEnable);
    pendingInt[1] |= spiVect;
    if (irqEnable && enabled) {
        updateIntState(-1);
    }
}

void
Pl390::driveIrqEn( bool state)
{
    irqEnable = state;
    DPRINTF(GIC, " Enabling Irq\n");
    updateIntState(-1);
}

void
Pl390::driveLegIRQ(bool state)
{
    if (irqEnable && !(!enabled && cpuEnabled[0])) {
        if (state) {
            DPRINTF(GIC, "Driving Legacy Irq\n");
            platform->intrctrl->post(0, ArmISA::INT_IRQ, 0);
        }
        else platform->intrctrl->clear(0, ArmISA::INT_IRQ, 0);
    }
}

void
Pl390::driveLegFIQ(bool state)
{
    if (state)
        platform->intrctrl->post(0, ArmISA::INT_FIQ, 0);
    else platform->intrctrl->clear(0, ArmISA::INT_FIQ, 0);
}
