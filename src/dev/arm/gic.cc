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
#include "cpu/intr_control.hh"
#include "dev/arm/gic.hh"
#include "dev/platform.hh"
#include "dev/terminal.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Gic::Gic(const Params *p)
    : PioDevice(p),distAddr(p->dist_addr), cpuAddr(p->cpu_addr),
      distPioDelay(p->dist_pio_delay), cpuPioDelay(p->cpu_pio_delay),
      enabled(false), itLines(p->it_lines)
{
    itLinesLog2 = ceilLog2(itLines);

    for (int x = 0; x < 8; x++) {
        cpuEnabled[x] = false;
        cpuPriority[x] = 0;
        cpuBpr[x] = 0;
        // Initialize cpu highest int
        cpuHighestInt[x] = SPURIOUS_INT;
    }

    for (int x = 0; x < 32; x++) {
        intEnabled[x] = 0;
        pendingInt[x] = 0;
        activeInt[x] = 0;
    }

    for (int x = 0; x < 1020; x++) {
        intPriority[x] = 0;
        cpuTarget[x] = 0;
    }

    for (int x = 0; x < 64; x++) {
        intConfig[x] = 0;
    }
}

Tick
Gic::read(PacketPtr pkt)
{

    Addr addr = pkt->getAddr();

    if (addr >= distAddr && addr < distAddr + DIST_SIZE)
        return readDistributor(pkt);
    else if (addr >= cpuAddr && addr < cpuAddr + CPU_SIZE)
        return readCpu(pkt);
    else
        panic("Read to unknown address %#x\n", pkt->getAddr());
}


Tick
Gic::write(PacketPtr pkt)
{

    Addr addr = pkt->getAddr();

    if (addr >= distAddr && addr < distAddr + DIST_SIZE)
        return writeDistributor(pkt);
    else if (addr >= cpuAddr && addr < cpuAddr + CPU_SIZE)
        return writeCpu(pkt);
    else
        panic("Write to unknown address %#x\n", pkt->getAddr());
}

Tick
Gic::readDistributor(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - distAddr;
    pkt->allocate();

    DPRINTF(Interrupt, "gic distributor read register %#x\n", daddr);

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
        assert(int_num < 1020);
        DPRINTF(Interrupt, "Reading interrupt priority at int# %#x \n",int_num);
        switch(pkt->getSize()){
          case 1:
            pkt->set<uint8_t>(intPriority[int_num]);
            break;
          case 2:
            pkt->set<uint16_t>(intPriority[int_num] |
                               intPriority[int_num+1] << 8);
            break;
          case 4:
            pkt->set<uint32_t>(intPriority[int_num] |
                               intPriority[int_num+1] << 8 |
                               intPriority[int_num+2] << 16 |
                               intPriority[int_num+3] << 24);
            break;
          default:
            panic("Invalid access size while reading, priority registers in Gic: %d", pkt->getSize());
        }
        goto done;
    }

    if (daddr >= ICDIPTR_ST && daddr < ICDIPTR_ED + 4) {
        Addr int_num;
        int_num = (daddr-ICDIPTR_ST) << 2;
        assert(int_num < 1020);

        // First 31 interrupts only target single processor
        if (int_num > 31) {
            pkt->set<uint32_t>(cpuTarget[int_num] |
                               cpuTarget[int_num+1] << 8 |
                               cpuTarget[int_num+2] << 16 |
                               cpuTarget[int_num+3] << 24) ;
        } else {
            /** @todo should be processor id */
            pkt->set<uint32_t>(0);
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
        /* @todo this needs to refelct the number of CPUs in the system */
        uint32_t tmp;
        tmp = 0 << 5 | // cpu number
              (itLines/32 -1);
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
Gic::readCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - cpuAddr;
    pkt->allocate();

    DPRINTF(Interrupt, "gic cpu read register %#x\n", daddr);

    switch(daddr) {
      case ICCICR:
        pkt->set<uint32_t>(cpuEnabled[0]);
        break;
      case ICCPMR:
        pkt->set<uint32_t>(cpuPriority[0]);
        break;
      case ICCBPR:
        pkt->set<uint32_t>(cpuBpr[0]);
        break;
      case ICCIAR:
        DPRINTF(Interrupt, "CPU reading IAR = %d\n", cpuHighestInt[0]);
        if(enabled && cpuEnabled[0]){
            pkt->set<uint32_t>(cpuHighestInt[0]);
            activeInt[intNumToWord(cpuHighestInt[0])] |=
                1 << intNumToBit(cpuHighestInt[0]);
            updateRunPri();
            pendingInt[intNumToWord(cpuHighestInt[0])] &=
                ~(1 << intNumToBit(cpuHighestInt[0]));
            cpuHighestInt[0] = SPURIOUS_INT;
            updateIntState(-1);
            platform->intrctrl->clear(0, ArmISA::INT_IRQ, 0);
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
Gic::writeDistributor(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - distAddr;
    pkt->allocate();

    DPRINTF(Interrupt, "gic distributor write register %#x size %#x\n",
            daddr, pkt->getSize());

    if (daddr >= ICDISER_ST && daddr < ICDISER_ED + 4) {
        assert((daddr-ICDISER_ST) >> 2 < 32);
        intEnabled[(daddr-ICDISER_ST)>>2] |= pkt->get<uint32_t>();
        goto done;
    }

    if (daddr >= ICDICER_ST && daddr < ICDICER_ED + 4) {
        assert((daddr-ICDICER_ST) >> 2 < 32);
        intEnabled[(daddr-ICDICER_ST)>>2] &= ~pkt->get<uint32_t>();
        goto done;
    }

    if (daddr >= ICDISPR_ST && daddr < ICDISPR_ED + 4) {
        assert((daddr-ICDISPR_ST) >> 2 < 32);
        pendingInt[(daddr-ICDISPR_ST)>>2] |= pkt->get<uint32_t>();
        updateIntState((daddr-ICDISPR_ST)>>2);
        goto done;
    }

    if (daddr >= ICDICPR_ST && daddr < ICDICPR_ED + 4) {
        assert((daddr-ICDICPR_ST) >> 2 < 32);
        pendingInt[(daddr-ICDICPR_ST)>>2] &= ~pkt->get<uint32_t>();
        updateIntState((daddr-ICDICPR_ST)>>2);
        goto done;
    }

    if (daddr >= ICDIPR_ST && daddr < ICDIPR_ED + 4) {
        Addr int_num = daddr - ICDIPR_ST;
        assert(int_num < 1020);
        uint32_t tmp;
        switch(pkt->getSize()){
          case 1:
            tmp = pkt->get<uint8_t>();
            intPriority[int_num] = tmp & 0xff;
            break;
          case 2:
            tmp = pkt->get<uint16_t>();
            intPriority[int_num] = tmp & 0xff;
            intPriority[int_num + 1] = (tmp >> 8) & 0xff;
            break;
          case 4:
            tmp = pkt->get<uint32_t>();
            intPriority[int_num] = tmp & 0xff;
            intPriority[int_num + 1] = (tmp >> 8) & 0xff;
            intPriority[int_num + 2] = (tmp >> 16) & 0xff;
            intPriority[int_num + 3] = (tmp >> 24) & 0xff;
            break;
          default:
            panic("Invalid access size while writing to, priority registers in Gic: %d", pkt->getSize());
        }

        updateIntState(-1);
        updateRunPri();
        goto done;
    }

    if (daddr >= ICDIPTR_ST && daddr < ICDIPTR_ED + 4) {
        Addr int_num = (daddr-ICDIPTR_ST) << 2;
        assert(int_num < 1020);

        // First 31 interrupts only target single processor
        if (int_num > 31) {
            uint32_t tmp = pkt->get<uint32_t>();
            cpuTarget[int_num] = tmp & 0xff;
            cpuTarget[int_num+1] = (tmp >> 8) & 0xff;
            cpuTarget[int_num+2] = (tmp >> 16) & 0xff;
            cpuTarget[int_num+3] = (tmp >> 24) & 0xff;
            updateIntState((daddr-ICDIPTR_ST)>>2);
        }
        goto done;
    }

    if (daddr >= ICDICFR_ST && daddr < ICDICFR_ED + 4) {
        assert((daddr-ICDICFR_ST) >> 2 < 64);
        intConfig[(daddr-ICDICFR_ST)>>2] = pkt->get<uint32_t>();
        goto done;
    }

    switch(daddr) {
      case ICDDCR:
        enabled = pkt->get<uint32_t>();
        DPRINTF(Interrupt, "Distributor enable flag set to = %d\n", enabled);
        break;
      case ICDSGIR:
        softInt(pkt->get<uint32_t>());
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
Gic::writeCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - cpuAddr;
    pkt->allocate();

    DPRINTF(Interrupt, "gic cpu write register %#x val: %#x\n",
            daddr, pkt->get<uint32_t>());

    switch(daddr) {
      case ICCICR:
        cpuEnabled[0] = pkt->get<uint32_t>();
        updateIntState(-1);
        break;
      case ICCPMR:
        cpuPriority[0] = pkt->get<uint32_t>();
        updateIntState(-1);
        break;
      case ICCBPR:
        cpuBpr[0] = pkt->get<uint32_t>();
        updateIntState(-1);
        break;
      case ICCEOIR:
        uint32_t tmp;
        tmp = pkt->get<uint32_t>();
        if (!(activeInt[intNumToWord(tmp)] & (1 << intNumToBit(tmp))))
            panic("Done handling interrupt that isn't active?\n");
        activeInt[intNumToWord(tmp)] &= ~(1 << intNumToBit(tmp));
        updateRunPri();
        DPRINTF(Interrupt, "CPU done handling interrupt IAR = %d\n", tmp);
        break;
      default:
        panic("Tried to write Gic cpu at offset %#x\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return cpuPioDelay;
}

void
Gic::softInt(SWI swi)
{
    warn("Should be causing software interrupt");
}

void
Gic::updateIntState(int hint)
{
    /*@todo use hint to do less work. */
    int highest_int = SPURIOUS_INT;
    // Priorities below that set in ICCPMR can be ignored
    uint8_t highest_pri = cpuPriority[0];

    for (int x = 0; x < (itLines/32) ; x++) {
        if (intEnabled[x] & pendingInt[x]) {
            for (int y = 0; y < 32; y++) {
                if (bits(intEnabled[x], y) & bits(pendingInt[x], y))
                    if (intPriority[x*32+y] < highest_pri) {
                        highest_pri = intPriority[x*32+y];
                        highest_int = x*32 + y;
                    }
            }
        }
    }

    if (highest_int == SPURIOUS_INT)
        return;

    cpuHighestInt[0] = highest_int;


    /* @todo make this work for more than one cpu, need to handle 1:N, N:N
     * models */
    if (enabled && cpuEnabled[0] && (highest_pri < cpuPriority[0])) {
        /* @todo delay interrupt by some time to deal with calculation delay */
        /* @todo only interrupt if we've haven't already interrupted for this
         * int !!!!!!!!!! */
        DPRINTF(Interrupt, "Posting interrupt %d to cpu0\n", highest_int);
        platform->intrctrl->post(0, ArmISA::INT_IRQ, 0);
    }
}

void
Gic::updateRunPri()
{
    uint8_t maxPriority = 0xff;
    for (int i = 0 ; i < itLines ; i++){
        if ( activeInt[intNumToWord(i)] & (1 << intNumToBit(i))){
            if (intPriority[i] < maxPriority) maxPriority = intPriority[i];
        }
    }
    iccrpr[0] = maxPriority;
}
void
Gic::sendInt(uint32_t num)
{
    DPRINTF(Interrupt, "Received Interupt number %d\n", num);
    pendingInt[intNumToWord(num)] |= 1 << intNumToBit(num);
    updateIntState(intNumToWord(num));

}

void
Gic::clearInt(uint32_t number)
{
    /* @todo assume edge triggered only at the moment. Nothing to do. */
}

void
Gic::addressRanges(AddrRangeList &range_list)
{
    range_list.clear();
    range_list.push_back(RangeSize(distAddr, DIST_SIZE));
    range_list.push_back(RangeSize(cpuAddr, CPU_SIZE));
}


void
Gic::serialize(std::ostream &os)
{
    DPRINTF(Checkpoint, "Serializing Arm GIC\n");

    SERIALIZE_SCALAR(distAddr);
    SERIALIZE_SCALAR(cpuAddr);
    SERIALIZE_SCALAR(distPioDelay);
    SERIALIZE_SCALAR(cpuPioDelay);
    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(itLines);
    SERIALIZE_SCALAR(itLinesLog2);
    SERIALIZE_ARRAY(intEnabled, 32);
    SERIALIZE_ARRAY(pendingInt, 32);
    SERIALIZE_ARRAY(activeInt, 32);
    SERIALIZE_ARRAY(iccrpr, 8);
    SERIALIZE_ARRAY(intPriority, 1020);
    SERIALIZE_ARRAY(cpuTarget, 1020);
    SERIALIZE_ARRAY(intConfig, 64);
    SERIALIZE_ARRAY(cpuEnabled, 8);
    SERIALIZE_ARRAY(cpuPriority, 8);
    SERIALIZE_ARRAY(cpuBpr, 8);
    SERIALIZE_ARRAY(cpuHighestInt, 8);
    SERIALIZE_SCALAR(irqEnable);
}

void
Gic::unserialize(Checkpoint *cp, const std::string &section)
{
    DPRINTF(Checkpoint, "Unserializing Arm GIC\n");

    UNSERIALIZE_SCALAR(distAddr);
    UNSERIALIZE_SCALAR(cpuAddr);
    UNSERIALIZE_SCALAR(distPioDelay);
    UNSERIALIZE_SCALAR(cpuPioDelay);
    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(itLines);
    UNSERIALIZE_SCALAR(itLinesLog2);
    UNSERIALIZE_ARRAY(intEnabled, 32);
    UNSERIALIZE_ARRAY(pendingInt, 32);
    UNSERIALIZE_ARRAY(activeInt, 32);
    UNSERIALIZE_ARRAY(iccrpr, 8);
    UNSERIALIZE_ARRAY(intPriority, 1020);
    UNSERIALIZE_ARRAY(cpuTarget, 1020);
    UNSERIALIZE_ARRAY(intConfig, 64);
    UNSERIALIZE_ARRAY(cpuEnabled, 8);
    UNSERIALIZE_ARRAY(cpuPriority, 8);
    UNSERIALIZE_ARRAY(cpuBpr, 8);
    UNSERIALIZE_ARRAY(cpuHighestInt, 8);
    UNSERIALIZE_SCALAR(irqEnable);
}

Gic *
GicParams::create()
{
    return new Gic(this);
}

/* Functions for debugging and testing */
void
Gic::driveSPI(unsigned int spiVect)
{
    DPRINTF(GIC, "Received SPI Vector:%x Enable: %d\n", spiVect, irqEnable);
    if( irqEnable && enabled ){
        pendingInt[1] |= spiVect;
        updateIntState(-1);
    }
}

void
Gic::driveIrqEn( bool state)
{
    irqEnable = state;
}

void
Gic::driveLegIRQ(bool state)
{
    if (irqEnable && !(!enabled && cpuEnabled[0])){
        if(state){
            DPRINTF(GIC, "Driving Legacy Irq\n");
            platform->intrctrl->post(0, ArmISA::INT_IRQ, 0);
        }
        else platform->intrctrl->clear(0, ArmISA::INT_IRQ, 0);
    }
}

void
Gic::driveLegFIQ(bool state)
{
    if (state)
        platform->intrctrl->post(0, ArmISA::INT_FIQ, 0);
    else platform->intrctrl->clear(0, ArmISA::INT_FIQ, 0);
}
