/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
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
 *          William Wang
 */

#include "dev/arm/kmi.hh"

#include "base/trace.hh"
#include "base/vnc/vncinput.hh"
#include "debug/Pl050.hh"
#include "dev/arm/amba_device.hh"
#include "dev/ps2/device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Pl050::Pl050(const Pl050Params *p)
    : AmbaIntDevice(p, 0xfff), control(0), status(0x43), clkdiv(0),
      rawInterrupts(0),
      intEvent([this]{ generateInterrupt(); }, name()),
      ps2(p->ps2)
{
    ps2->hostRegDataAvailable([this]() { this->updateIntStatus(); });
}

Tick
Pl050::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    uint32_t data = 0;

    switch (daddr) {
      case kmiCr:
        DPRINTF(Pl050, "Read Commmand: %#x\n", (uint32_t)control);
        data = control;
        break;

      case kmiStat:
        status.rxfull = ps2->hostDataAvailable() ? 1 : 0;
        DPRINTF(Pl050, "Read Status: %#x\n", (uint32_t)status);
        data = status;
        break;

      case kmiData:
        data = ps2->hostDataAvailable() ? ps2->hostRead() : 0;
        DPRINTF(Pl050, "Read Data: %#x\n", (uint32_t)data);
        updateIntStatus();
        break;

      case kmiClkDiv:
        data = clkdiv;
        break;

      case kmiISR:
        data = getInterrupt();
        DPRINTF(Pl050, "Read Interrupts: %#x\n", getInterrupt());
        break;

      default:
        if (readId(pkt, ambaId, pioAddr)) {
            // Hack for variable size accesses
            data = pkt->get<uint32_t>();
            break;
        }

        warn("Tried to read PL050 at offset %#x that doesn't exist\n", daddr);
        break;
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
        panic("KMI read size too big?\n");
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Pl050::write(PacketPtr pkt)
{

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    assert(pkt->getSize() == sizeof(uint8_t));


    switch (daddr) {
      case kmiCr:
        DPRINTF(Pl050, "Write Commmand: %#x\n", (uint32_t)pkt->get<uint8_t>());
        control = pkt->get<uint8_t>();
        updateIntStatus();
        break;

      case kmiData:
        DPRINTF(Pl050, "Write Data: %#x\n", (uint32_t)pkt->get<uint8_t>());
        ps2->hostWrite(pkt->get<uint8_t>());
        updateIntStatus();
        break;

      case kmiClkDiv:
        clkdiv = pkt->get<uint8_t>();
        break;

      default:
        warn("Tried to write PL050 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}


void
Pl050::updateIntStatus()
{
    const bool old_interrupt(getInterrupt());

    rawInterrupts.rx = ps2->hostDataAvailable() ? 1 : 0;

    if ((!old_interrupt && getInterrupt()) && !intEvent.scheduled()) {
        schedule(intEvent, curTick() + intDelay);
    } else if (old_interrupt && !(getInterrupt())) {
            gic->clearInt(intNum);
    }
}

Pl050::InterruptReg
Pl050::getInterrupt() const
{
    InterruptReg tmp_interrupt(0);

    tmp_interrupt.tx = rawInterrupts.tx & control.txint_enable;
    tmp_interrupt.rx = rawInterrupts.rx & control.rxint_enable;

    return tmp_interrupt;
}

void
Pl050::generateInterrupt()
{
    DPRINTF(Pl050, "Generate Interrupt: rawInt=%#x ctrl=%#x int=%#x\n",
            rawInterrupts, control, getInterrupt());

    if (getInterrupt()) {
        gic->sendInt(intNum);
        DPRINTF(Pl050, " -- Generated\n");
    }
}

void
Pl050::serialize(CheckpointOut &cp) const
{
    uint8_t ctrlreg = control;
    SERIALIZE_SCALAR(ctrlreg);

    uint8_t stsreg = status;
    SERIALIZE_SCALAR(stsreg);
    SERIALIZE_SCALAR(clkdiv);

    uint8_t raw_ints = rawInterrupts;
    SERIALIZE_SCALAR(raw_ints);
}

void
Pl050::unserialize(CheckpointIn &cp)
{
    uint8_t ctrlreg;
    UNSERIALIZE_SCALAR(ctrlreg);
    control = ctrlreg;

    uint8_t stsreg;
    UNSERIALIZE_SCALAR(stsreg);
    status = stsreg;

    UNSERIALIZE_SCALAR(clkdiv);

    uint8_t raw_ints;
    UNSERIALIZE_SCALAR(raw_ints);
    rawInterrupts = raw_ints;
}

Pl050 *
Pl050Params::create()
{
    return new Pl050(this);
}
