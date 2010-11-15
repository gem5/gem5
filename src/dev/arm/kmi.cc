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
 * Authors: William Wang
 */

#include "base/trace.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/kmi.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

Pl050::Pl050(const Params *p)
    : AmbaDevice(p), control(0x00), status(0x43), kmidata(0x00), clkdiv(0x00),
      intreg(0x00), intNum(p->int_num), gic(p->gic), intDelay(p->int_delay),
      intEvent(this)
{
    pioSize = 0xfff;
}

Tick
Pl050::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;
    pkt->allocate();

    DPRINTF(Pl050, " read register %#x size=%d\n", daddr, pkt->getSize());

    // use a temporary data since the KMI registers are read/written with
    // different size operations
    //
    uint32_t data = 0;

    switch (daddr) {
      case kmiCr:
        data = control;
        break;
      case kmiStat:
        data = status;
        break;
      case kmiData:
        data = kmidata;
        break;
      case kmiClkDiv:
        data = clkdiv;
        break;
      case kmiISR:
        data = intreg;
        break;
      default:
        if (AmbaDev::readId(pkt, ambaId, pioAddr)) {
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

    DPRINTF(Pl050, " write register %#x value %#x size=%d\n", daddr,
            pkt->get<uint8_t>(), pkt->getSize());

    // use a temporary data since the KMI registers are read/written with
    // different size operations
    //
    uint32_t data = 0;

    switch (pkt->getSize()) {
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
        panic("KMI write size too big?\n");
        break;
    }


    switch (daddr) {
      case kmiCr:
        control = data;
        break;
      case kmiStat:
        panic("Tried to write PL050 register(read only) at offset %#x\n",
              daddr);
        break;
      case kmiData:
        kmidata = data;
        break;
      case kmiClkDiv:
        clkdiv = data;
        break;
      case kmiISR:
        panic("Tried to write PL050 register(read only) at offset %#x\n",
              daddr);
        break;
      default:
        warn("Tried to write PL050 at offset %#x that doesn't exist\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Pl050::generateInterrupt()
{
    if (intreg.rxintr || intreg.txintr) {
        gic->sendInt(intNum);
        DPRINTF(Pl050, " -- Generated\n");
    }
}

Pl050 *
Pl050Params::create()
{
    return new Pl050(this);
}
