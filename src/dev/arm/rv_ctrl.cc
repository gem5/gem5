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
 * Authors: Ali Saidi
 */

#include "base/trace.hh"
#include "dev/arm/rv_ctrl.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

RealViewCtrl::RealViewCtrl(Params *p)
    : BasicPioDevice(p), flags(0)
{
    pioSize = 0xD4;
}

Tick
RealViewCtrl::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    pkt->allocate();

    switch(daddr) {
      case ProcId0:
        pkt->set(params()->proc_id0);
        break;
      case ProcId1:
        pkt->set(params()->proc_id1);
        break;
      case Clock24:
        Tick clk;
        clk = SimClock::Float::MHz * curTick() * 24;
        pkt->set((uint32_t)(clk));
        break;
      case Clock100:
        Tick clk100;
        clk100 = SimClock::Float::MHz * curTick() * 100;
        pkt->set((uint32_t)(clk100));
        break;
      case Flash:
        pkt->set<uint32_t>(0);
        break;
      case Clcd:
        pkt->set<uint32_t>(0x00001F00);
        break;
      case Osc0:
        pkt->set<uint32_t>(0x00012C5C);
        break;
      case Osc1:
        pkt->set<uint32_t>(0x00002CC0);
        break;
      case Osc2:
        pkt->set<uint32_t>(0x00002C75);
        break;
      case Osc3:
        pkt->set<uint32_t>(0x00020211);
        break;
      case Osc4:
        pkt->set<uint32_t>(0x00002C75);
        break;
      case Lock:
        pkt->set<uint32_t>(sysLock);
        break;
      case Flags:
        pkt->set<uint32_t>(flags);
        break;
      case IdReg:
        pkt->set<uint32_t>(params()->idreg);
        break;
      case CfgStat:
        pkt->set<uint32_t>(1);
        break;
      default:
        warn("Tried to read RealView I/O at offset %#x that doesn't exist\n",
             daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;

}

Tick
RealViewCtrl::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;
    switch (daddr) {
      case Flash:
      case Clcd:
      case Osc0:
      case Osc1:
      case Osc2:
      case Osc3:
      case Osc4:
        break;
      case Lock:
        sysLock.lockVal = pkt->get<uint16_t>();
        break;
      case Flags:
        flags = pkt->get<uint32_t>();
        break;
      case FlagsClr:
        flags = 0;
        break;
      default:
        warn("Tried to write RVIO at offset %#x that doesn't exist\n",
             daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
RealViewCtrl::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(flags);
}

void
RealViewCtrl::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(flags);
}

RealViewCtrl *
RealViewCtrlParams::create()
{
    return new RealViewCtrl(this);
}
