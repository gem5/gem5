/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "dev/x86/south_bridge/i8254.hh"
#include "mem/packet_access.hh"

Tick
X86ISA::I8254::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - addrRange.start)
    {
      case 0x0:
        pkt->set(pit.readCounter(0));
        break;
      case 0x1:
        pkt->set(pit.readCounter(1));
        break;
      case 0x2:
        pkt->set(pit.readCounter(2));
        break;
      case 0x3:
        pkt->set(uint8_t(-1));
        break;
      default:
        panic("Read from undefined i8254 register.\n");
    }
    return latency;
}

Tick
X86ISA::I8254::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - addrRange.start)
    {
      case 0x0:
        pit.writeCounter(0, pkt->get<uint8_t>());
        break;
      case 0x1:
        pit.writeCounter(1, pkt->get<uint8_t>());
        break;
      case 0x2:
        pit.writeCounter(2, pkt->get<uint8_t>());
        break;
      case 0x3:
        pit.writeControl(pkt->get<uint8_t>());
        break;
      default:
        panic("Write to undefined i8254 register.\n");
    }
    return latency;
}
