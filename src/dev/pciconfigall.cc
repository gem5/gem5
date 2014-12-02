/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Andrew Schultz
 *          Ali Saidi
 */

/* @file
 * PCI Configspace implementation
 */

#include "base/trace.hh"
#include "debug/PciConfigAll.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/PciConfigAll.hh"
#include "sim/system.hh"

PciConfigAll::PciConfigAll(const Params *p)
    : BasicPioDevice(p, p->size)
{
    // the pio_addr Python parameter is ignored, and overridden by
    // this caluclated value
    pioAddr = p->platform->calcPciConfigAddr(params()->bus,0,0);
}


Tick
PciConfigAll::read(PacketPtr pkt)
{
    DPRINTF(PciConfigAll, "read  va=%#x size=%d\n", pkt->getAddr(),
            pkt->getSize());

    switch (pkt->getSize()) {
      case sizeof(uint32_t):
         pkt->set<uint32_t>(0xFFFFFFFF);
         break;
      case sizeof(uint16_t):
         pkt->set<uint16_t>(0xFFFF);
         break;
      case sizeof(uint8_t):
         pkt->set<uint8_t>(0xFF);
         break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
PciConfigAll::write(PacketPtr pkt)
{
    panic("Attempting to write to config space on non-existent device\n");
    M5_DUMMY_RETURN
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS

PciConfigAll *
PciConfigAllParams::create()
{
    return new PciConfigAll(this);
}

#endif // DOXYGEN_SHOULD_SKIP_THIS
