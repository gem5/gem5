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
 * Authors: Ali Saidi
 *          Andrew Schultz
 */

/** @file
 * Tsunami PChip (pci)
 */
#include "dev/alpha/tsunami_pchip.hh"

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/Tsunami.hh"
#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunami_cchip.hh"
#include "dev/alpha/tsunamireg.h"
#include "dev/pci/device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

using namespace std;
//Should this be AlphaISA?
using namespace TheISA;

TsunamiPChip::TsunamiPChip(const Params *p)
    : GenericPciHost(p),
      pioRange(RangeSize(p->pio_addr, 0x1000)),
      pioDelay(p->pio_latency)
{
    for (int i = 0; i < 4; i++) {
        wsba[i] = 0;
        wsm[i] = 0;
        tba[i] = 0;
    }

    // initialize pchip control register
    pctl = (ULL(0x1) << 20) | (ULL(0x1) << 32) | (ULL(0x2) << 36);

    //Set back pointer in tsunami
    p->tsunami->pchip = this;
}

Tick
TsunamiPChip::read(PacketPtr pkt)
{
    // We only need to handle our own configuration registers, pass
    // unknown addresses to the generic code.
    if (!pioRange.contains(pkt->getAddr()))
        return GenericPciHost::read(pkt);

    Addr daddr = (pkt->getAddr() - pioRange.start()) >> 6;;
    assert(pkt->getSize() == sizeof(uint64_t));


    DPRINTF(Tsunami, "read  va=%#x size=%d\n", pkt->getAddr(), pkt->getSize());

    switch(daddr) {
      case TSDEV_PC_WSBA0:
            pkt->set(wsba[0]);
            break;
      case TSDEV_PC_WSBA1:
            pkt->set(wsba[1]);
            break;
      case TSDEV_PC_WSBA2:
            pkt->set(wsba[2]);
            break;
      case TSDEV_PC_WSBA3:
            pkt->set(wsba[3]);
            break;
      case TSDEV_PC_WSM0:
            pkt->set(wsm[0]);
            break;
      case TSDEV_PC_WSM1:
            pkt->set(wsm[1]);
            break;
      case TSDEV_PC_WSM2:
            pkt->set(wsm[2]);
            break;
      case TSDEV_PC_WSM3:
            pkt->set(wsm[3]);
            break;
      case TSDEV_PC_TBA0:
            pkt->set(tba[0]);
            break;
      case TSDEV_PC_TBA1:
            pkt->set(tba[1]);
            break;
      case TSDEV_PC_TBA2:
            pkt->set(tba[2]);
            break;
      case TSDEV_PC_TBA3:
            pkt->set(tba[3]);
            break;
      case TSDEV_PC_PCTL:
            pkt->set(pctl);
            break;
      case TSDEV_PC_PLAT:
            panic("PC_PLAT not implemented\n");
      case TSDEV_PC_RES:
            panic("PC_RES not implemented\n");
      case TSDEV_PC_PERROR:
            pkt->set((uint64_t)0x00);
            break;
      case TSDEV_PC_PERRMASK:
            pkt->set((uint64_t)0x00);
            break;
      case TSDEV_PC_PERRSET:
            panic("PC_PERRSET not implemented\n");
      case TSDEV_PC_TLBIV:
            panic("PC_TLBIV not implemented\n");
      case TSDEV_PC_TLBIA:
            pkt->set((uint64_t)0x00); // shouldn't be readable, but linux
            break;
      case TSDEV_PC_PMONCTL:
            panic("PC_PMONCTL not implemented\n");
      case TSDEV_PC_PMONCNT:
            panic("PC_PMONCTN not implemented\n");
      default:
          panic("Default in PChip Read reached reading 0x%x\n", daddr);
    }

    pkt->makeAtomicResponse();
    return pioDelay;

}

Tick
TsunamiPChip::write(PacketPtr pkt)
{
    // We only need to handle our own configuration registers, pass
    // unknown addresses to the generic code.
    if (!pioRange.contains(pkt->getAddr()))
        return GenericPciHost::write(pkt);

    Addr daddr = (pkt->getAddr() - pioRange.start()) >> 6;

    assert(pkt->getSize() == sizeof(uint64_t));

    DPRINTF(Tsunami, "write - va=%#x size=%d \n", pkt->getAddr(), pkt->getSize());

    switch(daddr) {
        case TSDEV_PC_WSBA0:
              wsba[0] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSBA1:
              wsba[1] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSBA2:
              wsba[2] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSBA3:
              wsba[3] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSM0:
              wsm[0] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSM1:
              wsm[1] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSM2:
              wsm[2] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_WSM3:
              wsm[3] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_TBA0:
              tba[0] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_TBA1:
              tba[1] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_TBA2:
              tba[2] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_TBA3:
              tba[3] = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_PCTL:
              pctl = pkt->get<uint64_t>();
              break;
        case TSDEV_PC_PLAT:
              panic("PC_PLAT not implemented\n");
        case TSDEV_PC_RES:
              panic("PC_RES not implemented\n");
        case TSDEV_PC_PERROR:
              break;
        case TSDEV_PC_PERRMASK:
              panic("PC_PERRMASK not implemented\n");
        case TSDEV_PC_PERRSET:
              panic("PC_PERRSET not implemented\n");
        case TSDEV_PC_TLBIV:
              panic("PC_TLBIV not implemented\n");
        case TSDEV_PC_TLBIA:
              break; // value ignored, supposted to invalidate SG TLB
        case TSDEV_PC_PMONCTL:
              panic("PC_PMONCTL not implemented\n");
        case TSDEV_PC_PMONCNT:
              panic("PC_PMONCTN not implemented\n");
        default:
            panic("Default in PChip write reached reading 0x%x\n", daddr);

    } // uint64_t

    pkt->makeAtomicResponse();
    return pioDelay;
}


AddrRangeList
TsunamiPChip::getAddrRanges() const
{
    return AddrRangeList({
            RangeSize(confBase, confSize),
            pioRange
        });
}


#define DMA_ADDR_MASK ULL(0x3ffffffff)

Addr
TsunamiPChip::dmaAddr(const PciBusAddr &dev, Addr busAddr) const
{
    // compare the address to the window base registers
    uint64_t tbaMask = 0;
    uint64_t baMask = 0;

    uint64_t windowMask = 0;
    uint64_t windowBase = 0;

    uint64_t pteEntry = 0;

    Addr pteAddr;
    Addr dmaAddr;

#if 0
    DPRINTF(IdeDisk, "Translation for bus address: %#x\n", busAddr);
    for (int i = 0; i < 4; i++) {
        DPRINTF(IdeDisk, "(%d) base:%#x mask:%#x\n",
                i, wsba[i], wsm[i]);

        windowBase = wsba[i];
        windowMask = ~wsm[i] & (ULL(0xfff) << 20);

        if ((busAddr & windowMask) == (windowBase & windowMask)) {
            DPRINTF(IdeDisk, "Would have matched %d (wb:%#x wm:%#x --> ba&wm:%#x wb&wm:%#x)\n",
                    i, windowBase, windowMask, (busAddr & windowMask),
                    (windowBase & windowMask));
        }
    }
#endif

    for (int i = 0; i < 4; i++) {

        windowBase = wsba[i];
        windowMask = ~wsm[i] & (ULL(0xfff) << 20);

        if ((busAddr & windowMask) == (windowBase & windowMask)) {

            if (wsba[i] & 0x1) {   // see if enabled
                if (wsba[i] & 0x2) { // see if SG bit is set
                    /** @todo
                        This currently is faked by just doing a direct
                        read from memory, however, to be realistic, this
                        needs to actually do a bus transaction.  The process
                        is explained in the tsunami documentation on page
                        10-12 and basically munges the address to look up a
                        PTE from a table in memory and then uses that mapping
                        to create an address for the SG page
                    */

                    tbaMask = ~(((wsm[i] & (ULL(0xfff) << 20)) >> 10) | ULL(0x3ff));
                    baMask = (wsm[i] & (ULL(0xfff) << 20)) | (ULL(0x7f) << 13);
                    pteAddr = (tba[i] & tbaMask) | ((busAddr & baMask) >> 10);

                    sys->physProxy.readBlob(pteAddr, (uint8_t*)&pteEntry,
                                            sizeof(uint64_t));

                    dmaAddr = ((pteEntry & ~ULL(0x1)) << 12) | (busAddr & ULL(0x1fff));

                } else {
                    baMask = (wsm[i] & (ULL(0xfff) << 20)) | ULL(0xfffff);
                    tbaMask = ~baMask;
                    dmaAddr = (tba[i] & tbaMask) | (busAddr & baMask);
                }

                return (dmaAddr & DMA_ADDR_MASK);
            }
        }
    }

    // if no match was found, then return the original address
    return busAddr;
}

void
TsunamiPChip::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(pctl);
    SERIALIZE_ARRAY(wsba, 4);
    SERIALIZE_ARRAY(wsm, 4);
    SERIALIZE_ARRAY(tba, 4);
}

void
TsunamiPChip::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(pctl);
    UNSERIALIZE_ARRAY(wsba, 4);
    UNSERIALIZE_ARRAY(wsm, 4);
    UNSERIALIZE_ARRAY(tba, 4);
}


TsunamiPChip *
TsunamiPChipParams::create()
{
    return new TsunamiPChip(this);
}
