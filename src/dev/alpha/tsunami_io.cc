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
 *          Miguel Serrano
 */

/** @file
 * Tsunami I/O including PIC, PIT, RTC, DMA
 */

#include "dev/alpha/tsunami_io.hh"

#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/time.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/Tsunami.hh"
#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunami_cchip.hh"
#include "dev/alpha/tsunamireg.h"
#include "dev/rtcreg.h"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "sim/system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::string;
using std::ostream;

//Should this be AlphaISA?
using namespace TheISA;

TsunamiIO::RTC::RTC(const string &n, const TsunamiIOParams *p)
    : MC146818(p->tsunami, n, p->time, p->year_is_bcd, p->frequency),
      tsunami(p->tsunami)
{
}

TsunamiIO::TsunamiIO(const Params *p)
    : BasicPioDevice(p, 0x100), tsunami(p->tsunami),
      pitimer(this, p->name + "pitimer"), rtc(p->name + ".rtc", p)
{
    // set the back pointer from tsunami to myself
    tsunami->io = this;

    timerData = 0;
    picr = 0;
    picInterrupting = false;
}

Tick
TsunamiIO::frequency() const
{
    return SimClock::Frequency / params()->frequency;
}

Tick
TsunamiIO::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Tsunami, "io read  va=%#x size=%d IOPorrt=%#x\n", pkt->getAddr(),
            pkt->getSize(), daddr);

    if (pkt->getSize() == sizeof(uint8_t)) {
        switch(daddr) {
          // PIC1 mask read
          case TSDEV_PIC1_MASK:
            pkt->set(~mask1);
            break;
          case TSDEV_PIC2_MASK:
            pkt->set(~mask2);
            break;
          case TSDEV_PIC1_ISR:
              // !!! If this is modified 64bit case needs to be too
              // Pal code has to do a 64 bit physical read because there is
              // no load physical byte instruction
              pkt->set(picr);
              break;
          case TSDEV_PIC2_ISR:
              // PIC2 not implemnted... just return 0
              pkt->set(0x00);
              break;
          case TSDEV_TMR0_DATA:
            pkt->set(pitimer.readCounter(0));
            break;
          case TSDEV_TMR1_DATA:
            pkt->set(pitimer.readCounter(1));
            break;
          case TSDEV_TMR2_DATA:
            pkt->set(pitimer.readCounter(2));
            break;
          case TSDEV_RTC_DATA:
            pkt->set(rtc.readData(rtcAddr));
            break;
          case TSDEV_CTRL_PORTB:
            if (pitimer.outputHigh(2))
                pkt->set(PORTB_SPKR_HIGH);
            else
                pkt->set(0x00);
            break;
          default:
            panic("I/O Read - va%#x size %d\n", pkt->getAddr(), pkt->getSize());
        }
    } else if (pkt->getSize() == sizeof(uint64_t)) {
        if (daddr == TSDEV_PIC1_ISR)
            pkt->set<uint64_t>(picr);
        else
           panic("I/O Read - invalid addr - va %#x size %d\n",
                   pkt->getAddr(), pkt->getSize());
    } else {
       panic("I/O Read - invalid size - va %#x size %d\n", pkt->getAddr(), pkt->getSize());
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
TsunamiIO::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Tsunami, "io write - va=%#x size=%d IOPort=%#x Data=%#x\n",
            pkt->getAddr(), pkt->getSize(), pkt->getAddr() & 0xfff, (uint32_t)pkt->get<uint8_t>());

    assert(pkt->getSize() == sizeof(uint8_t));

    switch(daddr) {
      case TSDEV_PIC1_MASK:
        mask1 = ~(pkt->get<uint8_t>());
        if ((picr & mask1) && !picInterrupting) {
            picInterrupting = true;
            tsunami->cchip->postDRIR(55);
            DPRINTF(Tsunami, "posting pic interrupt to cchip\n");
        }
        if ((!(picr & mask1)) && picInterrupting) {
            picInterrupting = false;
            tsunami->cchip->clearDRIR(55);
            DPRINTF(Tsunami, "clearing pic interrupt\n");
        }
        break;
      case TSDEV_PIC2_MASK:
        mask2 = pkt->get<uint8_t>();
        //PIC2 Not implemented to interrupt
        break;
      case TSDEV_PIC1_ACK:
        // clear the interrupt on the PIC
        picr &= ~(1 << (pkt->get<uint8_t>() & 0xF));
        if (!(picr & mask1))
            tsunami->cchip->clearDRIR(55);
        break;
      case TSDEV_DMA1_MODE:
        mode1 = pkt->get<uint8_t>();
        break;
      case TSDEV_DMA2_MODE:
        mode2 = pkt->get<uint8_t>();
        break;
      case TSDEV_TMR0_DATA:
        pitimer.writeCounter(0, pkt->get<uint8_t>());
        break;
      case TSDEV_TMR1_DATA:
        pitimer.writeCounter(1, pkt->get<uint8_t>());
        break;
      case TSDEV_TMR2_DATA:
        pitimer.writeCounter(2, pkt->get<uint8_t>());
        break;
      case TSDEV_TMR_CTRL:
        pitimer.writeControl(pkt->get<uint8_t>());
        break;
      case TSDEV_RTC_ADDR:
        rtcAddr = pkt->get<uint8_t>();
        break;
      case TSDEV_RTC_DATA:
        rtc.writeData(rtcAddr, pkt->get<uint8_t>());
        break;
      case TSDEV_KBD:
      case TSDEV_DMA1_CMND:
      case TSDEV_DMA2_CMND:
      case TSDEV_DMA1_MMASK:
      case TSDEV_DMA2_MMASK:
      case TSDEV_PIC2_ACK:
      case TSDEV_DMA1_RESET:
      case TSDEV_DMA2_RESET:
      case TSDEV_DMA1_MASK:
      case TSDEV_DMA2_MASK:
      case TSDEV_CTRL_PORTB:
        break;
      default:
        panic("I/O Write - va%#x size %d data %#x\n", pkt->getAddr(), pkt->getSize(), pkt->get<uint8_t>());
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
TsunamiIO::postPIC(uint8_t bitvector)
{
    //PIC2 Is not implemented, because nothing of interest there
    picr |= bitvector;
    if (picr & mask1) {
        tsunami->cchip->postDRIR(55);
        DPRINTF(Tsunami, "posting pic interrupt to cchip\n");
    }
}

void
TsunamiIO::clearPIC(uint8_t bitvector)
{
    //PIC2 Is not implemented, because nothing of interest there
    picr &= ~bitvector;
    if (!(picr & mask1)) {
        tsunami->cchip->clearDRIR(55);
        DPRINTF(Tsunami, "clearing pic interrupt to cchip\n");
    }
}

void
TsunamiIO::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(rtcAddr);
    SERIALIZE_SCALAR(timerData);
    SERIALIZE_SCALAR(mask1);
    SERIALIZE_SCALAR(mask2);
    SERIALIZE_SCALAR(mode1);
    SERIALIZE_SCALAR(mode2);
    SERIALIZE_SCALAR(picr);
    SERIALIZE_SCALAR(picInterrupting);

    // Serialize the timers
    pitimer.serialize("pitimer", cp);
    rtc.serialize("rtc", cp);
}

void
TsunamiIO::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(rtcAddr);
    UNSERIALIZE_SCALAR(timerData);
    UNSERIALIZE_SCALAR(mask1);
    UNSERIALIZE_SCALAR(mask2);
    UNSERIALIZE_SCALAR(mode1);
    UNSERIALIZE_SCALAR(mode2);
    UNSERIALIZE_SCALAR(picr);
    UNSERIALIZE_SCALAR(picInterrupting);

    // Unserialize the timers
    pitimer.unserialize("pitimer", cp);
    rtc.unserialize("rtc", cp);
}

void
TsunamiIO::startup()
{
    rtc.startup();
    pitimer.startup();
}

TsunamiIO *
TsunamiIOParams::create()
{
    return new TsunamiIO(this);
}
