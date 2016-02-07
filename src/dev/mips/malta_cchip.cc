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
 *          Rick Strong
 */

/** @file
 * Emulation of the Malta CChip CSRs
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/intr_control.hh"
#include "cpu/thread_context.hh"
#include "debug/Malta.hh"
#include "dev/mips/malta.hh"
#include "dev/mips/malta_cchip.hh"
#include "dev/mips/maltareg.h"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/MaltaCChip.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

MaltaCChip::MaltaCChip(Params *p)
    : BasicPioDevice(p, 0xfffffff), malta(p->malta)
{
    warn("MaltaCCHIP::MaltaCChip() not implemented.");

    //Put back pointer in malta
    malta->cchip = this;

}

Tick
MaltaCChip::read(PacketPtr pkt)
{
                panic("MaltaCCHIP::read() not implemented.");
                return pioDelay;
                /*
    DPRINTF(Malta, "read  va=%#x size=%d\n", pkt->getAddr(), pkt->getSize());

    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr regnum = (pkt->getAddr() - pioAddr) >> 6;
    Addr daddr = (pkt->getAddr() - pioAddr);

    switch (pkt->getSize()) {

      case sizeof(uint64_t):
          if (daddr & TSDEV_CC_BDIMS)
          {
              pkt->set(dim[(daddr >> 4) & 0x3F]);
              break;
          }

          if (daddr & TSDEV_CC_BDIRS)
          {
              pkt->set(dir[(daddr >> 4) & 0x3F]);
              break;
          }

          switch(regnum) {
              case TSDEV_CC_CSR:
                  pkt->set(0x0);
                  break;
              case TSDEV_CC_MTR:
                  panic("TSDEV_CC_MTR not implemeted\n");
                   break;
              case TSDEV_CC_MISC:
                  pkt->set((ipint << 8) & 0xF | (itint << 4) & 0xF |
                                     (pkt->req->contextId() & 0x3));
                  break;
              case TSDEV_CC_AAR0:
              case TSDEV_CC_AAR1:
              case TSDEV_CC_AAR2:
              case TSDEV_CC_AAR3:
                  pkt->set(0);
                  break;
              case TSDEV_CC_DIM0:
                  pkt->set(dim[0]);
                  break;
              case TSDEV_CC_DIM1:
                  pkt->set(dim[1]);
                  break;
              case TSDEV_CC_DIM2:
                  pkt->set(dim[2]);
                  break;
              case TSDEV_CC_DIM3:
                  pkt->set(dim[3]);
                  break;
              case TSDEV_CC_DIR0:
                  pkt->set(dir[0]);
                  break;
              case TSDEV_CC_DIR1:
                  pkt->set(dir[1]);
                  break;
              case TSDEV_CC_DIR2:
                  pkt->set(dir[2]);
                  break;
              case TSDEV_CC_DIR3:
                  pkt->set(dir[3]);
                  break;
              case TSDEV_CC_DRIR:
                  pkt->set(drir);
                  break;
              case TSDEV_CC_PRBEN:
                  panic("TSDEV_CC_PRBEN not implemented\n");
                  break;
              case TSDEV_CC_IIC0:
              case TSDEV_CC_IIC1:
              case TSDEV_CC_IIC2:
              case TSDEV_CC_IIC3:
                  panic("TSDEV_CC_IICx not implemented\n");
                  break;
              case TSDEV_CC_MPR0:
              case TSDEV_CC_MPR1:
              case TSDEV_CC_MPR2:
              case TSDEV_CC_MPR3:
                  panic("TSDEV_CC_MPRx not implemented\n");
                  break;
              case TSDEV_CC_IPIR:
                  pkt->set(ipint);
                  break;
              case TSDEV_CC_ITIR:
                  pkt->set(itint);
                  break;
              default:
                  panic("default in cchip read reached, accessing 0x%x\n");
           } // uint64_t

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for malta register!\n");
    }
    DPRINTF(Malta, "Malta CChip: read  regnum=%#x size=%d data=%lld\n",
            regnum, pkt->getSize(), pkt->get<uint64_t>());

    pkt->result = Packet::Success;
    return pioDelay;
    */
}

Tick
MaltaCChip::write(PacketPtr pkt)
{
                panic("MaltaCCHIP::write() not implemented.");
                return pioDelay;
                /*
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;
    Addr regnum = (pkt->getAddr() - pioAddr) >> 6 ;


    assert(pkt->getSize() == sizeof(uint64_t));

    DPRINTF(Malta, "write - addr=%#x value=%#x\n", pkt->getAddr(), pkt->get<uint64_t>());

    bool supportedWrite = false;


    if (daddr & TSDEV_CC_BDIMS)
    {
        int number = (daddr >> 4) & 0x3F;

        uint64_t bitvector;
        uint64_t olddim;
        uint64_t olddir;

        olddim = dim[number];
        olddir = dir[number];
        dim[number] = pkt->get<uint64_t>();
        dir[number] = dim[number] & drir;
        for (int x = 0; x < Malta::Max_CPUs; x++)
        {
            bitvector = ULL(1) << x;
            // Figure out which bits have changed
            if ((dim[number] & bitvector) != (olddim & bitvector))
            {
                // The bit is now set and it wasn't before (set)
                if ((dim[number] & bitvector) && (dir[number] & bitvector))
                {
                    malta->intrctrl->post(number, TheISA::INTLEVEL_IRQ1, x);
                    DPRINTF(Malta, "dim write resulting in posting dir"
                            " interrupt to cpu %d\n", number);
                }
                else if ((olddir & bitvector) &&
                        !(dir[number] & bitvector))
                {
                    // The bit was set and now its now clear and
                    // we were interrupting on that bit before
                    malta->intrctrl->clear(number, TheISA::INTLEVEL_IRQ1, x);
                    DPRINTF(Malta, "dim write resulting in clear"
                            " dir interrupt to cpu %d\n", number);

                }


            }
        }
    } else {
        switch(regnum) {
          case TSDEV_CC_CSR:
              panic("TSDEV_CC_CSR write\n");
          case TSDEV_CC_MTR:
              panic("TSDEV_CC_MTR write not implemented\n");
          case TSDEV_CC_MISC:
            uint64_t ipreq;
            ipreq = (pkt->get<uint64_t>() >> 12) & 0xF;
            //If it is bit 12-15, this is an IPI post
            if (ipreq) {
                reqIPI(ipreq);
                supportedWrite = true;
            }

            //If it is bit 8-11, this is an IPI clear
            uint64_t ipintr;
            ipintr = (pkt->get<uint64_t>() >> 8) & 0xF;
            if (ipintr) {
                clearIPI(ipintr);
                supportedWrite = true;
            }

            //If it is the 4-7th bit, clear the RTC interrupt
            uint64_t itintr;
              itintr = (pkt->get<uint64_t>() >> 4) & 0xF;
            if (itintr) {
                  clearITI(itintr);
                supportedWrite = true;
            }

              // ignore NXMs
              if (pkt->get<uint64_t>() & 0x10000000)
                  supportedWrite = true;

            if (!supportedWrite)
                  panic("TSDEV_CC_MISC write not implemented\n");

            break;
            case TSDEV_CC_AAR0:
            case TSDEV_CC_AAR1:
            case TSDEV_CC_AAR2:
            case TSDEV_CC_AAR3:
                panic("TSDEV_CC_AARx write not implemeted\n");
            case TSDEV_CC_DIM0:
            case TSDEV_CC_DIM1:
            case TSDEV_CC_DIM2:
            case TSDEV_CC_DIM3:
                int number;
                if (regnum == TSDEV_CC_DIM0)
                    number = 0;
                else if (regnum == TSDEV_CC_DIM1)
                    number = 1;
                else if (regnum == TSDEV_CC_DIM2)
                    number = 2;
                else
                    number = 3;

                uint64_t bitvector;
                uint64_t olddim;
                uint64_t olddir;

                olddim = dim[number];
                olddir = dir[number];
                dim[number] = pkt->get<uint64_t>();
                dir[number] = dim[number] & drir;
                for (int x = 0; x < 64; x++)
                {
                    bitvector = ULL(1) << x;
                    // Figure out which bits have changed
                    if ((dim[number] & bitvector) != (olddim & bitvector))
                    {
                        // The bit is now set and it wasn't before (set)
                        if ((dim[number] & bitvector) && (dir[number] & bitvector))
                        {
                          malta->intrctrl->post(number, TheISA::INTLEVEL_IRQ1, x);
                          DPRINTF(Malta, "posting dir interrupt to cpu 0\n");
                        }
                        else if ((olddir & bitvector) &&
                                !(dir[number] & bitvector))
                        {
                            // The bit was set and now its now clear and
                            // we were interrupting on that bit before
                            malta->intrctrl->clear(number, TheISA::INTLEVEL_IRQ1, x);
                          DPRINTF(Malta, "dim write resulting in clear"
                                    " dir interrupt to cpu %d\n",
                                    x);

                        }


                    }
                }
                break;
            case TSDEV_CC_DIR0:
            case TSDEV_CC_DIR1:
            case TSDEV_CC_DIR2:
            case TSDEV_CC_DIR3:
                panic("TSDEV_CC_DIR write not implemented\n");
            case TSDEV_CC_DRIR:
                panic("TSDEV_CC_DRIR write not implemented\n");
            case TSDEV_CC_PRBEN:
                panic("TSDEV_CC_PRBEN write not implemented\n");
            case TSDEV_CC_IIC0:
            case TSDEV_CC_IIC1:
            case TSDEV_CC_IIC2:
            case TSDEV_CC_IIC3:
                panic("TSDEV_CC_IICx write not implemented\n");
            case TSDEV_CC_MPR0:
            case TSDEV_CC_MPR1:
            case TSDEV_CC_MPR2:
            case TSDEV_CC_MPR3:
                panic("TSDEV_CC_MPRx write not implemented\n");
            case TSDEV_CC_IPIR:
                clearIPI(pkt->get<uint64_t>());
                break;
            case TSDEV_CC_ITIR:
                clearITI(pkt->get<uint64_t>());
                break;
            case TSDEV_CC_IPIQ:
                reqIPI(pkt->get<uint64_t>());
                break;
            default:
              panic("default in cchip read reached, accessing 0x%x\n");
        }  // swtich(regnum)
    } // not BIG_TSUNAMI write
    pkt->result = Packet::Success;
    return pioDelay;
    */
}

void
MaltaCChip::clearIPI(uint64_t ipintr)
{
                panic("MaltaCCHIP::clear() not implemented.");
                /*
    int numcpus = malta->intrctrl->cpu->system->threadContexts.size();
    assert(numcpus <= Malta::Max_CPUs);

    if (ipintr) {
        for (int cpunum=0; cpunum < numcpus; cpunum++) {
            // Check each cpu bit
            uint64_t cpumask = ULL(1) << cpunum;
            if (ipintr & cpumask) {
                // Check if there is a pending ipi
                if (ipint & cpumask) {
                    ipint &= ~cpumask;
                    malta->intrctrl->clear(cpunum, TheISA::INTLEVEL_IRQ3, 0);
                    DPRINTF(IPI, "clear IPI IPI cpu=%d\n", cpunum);
                }
                else
                    warn("clear IPI for CPU=%d, but NO IPI\n", cpunum);
            }
        }
    }
    else
        panic("Big IPI Clear, but not processors indicated\n");
        */
}

void
MaltaCChip::clearITI(uint64_t itintr)
{
                panic("MaltaCCHIP::clearITI() not implemented.");
                /*
    int numcpus = malta->intrctrl->cpu->system->threadContexts.size();
    assert(numcpus <= Malta::Max_CPUs);

    if (itintr) {
        for (int i=0; i < numcpus; i++) {
            uint64_t cpumask = ULL(1) << i;
            if (itintr & cpumask & itint) {
                malta->intrctrl->clear(i, TheISA::INTLEVEL_IRQ2, 0);
                itint &= ~cpumask;
                DPRINTF(Malta, "clearing rtc interrupt to cpu=%d\n", i);
            }
        }
    }
    else
        panic("Big ITI Clear, but not processors indicated\n");
    */
}

void
MaltaCChip::reqIPI(uint64_t ipreq)
{
                panic("MaltaCCHIP::reqIPI() not implemented.");

                /*
    int numcpus = malta->intrctrl->cpu->system->threadContexts.size();
    assert(numcpus <= Malta::Max_CPUs);

    if (ipreq) {
        for (int cpunum=0; cpunum < numcpus; cpunum++) {
            // Check each cpu bit
            uint64_t cpumask = ULL(1) << cpunum;
            if (ipreq & cpumask) {
                // Check if there is already an ipi (bits 8:11)
                if (!(ipint & cpumask)) {
                    ipint  |= cpumask;
                    malta->intrctrl->post(cpunum, TheISA::INTLEVEL_IRQ3, 0);
                    DPRINTF(IPI, "send IPI cpu=%d\n", cpunum);
                }
                else
                    warn("post IPI for CPU=%d, but IPI already\n", cpunum);
            }
        }
    }
    else
        panic("Big IPI Request, but not processors indicated\n");
   */

}


void
MaltaCChip::postRTC()
{
                panic("MaltaCCHIP::postRTC() not implemented.");

                /*
    int size = malta->intrctrl->cpu->system->threadContexts.size();
    assert(size <= Malta::Max_CPUs);

    for (int i = 0; i < size; i++) {
        uint64_t cpumask = ULL(1) << i;
       if (!(cpumask & itint)) {
           itint |= cpumask;
           malta->intrctrl->post(i, TheISA::INTLEVEL_IRQ2, 0);
           DPRINTF(Malta, "Posting RTC interrupt to cpu=%d", i);
       }
    }
    */

}

void
MaltaCChip::postIntr(uint32_t interrupt)
{
    uint64_t size = sys->threadContexts.size();
    assert(size <= Malta::Max_CPUs);

    for (int i=0; i < size; i++) {
                                        //Note: Malta does not use index, but this was added to use the pre-existing implementation
              malta->intrctrl->post(i, interrupt, 0);
              DPRINTF(Malta, "posting  interrupt to cpu %d,"
                        "interrupt %d\n",i, interrupt);
   }

}

void
MaltaCChip::clearIntr(uint32_t interrupt)
{
    uint64_t size = sys->threadContexts.size();
    assert(size <= Malta::Max_CPUs);

    for (int i=0; i < size; i++) {
                                        //Note: Malta does not use index, but this was added to use the pre-existing implementation
              malta->intrctrl->clear(i, interrupt, 0);
              DPRINTF(Malta, "clearing interrupt to cpu %d,"
                        "interrupt %d\n",i, interrupt);
   }
}


void
MaltaCChip::serialize(CheckpointOut &cp) const
{
   // SERIALIZE_ARRAY(dim, Malta::Max_CPUs);
    //SERIALIZE_ARRAY(dir, Malta::Max_CPUs);
    //SERIALIZE_SCALAR(ipint);
    //SERIALIZE_SCALAR(itint);
    //SERIALIZE_SCALAR(drir);
}

void
MaltaCChip::unserialize(CheckpointIn &cp)
{
    //UNSERIALIZE_ARRAY(dim, Malta::Max_CPUs);
    //UNSERIALIZE_ARRAY(dir, Malta::Max_CPUs);
    //UNSERIALIZE_SCALAR(ipint);
    //UNSERIALIZE_SCALAR(itint);
    //UNSERIALIZE_SCALAR(drir);
}

MaltaCChip *
MaltaCChipParams::create()
{
    return new MaltaCChip(this);
}

