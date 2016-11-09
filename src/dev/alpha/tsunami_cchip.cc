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
 *          Ron Dreslinski
 */

/** @file
 * Emulation of the Tsunami CChip CSRs
 */

#include "dev/alpha/tsunami_cchip.hh"

#include <deque>
#include <string>
#include <vector>

#include "arch/alpha/ev5.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/intr_control.hh"
#include "cpu/thread_context.hh"
#include "debug/IPI.hh"
#include "debug/Tsunami.hh"
#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunamireg.h"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/TsunamiCChip.hh"
#include "sim/system.hh"

//Should this be AlphaISA?
using namespace TheISA;

TsunamiCChip::TsunamiCChip(const Params *p)
    : BasicPioDevice(p, 0x10000000), tsunami(p->tsunami)
{
    drir = 0;
    ipint = 0;
    itint = 0;

    for (int x = 0; x < Tsunami::Max_CPUs; x++)
    {
        dim[x] = 0;
        dir[x] = 0;
    }

    //Put back pointer in tsunami
    tsunami->cchip = this;
}

Tick
TsunamiCChip::read(PacketPtr pkt)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n", pkt->getAddr(), pkt->getSize());

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr regnum = (pkt->getAddr() - pioAddr) >> 6;
    Addr daddr = (pkt->getAddr() - pioAddr);

    switch (pkt->getSize()) {

      case sizeof(uint64_t):
          pkt->set<uint64_t>(0);

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
                  pkt->set(((ipint << 8) & 0xF) | ((itint << 4) & 0xF) |
                                     (pkt->req->contextId() & 0x3));
                  // currently, FS cannot handle MT so contextId and
                  // cpuId are effectively the same, don't know if it will
                  // matter if FS becomes MT enabled.  I suspect no because
                  // we are currently able to boot up to 64 procs anyway
                  // which would render the CPUID of this register useless
                  // anyway
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
        panic("invalid access size(?) for tsunami register!\n");
    }
    DPRINTF(Tsunami, "Tsunami CChip: read  regnum=%#x size=%d data=%lld\n",
            regnum, pkt->getSize(), pkt->get<uint64_t>());

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
TsunamiCChip::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;
    Addr regnum = (pkt->getAddr() - pioAddr) >> 6 ;


    assert(pkt->getSize() == sizeof(uint64_t));

    DPRINTF(Tsunami, "write - addr=%#x value=%#x\n", pkt->getAddr(), pkt->get<uint64_t>());

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
        for (int x = 0; x < Tsunami::Max_CPUs; x++)
        {
            bitvector = ULL(1) << x;
            // Figure out which bits have changed
            if ((dim[number] & bitvector) != (olddim & bitvector))
            {
                // The bit is now set and it wasn't before (set)
                if ((dim[number] & bitvector) && (dir[number] & bitvector))
                {
                    tsunami->intrctrl->post(number, TheISA::INTLEVEL_IRQ1, x);
                    DPRINTF(Tsunami, "dim write resulting in posting dir"
                            " interrupt to cpu %d\n", number);
                }
                else if ((olddir & bitvector) &&
                        !(dir[number] & bitvector))
                {
                    // The bit was set and now its now clear and
                    // we were interrupting on that bit before
                    tsunami->intrctrl->clear(number, TheISA::INTLEVEL_IRQ1, x);
                    DPRINTF(Tsunami, "dim write resulting in clear"
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
                          tsunami->intrctrl->post(number, TheISA::INTLEVEL_IRQ1, x);
                          DPRINTF(Tsunami, "posting dir interrupt to cpu 0\n");
                        }
                        else if ((olddir & bitvector) &&
                                !(dir[number] & bitvector))
                        {
                            // The bit was set and now its now clear and
                            // we were interrupting on that bit before
                            tsunami->intrctrl->clear(number, TheISA::INTLEVEL_IRQ1, x);
                          DPRINTF(Tsunami, "dim write resulting in clear"
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
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
TsunamiCChip::clearIPI(uint64_t ipintr)
{
    int numcpus = sys->threadContexts.size();
    assert(numcpus <= Tsunami::Max_CPUs);

    if (ipintr) {
        for (int cpunum=0; cpunum < numcpus; cpunum++) {
            // Check each cpu bit
            uint64_t cpumask = ULL(1) << cpunum;
            if (ipintr & cpumask) {
                // Check if there is a pending ipi
                if (ipint & cpumask) {
                    ipint &= ~cpumask;
                    tsunami->intrctrl->clear(cpunum, TheISA::INTLEVEL_IRQ3, 0);
                    DPRINTF(IPI, "clear IPI IPI cpu=%d\n", cpunum);
                }
                else
                    warn("clear IPI for CPU=%d, but NO IPI\n", cpunum);
            }
        }
    }
    else
        panic("Big IPI Clear, but not processors indicated\n");
}

void
TsunamiCChip::clearITI(uint64_t itintr)
{
    int numcpus = sys->threadContexts.size();
    assert(numcpus <= Tsunami::Max_CPUs);

    if (itintr) {
        for (int i=0; i < numcpus; i++) {
            uint64_t cpumask = ULL(1) << i;
            if (itintr & cpumask & itint) {
                tsunami->intrctrl->clear(i, TheISA::INTLEVEL_IRQ2, 0);
                itint &= ~cpumask;
                DPRINTF(Tsunami, "clearing rtc interrupt to cpu=%d\n", i);
            }
        }
    }
    else
        panic("Big ITI Clear, but not processors indicated\n");
}

void
TsunamiCChip::reqIPI(uint64_t ipreq)
{
    int numcpus = sys->threadContexts.size();
    assert(numcpus <= Tsunami::Max_CPUs);

    if (ipreq) {
        for (int cpunum=0; cpunum < numcpus; cpunum++) {
            // Check each cpu bit
            uint64_t cpumask = ULL(1) << cpunum;
            if (ipreq & cpumask) {
                // Check if there is already an ipi (bits 8:11)
                if (!(ipint & cpumask)) {
                    ipint  |= cpumask;
                    tsunami->intrctrl->post(cpunum, TheISA::INTLEVEL_IRQ3, 0);
                    DPRINTF(IPI, "send IPI cpu=%d\n", cpunum);
                }
                else
                    warn("post IPI for CPU=%d, but IPI already\n", cpunum);
            }
        }
    }
    else
        panic("Big IPI Request, but not processors indicated\n");
}


void
TsunamiCChip::postRTC()
{
    int size = sys->threadContexts.size();
    assert(size <= Tsunami::Max_CPUs);

    for (int i = 0; i < size; i++) {
        uint64_t cpumask = ULL(1) << i;
       if (!(cpumask & itint)) {
           itint |= cpumask;
           tsunami->intrctrl->post(i, TheISA::INTLEVEL_IRQ2, 0);
           DPRINTF(Tsunami, "Posting RTC interrupt to cpu=%d\n", i);
       }
    }

}

void
TsunamiCChip::postDRIR(uint32_t interrupt)
{
    uint64_t bitvector = ULL(1) << interrupt;
    uint64_t size = sys->threadContexts.size();
    assert(size <= Tsunami::Max_CPUs);
    drir |= bitvector;

    for (int i=0; i < size; i++) {
        dir[i] = dim[i] & drir;
       if (dim[i] & bitvector) {
              tsunami->intrctrl->post(i, TheISA::INTLEVEL_IRQ1, interrupt);
              DPRINTF(Tsunami, "posting dir interrupt to cpu %d,"
                        "interrupt %d\n",i, interrupt);
       }
    }
}

void
TsunamiCChip::clearDRIR(uint32_t interrupt)
{
    uint64_t bitvector = ULL(1) << interrupt;
    uint64_t size = sys->threadContexts.size();
    assert(size <= Tsunami::Max_CPUs);

    if (drir & bitvector)
    {
        drir &= ~bitvector;
        for (int i=0; i < size; i++) {
           if (dir[i] & bitvector) {
               tsunami->intrctrl->clear(i, TheISA::INTLEVEL_IRQ1, interrupt);
               DPRINTF(Tsunami, "clearing dir interrupt to cpu %d,"
                    "interrupt %d\n",i, interrupt);

           }
           dir[i] = dim[i] & drir;
        }
    }
    else
        DPRINTF(Tsunami, "Spurrious clear? interrupt %d\n", interrupt);
}


void
TsunamiCChip::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    SERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    SERIALIZE_SCALAR(ipint);
    SERIALIZE_SCALAR(itint);
    SERIALIZE_SCALAR(drir);
}

void
TsunamiCChip::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    UNSERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    UNSERIALIZE_SCALAR(ipint);
    UNSERIALIZE_SCALAR(itint);
    UNSERIALIZE_SCALAR(drir);
}

TsunamiCChip *
TsunamiCChipParams::create()
{
    return new TsunamiCChip(this);
}
