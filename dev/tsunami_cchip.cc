/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 */

/** @file
 * Emulation of the Tsunami CChip CSRs
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/tsunami_cchip.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "cpu/intr_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiCChip::TsunamiCChip(const string &name, Tsunami *t, Addr a,
                           MemoryController *mmu, HierParams *hier, Bus* bus,
                           Tick pio_latency)
    : PioDevice(name, t), addr(a), tsunami(t)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &TsunamiCChip::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * bus->clockRate;
    }

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

Fault
TsunamiCChip::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n", req->vaddr, req->size);

    Addr regnum = (req->paddr - (addr & EV5::PAddrImplMask)) >> 6;
    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask));

    ExecContext *xc = req->xc;

    switch (req->size) {

      case sizeof(uint64_t):
          if (daddr & TSDEV_CC_BDIMS)
          {
              *(uint64_t*)data = dim[(daddr >> 4) & 0x3F];
              return No_Fault;
          }

          if (daddr & TSDEV_CC_BDIRS)
          {
              *(uint64_t*)data = dir[(daddr >> 4) & 0x3F];
              return No_Fault;
          }

          switch(regnum) {
              case TSDEV_CC_CSR:
                  *(uint64_t*)data = 0x0;
                  return No_Fault;
              case TSDEV_CC_MTR:
                  panic("TSDEV_CC_MTR not implemeted\n");
                   return No_Fault;
              case TSDEV_CC_MISC:
                  *(uint64_t*)data = (ipint << 8) & 0xF |
                                     (itint << 4) & 0xF |
                                     (xc->cpu_id & 0x3);
                  return No_Fault;
              case TSDEV_CC_AAR0:
              case TSDEV_CC_AAR1:
              case TSDEV_CC_AAR2:
              case TSDEV_CC_AAR3:
                  *(uint64_t*)data = 0;
                  return No_Fault;
              case TSDEV_CC_DIM0:
                  *(uint64_t*)data = dim[0];
                  return No_Fault;
              case TSDEV_CC_DIM1:
                  *(uint64_t*)data = dim[1];
                  return No_Fault;
              case TSDEV_CC_DIM2:
                  *(uint64_t*)data = dim[2];
                  return No_Fault;
              case TSDEV_CC_DIM3:
                  *(uint64_t*)data = dim[3];
                  return No_Fault;
              case TSDEV_CC_DIR0:
                  *(uint64_t*)data = dir[0];
                  return No_Fault;
              case TSDEV_CC_DIR1:
                  *(uint64_t*)data = dir[1];
                  return No_Fault;
              case TSDEV_CC_DIR2:
                  *(uint64_t*)data = dir[2];
                  return No_Fault;
              case TSDEV_CC_DIR3:
                  *(uint64_t*)data = dir[3];
                  return No_Fault;
              case TSDEV_CC_DRIR:
                  *(uint64_t*)data = drir;
                  return No_Fault;
              case TSDEV_CC_PRBEN:
                  panic("TSDEV_CC_PRBEN not implemented\n");
                  return No_Fault;
              case TSDEV_CC_IIC0:
              case TSDEV_CC_IIC1:
              case TSDEV_CC_IIC2:
              case TSDEV_CC_IIC3:
                  panic("TSDEV_CC_IICx not implemented\n");
                  return No_Fault;
              case TSDEV_CC_MPR0:
              case TSDEV_CC_MPR1:
              case TSDEV_CC_MPR2:
              case TSDEV_CC_MPR3:
                  panic("TSDEV_CC_MPRx not implemented\n");
                  return No_Fault;
              case TSDEV_CC_IPIR:
                  *(uint64_t*)data = ipint;
                  return No_Fault;
              case TSDEV_CC_ITIR:
                  *(uint64_t*)data = itint;
                  return No_Fault;
              default:
                  panic("default in cchip read reached, accessing 0x%x\n");
           } // uint64_t

      break;
      case sizeof(uint32_t):
          if (regnum == TSDEV_CC_DRIR) {
              warn("accessing DRIR with 32 bit read, "
                   "hopefully your just reading this for timing");
              *(uint32_t*)data = drir;
          } else
              panic("invalid access size(?) for tsunami register!\n");
          return No_Fault;
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n");
    }
    DPRINTFN("Tsunami CChip ERROR: read  regnum=%#x size=%d\n", regnum, req->size);

    return No_Fault;
}

Fault
TsunamiCChip::write(MemReqPtr &req, const uint8_t *data)
{
    DPRINTF(Tsunami, "write - va=%#x value=%#x size=%d \n",
            req->vaddr, *(uint64_t*)data, req->size);

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask));
    Addr regnum = (req->paddr - (addr & EV5::PAddrImplMask)) >> 6;

    bool supportedWrite = false;

    switch (req->size) {

      case sizeof(uint64_t):
          if (daddr & TSDEV_CC_BDIMS)
          {
              int number = (daddr >> 4) & 0x3F;

              uint64_t bitvector;
              uint64_t olddim;
              uint64_t olddir;

              olddim = dim[number];
              olddir = dir[number];
              dim[number] = *(uint64_t*)data;
              dir[number] = dim[number] & drir;
              for(int x = 0; x < Tsunami::Max_CPUs; x++)
              {
                  bitvector = ULL(1) << x;
                  // Figure out which bits have changed
                  if ((dim[number] & bitvector) != (olddim & bitvector))
                  {
                      // The bit is now set and it wasn't before (set)
                      if((dim[number] & bitvector) && (dir[number] & bitvector))
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
              return No_Fault;
          }

          switch(regnum) {
              case TSDEV_CC_CSR:
                  panic("TSDEV_CC_CSR write\n");
                  return No_Fault;
              case TSDEV_CC_MTR:
                  panic("TSDEV_CC_MTR write not implemented\n");
                   return No_Fault;
              case TSDEV_CC_MISC:
                uint64_t ipreq;
                ipreq = (*(uint64_t*)data >> 12) & 0xF;
                //If it is bit 12-15, this is an IPI post
                if (ipreq) {
                    reqIPI(ipreq);
                    supportedWrite = true;
                }

                //If it is bit 8-11, this is an IPI clear
                uint64_t ipintr;
                ipintr = (*(uint64_t*)data >> 8) & 0xF;
                if (ipintr) {
                    clearIPI(ipintr);
                    supportedWrite = true;
                }

                //If it is the 4-7th bit, clear the RTC interrupt
                uint64_t itintr;
                itintr = (*(uint64_t*)data >> 4) & 0xF;
                if (itintr) {
                    clearITI(itintr);
                    supportedWrite = true;
                }

                // ignore NXMs
                if (*(uint64_t*)data & 0x10000000)
                    supportedWrite = true;

                if(!supportedWrite)
                    panic("TSDEV_CC_MISC write not implemented\n");

                return No_Fault;
              case TSDEV_CC_AAR0:
              case TSDEV_CC_AAR1:
              case TSDEV_CC_AAR2:
              case TSDEV_CC_AAR3:
                  panic("TSDEV_CC_AARx write not implemeted\n");
                  return No_Fault;
              case TSDEV_CC_DIM0:
              case TSDEV_CC_DIM1:
              case TSDEV_CC_DIM2:
              case TSDEV_CC_DIM3:
                  int number;
                  if(regnum == TSDEV_CC_DIM0)
                      number = 0;
                  else if(regnum == TSDEV_CC_DIM1)
                      number = 1;
                  else if(regnum == TSDEV_CC_DIM2)
                      number = 2;
                  else
                      number = 3;

                  uint64_t bitvector;
                  uint64_t olddim;
                  uint64_t olddir;

                  olddim = dim[number];
                  olddir = dir[number];
                  dim[number] = *(uint64_t*)data;
                  dir[number] = dim[number] & drir;
                  for(int x = 0; x < 64; x++)
                  {
                      bitvector = ULL(1) << x;
                      // Figure out which bits have changed
                      if ((dim[number] & bitvector) != (olddim & bitvector))
                      {
                          // The bit is now set and it wasn't before (set)
                          if((dim[number] & bitvector) && (dir[number] & bitvector))
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
                  return No_Fault;
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
                  clearIPI(*(uint64_t*)data);
                  return No_Fault;
              case TSDEV_CC_ITIR:
                  clearITI(*(uint64_t*)data);
                  return No_Fault;
              case TSDEV_CC_IPIQ:
                  reqIPI(*(uint64_t*)data);
                  return No_Fault;
              default:
                  panic("default in cchip read reached, accessing 0x%x\n");
          }

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n");
    }

    DPRINTFN("Tsunami ERROR: write daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

void
TsunamiCChip::clearIPI(uint64_t ipintr)
{
    int numcpus = tsunami->intrctrl->cpu->system->execContexts.size();
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
    int numcpus = tsunami->intrctrl->cpu->system->execContexts.size();
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
    int numcpus = tsunami->intrctrl->cpu->system->execContexts.size();
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
    int size = tsunami->intrctrl->cpu->system->execContexts.size();
    assert(size <= Tsunami::Max_CPUs);

    for (int i = 0; i < size; i++) {
        uint64_t cpumask = ULL(1) << i;
        if (!(cpumask & itint)) {
            itint |= cpumask;
            tsunami->intrctrl->post(i, TheISA::INTLEVEL_IRQ2, 0);
            DPRINTF(Tsunami, "Posting RTC interrupt to cpu=%d", i);
        }
    }

}

void
TsunamiCChip::postDRIR(uint32_t interrupt)
{
    uint64_t bitvector = ULL(1) << interrupt;
    uint64_t size = tsunami->intrctrl->cpu->system->execContexts.size();
    assert(size <= Tsunami::Max_CPUs);
    drir |= bitvector;

    for(int i=0; i < size; i++) {
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
    uint64_t size = tsunami->intrctrl->cpu->system->execContexts.size();
    assert(size <= Tsunami::Max_CPUs);

    if (drir & bitvector)
    {
        drir &= ~bitvector;
        for(int i=0; i < size; i++) {
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

Tick
TsunamiCChip::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}


void
TsunamiCChip::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    SERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    SERIALIZE_SCALAR(ipint);
    SERIALIZE_SCALAR(itint);
    SERIALIZE_SCALAR(drir);
}

void
TsunamiCChip::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    UNSERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    UNSERIALIZE_SCALAR(ipint);
    UNSERIALIZE_SCALAR(itint);
    UNSERIALIZE_SCALAR(drir);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiCChip)

    SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<Bus*> io_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiCChip)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiCChip)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(TsunamiCChip)

CREATE_SIM_OBJECT(TsunamiCChip)
{
    return new TsunamiCChip(getInstanceName(), tsunami, addr, mmu, hier,
                            io_bus, pio_latency);
}

REGISTER_SIM_OBJECT("TsunamiCChip", TsunamiCChip)
