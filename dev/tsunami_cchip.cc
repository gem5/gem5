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

/* @file
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
#include "mem/functional_mem/memory_control.hh"
#include "cpu/intr_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiCChip::TsunamiCChip(const string &name, Tsunami *t, Addr a,
                           MemoryController *mmu, HierParams *hier, Bus* bus)
    : PioDevice(name), addr(a), tsunami(t)
{
    mmu->add_child(this, Range<Addr>(addr, addr + size));

    for(int i=0; i < Tsunami::Max_CPUs; i++) {
        dim[i] = 0;
        dir[i] = 0;
        dirInterrupting[i] = false;
        ipiInterrupting[i] = false;
        RTCInterrupting[i] = false;
    }

    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &TsunamiCChip::cacheAccess);
        pioInterface->addAddrRange(addr, addr + size - 1);
    }

    drir = 0;
    misc = 0;

    //Put back pointer in tsunami
    tsunami->cchip = this;
}

Fault
TsunamiCChip::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr - (addr & PA_IMPL_MASK)) >> 6;
    ExecContext *xc = req->xc;

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
              case TSDEV_CC_CSR:
                  *(uint64_t*)data = 0x0;
                  return No_Fault;
              case TSDEV_CC_MTR:
                  panic("TSDEV_CC_MTR not implemeted\n");
                   return No_Fault;
              case TSDEV_CC_MISC:
                *(uint64_t*)data = misc | (xc->cpu_id & 0x3);
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
    DPRINTFN("Tsunami CChip ERROR: read  daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

Fault
TsunamiCChip::write(MemReqPtr &req, const uint8_t *data)
{
    DPRINTF(Tsunami, "write - va=%#x value=%#x size=%d \n",
            req->vaddr, *(uint64_t*)data, req->size);

    Addr daddr = (req->paddr - (addr & PA_IMPL_MASK)) >> 6;

    bool supportedWrite = false;
    uint64_t size = tsunami->intrctrl->cpu->system->execContexts.size();

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
            case TSDEV_CC_CSR:
                  panic("TSDEV_CC_CSR write\n");
                  return No_Fault;
              case TSDEV_CC_MTR:
                  panic("TSDEV_CC_MTR write not implemented\n");
                   return No_Fault;
              case TSDEV_CC_MISC:
                //If it is the 4-7th bit, clear the RTC interrupt
                uint64_t itintr;
                if ((itintr = (*(uint64_t*) data) & (0xf<<4))) {
                    //Clear the bits in ITINTR
                    misc &= ~(itintr);
                    for (int i=0; i < size; i++) {
                        if ((itintr & (1 << (i+4))) && RTCInterrupting[i]) {
                            tsunami->intrctrl->clear(i, TheISA::INTLEVEL_IRQ2, 0);
                            RTCInterrupting[i] = false;
                            DPRINTF(Tsunami, "clearing rtc interrupt to cpu=%d\n", i);
                        }
                    }
                    supportedWrite = true;
                }
                //If it is 12th-15th bit, IPI sent to Processor 1
                uint64_t ipreq;
                if ((ipreq = (*(uint64_t*) data) & (0xf << 12))) {
                    //Set the bits in IPINTR
                    misc |= (ipreq >> 4);
                    for (int i=0; i < size; i++) {
                        if ((ipreq & (1 << (i + 12)))) {
                            if (!ipiInterrupting[i])
                                tsunami->intrctrl->post(i, TheISA::INTLEVEL_IRQ3, 0);
                            ipiInterrupting[i]++;
                            DPRINTF(IPI, "send cpu=%d pending=%d from=%d\n", i,
                                    ipiInterrupting[i], req->cpu_num);
                        }
                    }
                    supportedWrite = true;
                }
                //If it is bits 8-11, then clearing IPI's
                uint64_t ipintr;
                if ((ipintr = (*(uint64_t*) data) & (0xf << 8))) {
                    //Clear the bits in IPINTR
                    misc &= ~(ipintr);
                    for (int i=0; i < size; i++) {
                        if ((ipintr & (1 << (i + 8))) && ipiInterrupting[i]) {
                            if (!(--ipiInterrupting[i]))
                                tsunami->intrctrl->clear(i, TheISA::INTLEVEL_IRQ3, 0);
                            DPRINTF(IPI, "clearing cpu=%d pending=%d from=%d\n", i,
                                    ipiInterrupting[i] + 1, req->cpu_num);
                        }
                    }
                    supportedWrite = true;
                }

        // ignore NXMs
        if (*(uint64_t*)data & 0x10000000)
            supportedWrite = true;

                if(!supportedWrite) panic("TSDEV_CC_MISC write not implemented\n");
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
                  if(daddr == TSDEV_CC_DIM0)
                      number = 0;
                  else if(daddr == TSDEV_CC_DIM1)
                      number = 1;
                  else if(daddr == TSDEV_CC_DIM2)
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
                      bitvector = (uint64_t)1 << x;
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
                                      "dir interrupt to cpu 0\n");

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
TsunamiCChip::postRTC()
{
    int size = tsunami->intrctrl->cpu->system->execContexts.size();

    for (int i = 0; i < size; i++) {
        if (!RTCInterrupting[i]) {
            misc |= 16 << i;
            RTCInterrupting[i] = true;
            tsunami->intrctrl->post(i, TheISA::INTLEVEL_IRQ2, 0);
            DPRINTF(Tsunami, "Posting RTC interrupt to cpu=%d", i);
        }
    }

}

void
TsunamiCChip::postDRIR(uint32_t interrupt)
{
    uint64_t bitvector = (uint64_t)0x1 << interrupt;
    drir |= bitvector;
    uint64_t size = tsunami->intrctrl->cpu->system->execContexts.size();
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
    uint64_t bitvector = (uint64_t)0x1 << interrupt;
    uint64_t size = tsunami->intrctrl->cpu->system->execContexts.size();
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
    return curTick + 1000;
}


void
TsunamiCChip::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    SERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    SERIALIZE_ARRAY(dirInterrupting, Tsunami::Max_CPUs);
    SERIALIZE_ARRAY(ipiInterrupting, Tsunami::Max_CPUs);
    SERIALIZE_SCALAR(drir);
    SERIALIZE_SCALAR(misc);
    SERIALIZE_ARRAY(RTCInterrupting, Tsunami::Max_CPUs);
}

void
TsunamiCChip::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(dim, Tsunami::Max_CPUs);
    UNSERIALIZE_ARRAY(dir, Tsunami::Max_CPUs);
    UNSERIALIZE_ARRAY(dirInterrupting, Tsunami::Max_CPUs);
    UNSERIALIZE_ARRAY(ipiInterrupting, Tsunami::Max_CPUs);
    UNSERIALIZE_SCALAR(drir);
    UNSERIALIZE_SCALAR(misc);
    UNSERIALIZE_ARRAY(RTCInterrupting, Tsunami::Max_CPUs);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiCChip)

    SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<Bus*> io_bus;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiCChip)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiCChip)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(TsunamiCChip)

CREATE_SIM_OBJECT(TsunamiCChip)
{
    return new TsunamiCChip(getInstanceName(), tsunami, addr, mmu, hier, io_bus);
}

REGISTER_SIM_OBJECT("TsunamiCChip", TsunamiCChip)
