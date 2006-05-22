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
 */

/* @file
 * PCI Configspace implementation
 */

#include <deque>
#include <string>
#include <vector>
#include <bitset>

#include "base/trace.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcidev.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

PciConfigAll::PciConfigAll(Params *p)
    : BasicPioDevice(p)
{
    pioSize = 0xffffff;

    // Set backpointer for pci config. Really the config stuff should be able to
    // automagically do this
    p->platform->pciconfig = this;

    // Make all the pointers to devices null
    for(int x=0; x < MAX_PCI_DEV; x++)
        for(int y=0; y < MAX_PCI_FUNC; y++)
            devices[x][y] = NULL;
}

// If two interrupts share the same line largely bad things will happen.
// Since we don't track how many times an interrupt was set and correspondingly
// cleared two devices on the same interrupt line and assert and deassert each
// others interrupt "line". Interrupts will not work correctly.
void
PciConfigAll::startup()
{
    bitset<256> intLines;
    PciDev *tempDev;
    uint8_t intline;

    for (int x = 0; x < MAX_PCI_DEV; x++) {
        for (int y = 0; y < MAX_PCI_FUNC; y++) {
           if (devices[x][y] != NULL) {
               tempDev = devices[x][y];
               intline = tempDev->interruptLine();
               if (intLines.test(intline))
                   warn("Interrupt line %#X is used multiple times"
                        "(You probably want to fix this).\n", (uint32_t)intline);
               else
                   intLines.set(intline);
           } // devices != NULL
        } // PCI_FUNC
    } // PCI_DEV

}

Tick
PciConfigAll::read(Packet *pkt)
{
    assert(pkt->result == Unknown);
    assert(pkt->addr >= pioAddr && pkt->addr < pioAddr + pioSize);

    Addr daddr = pkt->addr - pioAddr;
    int device = (daddr >> 11) & 0x1F;
    int func = (daddr >> 8) & 0x7;
    int reg = daddr & 0xFF;

    pkt->time += pioDelay;
    pkt->allocate();

    DPRINTF(PciConfigAll, "read  va=%#x da=%#x size=%d\n", pkt->addr, daddr,
            pkt->size);

    switch (pkt->size) {
      case sizeof(uint32_t):
         if (devices[device][func] == NULL)
             pkt->set<uint32_t>(0xFFFFFFFF);
         else
             devices[device][func]->readConfig(reg, pkt->getPtr<uint32_t>());
         break;
      case sizeof(uint16_t):
         if (devices[device][func] == NULL)
             pkt->set<uint16_t>(0xFFFF);
         else
             devices[device][func]->readConfig(reg, pkt->getPtr<uint16_t>());
         break;
      case sizeof(uint8_t):
         if (devices[device][func] == NULL)
             pkt->set<uint8_t>(0xFF);
         else
             devices[device][func]->readConfig(reg, pkt->getPtr<uint8_t>());
         break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->result = Success;
    return pioDelay;
}

Tick
PciConfigAll::write(Packet *pkt)
{
    pkt->time += pioDelay;

    assert(pkt->result == Unknown);
    assert(pkt->addr >= pioAddr && pkt->addr < pioAddr + pioSize);
    assert(pkt->size == sizeof(uint8_t) || pkt->size == sizeof(uint16_t) ||
            pkt->size == sizeof(uint32_t));
    Addr daddr = pkt->addr - pioAddr;

    int device = (daddr >> 11) & 0x1F;
    int func = (daddr >> 8) & 0x7;
    int reg = daddr & 0xFF;

    if (devices[device][func] == NULL)
        panic("Attempting to write to config space on non-existant device\n");

    DPRINTF(PciConfigAll, "write - va=%#x size=%d data=%#x\n",
            pkt->addr, pkt->size, pkt->get<uint32_t>());

    switch (pkt->size) {
      case sizeof(uint8_t):
        devices[device][func]->writeConfig(reg, pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        devices[device][func]->writeConfig(reg, pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        devices[device][func]->writeConfig(reg, pkt->get<uint32_t>());
        break;
      default:
        panic("invalid pci config write size\n");
    }
    pkt->result = Success;
    return pioDelay;
}

void
PciConfigAll::serialize(std::ostream &os)
{
    /*
     * There is no state associated with this object that requires
     * serialization.  The only real state are the device pointers
     * which are all setup by the constructor of the PciDev class
     */
}

void
PciConfigAll::unserialize(Checkpoint *cp, const std::string &section)
{
    /*
     * There is no state associated with this object that requires
     * serialization.  The only real state are the device pointers
     * which are all setup by the constructor of the PciDev class
     */
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object")

END_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

CREATE_SIM_OBJECT(PciConfigAll)
{
    BasicPioDevice::Params *p = new BasicPioDevice::Params;
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    return new PciConfigAll(p);
}

REGISTER_SIM_OBJECT("PciConfigAll", PciConfigAll)

#endif // DOXYGEN_SHOULD_SKIP_THIS
