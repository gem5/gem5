/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 */

/* @file
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 */

#include "base/inet.hh"
#include "dev/i8254xGBe.hh"
#include "mem/packet.hh"
#include "sim/builder.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

IGbE::IGbE(Params *p)
    : PciDev(p), etherInt(NULL)
{

}


Tick
IGbE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDev::writeConfig(pkt);
    else
        panic("Device specific PCI config space not implemented.\n");

    ///
    /// Some work may need to be done here based for the pci COMMAND bits.
    ///

    return pioDelay;
}

Tick
IGbE::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    DPRINTF(Ethernet, "Accessed devie register %#X\n", daddr);

    pkt->allocate();


    ///
    /// Handle read of register here
    ///

    pkt->result = Packet::Success;
    return pioDelay;
}

Tick
IGbE::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    DPRINTF(Ethernet, "Accessed devie register %#X\n", daddr);

    ///
    /// Handle write of register here
    ///

    pkt->result = Packet::Success;
    return pioDelay;
}


bool
IGbE::ethRxPkt(EthPacketPtr packet)
{
    panic("Need to implemenet\n");
}


void
IGbE::ethTxDone()
{
    panic("Need to implemenet\n");
}

void
IGbE::serialize(std::ostream &os)
{
    panic("Need to implemenet\n");
}

void
IGbE::unserialize(Checkpoint *cp, const std::string &section)
{
    panic("Need to implemenet\n");
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(IGbEInt)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<IGbE *> device;

END_DECLARE_SIM_OBJECT_PARAMS(IGbEInt)

BEGIN_INIT_SIM_OBJECT_PARAMS(IGbEInt)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM(device, "Ethernet device of this interface")

END_INIT_SIM_OBJECT_PARAMS(IGbEInt)

CREATE_SIM_OBJECT(IGbEInt)
{
    IGbEInt *dev_int = new IGbEInt(getInstanceName(), device);

    EtherInt *p = (EtherInt *)peer;
    if (p) {
        dev_int->setPeer(p);
        p->setPeer(dev_int);
    }

    return dev_int;
}

REGISTER_SIM_OBJECT("IGbEInt", IGbEInt)


BEGIN_DECLARE_SIM_OBJECT_PARAMS(IGbE)

    SimObjectParam<System *> system;
    SimObjectParam<Platform *> platform;
    SimObjectParam<PciConfigData *> configdata;
    Param<uint32_t> pci_bus;
    Param<uint32_t> pci_dev;
    Param<uint32_t> pci_func;
    Param<Tick> pio_latency;
    Param<Tick> config_latency;

END_DECLARE_SIM_OBJECT_PARAMS(IGbE)

BEGIN_INIT_SIM_OBJECT_PARAMS(IGbE)

    INIT_PARAM(system, "System pointer"),
    INIT_PARAM(platform, "Platform pointer"),
    INIT_PARAM(configdata, "PCI Config data"),
    INIT_PARAM(pci_bus, "PCI bus ID"),
    INIT_PARAM(pci_dev, "PCI device number"),
    INIT_PARAM(pci_func, "PCI function code"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM(config_latency, "Number of cycles for a config read or write")

END_INIT_SIM_OBJECT_PARAMS(IGbE)


CREATE_SIM_OBJECT(IGbE)
{
    IGbE::Params *params = new IGbE::Params;

    params->name = getInstanceName();
    params->platform = platform;
    params->system = system;
    params->configData = configdata;
    params->busNum = pci_bus;
    params->deviceNum = pci_dev;
    params->functionNum = pci_func;
    params->pio_delay = pio_latency;
    params->config_delay = config_latency;

    return new IGbE(params);
}

REGISTER_SIM_OBJECT("IGbE", IGbE)
