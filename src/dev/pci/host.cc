/*
 * Copyright (c) 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "dev/pci/host.hh"

#include "base/addr_range.hh"
#include "debug/PciHost.hh"
#include "dev/pci/device.hh"
#include "dev/pci/types.hh"
#include "dev/pci/upstream.hh"
#include "dev/platform.hh"
#include "params/GenericPciHost.hh"
#include "params/PciHost.hh"

namespace gem5
{

PciHost::PciHost(const PciHostParams &p)
    : ClockedObject(p),
      PciUpstream(p.up_to_down, p.config_error, p.devices, p.name)
{}

PciHost::~PciHost()
{
}

void
PciHost::init()
{
    ClockedObject::init();
    PciUpstream::init();
}

GenericPciHost::GenericPciHost(const GenericPciHostParams &p)
    : PciHost(p),
      platform(*p.platform),
      confBase(p.conf_base), confSize(p.conf_size),
      confDeviceBits(p.conf_device_bits),
      pciPioBase(p.pci_pio_base), pciMemBase(p.pci_mem_base),
      pciDmaBase(p.pci_dma_base)
{
}

GenericPciHost::~GenericPciHost()
{
}

Addr
GenericPciHost::devConfigAddr(PciBusNum bus_num,
                              const PciDevAddr &dev_addr) const
{
    Addr bus_addr = (bus_num << BUS_OFFSET) + (dev_addr.dev << DEVICE_OFFSET) +
                    (dev_addr.func << FUNCTION_OFFSET);

    return confBase + (bus_addr << confDeviceBits);
}

AddrRange
GenericPciHost::getConfigAddrRange() const
{
    return RangeSize(confBase, confSize);
}

AddrRange
GenericPciHost::interfaceConfigRange(PciBusNum bus_num,
                                     const PciDevice &device) const
{
    Addr start = devConfigAddr(bus_num, device.devAddr());

    return RangeSize(start, 1 << confDeviceBits);
}

AddrRange
GenericPciHost::interfaceBusConfigRange(PciBusNum start_bus,
                                        PciBusNum end_bus) const
{
    if (end_bus < start_bus) {
        return AddrRange();
    }

    Addr start = devConfigAddr(start_bus, PciDevAddr(0, 0));
    Addr size = static_cast<Addr>(end_bus - start_bus + 1)
                << (BUS_OFFSET + confDeviceBits);

    return RangeSize(start, size);
}

void
GenericPciHost::interfacePostInt(PciBusNum bus_num, const PciDevice &device)
{
    platform.postPciInt(mapPciInterrupt(device));
}

void
GenericPciHost::interfaceClearInt(PciBusNum bus_num, const PciDevice &device)
{
    platform.clearPciInt(mapPciInterrupt(device));
}

uint32_t
GenericPciHost::mapPciInterrupt(const PciDevice &device) const
{
    return device.interruptLine();
}

} // namespace gem5
