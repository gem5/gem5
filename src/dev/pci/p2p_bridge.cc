/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
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

#include "dev/pci/p2p_bridge.hh"

#include "base/addr_range.hh"
#include "base/types.hh"
#include "dev/pci/types.hh"
#include "dev/pci/upstream.hh"
#include "params/PciToPciBridge.hh"

namespace gem5
{

PciToPciBridge::PciToPciBridge(const Params &p)
    : PciType1Device(p),
      PciUpstream(p.up_to_down, p.config_error, p.devices, name())
{}

PciToPciBridge::~PciToPciBridge()
{}

void
PciToPciBridge::init()
{
    PciType1Device::init();
    PciUpstream::init();
}

PciBusNum
PciToPciBridge::getBusNum() const
{
    return config().secondaryBusNum;
}

Tick
PciToPciBridge::writeConfig(PacketPtr pkt)
{
    Tick tick = PciType1Device::writeConfig(pkt);

    auto offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    auto end_offset = offset + pkt->getSize();

    // All downstream devices need to update their address ranges when
    // secondary bus number has changed.
    if ((offset <= PCI1_SEC_BUS_NUM && PCI1_SEC_BUS_NUM < end_offset) ||
        (offset <= PCI1_SUB_BUS_NUM && PCI1_SUB_BUS_NUM < end_offset)) {
        sendBusChange();
    }

    return tick;
}

bool
PciToPciBridge::isDownstreamBus(PciBusNum bus_num) const
{
    // If this bridge number is zero, this means that the bridge isn't
    // configured yet, and so there is no subordinate bus.
    if (getBusNum() == 0) {
        return false;
    }

    return (getBusNum() <= bus_num) && (bus_num <= config().subordinateBusNum);
}

AddrRange
PciToPciBridge::getConfigAddrRange() const
{
    // No valid bus assigned, return invalid range
    const auto bus_num = getBusNum();
    const auto sub_num = config().subordinateBusNum;
    if ((bus_num == 0) || sub_num < bus_num) {
        return AddrRange();
    }

    return upstreamInterface->busConfigRange(bus_num, sub_num);
}

AddrRange
PciToPciBridge::interfaceConfigRange(PciBusNum bus_num,
                                     const PciDevice &device) const
{
    if (!isDownstreamBus(bus_num)) {
        return AddrRange();
    }

    return upstreamInterface->configRange(bus_num, device);
}

Addr
PciToPciBridge::interfacePioAddr(PciBusNum bus_num, const PciDevice &device,
                                 Addr pci_addr) const
{
    if (!isDownstreamBus(bus_num)) {
        return 0;
    }

    return upstreamInterface->pioAddr(bus_num, device, pci_addr);
}

Addr
PciToPciBridge::interfaceMemAddr(PciBusNum bus_num, const PciDevice &device,
                                 Addr pci_addr) const
{
    if (!isDownstreamBus(bus_num)) {
        return 0;
    }

    return upstreamInterface->memAddr(bus_num, device, pci_addr);
}

Addr
PciToPciBridge::interfaceDmaAddr(PciBusNum bus_num, const PciDevice &device,
                                 Addr pci_addr) const
{
    if (!isDownstreamBus(bus_num)) {
        return 0;
    }

    return upstreamInterface->dmaAddr(bus_num, device, pci_addr);
}

void
PciToPciBridge::interfacePostInt(PciBusNum bus_num, const PciDevice &device)
{
    if (!isDownstreamBus(bus_num)) {
        warn("Posting interrupt on unmapped bus\n");
        return;
    }

    upstreamInterface->postInt(bus_num, device);
}

void
PciToPciBridge::interfaceClearInt(PciBusNum bus_num, const PciDevice &device)
{
    if (!isDownstreamBus(bus_num)) {
        warn("Clearing interrupt on unmapped bus\n");
        return;
    }

    upstreamInterface->clearInt(bus_num, device);
}

AddrRange
PciToPciBridge::interfaceBusConfigRange(PciBusNum start_bus,
                                        PciBusNum end_bus) const
{
    if (!isDownstreamBus(start_bus) || !isDownstreamBus(end_bus)) {
        return AddrRange();
    }

    return upstreamInterface->busConfigRange(start_bus, end_bus);
}

} // namespace gem5
