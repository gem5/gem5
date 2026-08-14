/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
 * All rights reserved
 *
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

#include "dev/pci/upstream.hh"

#include "debug/PciUpstream.hh"
#include "dev/pci/device.hh"
#include "dev/pci/types.hh"
#include "params/PciConfigError.hh"

namespace gem5
{

PciConfigError::PciConfigError(const PciConfigErrorParams &p) : IsaFake(p)
{}

void
PciConfigError::setAddrRange(AddrRange range)
{
    if (range.valid()) {
        pioAddr = range.start();
        pioSize = range.size();
    } else {
        pioAddr = 0;
        pioSize = 0;
    }

    pioPort.sendRangeChange();
}

AddrRangeList
PciConfigError::getAddrRanges() const
{
    // Avoid triggering assertion error with zero size range, which is the case
    // when a PCI-to-PCI bridge isn't configured yet.
    if (pioSize == 0) {
        return AddrRangeList();
    }

    return IsaFake::getAddrRanges();
}

PciUpstream::PciUpstream(PciUpDownBridge *up_to_down,
                         PciConfigError *config_error_dev,
                         const std::vector<PciDevice *> &pci_devices,
                         std::string upstream_name)
    : upToDown(up_to_down),
      configErrorDevice(config_error_dev),
      upstreamName(upstream_name)
{
    for (PciDevice *device : pci_devices) {
        PciDevAddr dev_addr = device->devAddr();
        DPRINTF(PciUpstream, "%02x.%i: Registering device\n", dev_addr.dev,
                dev_addr.func);

        auto map_entry = devices.emplace(dev_addr, device);
        fatal_if(!map_entry.second, "%s - %02x.%i: PCI bus ID collision\n",
                 upstreamName, dev_addr.dev, dev_addr.func);

        // Manually allocate instead of using make_unique due to
        // DeviceInterface constructor being protected.
        device->setUpstreamInterface(std::unique_ptr<DeviceInterface>(
            new DeviceInterface(*this, *device)));
    }
}

void
PciUpstream::init()
{
    sendBusChange();
}

std::string
PciUpstream::name() const
{
    return upstreamName;
}

PciDevice *
PciUpstream::getDevice(const PciDevAddr &addr)
{
    auto device = devices.find(addr);
    return device != devices.end() ? device->second : nullptr;
}

const PciDevice *
PciUpstream::getDevice(const PciDevAddr &addr) const
{
    auto device = devices.find(addr);
    return device != devices.end() ? device->second : nullptr;
}

PciUpstream::DeviceInterface::DeviceInterface(PciUpstream &upstream,
                                              PciDevice &device)
    : upstream(upstream), device(device)
{}

const std::string
PciUpstream::DeviceInterface::name() const
{
    PciDevAddr dev_addr = device.devAddr();
    return csprintf("%s.interface[%02x:%02x.%i]", upstream.name(),
                    upstream.getBusNum(), dev_addr.dev, dev_addr.func);
}

PciBusNum
PciUpstream::DeviceInterface::getBusNum() const
{
    return upstream.getBusNum();
}

void
PciUpstream::DeviceInterface::postInt()
{
    DPRINTF(PciUpstream, "postInt\n");

    upstream.interfacePostInt(device);
}

void
PciUpstream::DeviceInterface::postInt(const PciDevice &device)
{
    DPRINTF(PciUpstream, "postInt\n");

    upstream.interfacePostInt(device);
}

void
PciUpstream::DeviceInterface::clearInt()
{
    DPRINTF(PciUpstream, "clearInt\n");

    upstream.interfaceClearInt(device);
}

void
PciUpstream::DeviceInterface::clearInt(const PciDevice &device)
{
    DPRINTF(PciUpstream, "clearInt\n");

    upstream.interfaceClearInt(device);
}

AddrRange
PciUpstream::DeviceInterface::configRange() const
{
    return upstream.interfaceConfigRange(device);
}

AddrRange
PciUpstream::DeviceInterface::configRange(const PciDevice &device) const
{
    return upstream.interfaceConfigRange(device);
}

Addr
PciUpstream::DeviceInterface::pioAddr(Addr addr) const
{
    return upstream.interfacePioAddr(device, addr);
}

Addr
PciUpstream::DeviceInterface::pioAddr(const PciDevice &device, Addr addr) const
{
    return upstream.interfacePioAddr(device, addr);
}

Addr
PciUpstream::DeviceInterface::memAddr(Addr addr) const
{
    return upstream.interfaceMemAddr(device, addr);
}

Addr
PciUpstream::DeviceInterface::memAddr(const PciDevice &device, Addr addr) const
{
    return upstream.interfaceMemAddr(device, addr);
}

Addr
PciUpstream::DeviceInterface::dmaAddr(Addr addr) const
{
    return upstream.interfaceDmaAddr(device, addr);
}

Addr
PciUpstream::DeviceInterface::dmaAddr(const PciDevice &device, Addr addr) const
{
    return upstream.interfaceDmaAddr(device, addr);
}

AddrRange
PciUpstream::DeviceInterface::busConfigRange(PciBusNum start_bus,
                                             PciBusNum end_bus) const
{
    return upstream.interfaceBusConfigRange(start_bus, end_bus);
}

void
PciUpstream::sendBusChange()
{
    // Update bridge and error device configuration range
    upToDown->setConfigRange(getConfigAddrRange());
    configErrorDevice->setAddrRange(getConfigAddrRange());

    for (std::pair<PciDevAddr, PciDevice *> device : devices) {
        device.second->recvBusChange();
    }
}

} // namespace gem5
