/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
 * All rights reserved
 *
 * Copyright (c) 2013, 2015 ARM Limited
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
 * A single PCI device configuration space entry.
 */

#include "dev/pci/device.hh"

#include <initializer_list>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PciDevice.hh"
#include "dev/pci/capabilities.hh"
#include "dev/pci/config.hh"
#include "mem/packet.hh"

namespace gem5
{

using namespace std::placeholders;

PciDevice::PciDevice(const PciDeviceParams &p,
                     std::initializer_list<PciBar *> BARs_init,
                     PciConfigBase *config)
    : DmaDevice(p),
      _config(config),
      _configSpace(),
      _devAddr(p.pci_dev, p.pci_func),
      BARs(BARs_init),
      upstreamInterface(nullptr),
      pioDelay(p.pio_latency),
      configDelay(p.config_latency)
{
    fatal_if(p.InterruptPin >= 5,
             "Invalid PCI interrupt '%i' specified.", p.InterruptPin);

    fatal_if(_config == nullptr, "No configuration registers provided.");

    addConfigRegisterBank(config);

    int idx = 0;
    for (auto *bar: BARs) {
        auto *mu = dynamic_cast<PciMemUpperBar *>(bar);
        // If this is the upper 32 bits of a memory BAR, try to connect it to
        // the lower 32 bits.
        if (mu) {
            fatal_if(idx == 0,
                    "First BAR in %s is upper 32 bits of a memory BAR.", idx);
            auto *ml = dynamic_cast<PciMemBar *>(BARs[idx - 1]);
            fatal_if(!ml, "Upper 32 bits of memory BAR in %s doesn't come "
                    "after the lower 32.");
            mu->lower(ml);
        }

        _config->baseAddr[idx].update(bar->write(*upstreamInterface, 0));
        _config->baseAddr[idx].writer(
            std::bind(&PciDevice::barConfigWriter, this, _1, _2, bar));

        idx++;
    }

    // Construct PCI capability linked list
    RegisterBankLE::Register8 *nextCapability = &_config->capabilityPtr;
    for (auto cap : p.capabilities) {
        nextCapability->update(cap->base());
        nextCapability->resetInitialValue();

        addConfigRegisterBank(cap);
    }

    _config->command.writer(this, &PciDevice::commandConfigWriter);
    _config->expansionROM.writer(this, &PciDevice::expansionRomConfigWriter);
}

void
PciDevice::init()
{
    DmaDevice::init();

    fatal_if(upstreamInterface == nullptr,
             "%s: Missing upstream interface, ensure this device is connected "
             "to a PCI upstream (host bridge or PCI to PCI bridge).",
             name());
}

void
PciDevice::setUpstreamInterface(
    std::unique_ptr<PciUpstream::DeviceInterface> &&interface)
{
    fatal_if(upstreamInterface,
             "%s: PCI device already has an upstream interface.", name());
    upstreamInterface = std::move(interface);
}

void
PciDevice::addConfigRegisterBank(RegisterBankLE *bank)
{
    AddrRange bank_range = RangeSize(bank->base(), bank->size());
    auto it = _configSpace.lower_bound(bank_range.start());

    // Ensure no overlaping with next bank.
    if (it != _configSpace.end()) {
        fatal_if(it->first < bank_range.end(),
                 "Bank %s overlaps with another bank %s\n", bank->name(),
                 it->second->name());
    }

    // Ensure no overlaping with previous bank.
    if (it != _configSpace.begin()) {
        auto prev = std::prev(it);
        AddrRange prev_range = RangeSize(prev->first, prev->second->size());
        fatal_if(prev_range.end() > bank_range.start(),
                 "Bank %s overlaps with another bank %s\n", bank->name(),
                 prev->second->name());
    }

    auto ret = _configSpace.emplace(bank->base(), bank);
    fatal_if(!ret.second, "Failed to insert config bank %s.", bank->name());
}

Tick
PciDevice::readConfig(PacketPtr pkt, Addr offset)
{
    // Get configuration register bank for given offset.
    auto it = _configSpace.lower_bound(offset);
    if (it == _configSpace.end() || it->first > offset) {
        it--;
    }

    AddrRange config_range = RangeSize(it->second->base(), it->second->size());
    if (config_range.contains(offset)) {
        it->second->read(offset, pkt->getPtr<void>(), pkt->getSize());
        DPRINTF(PciDevice,
                "readConfig:  dev %#x func %#x reg %#x %s[%#x] %d bytes: data "
                "= %#x\n",
                _devAddr.dev, _devAddr.func, offset, it->second->name(),
                offset - it->second->base(), pkt->getSize(),
                pkt->getUintX(ByteOrder::little));
    } else {
        // The offset isn't pointing to a register bank.
        pkt->setUintX(0, ByteOrder::little);
        DPRINTF(PciDevice,
                "readConfig:  dev %#x func %#x reg %#x nobank %d bytes: data "
                "= %#x\n",
                _devAddr.dev, _devAddr.func, offset, pkt->getSize(),
                pkt->getUintX(ByteOrder::little));
    }

    pkt->makeAtomicResponse();
    return configDelay;
}

Tick
PciDevice::read(PacketPtr pkt)
{
    AddrRange config_range = upstreamInterface->configRange();
    if (config_range.contains(pkt->getAddr())) {
        return readConfig(pkt, config_range.getOffset(pkt->getAddr()));
    }

    return readDevice(pkt);
}

AddrRangeList
PciDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    PciCommand command = _config->command.get();
    for (auto *bar: BARs) {
        if (command.ioSpace && bar->isIo())
            ranges.push_back(bar->range());
        if (command.memorySpace && bar->isMem())
            ranges.push_back(bar->range());
    }

    ranges.push_back(upstreamInterface->configRange());

    return ranges;
}

Tick
PciDevice::writeConfig(PacketPtr pkt, Addr offset)
{
    // Get configuration register bank for given offset.
    auto it = _configSpace.lower_bound(offset);
    if (it == _configSpace.end() || it->first > offset) {
        it--;
    }

    AddrRange configRange = RangeSize(it->second->base(), it->second->size());
    if (configRange.contains(offset)) {
        it->second->write(offset, pkt->getConstPtr<void>(), pkt->getSize());
        DPRINTF(
            PciDevice,
            "writeConfig:  dev %#x func %#x reg %#x %s[%#x] %d bytes: data "
            "= %#x\n",
            _devAddr.dev, _devAddr.func, offset, it->second->name(),
            offset - it->second->base(), pkt->getSize(),
            pkt->getUintX(ByteOrder::little));
    } else {
        // The offset isn't pointing to a register bank.
        DPRINTF(PciDevice,
                "writeConfig:  dev %#x func %#x reg %#x nobank %d bytes: data "
                "= %#x\n",
                _devAddr.dev, _devAddr.func, offset, pkt->getSize(),
                pkt->getUintX(ByteOrder::little));
    }

    pkt->makeAtomicResponse();
    return configDelay;
}

Tick
PciDevice::write(PacketPtr pkt)
{
    AddrRange configRange = upstreamInterface->configRange();
    if (configRange.contains(pkt->getAddr())) {
        return writeConfig(pkt, configRange.getOffset(pkt->getAddr()));
    }

    return writeDevice(pkt);
}

void
PciDevice::barConfigWriter(PciConfigBase::Register32 &reg,
                           const uint32_t &value, PciBar *bar)
{
    reg.update(bar->write(*upstreamInterface, value));
    pioPort.sendRangeChange();
}

void
PciDevice::commandConfigWriter(PciConfigBase::PciCommandRegister &reg,
                               const PciCommand &value)
{
    // Command register is used to enable the device, meaning that
    // responding address ranges from BAR will be active. So send range
    // change to inform connected bus of the new ranges.
    reg.update(value);
    pioPort.sendRangeChange();
}

void
PciDevice::expansionRomConfigWriter(PciConfigBase::Register32 &reg,
                                    const uint32_t &value)
{
    if (value == 0xfffffffe) {
        reg.update(0xffffffff);
    } else {
        reg.update(value);
    }
}

void
PciDevice::serialize(CheckpointOut &cp) const
{
    _config->serialize(cp);
}

void
PciDevice::unserialize(CheckpointIn &cp)
{
    _config->unserialize(cp);

    // Update all BAR address ranges
    recvBusChange();
}

PciEndpoint::PciEndpoint(const PciEndpointParams &p)
    : PciDevice(p, {p.BAR0, p.BAR1, p.BAR2, p.BAR3, p.BAR4, p.BAR5},
                new PciConfigType0(p))
{}

PciEndpoint::~PciEndpoint()
{
    delete _config;
}

void
PciEndpoint::init()
{
    PciDevice::init();

    int idx = 0;
    for (auto *bar : BARs) {
        _config->baseAddr[idx++].update(bar->write(*upstreamInterface, 0));
    }

    pioPort.sendRangeChange();
}

void
PciEndpoint::recvBusChange()
{
    for (int idx = 0; idx < BARs.size(); idx++) {
        BARs[idx]->write(*upstreamInterface, _config->baseAddr[idx].get());
    }

    pioPort.sendRangeChange();
}

PciType1Device::PciType1Device(const PciType1DeviceParams &p)
    : PciDevice(p, {p.BAR0, p.BAR1}, new PciConfigType1(p))
{}

PciType1Device::~PciType1Device()
{
    delete _config;
}

void
PciType1Device::init()
{
    PciDevice::init();

    int idx = 0;
    for (auto *bar : BARs) {
        _config->baseAddr[idx++].update(bar->write(*upstreamInterface, 0));
    }

    pioPort.sendRangeChange();
}

void
PciType1Device::recvBusChange()
{
    for (int idx = 0; idx < BARs.size(); idx++) {
        BARs[idx]->write(*upstreamInterface, _config->baseAddr[idx].get());
    }

    pioPort.sendRangeChange();
}

} // namespace gem5
