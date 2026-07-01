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

#include "dev/pci/config.hh"

#include "base/cprintf.hh"

namespace gem5
{

PciConfigBase::PciConfigBase(const PciDeviceParams &p, uint8_t header_type,
                             size_t nb_bar)
    : RegisterBankLE(p.name + ".header", 0),
      vendorId("vendor", p.VendorID),
      deviceId("device", p.DeviceID),
      command("command", p.Command),
      status("status", p.Status),
      revision("revision", p.Revision),
      progIF("prog_if", p.ProgIF),
      subClassCode("sub_class_code", p.SubClassCode),
      classCode("class_code", p.ClassCode),
      cacheLineSize("cache_line_size", p.CacheLineSize),
      latencyTimer("latency_timer", p.LatencyTimer),
      headerType("header_type", header_type),
      bist("bist", p.BIST),
      expansionROM("expansion_rom", p.ExpansionROM),
      capabilityPtr("capability_ptr", 0),
      interruptLine("interrupt_line", p.InterruptLine),
      interruptPin("interrupt_pin", p.InterruptPin)
{
    if (p.MultiFunction) {
        headerType.update((1 << 7) | header_type);
    }

    addRegisters({
        {PCI_VENDOR_ID, vendorId},
        {PCI_DEVICE_ID, deviceId},
        {PCI_COMMAND, command},
        {PCI_STATUS, status},
        {PCI_REVISION_ID, revision},
        {PCI_PROG_IF, progIF},
        {PCI_SUB_CLASS_CODE, subClassCode},
        {PCI_BASE_CLASS_CODE, classCode},
        {PCI_CACHE_LINE_SIZE, cacheLineSize},
        {PCI_LATENCY_TIMER, latencyTimer},
        {PCI_HEADER_TYPE, headerType},
        {PCI_BIST, bist},
    });

    vendorId.readonly();
    deviceId.readonly();
    revision.readonly();
    progIF.readonly();
    subClassCode.readonly();
    classCode.readonly();
    headerType.readonly();
    capabilityPtr.readonly();
    interruptPin.readonly();

    // Generate base address registers.
    baseAddr.reserve(nb_bar);
    for (size_t i = 0; i < nb_bar; i++) {
        baseAddr.emplace_back(csprintf("bar_%d", i), 0);
        addRegister(baseAddr.back());
    }

    // Expansion ROM, capability pointer and interrupt line/PIN aren't added
    // here as they are after the config type specific registers.
}

void
PciConfigBase::serialize(CheckpointOut &cp) const
{
    for (const Register32 &bar : baseAddr) {
        paramOut(cp, bar.name(), bar);
    }

    SERIALIZE_SCALAR(vendorId);
    SERIALIZE_SCALAR(deviceId);
    SERIALIZE_SCALAR(command);
    SERIALIZE_SCALAR(status);
    SERIALIZE_SCALAR(revision);
    SERIALIZE_SCALAR(progIF);
    SERIALIZE_SCALAR(subClassCode);
    SERIALIZE_SCALAR(classCode);
    SERIALIZE_SCALAR(cacheLineSize);
    SERIALIZE_SCALAR(latencyTimer);
    SERIALIZE_SCALAR(headerType);
    SERIALIZE_SCALAR(bist);
    SERIALIZE_SCALAR(expansionROM);
    SERIALIZE_SCALAR(capabilityPtr);
    SERIALIZE_SCALAR(interruptLine);
    SERIALIZE_SCALAR(interruptPin);
}

void
PciConfigBase::unserialize(CheckpointIn &cp)
{
    for (Register32 &bar : baseAddr) {
        paramIn(cp, bar.name(), bar);
    }

    UNSERIALIZE_SCALAR(vendorId);
    UNSERIALIZE_SCALAR(deviceId);
    UNSERIALIZE_SCALAR(command);
    UNSERIALIZE_SCALAR(status);
    UNSERIALIZE_SCALAR(revision);
    UNSERIALIZE_SCALAR(progIF);
    UNSERIALIZE_SCALAR(subClassCode);
    UNSERIALIZE_SCALAR(classCode);
    UNSERIALIZE_SCALAR(cacheLineSize);
    UNSERIALIZE_SCALAR(latencyTimer);
    UNSERIALIZE_SCALAR(headerType);
    UNSERIALIZE_SCALAR(bist);
    UNSERIALIZE_SCALAR(expansionROM);
    UNSERIALIZE_SCALAR(capabilityPtr);
    UNSERIALIZE_SCALAR(interruptLine);
    UNSERIALIZE_SCALAR(interruptPin);
}

PciConfigType0::PciConfigType0(const PciEndpointParams &p)
    : PciConfigBase(p, 0, PCI0_BASE_ADDR_COUNT),
      cardbusCIS("cardbus_cis", p.CardbusCIS),
      subsystemVendorID("subsystem_vendor_id", p.SubsystemVendorID),
      subsystemID("subsystem_id", p.SubsystemID),
      minimumGrant("minimum_grant", p.MinimumGrant),
      maximumLatency("maximum_latency", p.MaximumLatency)
{
    // Ensure reserved offsets will return 0 using RAZ register
    addRegistersAt<RegisterRaz>({
        {PCI0_CIS, cardbusCIS},
        {PCI0_SUB_VENDOR_ID, subsystemVendorID},
        {PCI0_SUB_SYSTEM_ID, subsystemID},
        {PCI0_ROM_BASE_ADDR, expansionROM},
        {PCI0_MINIMUM_GRANT, minimumGrant},
        {PCI0_MAXIMUM_LATENCY, maximumLatency},

        // Add missing common registers
        {PCI_CAP_PTR, capabilityPtr},
        {PCI_INTERRUPT_LINE, interruptLine},
        {PCI_INTERRUPT_PIN, interruptPin},
    });

    subsystemVendorID.readonly();
    subsystemID.readonly();
}

void
PciConfigType0::serialize(CheckpointOut &cp) const
{
    PciConfigBase::serialize(cp);

    SERIALIZE_SCALAR(cardbusCIS);
    SERIALIZE_SCALAR(subsystemVendorID);
    SERIALIZE_SCALAR(subsystemID);
    SERIALIZE_SCALAR(minimumGrant);
    SERIALIZE_SCALAR(maximumLatency);
}

void
PciConfigType0::unserialize(CheckpointIn &cp)
{
    PciConfigBase::unserialize(cp);

    UNSERIALIZE_SCALAR(cardbusCIS);
    UNSERIALIZE_SCALAR(subsystemVendorID);
    UNSERIALIZE_SCALAR(subsystemID);
    UNSERIALIZE_SCALAR(minimumGrant);
    UNSERIALIZE_SCALAR(maximumLatency);
}

PciConfigType1::PciConfigType1(const PciType1DeviceParams &p)
    : PciConfigBase(p, 1, PCI1_BASE_ADDR_COUNT),
      primaryBusNum("primary_bus_num", p.PrimaryBusNumber),
      secondaryBusNum("secondary_bus_num", p.SecondaryBusNumber),
      subordinateBusNum("subordinate_bus_num", p.SubordinateBusNumber),
      secondaryLatencyTimer("secondary_latency_timer",
                            p.SecondaryLatencyTimer),
      ioBase("io_base", p.IOBase),
      ioLimit("io_limit", p.IOLimit),
      secondaryStatus("secondary_status", p.SecondaryStatus),
      memBase("mem_base", p.MemoryBase),
      memLimit("mem_limit", p.MemoryLimit),
      prefetchMemBase("prefetch_mem_base", p.PrefetchableMemoryBase),
      prefetchMemLimit("prefetch_mem_limit", p.PrefetchableMemoryLimit),
      prefetchBaseUpper("prefetch_base_upper", p.PrefetchableBaseUpper),
      prefetchLimitUpper("prefetch_limit_upper", p.PrefetchableLimitUpper),
      ioBaseUpper("io_base_upper", p.IOBaseUpper),
      ioLimitUpper("io_limit_upper", p.IOLimitUpper),
      bridgeControl("bridge_control", p.BridgeControl)
{
    // Ensure reserved offsets will return 0 using RAZ register
    addRegistersAt<RegisterRaz>({
        {PCI1_PRI_BUS_NUM, primaryBusNum},
        {PCI1_SEC_BUS_NUM, secondaryBusNum},
        {PCI1_SUB_BUS_NUM, subordinateBusNum},
        {PCI1_SEC_LAT_TIMER, secondaryLatencyTimer},
        {PCI1_IO_BASE, ioBase},
        {PCI1_IO_LIMIT, ioLimit},
        {PCI1_SECONDARY_STATUS, secondaryStatus},
        {PCI1_MEM_BASE, memBase},
        {PCI1_MEM_LIMIT, memLimit},
        {PCI1_PRF_MEM_BASE, prefetchMemBase},
        {PCI1_PRF_MEM_LIMIT, prefetchMemLimit},
        {PCI1_PRF_BASE_UPPER, prefetchBaseUpper},
        {PCI1_PRF_LIMIT_UPPER, prefetchLimitUpper},
        {PCI1_IO_BASE_UPPER, ioBaseUpper},
        {PCI1_IO_LIMIT_UPPER, ioLimitUpper},
        {PCI1_ROM_BASE_ADDR, expansionROM},
        {PCI1_BRIDGE_CTRL, bridgeControl},

        // Add missing common registers
        {PCI_CAP_PTR, capabilityPtr},
        {PCI_INTERRUPT_LINE, interruptLine},
        {PCI_INTERRUPT_PIN, interruptPin},
    });
}

void
PciConfigType1::serialize(CheckpointOut &cp) const
{
    PciConfigBase::serialize(cp);

    SERIALIZE_SCALAR(primaryBusNum);
    SERIALIZE_SCALAR(secondaryBusNum);
    SERIALIZE_SCALAR(subordinateBusNum);
    SERIALIZE_SCALAR(secondaryLatencyTimer);
    SERIALIZE_SCALAR(ioBase);
    SERIALIZE_SCALAR(ioLimit);
    SERIALIZE_SCALAR(secondaryStatus);
    SERIALIZE_SCALAR(memBase);
    SERIALIZE_SCALAR(memLimit);
    SERIALIZE_SCALAR(prefetchMemBase);
    SERIALIZE_SCALAR(prefetchMemLimit);
    SERIALIZE_SCALAR(prefetchBaseUpper);
    SERIALIZE_SCALAR(prefetchLimitUpper);
    SERIALIZE_SCALAR(ioBaseUpper);
    SERIALIZE_SCALAR(ioLimitUpper);
    SERIALIZE_SCALAR(bridgeControl);
}

void
PciConfigType1::unserialize(CheckpointIn &cp)
{
    PciConfigBase::unserialize(cp);

    UNSERIALIZE_SCALAR(primaryBusNum);
    UNSERIALIZE_SCALAR(secondaryBusNum);
    UNSERIALIZE_SCALAR(subordinateBusNum);
    UNSERIALIZE_SCALAR(secondaryLatencyTimer);
    UNSERIALIZE_SCALAR(ioBase);
    UNSERIALIZE_SCALAR(ioLimit);
    UNSERIALIZE_SCALAR(secondaryStatus);
    UNSERIALIZE_SCALAR(memBase);
    UNSERIALIZE_SCALAR(memLimit);
    UNSERIALIZE_SCALAR(prefetchMemBase);
    UNSERIALIZE_SCALAR(prefetchMemLimit);
    UNSERIALIZE_SCALAR(prefetchBaseUpper);
    UNSERIALIZE_SCALAR(prefetchLimitUpper);
    UNSERIALIZE_SCALAR(ioBaseUpper);
    UNSERIALIZE_SCALAR(ioLimitUpper);
    UNSERIALIZE_SCALAR(bridgeControl);
}

} // namespace gem5
