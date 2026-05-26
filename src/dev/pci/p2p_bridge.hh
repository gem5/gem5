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

#ifndef __DEV_PCI_P2P_BRIDGE_HH__
#define __DEV_PCI_P2P_BRIDGE_HH__

#include "dev/pci/device.hh"
#include "dev/pci/types.hh"
#include "dev/pci/upstream.hh"

namespace gem5
{

class Platform;
struct PciToPciBridgeParams;

/**
 * The PCI to PCI bridge upstream implements the required interface to let
 * PCI device connect to them. PciToPciBridge class will forward address and
 * range request to the next upstream, which can be the PCI host or another PCI
 * bridge. If downstream bus number is set to 0, the bridge will be considered
 * as unconfigured and then return empty ranges or invalid addresses.
 */
class PciToPciBridge : public PciType1Device, public PciUpstream
{
  public:
    /* Remove name() ambiguity. */
    using PciType1Device::name;

    PARAMS(PciToPciBridge);
    PciToPciBridge(const Params &p);
    virtual ~PciToPciBridge();

    void init() override;

    /**
     * Get the range for the configuration memory space for which this PCI
     * bridge is responsible. The range should include the full configuration
     * space even where no bus/device are present.
     */
    AddrRange getConfigAddrRange() const override;

  protected: // PciUpstream
    AddrRange interfaceConfigRange(PciBusNum bus_num,
                                   const PciDevice &device) const override;

    Addr interfacePioAddr(PciBusNum bus_num, const PciDevice &device,
                          Addr pci_addr) const override;

    Addr interfaceMemAddr(PciBusNum bus_num, const PciDevice &device,
                          Addr pci_addr) const override;

    Addr interfaceDmaAddr(PciBusNum bus_num, const PciDevice &device,
                          Addr pci_addr) const override;

    AddrRange interfaceBusConfigRange(PciBusNum start_bus,
                                      PciBusNum end_bus) const override;

    void interfacePostInt(PciBusNum bus_num, const PciDevice &device) override;
    void interfaceClearInt(PciBusNum bus_num,
                           const PciDevice &device) override;

    PciBusNum getBusNum() const override;

    /**
     * Check whatever a bus number is a valid downstream bus number.
     * A valid downstream bus number is contained in the range between
     * secondary and subordinate number for configuration header.
     */
    bool isDownstreamBus(PciBusNum bus_num) const;

  protected: // PciDevice
    Tick writeConfig(PacketPtr pkt) override;

    Tick
    writeDevice(PacketPtr pkt) override
    {
        return 0;
    }

    Tick
    readDevice(PacketPtr pkt) override
    {
        return 0;
    }
};

} // namespace gem5

#endif // __DEV_PCI_P2P_BRIDGE_HH__
