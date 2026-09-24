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

#ifndef __DEV_PCI_UPSTREAM_HH__
#define __DEV_PCI_UPSTREAM_HH__

#include <map>

#include "base/addr_range.hh"
#include "dev/isa_fake.hh"
#include "dev/pci/types.hh"
#include "dev/pci/up_down_bridge.hh"
#include "params/PciConfigError.hh"

namespace gem5
{

class PciDevice;

class PciConfigError : public IsaFake
{
  public:
    PARAMS(PciConfigError);

    PciConfigError(const Params &p);

    void setAddrRange(AddrRange range);

    AddrRangeList getAddrRanges() const override;
};

/**
 * The PCI upstream describes any device (PCI host bridge, PCI-PCI bridge)
 * that are connected upstream of PCI devices (endpoint or bridge).
 *
 * The PCI upstream controller has three main responsibilities:
 * <ol>
 *     <li>Bridge all packets between two buses (e.g. system and PCI bus)
 *     <li>Map and deliver interrupts to the next upstream or CPU.
 *     <li>Map memory addresses from the PCI bus's various memory
 *         spaces (Configuration, legacy IO, non-prefetchable memory, and
 *         prefetchable memory) to physical memory.
 * </ol>
 *
 * PCI devices directly downstream will receive a PciUpstream::DeviceInterface
 * that provides for common functionality such as interrupt delivery and memory
 * mapping.
 *
 * The PciUpstream is an abstract class that provides the device registering
 * part and should be inherited by the actual upstream classes, which must
 * provides implementation for the interrupts and memory mapping.
 *
 * It can be double inherited with a SimObject to implement an actual objects.
 * In that case, like shown in the following example, the declaration "using
 * SimObject::name;" must be added in the derived class to avoid any ambiguity
 * of name(). (SimObject must be replaced by the actual class name of the
 * simulation object)
 *
 *   class Example : public SimObject, public PciUpstream
 *   {
 *     public:
 *       using SimObject::name;
 *   };
 */
class PciUpstream
{
  public:
    PciUpstream(PciUpDownBridge *up_to_down, PciConfigError *config_error_dev,
                const std::vector<PciDevice *> &pci_devices,
                std::string upstream_name);

    void init();

    /**
     * Get name of the upstream. Used to print correct object name with
     * DPRINTF.
     */
    std::string name() const;

    /**
     * @{
     * @name Device interface
     */

    /**
     * Callback interface from PCI devices to the upstream.
     *
     * Devices get an instance of this object during PciUpstream
     * initialization.
     */
    class DeviceInterface
    {
        friend class gem5::PciUpstream;

      protected:
        /**
         * Instantiate a device interface
         *
         * @param upstream PCI upstream that this device belongs to.
         * @param device The device associated with the interface
         */
        DeviceInterface(PciUpstream &upstream, PciDevice &device);

      public:
        DeviceInterface() = delete;
        void operator=(const DeviceInterface &) = delete;

        const std::string name() const;

        /**
         * Get the PCI bus number of the upstream bridge.
         */
        PciBusNum getBusNum() const;

        /**
         * Post a PCI interrupt to the CPU.
         */
        void postInt();

        /**
         * Post a PCI interrupt to the CPU.
         * This function should be used by bridges to post interrupts
         * from devices downstream of them.
         *
         * @param device PCI device posting the interrupts.
         */
        void postInt(const PciDevice &device);

        /**
         * Clear a posted PCI interrupt
         */
        void clearInt();

        /**
         * Clear a posted PCI interrupt.
         * This function should be used by bridges to clear interrupts
         * from devices downstream of them.
         *
         * @param device PCI device posting the interrupts.
         */
        void clearInt(const PciDevice &device);

        /**
         * Calculate the physical address range of the PCI device
         * configuration space.
         *
         * @return Address range in the system's physical address space.
         */
        AddrRange configRange() const;

        /**
         * Calculate the physical address range of the PCI device
         * configuration space. This function should be used by bridges to get
         * ranges from devices downstream of them.
         *
         * @param device PCI device requesting configuration range.
         * @return Address range in the system's physical address space.
         */
        AddrRange configRange(const PciDevice &device) const;

        /**
         * Calculate the physical address of an IO location on the PCI
         * bus.
         *
         * @param addr Address in the PCI IO address space
         * @return Address in the system's physical address space.
         */
        Addr pioAddr(Addr addr) const;

        /**
         * Calculate the physical address of an IO location on the PCI
         * bus. This function should be used by bridges to get address
         * from devices downstream of them.
         *
         * @param device PCI device requesting PIO addr.
         * @param addr Address in the PCI IO address space
         * @return Address in the system's physical address space.
         */
        Addr pioAddr(const PciDevice &device, Addr addr) const;

        /**
         * Calculate the physical address of a non-prefetchable memory
         * location in the PCI address space.
         *
         * @param addr Address in the PCI memory address space
         * @return Address in the system's physical address space.
         */
        Addr memAddr(Addr addr) const;

        /**
         * Calculate the physical address of a non-prefetchable memory
         * location in the PCI address space. This function should be used by
         * bridge to get address from devices downstream of them.
         *
         * @param device PCI device requesting memory address translation.
         * @param addr Address in the PCI memory address space
         * @return Address in the system's physical address space.
         */
        Addr memAddr(const PciDevice &device, Addr addr) const;

        /**
         * Calculate the physical address of a prefetchable memory
         * location in the PCI address space.
         *
         * @param addr Address in the PCI DMA memory address space
         * @return Address in the system's physical address space.
         */
        Addr dmaAddr(Addr addr) const;

        /**
         * Calculate the physical address of a prefetchable memory
         * location in the PCI address space. This function should be used by
         * bridge to get address from devices downstream of them.
         *
         * @param device PCI device request DMA address.
         * @param addr Address in the PCI DMA memory address space
         * @return Address in the system's physical address space.
         */
        Addr dmaAddr(const PciDevice &device, Addr addr) const;

        /**
         * Calculate the physical address range of the PCI configuration space
         * for a range of buses.
         *
         * @param start_bus First bus of the range to get
         * @param end_bus Last bus of the range to get, included
         * @return Address range in the system's physical address space.
         */
        AddrRange busConfigRange(PciBusNum start_bus, PciBusNum end_bus) const;

      protected:
        PciUpstream &upstream;
        PciDevice &device;
    };

    /** @} */

    /**
     * Inform each PCI devices connected to this upstream of a bus number
     * change.
     */
    void sendBusChange();

  protected:
    /**
     * Get the range for the configuration memory space for which this PCI
     * upstream is responsible. The range should include the full configuration
     * space even where no bus/device are present.
     */
    virtual AddrRange getConfigAddrRange() const = 0;

    /**
     * @{
     * @name PciUpstream controller interface
     */

    /**
     * Post an interrupt to the CPU.
     *
     * @param device The requesting PCI device
     */
    virtual void interfacePostInt(const PciDevice &device) = 0;

    /**
     * Clear an interrupt to the CPU.
     *
     * @param device The requesting PCI device
     */
    virtual void interfaceClearInt(const PciDevice &device) = 0;

    /**
     * Calculate the physical address range of the PCI device
     * configuration space.
     *
     * @param device The requesting PCI device
     * @return Configuration address range in the system's physical address
     *         space.
     */
    virtual AddrRange interfaceConfigRange(const PciDevice &device) const = 0;

    /**
     * Calculate the physical address of an IO location on the PCI
     * bus.
     *
     * @param device The requesting PCI device
     * @param pci_addr Address in the PCI IO address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfacePioAddr(const PciDevice &device,
                                  Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a non-prefetchable memory
     * location in the PCI address space.
     *
     * @param device The requesting PCI device
     * @param pci_addr Address in the PCI memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfaceMemAddr(const PciDevice &device,
                                  Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a prefetchable memory
     * location in the PCI address space.
     *
     * @param device The requesting PCI device
     * @param pci_addr Address in the PCI DMA memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfaceDmaAddr(const PciDevice &device,
                                  Addr pci_addr) const = 0;

    /**
     * Get the range for the configuration memory space for which a downstream
     * PCI bridge is responsible for.
     *
     * @param start_bus First bus for which the bridge is responsible
     * (secondary bus number)
     * @param end_bus Last bus for which the bridge is responsible, included
     * (subordinate bus number)
     */
    virtual AddrRange interfaceBusConfigRange(PciBusNum start_bus,
                                              PciBusNum end_bus) const = 0;

    /** @} */

    /**
     * Get the PCI bus number assign to that upstream.
     */
    virtual PciBusNum getBusNum() const = 0;

  protected:
    /**
     * Retrieve a PCI device from its bus address.
     *
     * @return Pointer to a PciDevice instance or nullptr if the
     *         device doesn't exist.
     */
    PciDevice *getDevice(const PciDevAddr &addr);

    /**
     * Retrieve a PCI device from its bus address.
     *
     * @return Pointer to a constant PciDevice instance or nullptr if
     *         the device doesn't exist.
     */
    const PciDevice *getDevice(const PciDevAddr &addr) const;

  private:
    /** Currently registered PCI interfaces */
    std::map<PciDevAddr, PciDevice *> devices;

    /** The two one way bridges to connect both side buses */
    PciUpDownBridge *upToDown;

    PciConfigError *configErrorDevice;

    std::string upstreamName;
};

} // namespace gem5

#endif // __DEV_PCI_UPSTREAM_HH__
