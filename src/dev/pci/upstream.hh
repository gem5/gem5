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
#include "params/PciUpstream.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class PciDevice;

class PciConfigError : public IsaFake
{
  public:
    PARAMS(PciConfigError);

    PciConfigError(const Params &p);

    void setAddrRange(AddrRange range);
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
 * PCI devices need to register themselves with a PCI upstream using the
 * PciUpstream::registerDevice() call. This call returns a
 * PciUpstream::DeviceInterface that provides for common functionality
 * such as interrupt delivery and memory mapping.
 *
 * The PciUpstream is an abstract class that provides the device registering
 * part and should be inherited by the actual upstream classes, which must
 * provides implementation for the interrupts and memory mapping.
 */
class PciUpstream : public ClockedObject
{
  public:
    PARAMS(PciUpstream);

    PciUpstream(const Params &p);

    void init() override;

    /**
     * @{
     * @name Device interface
     */

    /**
     * Callback interface from PCI devices to the upstream.
     *
     * Devices get an instance of this object by calling
     * PciUpstream::registerDevice() on their direct upstream.
     */
    class DeviceInterface
    {
        friend class gem5::PciUpstream;

      protected:
        /**
         * Instantiate a device interface
         *
         * @param upstream PCI upstream that this device belongs to.
         * @param dev_addr The device's position on the PCI bus
         * @param pin Interrupt pin
         */
        DeviceInterface(PciUpstream &upstream, const PciDevAddr &dev_addr,
                        PciIntPin pin);

      public:
        DeviceInterface() = delete;
        void operator=(const DeviceInterface &) = delete;

        const std::string name() const;

        /**
         * Post a PCI interrupt to the CPU.
         */
        void postInt();

        /**
         * Clear a posted PCI interrupt
         */
        void clearInt();

        /**
         * Calculate the physical address range of the PCI device
         * configuration space.
         *
         * @return Address range in the system's physical address space.
         */
        AddrRange
        configRange() const
        {
            return upstream.interfaceConfigRange(devAddr);
        }

        /**
         * Calculate the physical address of an IO location on the PCI
         * bus.
         *
         * @param addr Address in the PCI IO address space
         * @return Address in the system's physical address space.
         */
        Addr
        pioAddr(Addr addr) const
        {
            return upstream.interfacePioAddr(devAddr, addr);
        }

        /**
         * Calculate the physical address of a non-prefetchable memory
         * location in the PCI address space.
         *
         * @param addr Address in the PCI memory address space
         * @return Address in the system's physical address space.
         */
        Addr
        memAddr(Addr addr) const
        {
            return upstream.interfaceMemAddr(devAddr, addr);
        }

        /**
         * Calculate the physical address of a prefetchable memory
         * location in the PCI address space.
         *
         * @param addr Address in the PCI DMA memory address space
         * @return Address in the system's physical address space.
         */
        Addr
        dmaAddr(Addr addr) const
        {
            return upstream.interfaceDmaAddr(devAddr, addr);
        }

      protected:
        PciUpstream &upstream;

        const PciDevAddr devAddr;
        const PciIntPin interruptPin;
    };

    /**
     * Register a PCI device with the host.
     *
     * @param device Device to register
     * @param dev_addr The device's position on the PCI bus
     * @param pin Interrupt pin
     * @return A device-specific DeviceInterface instance.
     */
    virtual DeviceInterface registerDevice(PciDevice *device,
                                           PciDevAddr dev_addr, PciIntPin pin);

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
     * @param dev_addr The device's position on the PCI bus
     * @param pin PCI interrupt pin
     */
    virtual void interfacePostInt(const PciDevAddr &dev_addr,
                                  PciIntPin pin) = 0;

    /**
     * Post an interrupt to the CPU.
     *
     * @param dev_addr The device's position on the PCI bus
     * @param pin PCI interrupt pin
     */
    virtual void interfaceClearInt(const PciDevAddr &dev_addr,
                                   PciIntPin pin) = 0;

    /**
     * Calculate the physical address range of the PCI device
     * configuration space.
     *
     * @param dev_addr The device's position on the PCI bus
     * @return Configuration address range in the system's physical address
     *         space.
     */
    virtual AddrRange
    interfaceConfigRange(const PciDevAddr &dev_addr) const = 0;

    /**
     * Calculate the physical address of an IO location on the PCI
     * bus.
     *
     * @param dev_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI IO address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfacePioAddr(const PciDevAddr &dev_addr,
                                  Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a non-prefetchable memory
     * location in the PCI address space.
     *
     * @param dev_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfaceMemAddr(const PciDevAddr &dev_addr,
                                  Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a prefetchable memory
     * location in the PCI address space.
     *
     * @param dev_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI DMA memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr interfaceDmaAddr(const PciDevAddr &dev_addr,
                                  Addr pci_addr) const = 0;

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
};

} // namespace gem5

#endif // __DEV_PCI_UPSTREAM_HH__
