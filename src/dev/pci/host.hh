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

#ifndef __DEV_PCI_HOST_HH__
#define __DEV_PCI_HOST_HH__

#include "dev/io_device.hh"
#include "dev/pci/types.hh"

namespace gem5
{

struct PciHostParams;
struct GenericPciHostParams;

class PciDevice;
class Platform;

/**
 * The PCI host describes the interface between PCI devices and a
 * simulated system.
 *
 * The PCI host controller has three main responsibilities:
 * <ol>
 *     <li>Expose a configuration memory space that allows devices to
 *         be discovered and configured.
 *     <li>Map and deliver interrupts to the CPU.
 *     <li>Map memory addresses from the PCI bus's various memory
 *         spaces (Legacy IO, non-prefetchable memory, and
 *         prefetchable memory) to physical memory.
 * </ol>
 *
 * PCI devices need to register themselves with a PCI host using the
 * PciHost::registerDevice() call. This call returns a
 * PciHost::DeviceInterface that provides for common functionality
 * such as interrupt delivery and memory mapping.
 *
 * The PciHost class itself provides very little functionality. Simple
 * PciHost functionality is implemented by the GenericPciHost class.
 */
class PciHost : public PioDevice
{
  public:
    PciHost(const PciHostParams &p);
    virtual ~PciHost();

  public:
    /**
     * @{
     * @name Device interface
     */

    /**
     * Callback interface from PCI devices to the host.
     *
     * Devices get an instance of this object when they register
     * themselves with the host using the PciHost::registerDevice()
     * call.
     */
    class DeviceInterface
    {
        friend class gem5::PciHost;

      protected:
        /**
         * Instantiate a device interface
         *
         * @param host PCI host that this device belongs to.
         * @param bus_addr The device's position on the PCI bus
         * @param pin Interrupt pin
         */
        DeviceInterface(PciHost &host, PciBusAddr &bus_addr, PciIntPin pin);

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
         * Calculate the physical address of an IO location on the PCI
         * bus.
         *
         * @param addr Address in the PCI IO address space
         * @return Address in the system's physical address space.
         */
        Addr
        pioAddr(Addr addr) const
        {
            return host.pioAddr(busAddr, addr);
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
            return host.memAddr(busAddr, addr);
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
            return host.dmaAddr(busAddr, addr);
        }

      protected:
        PciHost &host;

        const PciBusAddr busAddr;
        const PciIntPin interruptPin;
    };

    /**
     * Register a PCI device with the host.
     *
     * @param device Device to register
     * @param bus_addr The device's position on the PCI bus
     * @param pin Interrupt pin
     * @return A device-specific DeviceInterface instance.
     */
    virtual DeviceInterface registerDevice(PciDevice *device,
                                           PciBusAddr bus_addr, PciIntPin pin);

    /** @} */

  protected:
    /**
     * @{
     * @name PciHost controller interface
     */

    /**
     * Post an interrupt to the CPU.
     *
     * @param bus_addr The device's position on the PCI bus
     * @param pin PCI interrupt pin
     */
    virtual void postInt(const PciBusAddr &bus_addr, PciIntPin pin) = 0;

    /**
     * Post an interrupt to the CPU.
     *
     * @param bus_addr The device's position on the PCI bus
     * @param pin PCI interrupt pin
     */
    virtual void clearInt(const PciBusAddr &bus_addr, PciIntPin pin) = 0;

    /**
     * Calculate the physical address of an IO location on the PCI
     * bus.
     *
     * @param bus_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI IO address space
     * @return Address in the system's physical address space.
     */
    virtual Addr pioAddr(const PciBusAddr &bus_addr, Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a non-prefetchable memory
     * location in the PCI address space.
     *
     * @param bus_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr memAddr(const PciBusAddr &bus_addr, Addr pci_addr) const = 0;

    /**
     * Calculate the physical address of a prefetchable memory
     * location in the PCI address space.
     *
     * @param bus_addr The device's position on the PCI bus
     * @param pci_addr Address in the PCI DMA memory address space
     * @return Address in the system's physical address space.
     */
    virtual Addr dmaAddr(const PciBusAddr &bus_addr, Addr pci_addr) const = 0;

    /** @} */

  protected:
    /**
     * Retrieve a PCI device from its bus address.
     *
     * @return Pointer to a PciDevice instance or nullptr if the
     *         device doesn't exist.
     */
    PciDevice *getDevice(const PciBusAddr &addr);

    /**
     * Retrieve a PCI device from its bus address.
     *
     * @return Pointer to a constant PciDevice instance or nullptr if
     *         the device doesn't exist.
     */
    const PciDevice *getDevice(const PciBusAddr &addr) const;

  private:
    /** Currently registered PCI devices */
    std::map<PciBusAddr, PciDevice *> devices;
};

/**
 * Configurable generic PCI host interface
 *
 * The GenericPciHost provides a configurable generic PCI host
 * implementation.
 *
 * The generic controller binds to one range of physical addresses to
 * implement the PCI subsystem's configuraiton space. The base
 * address, size and mapping between memory addresses and PCI devices
 * are all configurable as simulation parameters. The basic
 * implementation supports both the Configuration Access Mechanism
 * (CAM) and Enhanced Configuration Access Mechanism (ECAM)
 * configuration space layout. The layouts can be configured by
 * changing the number of bits allocated to each device in the
 * configuration space. ECAM uses 12 bits per device, while CAM uses 8
 * bits per device.
 *
 * Interrupts are delivered via the Platform::postInt() and
 * Platform::clearInt() calls. Interrupt numbers are mapped statically
 * using the interrupt line (PciDevice::interruptLine()) returned from
 * the device. Implementations may override mapPciInterrupt() to
 * dynamically map a PciBusAddr and PciIntPin to a platform-specific
 * interrupt.
 *
 * All PCI memory spaces (IO, prefetchable, and non-prefetchable)
 * support a simple base+offset mapping that can be configured using
 * simulation parameters. The base defaults to 0 for all of them.
 */
class GenericPciHost : public PciHost
{
  public:
    GenericPciHost(const GenericPciHostParams &p);
    virtual ~GenericPciHost();

  public: // PioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    AddrRangeList getAddrRanges() const override;

  protected: // PciHost
    Addr
    pioAddr(const PciBusAddr &bus_addr, Addr pci_addr) const override
    {
        return pciPioBase + pci_addr;
    }

    Addr
    memAddr(const PciBusAddr &bus_addr, Addr pci_addr) const override
    {
        return pciMemBase + pci_addr;
    }

    Addr
    dmaAddr(const PciBusAddr &bus_addr, Addr pci_addr) const override
    {
        return pciDmaBase + pci_addr;
    }

  protected: // Configuration address space handling
    /**
     * Decode a configuration space address.
     *
     *
     * @param addr Offset into the configuration space
     * @return Tuple containing the PCI bus address and an offset into
     *         the device's configuration space.
     */
    virtual std::pair<PciBusAddr, Addr> decodeAddress(Addr address);

  protected: // Interrupt handling
    void postInt(const PciBusAddr &addr, PciIntPin pin) override;
    void clearInt(const PciBusAddr &addr, PciIntPin pin) override;

    virtual uint32_t mapPciInterrupt(const PciBusAddr &bus_addr,
                                     PciIntPin pin) const;

  protected:
    Platform &platform;

    const Addr confBase;
    const Addr confSize;
    const uint8_t confDeviceBits;

    const Addr pciPioBase;
    const Addr pciMemBase;
    const Addr pciDmaBase;
};

} // namespace gem5

#endif // __DEV_PCI_HOST_HH__
