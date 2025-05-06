/*
 * Copyright (c) 2013 ARM Limited
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
 * Interface for devices using PCI configuration
 */

#ifndef __DEV_PCI_DEVICE_HH__
#define __DEV_PCI_DEVICE_HH__

#include <cstring>
#include <vector>

#include "dev/dma_device.hh"
#include "dev/pci/host.hh"
#include "dev/pci/pcireg.h"
#include "params/PciBar.hh"
#include "params/PciBarNone.hh"
#include "params/PciBridge.hh"
#include "params/PciDevice.hh"
#include "params/PciEndpoint.hh"
#include "params/PciIoBar.hh"
#include "params/PciLegacyIoBar.hh"
#include "params/PciMemBar.hh"
#include "params/PciMemUpperBar.hh"
#include "sim/byteswap.hh"

#define PCI0_BAR_NUMBER(x) (((x) - PCI0_BASE_ADDR0) >> 0x2);
#define PCI1_BAR_NUMBER(x) (((x) - PCI1_BASE_ADDR0) >> 0x2);

namespace gem5
{

class PciBar : public SimObject
{
  protected:
    // The address and size of the region this decoder recognizes.
    Addr _addr = 0;
    Addr _size = 0;

  public:
    PciBar(const PciBarParams &p) : SimObject(p) {}

    virtual bool isMem() const { return false; }
    virtual bool isIo() const { return false; }

    // Accepts a value written to config space, consumes it, and returns what
    // value config space should actually be set to. Both should be in host
    // endian format.
    virtual uint32_t write(const PciHost::DeviceInterface &host,
                           uint32_t val) = 0;

    AddrRange range() const { return AddrRange(_addr, _addr + _size); }
    Addr addr() const { return _addr; }
    Addr size() const { return _size; }

    // Hack for devices that don't know their BAR sizes ahead of time :-o.
    // Don't use unless you have to, since this may not propogate properly
    // outside of a small window.
    void size(Addr value) { _size = value; }
};

class PciBarNone : public PciBar
{
  public:
    PciBarNone(const PciBarNoneParams &p) : PciBar(p) {}

    uint32_t
    write(const PciHost::DeviceInterface &host, uint32_t val) override
    {
        return 0;
    }
};

class PciIoBar : public PciBar
{
  protected:
    BitUnion32(Bar)
        Bitfield<31, 2> addr;
        Bitfield<1> reserved;
        Bitfield<0> io;
    EndBitUnion(Bar)

  public:
    PciIoBar(const PciIoBarParams &p, bool legacy=false) : PciBar(p)
    {
        _size = p.size;
        if (!legacy) {
            Bar bar = _size;
            fatal_if(!_size || !isPowerOf2(_size) || bar.io || bar.reserved,
                    "Illegal size %d for bar %s.", _size, name());
        }
    }

    bool isIo() const override { return true; }

    uint32_t
    write(const PciHost::DeviceInterface &host, uint32_t val) override
    {
        // Mask away the bits fixed by hardware.
        Bar bar = val & ~(_size - 1);
        // Set the fixed bits to their correct values.
        bar.reserved = 0;
        bar.io = 1;

        // Update our address.
        _addr = host.pioAddr(bar.addr << 2);

        // Return what should go into config space.
        return bar;
    }
};

class PciLegacyIoBar : public PciIoBar
{
  protected:
    Addr fixedAddr;

  public:
    PciLegacyIoBar(const PciLegacyIoBarParams &p) : PciIoBar(p, true)
    {
        // Save the address until we get a host to translate it.
        fixedAddr = p.addr;
    }

    uint32_t
    write(const PciHost::DeviceInterface &host, uint32_t val) override
    {
        // Update the address now that we have a host to translate it.
        _addr = host.pioAddr(fixedAddr);
        // Ignore writes.
        return 0;
    }
};

class PciMemBar : public PciBar
{
  private:
    BitUnion32(Bar)
        Bitfield<31, 3> addr;
        SubBitUnion(type, 2, 1)
            Bitfield<2> wide;
            Bitfield<1> reserved;
        EndSubBitUnion(type)
        Bitfield<0> io;
    EndBitUnion(Bar)

    bool _wide = false;
    uint64_t _lower = 0;
    uint64_t _upper = 0;

  public:
    PciMemBar(const PciMemBarParams &p) : PciBar(p)
    {
        _size = p.size;
        Bar bar = _size;
        fatal_if(!_size || !isPowerOf2(_size) || bar.io || bar.type,
                "Illegal size %d for bar %s.", _size, name());
    }

    bool isMem() const override { return true; }

    uint32_t
    write(const PciHost::DeviceInterface &host, uint32_t val) override
    {
        // Mask away the bits fixed by hardware.
        Bar bar = val & ~(_size - 1);
        // Set the fixed bits to their correct values.
        bar.type.wide = wide() ? 1 : 0;
        bar.type.reserved = 0;
        bar.io = 0;

        // Keep track of our lower 32 bits.
        _lower = bar.addr << 3;

        // Update our address.
        _addr = host.memAddr(upper() + lower());

        // Return what should go into config space.
        return bar;
    }

    bool wide() const { return _wide; }
    void wide(bool val) { _wide = val; }

    uint64_t upper() const { return _upper; }
    void
    upper(const PciHost::DeviceInterface &host, uint32_t val)
    {
        _upper = (uint64_t)val << 32;

        // Update our address.
        _addr = host.memAddr(upper() + lower());
    }

    uint64_t lower() const { return _lower; }
};

class PciMemUpperBar : public PciBar
{
  private:
    PciMemBar *_lower = nullptr;

  public:
    PciMemUpperBar(const PciMemUpperBarParams &p) : PciBar(p)
    {}

    void
    lower(PciMemBar *val)
    {
        _lower = val;
        // Let our lower half know we're up here.
        _lower->wide(true);
    }

    uint32_t
    write(const PciHost::DeviceInterface &host, uint32_t val) override
    {
        assert(_lower);

        // Mask away bits fixed by hardware, if any.
        Addr upper = val & ~((_lower->size() - 1) >> 32);

        // Let our lower half know about the update.
        _lower->upper(host, upper);

        return upper;
    }
};

class PciEndpoint;
class PciBridge;

/**
 * PCI device, base implementation is only config space.
 */
class PciDevice : public DmaDevice
{
    friend PciEndpoint;
    friend PciBridge;

  private:
    /** The current config space.  */
    PCIConfig _config;

    bool
    isCommonConfig(Addr offs)
    {
        return (offs <= PCI_BIST) || (offs == PCI_CAP_PTR) ||
               (offs == PCI_INTERRUPT_LINE) || (offs == PCI_INTERRUPT_PIN);
    }

  protected:
    const PciBusAddr _busAddr;

    /** The capability list structures and base addresses
     * @{
     */
    const int PMCAP_BASE;
    const int PMCAP_ID_OFFSET;
    const int PMCAP_PC_OFFSET;
    const int PMCAP_PMCS_OFFSET;
    PMCAP pmcap;

    const int MSICAP_BASE;
    MSICAP msicap;

    const int MSIXCAP_BASE;
    const int MSIXCAP_ID_OFFSET;
    const int MSIXCAP_MXC_OFFSET;
    const int MSIXCAP_MTAB_OFFSET;
    const int MSIXCAP_MPBA_OFFSET;
    int MSIX_TABLE_OFFSET;
    int MSIX_TABLE_END;
    int MSIX_PBA_OFFSET;
    int MSIX_PBA_END;
    MSIXCAP msixcap;

    const int PXCAP_BASE;
    PXCAP pxcap;
    /** @} */

    /** MSIX Table and PBA Structures */
    std::vector<MSIXTable> msix_table;
    std::vector<MSIXPbaEntry> msix_pba;

    std::vector<PciBar *> BARs{};

    /**
     * Which base address register (if any) maps the given address?
     * @param addr The address to check.
     * @retval num The BAR number (0-5 inclusive),
     *             only valid if return value is true.
     * @retval offs The offset from the base address,
     *              only valid if return value is true.
     * @return True iff address maps to a base address register's region.
     */
    bool
    getBAR(Addr addr, int &num, Addr &offs)
    {
        for (int i = 0; i < BARs.size(); i++) {
            auto *bar = BARs[i];
            if (!bar || !bar->range().contains(addr))
                continue;
            num = i;
            offs = addr - bar->addr();
            return true;
        }
        return false;
    }

  public: // Host configuration interface
    /**
     * Write to the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param pkt packet containing the write the offset into config space
     */
    virtual Tick writeConfig(PacketPtr pkt);


    /**
     * Read from the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param pkt packet containing the write the offset into config space
     */
    virtual Tick readConfig(PacketPtr pkt);

  protected:
    PciHost::DeviceInterface hostInterface;

    Tick pioDelay;
    Tick configDelay;

  public:
    Addr
    pciToDma(Addr pci_addr) const
    {
        return hostInterface.dmaAddr(pci_addr);
    }

    void intrPost() { hostInterface.postInt(); }
    void intrClear() { hostInterface.clearInt(); }

    uint8_t
    interruptLine() const
    {
        return letoh(_config.common.interruptLine);
    }

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const override;

    /**
     * Constructor for PCI Dev. This function copies data from the
     * config file object PCIConfigData and registers the device with
     * a PciHost object.
     */
    PciDevice(const PciDeviceParams &params,
              std::initializer_list<PciBar *> BARs_init);

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    void serialize(CheckpointOut &cp) const override;

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(CheckpointIn &cp) override;

    const PciBusAddr &busAddr() const { return _busAddr; }
};

class PciEndpoint : public PciDevice
{
  protected:
    PCIConfigType0 &
    config()
    {
        return _config.type0;
    }

  public:
    /**
     * Constructor for PCI Dev. This function copies data from the
     * config file object PCIConfigData and registers the device with
     * a PciHost object.
     */
    PciEndpoint(const PciEndpointParams &params);

    /**
     * Write to the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param pkt packet containing the write the offset into config space
     */
    Tick writeConfig(PacketPtr pkt) override;

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(CheckpointIn &cp) override;
};

class PciBridge : public PciDevice
{
  protected:
    PCIConfigType1 &
    config()
    {
        return _config.type1;
    }

  public:
    /**
     * Constructor for PCI Dev. This function copies data from the
     * config file object PCIConfigData and registers the device with
     * a PciHost object.
     */
    PciBridge(const PciBridgeParams &params);

    /**
     * Write to the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param pkt packet containing the write the offset into config space
     */
    Tick writeConfig(PacketPtr pkt) override;

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_PCI_DEVICE_HH__
