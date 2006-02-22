/*
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

#ifndef __DEV_PCIDEV_HH__
#define __DEV_PCIDEV_HH__

#include "dev/io_device.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"

#define BAR_IO_MASK 0x3
#define BAR_MEM_MASK 0xF
#define BAR_IO_SPACE_BIT 0x1
#define BAR_IO_SPACE(x) ((x) & BAR_IO_SPACE_BIT)
#define BAR_NUMBER(x) (((x) - PCI0_BASE_ADDR0) >> 0x2);

class PciConfigAll;
class MemoryController;


/**
 * This class encapulates the first 64 bytes of a singles PCI
 * devices config space that in configured by the configuration file.
 */
class PciConfigData : public SimObject
{
  public:
    /**
     * Constructor to initialize the devices config space to 0.
     */
    PciConfigData(const std::string &name)
        : SimObject(name)
    {
        memset(config.data, 0, sizeof(config.data));
        memset(BARAddrs, 0, sizeof(BARAddrs));
        memset(BARSize, 0, sizeof(BARSize));
    }

    /** The first 64 bytes */
    PCIConfig config;

    /** The size of the BARs */
    uint32_t BARSize[6];

    /** The addresses of the BARs */
    Addr BARAddrs[6];
};

/**
 * PCI device, base implemnation is only config space.
 * Each device is connected to a PCIConfigSpace device
 * which returns -1 for everything but the pcidevs that
 * register with it. This object registers with the PCIConfig space
 * object.
 */
class PciDev : public DmaDevice
{
  public:
    struct Params
    {
        std::string name;
        Platform *plat;
        MemoryController *mmu;

        /**
         * A pointer to the configspace all object that calls us when
         * a read comes to this particular device/function.
         */
        PciConfigAll *configSpace;

        /**
         * A pointer to the object that contains the first 64 bytes of
         * config space
         */
        PciConfigData *configData;

        /** The bus number we are on */
        uint32_t busNum;

        /** The device number we have */
        uint32_t deviceNum;

        /** The function number */
        uint32_t functionNum;
    };

  protected:
    Params *_params;

  public:
    const Params *params() const { return _params; }

  protected:
    /** The current config space. Unlike the PciConfigData this is
     * updated during simulation while continues to reflect what was
     * in the config file.
     */
    PCIConfig config;

    /** The size of the BARs */
    uint32_t BARSize[6];

    /** The current address mapping of the BARs */
    Addr BARAddrs[6];

    bool
    isBAR(Addr addr, int bar) const
    {
        assert(bar >= 0 && bar < 6);
        return BARAddrs[bar] <= addr && addr < BARAddrs[bar] + BARSize[bar];
    }

    int
    getBAR(Addr addr)
    {
        for (int i = 0; i <= 5; ++i)
            if (isBAR(addr, i))
                return i;

        return -1;
    }

    bool
    getBAR(Addr paddr, Addr &daddr, int &bar)
    {
        int b = getBAR(paddr);
        if (b < 0)
            return false;

        daddr = paddr - BARAddrs[b];
        bar = b;
        return true;
    }

  protected:
    Platform *plat;
    PciConfigData *configData;

  public:
    Addr pciToDma(Addr pciAddr) const
    { return plat->pciToDma(pciAddr); }

    void
    intrPost()
    { plat->postPciInt(configData->config.interruptLine); }

    void
    intrClear()
    { plat->clearPciInt(configData->config.interruptLine); }

    uint8_t
    interruptLine()
    { return configData->config.interruptLine; }

  public:
    /**
     * Constructor for PCI Dev. This function copies data from the
     * config file object PCIConfigData and registers the device with
     * a PciConfigAll object.
     */
    PciDev(Params *params);

    virtual Fault read(MemReqPtr &req, uint8_t *data);
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

  public:
    /**
     * Implement the read/write as BAR accesses
     */
    Fault readBar(MemReqPtr &req, uint8_t *data);
    Fault writeBar(MemReqPtr &req, const uint8_t *data);

  public:
    /**
     * Read from a specific BAR
     */
    virtual Fault readBar0(MemReqPtr &req, Addr daddr, uint8_t *data);
    virtual Fault readBar1(MemReqPtr &req, Addr daddr, uint8_t *data);
    virtual Fault readBar2(MemReqPtr &req, Addr daddr, uint8_t *data);
    virtual Fault readBar3(MemReqPtr &req, Addr daddr, uint8_t *data);
    virtual Fault readBar4(MemReqPtr &req, Addr daddr, uint8_t *data);
    virtual Fault readBar5(MemReqPtr &req, Addr daddr, uint8_t *data);

  public:
    /**
     * Write to a specific BAR
     */
    virtual Fault writeBar0(MemReqPtr &req, Addr daddr, const uint8_t *data);
    virtual Fault writeBar1(MemReqPtr &req, Addr daddr, const uint8_t *data);
    virtual Fault writeBar2(MemReqPtr &req, Addr daddr, const uint8_t *data);
    virtual Fault writeBar3(MemReqPtr &req, Addr daddr, const uint8_t *data);
    virtual Fault writeBar4(MemReqPtr &req, Addr daddr, const uint8_t *data);
    virtual Fault writeBar5(MemReqPtr &req, Addr daddr, const uint8_t *data);

  public:
    /**
     * Write to the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param offset the offset into config space
     * @param size the size of the write
     * @param data the data to write
     */
    virtual void writeConfig(int offset, int size, const uint8_t* data);


    /**
     * Read from the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param offset the offset into config space
     * @param size the size of the read
     * @param data pointer to the location where the read value should be stored
     */
    virtual void readConfig(int offset, int size, uint8_t *data);

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    virtual void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

inline Fault
PciDev::readBar(MemReqPtr &req, uint8_t *data)
{
    if (isBAR(req->paddr, 0))
        return readBar0(req, req->paddr - BARAddrs[0], data);
    if (isBAR(req->paddr, 1))
        return readBar1(req, req->paddr - BARAddrs[1], data);
    if (isBAR(req->paddr, 2))
        return readBar2(req, req->paddr - BARAddrs[2], data);
    if (isBAR(req->paddr, 3))
        return readBar3(req, req->paddr - BARAddrs[3], data);
    if (isBAR(req->paddr, 4))
        return readBar4(req, req->paddr - BARAddrs[4], data);
    if (isBAR(req->paddr, 5))
        return readBar5(req, req->paddr - BARAddrs[5], data);
    return MachineCheckFault;
}

inline Fault
PciDev::writeBar(MemReqPtr &req, const uint8_t *data)
{
    if (isBAR(req->paddr, 0))
        return writeBar0(req, req->paddr - BARAddrs[0], data);
    if (isBAR(req->paddr, 1))
        return writeBar1(req, req->paddr - BARAddrs[1], data);
    if (isBAR(req->paddr, 2))
        return writeBar2(req, req->paddr - BARAddrs[2], data);
    if (isBAR(req->paddr, 3))
        return writeBar3(req, req->paddr - BARAddrs[3], data);
    if (isBAR(req->paddr, 4))
        return writeBar4(req, req->paddr - BARAddrs[4], data);
    if (isBAR(req->paddr, 5))
        return writeBar5(req, req->paddr - BARAddrs[5], data);
    return MachineCheckFault;
}

#endif // __DEV_PCIDEV_HH__
