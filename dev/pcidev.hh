/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
  protected:
    struct Params;
    Params *_params;

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
    const Params *params() const { return _params; }

  protected:
    /** The current config space. Unlike the PciConfigData this is
     * updated during simulation while continues to refelect what was
     * in the config file.
     */
    PCIConfig config;

    /** The size of the BARs */
    uint32_t BARSize[6];

    /** The current address mapping of the BARs */
    Addr BARAddrs[6];

  protected:
    Platform *plat;
    PciConfigData *configData;

  public:
    Addr pciToDma(Addr pciAddr) const
    { return plat->pciToDma(pciAddr); }

    void
    intrPost()
    { plat->postPciInt(configData->config.hdr.pci0.interruptLine); }

    void
    intrClear()
    { plat->clearPciInt(configData->config.hdr.pci0.interruptLine); }

  public:
    /**
     * Constructor for PCI Dev. This function copies data from the
     * config file object PCIConfigData and registers the device with
     * a PciConfigAll object.
     */
    PciDev(Params *params);

    virtual Fault read(MemReqPtr &req, uint8_t *data) {
        return No_Fault;
    }
    virtual Fault write(MemReqPtr &req, const uint8_t *data) {
        return No_Fault;
    }

    /**
     * Write to the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param offset the offset into config space
     * @param size the size of the write
     * @param data the data to write
     */
    virtual void WriteConfig(int offset, int size, uint32_t data);


    /**
     * Read from the PCI config space data that is stored locally. This may be
     * overridden by the device but at some point it will eventually call this
     * for normal operations that it does not need to override.
     * @param offset the offset into config space
     * @param size the size of the read
     * @param data pointer to the location where the read value should be stored
     */
    virtual void ReadConfig(int offset, int size, uint8_t *data);

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

#endif // __DEV_PCIDEV_HH__
