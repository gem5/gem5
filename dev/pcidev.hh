/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 * PCI configspace devices
 */

#ifndef __PCI_DEV_HH__
#define __PCI_DEV_HH__

#include "dev/pcireg.h"
#include "sim/sim_object.hh"
#include "mem/functional_mem/functional_memory.hh"

class PciConfigAll;
class MemoryController;

class PciConfigData : public SimObject
{
  public:
    PciConfigData(const std::string &name)
        : SimObject(name)
    {
        memset(config.data, 0, sizeof(config.data));
        memset(BARAddrs, 0, sizeof(BARAddrs));
        memset(BARSize, 0, sizeof(BARSize));
    }

    PCIConfig config;
    uint32_t BARSize[6];
    Addr BARAddrs[6];
};

/**
 * PCI device, base implemnation is only config space.
 * Each device is connected to a PCIConfigSpace device
 * which returns -1 for everything but the pcidevs that
 * register with it. This object registers with the PCIConfig space
 * object.
 */
class PciDev : public FunctionalMemory
{
  protected:
    MemoryController *MMU;
    PciConfigAll *configSpace;
    PciConfigData *configData;
    uint32_t bus;
    uint32_t device;
    uint32_t function;

    PCIConfig config;
    uint32_t BARSize[6];
    Addr BARAddrs[6];

  public:
    PciDev(const std::string &name, MemoryController *mmu, PciConfigAll *cf,
           PciConfigData *cd, uint32_t bus, uint32_t dev, uint32_t func);

    virtual Fault read(MemReqPtr &req, uint8_t *data) {
        return No_Fault;
    }
    virtual Fault write(MemReqPtr &req, const uint8_t *data) {
        return No_Fault;
    }

    virtual void WriteConfig(int offset, int size, uint32_t data);
    virtual void ReadConfig(int offset, int size, uint8_t *data);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __PCI_DEV_HH__
