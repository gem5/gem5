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

#include "mem/functional_mem/mmap_device.hh"
#include "dev/pcireg.h"
class PCIConfigAll;

/*
 * PCI device configuration device.
 * Each device is connected to a PCIConfigSpace device
 * which returns -1 for everything but the pcidevs that
 * register with it. This object registers with the PCIConfig space
 * object.
 */
class PciDev : public MMapDevice
{
  private:
    uint32_t Bus;
    uint32_t Device;
    uint32_t Function;
  public:
    PciDev(const std::string &name, PCIConfigAll *cf, uint32_t bus,
           uint32_t dev, uint32_t func);

    PCIConfigAll *ConfigSpace;
    PCIConfig config;
    uint32_t BARSize[6];

    virtual void WriteConfig(int offset, int size, uint32_t data);
    virtual void ReadConfig(int offset, int size, uint8_t *data);


    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __PCI_DEV_HH__
