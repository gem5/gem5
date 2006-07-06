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
 *
 * Authors: Andrew Schultz
 *          Ali Saidi
 */

/*
 * @file
 * PCI Config space implementation.
 */

#ifndef __PCICONFIGALL_HH__
#define __PCICONFIGALL_HH__

#include "dev/pcireg.h"
#include "base/range.hh"
#include "dev/io_device.hh"


static const uint32_t MAX_PCI_DEV = 32;
static const uint32_t MAX_PCI_FUNC = 8;

class PciDev;

/**
 * PCI Config Space
 * All of PCI config space needs to return -1 on Tsunami, except
 * the devices that exist. This device maps the entire bus config
 * space and passes the requests on to TsunamiPCIDev devices as
 * appropriate.
 */
class PciConfigAll : public BasicPioDevice
{
  private:
    /**
      * Pointers to all the devices that are registered with this
      * particular config space.
      */
    PciDev* devices[MAX_PCI_DEV][MAX_PCI_FUNC];

  public:
    /**
     * Constructor for PCIConfigAll
     * @param p parameters structure
     */
    PciConfigAll(Params *p);

    /**
     * Check if a device exists.
     * @param pcidev PCI device to check
     * @param pcifunc PCI function to check
     * @return true if device exists, false otherwise
     */
    bool deviceExists(uint32_t pcidev, uint32_t pcifunc)
                     { return devices[pcidev][pcifunc] != NULL ? true : false; }

    /**
     * Registers a device with the config space object.
     * @param pcidev PCI device to register
     * @param pcifunc PCI function to register
     * @param device device to register
     */
    void registerDevice(uint8_t pcidev, uint8_t pcifunc, PciDev *device)
                        { devices[pcidev][pcifunc] = device; }

    /**
     * Read something in PCI config space. If the device does not exist
     * -1 is returned, if the device does exist its PciDev::ReadConfig (or the
     * virtual function that overrides) it is called.
     * @param pkt Contains the address of the field to read.
     * @return Amount of time to do the read
     */
    virtual Tick read(Packet *pkt);

    /**
     * Write to PCI config spcae. If the device does not exit the simulator
     * panics. If it does it is passed on the PciDev::WriteConfig (or the virtual
     * function that overrides it).
     * @param req Contains the address to write to.
     * @param data The data to write.
     * @return The fault condition of the access.
     */

    virtual Tick write(Packet *pkt);

    /**
     * Start up function to check if more than one person is using an interrupt line
     * and print a warning if such a case exists
     */
    virtual void startup();

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

#endif // __PCICONFIGALL_HH__
