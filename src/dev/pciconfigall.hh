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

#include "dev/io_device.hh"
#include "dev/pcireg.h"
#include "params/PciConfigAll.hh"

/**
 * PCI Config Space
 * All of PCI config space needs to return -1 on Tsunami, except
 * the devices that exist. This device maps the entire bus config
 * space and passes the requests on to TsunamiPCIDevice devices as
 * appropriate.
 */
class PciConfigAll : public BasicPioDevice
{
  public:
    typedef PciConfigAllParams Params;
    const Params *params() const { return (const Params *)_params; }

    /**
     * Constructor for PCIConfigAll
     * @param p parameters structure
     */
    PciConfigAll(const Params *p);

    /**
     * Read something in PCI config space. If the device does not exist
     * -1 is returned, if the device does exist its PciDevice::ReadConfig
     * (or the virtual function that overrides) it is called.
     * @param pkt Contains information about the read operation
     * @return Amount of time to do the read
     */
    virtual Tick read(PacketPtr pkt);

    /**
     * Write to PCI config spcae. If the device does not exit the simulator
     * panics. If it does it is passed on the PciDevice::WriteConfig (or
     * the virtual function that overrides it).
     * @param pkt Contains information about the write operation
     * @return Amount of time to do the read
     */

    virtual Tick write(PacketPtr pkt);
};

#endif // __PCICONFIGALL_HH__
