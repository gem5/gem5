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

/*
 * @todo
 * Should derive Tsunami from common platform class so PCI will work with
 * multiple platforms
 *
 * @file
 * PCI Config space implementation.
 */

#ifndef __PCICONFIGALL_HH__
#define __PCICONFIGALL_HH__

#include "mem/functional_mem/functional_memory.hh"
#include "dev/tsunami.hh"
#include "dev/pcireg.h"

#define MAX_PCI_DEV     32
#define MAX_PCI_FUNC    8

class PciDev;

/**
 * PCI Config Space
 * All of PCI config space needs to return -1 on Tsunami, except
 * the devices that exist. This device maps the entire bus config
 * space and passes the requests on to TsunamiPCIDev devices as
 * appropriate.
 */
class PCIConfigAll : public FunctionalMemory
{
  private:
    Addr addr;
    static const Addr size = 0xffffff;

  protected:

    /**
      * Pointer to the Tsunmi Object so we can communicate
      * to other Tsunami devices in need be.
      * @todo Make this more generic for multiple platforms
      */
    Tsunami *tsunami;

  public:
    /**
      * Pointers to all the devices that are registered with this
      * particular config space.
      */
    PciDev* devices[MAX_PCI_DEV][MAX_PCI_FUNC];

    /**
      * The default constructor.
      */
    PCIConfigAll(const std::string &name, Tsunami *t, Addr a,
                 MemoryController *mmu);

    virtual Fault read(MemReqPtr &req, uint8_t *data);
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif // __PCICONFIGALL_HH__
