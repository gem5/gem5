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

/** @file
 * Tsunami PCI interface CSRs
 */

#ifndef __TSUNAMI_PCHIP_HH__
#define __TSUNAMI_PCHIP_HH__

#include "dev/tsunami.hh"
#include "base/range.hh"
#include "dev/io_device.hh"

/**
 * A very simple implementation of the Tsunami PCI interface chips.
 */
class TsunamiPChip : public PioDevice
{
  private:
    /** The base address of this device */
    Addr addr;

    /** The size of mappad from the above address */
    static const Addr size = 0xfff;

  protected:
    /**
     * pointer to the tsunami object.
     * This is our access to all the other tsunami
     * devices.
     */
    Tsunami *tsunami;

    /** Pchip control register */
    uint64_t pctl;

    /** Window Base addresses */
    uint64_t wsba[4];

    /** Window masks */
    uint64_t wsm[4];

    /** Translated Base Addresses */
    uint64_t tba[4];

  public:
    /**
     * Register the PChip with the mmu and init all wsba, wsm, and tba to 0
     * @param name the name of thes device
     * @param t a pointer to the tsunami device
     * @param a the address which we respond to
     * @param mmu the mmu we are to register with
     * @param hier object to store parameters universal the device hierarchy
     * @param bus The bus that this device is attached to
     */
    TsunamiPChip(const std::string &name, Tsunami *t, Addr a,
                 MemoryController *mmu, HierParams *hier, Bus *bus,
                 Tick pio_latency);

    /**
     * Translate a PCI bus address to a memory address for DMA.
     * @todo Andrew says this needs to be fixed. What's wrong with it?
     * @param busAddr PCI address to translate.
     * @return memory system address
     */
    Addr translatePciToDma(Addr busAddr);

     /**
      * Process a read to the PChip.
      * @param req Contains the address to read from.
      * @param data A pointer to write the read data to.
      * @return The fault condition of the access.
      */
    virtual Fault read(MemReqPtr &req, uint8_t *data);

    /**
      * Process a write to the PChip.
      * @param req Contains the address to write to.
      * @param data The data to write.
      * @return The fault condition of the access.
      */
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

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

    /**
     * Return how long this access will take.
     * @param req the memory request to calcuate
     * @return Tick when the request is done
     */
    Tick cacheAccess(MemReqPtr &req);
};

#endif // __TSUNAMI_PCHIP_HH__
