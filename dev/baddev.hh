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
 * This devices just panics when touched. For example if you have a
 * kernel that touches the frame buffer which isn't allowed.
 */

#ifndef __DEV_BADDEV_HH__
#define __DEV_BADDEV_HH__

#include "base/range.hh"
#include "dev/io_device.hh"

/**
 * BadDevice
 * This device just panics when accessed. It is supposed to warn
 * the user that the kernel they are running has unsupported
 * options (i.e. frame buffer)
 */
class BadDevice : public PioDevice
{
  private:
    Addr addr;
    static const Addr size = 0xf;

    std::string devname;

  public:
     /**
      * Constructor for the Baddev Class.
      * @param name name of the object
      * @param a base address of the write
      * @param mmu the memory controller
      * @param hier object to store parameters universal the device hierarchy
      * @param bus The bus that this device is attached to
      * @param devicename device that is not implemented
      */
    BadDevice(const std::string &name, Addr a, MemoryController *mmu,
              HierParams *hier, Bus *bus, const std::string &devicename);

    /**
      * On a read event we just panic aand hopefully print a
      * meaningful error message.
      * @param req Contains the address to read from.
      * @param data A pointer to write the read data to.
      * @return The fault condition of the access.
      */
    virtual Fault read(MemReqPtr &req, uint8_t *data);

    /**
      * On a write event we just panic aand hopefully print a
      * meaningful error message.
      * @param req Contains the address to write to.
      * @param data The data to write.
      * @return The fault condition of the access.
      */
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    /**
     * Return how long this access will take.
     * @param req the memory request to calcuate
     * @return Tick when the request is done
     */
    Tick cacheAccess(MemReqPtr &req);
};

#endif // __DEV_BADDEV_HH__
