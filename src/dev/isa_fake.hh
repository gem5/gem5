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
 * Authors: Ali Saidi
 */

/** @file
 * Declaration of a fake device.
 */

#ifndef __ISA_FAKE_HH__
#define __ISA_FAKE_HH__

#include <string>

#include "dev/io_device.hh"
// #include "dev/alpha/tsunami.hh"
#include "mem/packet.hh"
#include "params/IsaFake.hh"

/**
 * IsaFake is a device that returns, BadAddr, 1 or 0 on all reads and
 *  rites. It is meant to be placed at an address range
 * so that an mcheck doesn't occur when an os probes a piece of hw
 * that doesn't exist (e.g. UARTs1-3), or catch requests in the memory system
 * that have no responders..
 */
class IsaFake : public BasicPioDevice
{
  protected:
    uint8_t retData8;
    uint16_t retData16;
    uint32_t retData32;
    uint64_t retData64;

  public:
    typedef IsaFakeParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    /**
      * The constructor for Isa Fake just registers itself with the MMU.
      * @param p params structure
      */
    IsaFake(Params *p);

    /**
     * This read always returns -1.
     * @param pkt The memory request.
     * @param data Where to put the data.
     */
    virtual Tick read(PacketPtr pkt);

    /**
     * All writes are simply ignored.
     * @param pkt The memory request.
     * @param data the data to not write.
     */
    virtual Tick write(PacketPtr pkt);
};

#endif // __ISA_FAKE_HH__
