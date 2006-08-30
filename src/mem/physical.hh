/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 */

/* @file
 */

#ifndef __PHYSICAL_MEMORY_HH__
#define __PHYSICAL_MEMORY_HH__

#include "base/range.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/tport.hh"
#include "sim/eventq.hh"
#include <map>
#include <string>

//
// Functional model for a contiguous block of physical memory. (i.e. RAM)
//
class PhysicalMemory : public MemObject
{
    class MemoryPort : public SimpleTimingPort
    {
        PhysicalMemory *memory;

      public:

        MemoryPort(const std::string &_name, PhysicalMemory *_memory);

      protected:

        virtual Tick recvAtomic(Packet *pkt);

        virtual void recvFunctional(Packet *pkt);

        virtual void recvStatusChange(Status status);

        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop);

        virtual int deviceBlockSize();
    };

    int numPorts;


  private:
    // prevent copying of a MainMemory object
    PhysicalMemory(const PhysicalMemory &specmem);
    const PhysicalMemory &operator=(const PhysicalMemory &specmem);

  protected:
    uint8_t *pmemAddr;
    MemoryPort *port;
    int pagePtr;
    Tick lat;

  public:
    Addr new_page();
    uint64_t size() { return params()->addrRange.size(); }

    struct Params
    {
        std::string name;
        Range<Addr> addrRange;
        Tick latency;
    };

  protected:
    Params *_params;

  public:
    const Params *params() const { return _params; }
    PhysicalMemory(Params *p);
    virtual ~PhysicalMemory();

  public:
    int deviceBlockSize();
    void getAddressRanges(AddrRangeList &resp, AddrRangeList &snoop);
    virtual Port *getPort(const std::string &if_name, int idx = -1);
    void virtual init();
    unsigned int drain(Event *de);

  protected:
    void doFunctionalAccess(Packet *pkt);
    virtual Tick calculateLatency(Packet *pkt);
    void recvStatusChange(Port::Status status);

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif //__PHYSICAL_MEMORY_HH__
