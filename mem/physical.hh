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
 */

/* @file
 */

#ifndef __PHYSICAL_MEMORY_HH__
#define __PHYSICAL_MEMORY_HH__

#include "base/range.hh"
#include "mem/memory.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"
#include <map>
#include <string>
//
// Functional model for a contiguous block of physical memory. (i.e. RAM)
//
class PhysicalMemory : public Memory
{
    class MemoryPort : public Port
    {
        PhysicalMemory *memory;

      public:

        MemoryPort(PhysicalMemory *_memory);

      protected:

        virtual bool recvTiming(Packet &pkt);

        virtual Tick recvAtomic(Packet &pkt);

        virtual void recvFunctional(Packet &pkt);

        virtual void recvStatusChange(Status status);

        virtual void getDeviceAddressRanges(AddrRangeList &range_list,
                                            bool &owner);

        virtual int deviceBlockSize();
    };

    std::map<std::string, MemoryPort*> memoryPortList;

    virtual Port * getPort(const char *if_name);

    virtual Port * addPort(std::string portName);

    int numPorts;

    int lat;

    struct MemResponseEvent : public Event
    {
        Packet &pkt;
        MemoryPort *memoryPort;

        MemResponseEvent(Packet &pkt, MemoryPort *memoryPort);
        void process();
        const char *description();
    };

  private:
    // prevent copying of a MainMemory object
    PhysicalMemory(const PhysicalMemory &specmem);
    const PhysicalMemory &operator=(const PhysicalMemory &specmem);

  protected:
    Addr base_addr;
    Addr pmem_size;
    uint8_t *pmem_addr;
    int page_ptr;

  public:
    Addr new_page();
    uint64_t size() { return pmem_size; }

  public:
    PhysicalMemory(const std::string &n);
    virtual ~PhysicalMemory();

  public:
    int deviceBlockSize();

    // fast back-door memory access for vtophys(), remote gdb, etc.
    // uint64_t phys_read_qword(Addr addr) const;
  private:
    bool doTimingAccess(Packet &pkt, MemoryPort *memoryPort);
    Tick doAtomicAccess(Packet &pkt);
    void doFunctionalAccess(Packet &pkt);

    void recvStatusChange(Port::Status status);

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

/*uint64_t
PhysicalMemory::phys_read_qword(Addr addr) const
{
    if (addr + sizeof(uint64_t) > pmem_size)
        return 0;

    return *(uint64_t *)(pmem_addr + addr);
}*/


#endif //__PHYSICAL_MEMORY_HH__
