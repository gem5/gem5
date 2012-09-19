/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Hansson
 */

#ifndef __PHYSICAL_MEMORY_HH__
#define __PHYSICAL_MEMORY_HH__

#include "base/addr_range_map.hh"
#include "mem/abstract_mem.hh"
#include "mem/packet.hh"

/**
 * The physical memory encapsulates all memories in the system and
 * provides basic functionality for accessing those memories without
 * going through the memory system and interconnect.
 */
class PhysicalMemory
{

  private:

    // Global address map
    AddrRangeMap<AbstractMemory*> addrMap;

    // a mutable cache for the last range that matched an address
    mutable AddrRange rangeCache;

    // All address-mapped memories
    std::vector<AbstractMemory*> memories;

    // The total memory size
    uint64_t size;

    // Prevent copying
    PhysicalMemory(const PhysicalMemory&);

    // Prevent assignment
    PhysicalMemory& operator=(const PhysicalMemory&);

  public:

    /**
     * Create a physical memory object, wrapping a number of memories.
     */
    PhysicalMemory(const std::vector<AbstractMemory*>& _memories);

    /**
     * Nothing to destruct.
     */
    ~PhysicalMemory() { }

    /**
     * Check if a physical address is within a range of a memory that
     * is part of the global address map.
     *
     * @param addr A physical address
     * @return Whether the address corresponds to a memory
     */
    bool isMemAddr(Addr addr) const;

    /**
     * Get the memory ranges for all memories that are to be reported
     * to the configuration table.
     *
     * @return All configuration table memory ranges
     */
    AddrRangeList getConfAddrRanges() const;

    /**
     * Get the total physical memory size.
     *
     * @return The sum of all memory sizes
     */
    uint64_t totalSize() const { return size; }

    /**
     *
     */
    void access(PacketPtr pkt);
    void functionalAccess(PacketPtr pkt);
};




#endif //__PHYSICAL_MEMORY_HH__
