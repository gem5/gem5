/*
 * Copyright (c) 2017,2019 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_STRUCTURES_DIRECTORYMEMORY_HH__
#define __MEM_RUBY_STRUCTURES_DIRECTORYMEMORY_HH__

#include <iostream>
#include <string>

#include "base/addr_range.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/DirectoryRequestType.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "params/RubyDirectoryMemory.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class DirectoryMemory : public SimObject
{
  public:
    typedef RubyDirectoryMemoryParams Params;
    DirectoryMemory(const Params &p);
    ~DirectoryMemory();

    void init();

    /**
     * Return the index in the directory based on an address
     *
     * This function transforms an address which belongs to a not
     * necessarily continuous vector of address ranges into a flat
     * address that we use to index in the directory
     *
     * @param an input address
     * @return the corresponding index in the directory
     *
     */
    uint64_t mapAddressToLocalIdx(Addr address);

    uint64_t
    getSize()
    {
        return m_size_bytes;
    }

    bool isPresent(Addr address);
    AbstractCacheEntry *lookup(Addr address);
    AbstractCacheEntry *allocate(Addr address, AbstractCacheEntry *new_entry);

    // Explicitly free up this address
    void deallocate(Addr address);

    void print(std::ostream &out) const;
    void recordRequestType(DirectoryRequestType requestType);

  private:
    // Private copy constructor and assignment operator
    DirectoryMemory(const DirectoryMemory &obj);
    DirectoryMemory &operator=(const DirectoryMemory &obj);

  private:
    const std::string m_name;
    AbstractCacheEntry **m_entries;
    // int m_size;  // # of memory module blocks this directory is
    // responsible for
    uint64_t m_size_bytes;
    uint64_t m_size_bits;
    uint64_t m_num_entries;

    /**
     * The address range for which the directory responds. Normally
     * this is all possible memory addresses.
     */
    const AddrRangeList addrRanges;
};

inline std::ostream &
operator<<(std::ostream &out, const DirectoryMemory &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_DIRECTORYMEMORY_HH__
