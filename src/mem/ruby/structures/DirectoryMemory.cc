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
 * Copyright (c) 2017 Google Inc.
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

#include "mem/ruby/structures/DirectoryMemory.hh"

#include "base/addr_range.hh"
#include "base/intmath.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyStats.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

DirectoryMemory::DirectoryMemory(const Params &p)
    : SimObject(p), addrRanges(p.addr_ranges.begin(), p.addr_ranges.end())
{
    m_size_bytes = 0;
    for (const auto &r: addrRanges) {
        m_size_bytes += r.size();
    }
    m_size_bits = floorLog2(m_size_bytes);
    m_num_entries = 0;
}

void
DirectoryMemory::init()
{
    m_num_entries = m_size_bytes / m_block_size;
    m_entries = new AbstractCacheEntry*[m_num_entries];
    for (int i = 0; i < m_num_entries; i++)
        m_entries[i] = NULL;
}

DirectoryMemory::~DirectoryMemory()
{
    // free up all the directory entries
    for (uint64_t i = 0; i < m_num_entries; i++) {
        if (m_entries[i] != NULL) {
            delete m_entries[i];
        }
    }
    delete [] m_entries;
}

bool
DirectoryMemory::isPresent(Addr address)
{
    for (const auto& r: addrRanges) {
        if (r.contains(address)) {
            return true;
        }
    }
    return false;
}

uint64_t
DirectoryMemory::mapAddressToLocalIdx(Addr address)
{
    uint64_t ret = 0;
    for (const auto& r: addrRanges) {
        if (r.contains(address)) {
            ret += r.getOffset(address);
            break;
        }
        ret += r.size();
    }
    return ret >> RubySystem::getBlockSizeBits();
}

AbstractCacheEntry*
DirectoryMemory::lookup(Addr address)
{
    assert(isPresent(address));
    DPRINTF(RubyCache, "Looking up address: %#x\n", address);

    uint64_t idx = mapAddressToLocalIdx(address);
    assert(idx < m_num_entries);
    return m_entries[idx];
}

AbstractCacheEntry*
DirectoryMemory::allocate(Addr address, AbstractCacheEntry *entry)
{
    assert(isPresent(address));
    uint64_t idx;
    DPRINTF(RubyCache, "Looking up address: %#x\n", address);

    idx = mapAddressToLocalIdx(address);
    assert(idx < m_num_entries);
    assert(m_entries[idx] == NULL);
    entry->changePermission(AccessPermission_Read_Only);
    m_entries[idx] = entry;

    return entry;
}

void
DirectoryMemory::deallocate(Addr address)
{
    assert(isPresent(address));
    uint64_t idx;
    DPRINTF(RubyCache, "Removing entry for address: %#x\n", address);

    idx = mapAddressToLocalIdx(address);
    assert(idx < m_num_entries);
    assert(m_entries[idx] != NULL);
    delete m_entries[idx];
    m_entries[idx] = NULL;
}

void
DirectoryMemory::print(std::ostream& out) const
{
}

void
DirectoryMemory::recordRequestType(DirectoryRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            DirectoryRequestType_to_string(requestType));
}

} // namespace ruby
} // namespace gem5
