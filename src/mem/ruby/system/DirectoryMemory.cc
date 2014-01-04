/*
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

#include "base/intmath.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyStats.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/system/DirectoryMemory.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

int DirectoryMemory::m_num_directories = 0;
int DirectoryMemory::m_num_directories_bits = 0;
uint64_t DirectoryMemory::m_total_size_bytes = 0;
int DirectoryMemory::m_numa_high_bit = 0;

DirectoryMemory::DirectoryMemory(const Params *p)
    : SimObject(p)
{
    m_version = p->version;
    m_size_bytes = p->size;
    m_size_bits = floorLog2(m_size_bytes);
    m_num_entries = 0;
    m_use_map = p->use_map;
    m_map_levels = p->map_levels;
    m_numa_high_bit = p->numa_high_bit;
}

void
DirectoryMemory::init()
{
    m_num_entries = m_size_bytes / RubySystem::getBlockSizeBytes();

    if (m_use_map) {
        m_sparseMemory = new SparseMemory(m_map_levels);
        g_system_ptr->registerSparseMemory(m_sparseMemory);
    } else {
        m_entries = new AbstractEntry*[m_num_entries];
        for (int i = 0; i < m_num_entries; i++)
            m_entries[i] = NULL;
        m_ram = g_system_ptr->getMemoryVector();
    }

    m_num_directories++;
    m_num_directories_bits = ceilLog2(m_num_directories);
    m_total_size_bytes += m_size_bytes;

    if (m_numa_high_bit == 0) {
        m_numa_high_bit = RubySystem::getMemorySizeBits() - 1;
    }
    assert(m_numa_high_bit != 0);
}

DirectoryMemory::~DirectoryMemory()
{
    // free up all the directory entries
    if (m_entries != NULL) {
        for (uint64 i = 0; i < m_num_entries; i++) {
            if (m_entries[i] != NULL) {
                delete m_entries[i];
            }
        }
        delete [] m_entries;
    } else if (m_use_map) {
        delete m_sparseMemory;
    }
}

uint64
DirectoryMemory::mapAddressToDirectoryVersion(PhysAddress address)
{
    if (m_num_directories_bits == 0)
        return 0;

    uint64 ret = address.bitSelect(m_numa_high_bit - m_num_directories_bits + 1,
                                   m_numa_high_bit);
    return ret;
}

bool
DirectoryMemory::isPresent(PhysAddress address)
{
    bool ret = (mapAddressToDirectoryVersion(address) == m_version);
    return ret;
}

uint64
DirectoryMemory::mapAddressToLocalIdx(PhysAddress address)
{
    uint64 ret;
    if (m_num_directories_bits > 0) {
        ret = address.bitRemove(m_numa_high_bit - m_num_directories_bits + 1,
                                m_numa_high_bit);
    } else {
        ret = address.getAddress();
    }

    return ret >> (RubySystem::getBlockSizeBits());
}

AbstractEntry*
DirectoryMemory::lookup(PhysAddress address)
{
    assert(isPresent(address));
    DPRINTF(RubyCache, "Looking up address: %s\n", address);

    if (m_use_map) {
        return m_sparseMemory->lookup(address);
    } else {
        uint64_t idx = mapAddressToLocalIdx(address);
        assert(idx < m_num_entries);
        return m_entries[idx];
    }
}

AbstractEntry*
DirectoryMemory::allocate(const PhysAddress& address, AbstractEntry* entry)
{
    assert(isPresent(address));
    uint64 idx;
    DPRINTF(RubyCache, "Looking up address: %s\n", address);

    if (m_use_map) {
        m_sparseMemory->add(address, entry);
        entry->changePermission(AccessPermission_Read_Write);
    } else {
        idx = mapAddressToLocalIdx(address);
        assert(idx < m_num_entries);
        entry->getDataBlk().assign(m_ram->getBlockPtr(address));
        entry->changePermission(AccessPermission_Read_Only);
        m_entries[idx] = entry;
    }

    return entry;
}

void
DirectoryMemory::invalidateBlock(PhysAddress address)
{
    if (m_use_map) {
        assert(m_sparseMemory->exist(address));
        m_sparseMemory->remove(address);
    }
#if 0
    else {
        assert(isPresent(address));

        Index index = address.memoryModuleIndex();

        if (index < 0 || index > m_size) {
            ERROR_MSG("Directory Memory Assertion: "
                      "accessing memory out of range.");
        }

        if (m_entries[index] != NULL){
            delete m_entries[index];
            m_entries[index] = NULL;
        }
    }
#endif
}

void
DirectoryMemory::print(ostream& out) const
{
}

void
DirectoryMemory::regStats()
{
    if (m_use_map) {
        m_sparseMemory->regStats(name());
    }
}

void
DirectoryMemory::recordRequestType(DirectoryRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            DirectoryRequestType_to_string(requestType));
}

DirectoryMemory *
RubyDirectoryMemoryParams::create()
{
    return new DirectoryMemory(this);
}
