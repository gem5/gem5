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

#ifndef __MEM_RUBY_SYSTEM_DIRECTORYMEMORY_HH__
#define __MEM_RUBY_SYSTEM_DIRECTORYMEMORY_HH__

#include <iostream>
#include <string>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/slicc_interface/AbstractEntry.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/system/SparseMemory.hh"
#include "params/RubyDirectoryMemory.hh"
#include "sim/sim_object.hh"

class DirectoryMemory : public SimObject
{
  public:
    typedef RubyDirectoryMemoryParams Params;
    DirectoryMemory(const Params *p);
    ~DirectoryMemory();

    void init();

    uint64 mapAddressToLocalIdx(PhysAddress address);
    static uint64 mapAddressToDirectoryVersion(PhysAddress address);

    bool isSparseImplementation() { return m_use_map; }
    uint64 getSize() { return m_size_bytes; }

    void printConfig(std::ostream& out) const;
    static void printGlobalConfig(std::ostream & out);
    bool isPresent(PhysAddress address);
    AbstractEntry* lookup(PhysAddress address);
    AbstractEntry* allocate(const PhysAddress& address,
                            AbstractEntry* new_entry);

    void invalidateBlock(PhysAddress address);

    void print(std::ostream& out) const;
    void printStats(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    DirectoryMemory(const DirectoryMemory& obj);
    DirectoryMemory& operator=(const DirectoryMemory& obj);

  private:
    const std::string m_name;
    AbstractEntry **m_entries;
    // int m_size;  // # of memory module blocks this directory is
                    // responsible for
    uint64 m_size_bytes;
    uint64 m_size_bits;
    uint64 m_num_entries;
    int m_version;

    static int m_num_directories;
    static int m_num_directories_bits;
    static uint64_t m_total_size_bytes;
    static int m_numa_high_bit;

    MemoryVector* m_ram;
    SparseMemory* m_sparseMemory;
    bool m_use_map;
    int m_map_levels;
};

inline std::ostream&
operator<<(std::ostream& out, const DirectoryMemory& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SYSTEM_DIRECTORYMEMORY_HH__
