/*
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
 * Copyright (c) 2012 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SYSTEM_SPARSEMEMORY_HH__
#define __MEM_RUBY_SYSTEM_SPARSEMEMORY_HH__

#include <iostream>
#include <string>

#include "base/hashmap.hh"
#include "base/statistics.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/slicc_interface/AbstractEntry.hh"
#include "mem/ruby/system/CacheRecorder.hh"

typedef void* SparseMemEntry;
typedef m5::hash_map<Address, SparseMemEntry> SparseMapType;

struct CurNextInfo
{
    SparseMapType* curTable;
    int level;
    int highBit;
    int lowBit;
};

class SparseMemory
{
  public:
    SparseMemory(int number_of_levels);
    ~SparseMemory();

    bool exist(const Address& address) const;
    void add(const Address& address, AbstractEntry*);
    void remove(const Address& address);

    /*!
     * Function for recording the contents of memory. This function walks
     * through all the levels of the sparse memory in a breadth first
     * fashion. This might need more memory than a depth first approach.
     * But breadth first seems easier to me than a depth first approach.
     */
    void recordBlocks(int cntrl_id, CacheRecorder *) const;

    AbstractEntry* lookup(const Address& address);
    void regStats(const std::string &name);

  private:
    // Private copy constructor and assignment operator
    SparseMemory(const SparseMemory& obj);
    SparseMemory& operator=(const SparseMemory& obj);

    // Used by destructor to recursively remove all tables
    void recursivelyRemoveTables(SparseMapType* currentTable, int level);

    // recursive search for address and remove associated entries
    int recursivelyRemoveLevels(const Address& address, CurNextInfo& curInfo);

    // Data Members (m_prefix)
    SparseMapType* m_map_head;

    int m_total_number_of_bits;
    int m_number_of_levels;
    int* m_number_of_bits_per_level;

    Stats::Scalar m_total_adds;
    Stats::Vector m_adds_per_level;
    Stats::Scalar m_total_removes;
    Stats::Vector m_removes_per_level;
};

#endif // __MEM_RUBY_SYSTEM_SPARSEMEMORY_HH__
