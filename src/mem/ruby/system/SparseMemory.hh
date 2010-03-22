
/*
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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


#ifndef SPARSEMEMORY_H
#define SPARSEMEMORY_H

#include "mem/ruby/common/Global.hh"
#include "base/hashmap.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/protocol/Directory_Entry.hh"

typedef struct SparseMemEntry {
    void* entry;
} SparseMemEntry_t;

typedef m5::hash_map<Address, SparseMemEntry_t> SparseMapType;

typedef struct curNextInfo {
    SparseMapType* curTable;
    int level;
    int highBit;
    int lowBit;
};

class SparseMemory {
  public:

    // Constructors
    SparseMemory(int number_of_bits, int number_of_levels);
    
    // Destructor
    ~SparseMemory();
    
    // Public Methods
    
    void printConfig(ostream& out) { }
    
    bool exist(const Address& address) const;
    void add(const Address& address);
    void remove(const Address& address);
    
    Directory_Entry* lookup(const Address& address);
    
    // Print cache contents
    void print(ostream& out) const;
    void printStats(ostream& out) const;

  private:
    // Private Methods
    
    // Private copy constructor and assignment operator
    SparseMemory(const SparseMemory& obj);
    SparseMemory& operator=(const SparseMemory& obj);
    
    // Used by destructor to recursively remove all tables
    void recursivelyRemoveTables(SparseMapType* currentTable, int level);
    
    // recursive search for address and remove associated entries
    int recursivelyRemoveLevels(const Address& address, curNextInfo& curInfo);
    
    // Data Members (m_prefix)
    SparseMapType* m_map_head;
    
    int m_total_number_of_bits;
    int m_number_of_levels;
    int* m_number_of_bits_per_level;
    
    uint64_t m_total_adds;
    uint64_t m_total_removes;
    uint64_t* m_adds_per_level;
    uint64_t* m_removes_per_level;
};

// Output operator declaration
ostream& operator<<(ostream& out, const SparseMemEntry& obj);

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const SparseMemEntry& obj)
{
    out << "SparseMemEntry";
    out << flush;
    return out;
}


#endif //SPARSEMEMORY_H
