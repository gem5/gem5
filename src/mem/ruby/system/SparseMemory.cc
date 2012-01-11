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

#include <queue>

#include "debug/RubyCache.hh"
#include "mem/ruby/system/SparseMemory.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

SparseMemory::SparseMemory(int number_of_levels)
{
    int even_level_bits;
    int extra;
    m_total_number_of_bits = RubySystem::getMemorySizeBits() 
        - RubySystem::getBlockSizeBits();;

    m_number_of_levels = number_of_levels;

    //
    // Create the array that describes the bits per level
    //
    m_number_of_bits_per_level = new int[m_number_of_levels];
    even_level_bits = m_total_number_of_bits / m_number_of_levels;
    extra = m_total_number_of_bits % m_number_of_levels;
    for (int level = 0; level < m_number_of_levels; level++) {
        if (level < extra)
            m_number_of_bits_per_level[level] = even_level_bits + 1;
        else
            m_number_of_bits_per_level[level] = even_level_bits;
    }
    m_map_head = new SparseMapType;

    m_total_adds = 0;
    m_total_removes = 0;
    m_adds_per_level = new uint64_t[m_number_of_levels];
    m_removes_per_level = new uint64_t[m_number_of_levels];
    for (int level = 0; level < m_number_of_levels; level++) {
        m_adds_per_level[level] = 0;
        m_removes_per_level[level] = 0;
    }
}

SparseMemory::~SparseMemory()
{
    recursivelyRemoveTables(m_map_head, 0);
    delete m_map_head;
    delete [] m_number_of_bits_per_level;
    delete [] m_adds_per_level;
    delete [] m_removes_per_level;
}

// Recursively search table hierarchy for the lowest level table.
// Delete the lowest table first, the tables above
void
SparseMemory::recursivelyRemoveTables(SparseMapType* curTable, int curLevel)
{
    SparseMapType::iterator iter;

    for (iter = curTable->begin(); iter != curTable->end(); iter++) {
        SparseMemEntry entry = (*iter).second;

        if (curLevel != (m_number_of_levels - 1)) {
            // If the not at the last level, analyze those lower level
            // tables first, then delete those next tables
            SparseMapType* nextTable = (SparseMapType*)(entry);
            recursivelyRemoveTables(nextTable, (curLevel + 1));
            delete nextTable;
        } else {
            // If at the last level, delete the directory entry
            delete (AbstractEntry*)(entry);
        }
        entry = NULL;
    }

    // Once all entries have been deleted, erase the entries
    curTable->erase(curTable->begin(), curTable->end());
}

// tests to see if an address is present in the memory
bool
SparseMemory::exist(const Address& address) const
{
    SparseMapType* curTable = m_map_head;
    Address curAddress;

    // Initiallize the high bit to be the total number of bits plus
    // the block offset.  However the highest bit index is one less
    // than this value.
    int highBit = m_total_number_of_bits + RubySystem::getBlockSizeBits();
    int lowBit;
    assert(address == line_address(address));
    DPRINTF(RubyCache, "address: %s\n", address);

    for (int level = 0; level < m_number_of_levels; level++) {
        // Create the appropriate sub address for this level
        // Note: that set Address is inclusive of the specified range,
        // thus the high bit is one less than the total number of bits
        // used to create the address.
        lowBit = highBit - m_number_of_bits_per_level[level];
        curAddress.setAddress(address.bitSelect(lowBit, highBit - 1));

        DPRINTF(RubyCache, "level: %d, lowBit: %d, highBit - 1: %d, "
                "curAddress: %s\n",
                level, lowBit, highBit - 1, curAddress);

        // Adjust the highBit value for the next level
        highBit -= m_number_of_bits_per_level[level];

        // If the address is found, move on to the next level.
        // Otherwise, return not found
        if (curTable->count(curAddress) != 0) {
            curTable = (SparseMapType*)((*curTable)[curAddress]);
        } else {
            DPRINTF(RubyCache, "Not found\n");
            return false;
        }
    }

    DPRINTF(RubyCache, "Entry found\n");
    return true;
}

// add an address to memory
void
SparseMemory::add(const Address& address, AbstractEntry* entry)
{
    assert(address == line_address(address));
    assert(!exist(address));

    m_total_adds++;

    Address curAddress;
    SparseMapType* curTable = m_map_head;

    // Initiallize the high bit to be the total number of bits plus
    // the block offset.  However the highest bit index is one less
    // than this value.
    int highBit = m_total_number_of_bits + RubySystem::getBlockSizeBits();
    int lowBit;
    void* newEntry = NULL;

    for (int level = 0; level < m_number_of_levels; level++) {
        // create the appropriate address for this level
        // Note: that set Address is inclusive of the specified range,
        // thus the high bit is one less than the total number of bits
        // used to create the address.
        lowBit = highBit - m_number_of_bits_per_level[level];
        curAddress.setAddress(address.bitSelect(lowBit, highBit - 1));

        // Adjust the highBit value for the next level
        highBit -= m_number_of_bits_per_level[level];

        // if the address exists in the cur table, move on.  Otherwise
        // create a new table.
        if (curTable->count(curAddress) != 0) {
            curTable = (SparseMapType*)((*curTable)[curAddress]);
        } else {
            m_adds_per_level[level]++;

            // if the last level, add a directory entry.  Otherwise add a map.
            if (level == (m_number_of_levels - 1)) {
                entry->getDataBlk().clear();
                newEntry = (void*)entry;
            } else {
                SparseMapType* tempMap = new SparseMapType;
                newEntry = (void*)(tempMap);
            }

            // Create the pointer container SparseMemEntry and add it
            // to the table.
            (*curTable)[curAddress] = newEntry;

            // Move to the next level of the heirarchy
            curTable = (SparseMapType*)newEntry;
        }
    }

    assert(exist(address));
    return;
}

// recursively search table hierarchy for the lowest level table.
// remove the lowest entry and any empty tables above it.
int
SparseMemory::recursivelyRemoveLevels(const Address& address,
                                      CurNextInfo& curInfo)
{
    Address curAddress;
    CurNextInfo nextInfo;
    SparseMemEntry entry;

    // create the appropriate address for this level
    // Note: that set Address is inclusive of the specified range,
    // thus the high bit is one less than the total number of bits
    // used to create the address.
    curAddress.setAddress(address.bitSelect(curInfo.lowBit,
                                            curInfo.highBit - 1));

    DPRINTF(RubyCache, "address: %s, curInfo.level: %d, curInfo.lowBit: %d, "
            "curInfo.highBit - 1: %d, curAddress: %s\n",
            address, curInfo.level, curInfo.lowBit,
            curInfo.highBit - 1, curAddress);

    assert(curInfo.curTable->count(curAddress) != 0);

    entry = (*(curInfo.curTable))[curAddress];

    if (curInfo.level < (m_number_of_levels - 1)) {
        // set up next level's info
        nextInfo.curTable = (SparseMapType*)(entry);
        nextInfo.level = curInfo.level + 1;

        nextInfo.highBit = curInfo.highBit -
            m_number_of_bits_per_level[curInfo.level];

        nextInfo.lowBit = curInfo.lowBit -
            m_number_of_bits_per_level[curInfo.level + 1];

        // recursively search the table hierarchy
        int tableSize = recursivelyRemoveLevels(address, nextInfo);

        // If this table below is now empty, we must delete it and
        // erase it from our table.
        if (tableSize == 0) {
            m_removes_per_level[curInfo.level]++;
            delete nextInfo.curTable;
            entry = NULL;
            curInfo.curTable->erase(curAddress);
        }
    } else {
        // if this is the last level, we have reached the Directory
        // Entry and thus we should delete it including the
        // SparseMemEntry container struct.
        delete (AbstractEntry*)(entry);
        entry = NULL;
        curInfo.curTable->erase(curAddress);
        m_removes_per_level[curInfo.level]++;
    }
    return curInfo.curTable->size();
}

// remove an entry from the table
void
SparseMemory::remove(const Address& address)
{
    assert(address == line_address(address));
    assert(exist(address));

    m_total_removes++;

    CurNextInfo nextInfo;

    // Initialize table pointer and level value
    nextInfo.curTable = m_map_head;
    nextInfo.level = 0;

    // Initiallize the high bit to be the total number of bits plus
    // the block offset.  However the highest bit index is one less
    // than this value.
    nextInfo.highBit = m_total_number_of_bits + RubySystem::getBlockSizeBits();
    nextInfo.lowBit = nextInfo.highBit - m_number_of_bits_per_level[0];;

    // recursively search the table hierarchy for empty tables
    // starting from the level 0.  Note we do not check the return
    // value because the head table is never deleted;
    recursivelyRemoveLevels(address, nextInfo);

    assert(!exist(address));
    return;
}

// looks an address up in memory
AbstractEntry*
SparseMemory::lookup(const Address& address)
{
    assert(address == line_address(address));

    Address curAddress;
    SparseMapType* curTable = m_map_head;
    AbstractEntry* entry = NULL;

    // Initiallize the high bit to be the total number of bits plus
    // the block offset.  However the highest bit index is one less
    // than this value.
    int highBit = m_total_number_of_bits + RubySystem::getBlockSizeBits();
    int lowBit;

    for (int level = 0; level < m_number_of_levels; level++) {
        // create the appropriate address for this level
        // Note: that set Address is inclusive of the specified range,
        // thus the high bit is one less than the total number of bits
        // used to create the address.
        lowBit = highBit - m_number_of_bits_per_level[level];
        curAddress.setAddress(address.bitSelect(lowBit, highBit - 1));

        DPRINTF(RubyCache, "level: %d, lowBit: %d, highBit - 1: %d, "
                "curAddress: %s\n",
                level, lowBit, highBit - 1, curAddress);

        // Adjust the highBit value for the next level
        highBit -= m_number_of_bits_per_level[level];

        // If the address is found, move on to the next level.
        // Otherwise, return not found
        if (curTable->count(curAddress) != 0) {
            curTable = (SparseMapType*)((*curTable)[curAddress]);
        } else {
            DPRINTF(RubyCache, "Not found\n");
            return NULL;
        }
    }

    // The last entry actually points to the Directory entry not a table
    entry = (AbstractEntry*)curTable;

    return entry;
}

void
SparseMemory::recordBlocks(int cntrl_id, CacheRecorder* tr) const
{
    queue<SparseMapType*> unexplored_nodes[2];
    queue<physical_address_t> address_of_nodes[2];

    unexplored_nodes[0].push(m_map_head);
    address_of_nodes[0].push(0);

    int parity_of_level = 0;
    physical_address_t address, temp_address;
    Address curAddress;

    // Initiallize the high bit to be the total number of bits plus
    // the block offset.  However the highest bit index is one less
    // than this value.
    int highBit = m_total_number_of_bits + RubySystem::getBlockSizeBits();
    int lowBit;

    for (int cur_level = 0; cur_level < m_number_of_levels; cur_level++) {

        // create the appropriate address for this level
        // Note: that set Address is inclusive of the specified range,
        // thus the high bit is one less than the total number of bits
        // used to create the address.
        lowBit = highBit - m_number_of_bits_per_level[cur_level];

        while (!unexplored_nodes[parity_of_level].empty()) {

            SparseMapType* node = unexplored_nodes[parity_of_level].front();
            unexplored_nodes[parity_of_level].pop();

            address = address_of_nodes[parity_of_level].front();
            address_of_nodes[parity_of_level].pop();

            SparseMapType::iterator iter;

            for (iter = node->begin(); iter != node->end(); iter++) {
                SparseMemEntry entry = (*iter).second;
                curAddress = (*iter).first;

                if (cur_level != (m_number_of_levels - 1)) {
                    // If not at the last level, put this node in the queue
                    unexplored_nodes[1 - parity_of_level].push(
                                                     (SparseMapType*)(entry));
                    address_of_nodes[1 - parity_of_level].push(address |
                                         (curAddress.getAddress() << lowBit));
                } else {
                    // If at the last level, add a trace record
                    temp_address = address | (curAddress.getAddress()
                                                                   << lowBit);
                    DataBlock block = ((AbstractEntry*)entry)->getDataBlk();
                    tr->addRecord(cntrl_id, temp_address, 0, RubyRequestType_ST, 0,
                                  block);
                }
            }
        }

        // Adjust the highBit value for the next level
        highBit -= m_number_of_bits_per_level[cur_level];
        parity_of_level = 1 - parity_of_level;
    }
}

void
SparseMemory::print(ostream& out) const
{
}

void
SparseMemory::printStats(ostream& out) const
{
    out << "total_adds: " << m_total_adds << " [";
    for (int level = 0; level < m_number_of_levels; level++) {
        out << m_adds_per_level[level] << " ";
    }
    out << "]" << endl;
    out << "total_removes: " << m_total_removes << " [";
    for (int level = 0; level < m_number_of_levels; level++) {
        out << m_removes_per_level[level] << " ";
    }
    out << "]" << endl;
}
