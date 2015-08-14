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

#include "mem/ruby/structures/PersistentTable.hh"

using namespace std;

// randomize so that handoffs are not locality-aware
#if 0
int persistent_randomize[] = {0, 4, 8, 12, 1, 5, 9, 13, 2, 6,
                              10, 14, 3, 7, 11, 15};
int persistent_randomize[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                              10, 11, 12, 13, 14, 15};
#endif

PersistentTable::PersistentTable()
{
}

PersistentTable::~PersistentTable()
{
}

void
PersistentTable::persistentRequestLock(Addr address,
                                       MachineID locker,
                                       AccessType type)
{
#if 0
    if (locker == m_chip_ptr->getID())
        cout << "Chip " << m_chip_ptr->getID() << ": " << llocker
             << " requesting lock for " << address << endl;

    MachineID locker = (MachineID) persistent_randomize[llocker];
#endif

    assert(address == makeLineAddress(address));

    static const PersistentTableEntry dflt;
    pair<AddressMap::iterator, bool> r =
        m_map.insert(AddressMap::value_type(address, dflt));
    bool present = !r.second;
    AddressMap::iterator i = r.first;
    PersistentTableEntry &entry = i->second;

    if (present) {
        // Make sure we're not already in the locked set
        assert(!(entry.m_starving.isElement(locker)));
    }

    entry.m_starving.add(locker);
    if (type == AccessType_Write)
        entry.m_request_to_write.add(locker);

    if (present)
        assert(entry.m_marked.isSubset(entry.m_starving));
}

void
PersistentTable::persistentRequestUnlock(Addr address,
                                         MachineID unlocker)
{
#if 0
    if (unlocker == m_chip_ptr->getID())
        cout << "Chip " << m_chip_ptr->getID() << ": " << uunlocker
             << " requesting unlock for " << address << endl;

    MachineID unlocker = (MachineID) persistent_randomize[uunlocker];
#endif

    assert(address == makeLineAddress(address));
    assert(m_map.count(address));
    PersistentTableEntry& entry = m_map[address];

    //
    // Make sure we're in the locked set
    //
    assert(entry.m_starving.isElement(unlocker));
    assert(entry.m_marked.isSubset(entry.m_starving));
    entry.m_starving.remove(unlocker);
    entry.m_marked.remove(unlocker);
    entry.m_request_to_write.remove(unlocker);
    assert(entry.m_marked.isSubset(entry.m_starving));

    // Deallocate if empty
    if (entry.m_starving.isEmpty()) {
        assert(entry.m_marked.isEmpty());
        m_map.erase(address);
    }
}

bool
PersistentTable::okToIssueStarving(Addr address,
                                   MachineID machId) const
{
    assert(address == makeLineAddress(address));

    AddressMap::const_iterator i = m_map.find(address);
    if (i == m_map.end()) {
        // No entry present
        return true;
    }

    const PersistentTableEntry &entry = i->second;

    if (entry.m_starving.isElement(machId)) {
        // We can't issue another lockdown until are previous unlock
        // has occurred
        return false;
    }

    return entry.m_marked.isEmpty();
}

MachineID
PersistentTable::findSmallest(Addr address) const
{
    assert(address == makeLineAddress(address));
    AddressMap::const_iterator i = m_map.find(address);
    assert(i != m_map.end());
    const PersistentTableEntry& entry = i->second;
    return entry.m_starving.smallestElement();
}

AccessType
PersistentTable::typeOfSmallest(Addr address) const
{
    assert(address == makeLineAddress(address));
    AddressMap::const_iterator i = m_map.find(address);
    assert(i != m_map.end());
    const PersistentTableEntry& entry = i->second;
    if (entry.m_request_to_write.
        isElement(entry.m_starving.smallestElement())) {
        return AccessType_Write;
    } else {
        return AccessType_Read;
    }
}

void
PersistentTable::markEntries(Addr address)
{
    assert(address == makeLineAddress(address));
    AddressMap::iterator i = m_map.find(address);
    if (i == m_map.end())
        return;

    PersistentTableEntry& entry = i->second;

    // None should be marked
    assert(entry.m_marked.isEmpty());

    // Mark all the nodes currently in the table
    entry.m_marked = entry.m_starving;
}

bool
PersistentTable::isLocked(Addr address) const
{
    assert(address == makeLineAddress(address));

    // If an entry is present, it must be locked
    return m_map.count(address) > 0;
}

int
PersistentTable::countStarvingForAddress(Addr address) const
{
    assert(address == makeLineAddress(address));
    AddressMap::const_iterator i = m_map.find(address);
    if (i == m_map.end())
        return 0;

    const PersistentTableEntry& entry = i->second;
    return entry.m_starving.count();
}

int
PersistentTable::countReadStarvingForAddress(Addr address) const
{
    assert(address == makeLineAddress(address));
    AddressMap::const_iterator i = m_map.find(address);
    if (i == m_map.end())
        return 0;

    const PersistentTableEntry& entry = i->second;
    return entry.m_starving.count() - entry.m_request_to_write.count();
}

void
PersistentTable::print(ostream& out) const
{
}

