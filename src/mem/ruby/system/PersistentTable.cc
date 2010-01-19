
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

#include "mem/ruby/system/PersistentTable.hh"
#include "mem/gems_common/util.hh"

// randomize so that handoffs are not locality-aware
// int persistent_randomize[] = {0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15};
// int persistent_randomize[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};


PersistentTable::PersistentTable()
{
  m_map_ptr = new Map<Address, PersistentTableEntry>;
}

PersistentTable::~PersistentTable()
{
  delete m_map_ptr;
  m_map_ptr = NULL;
}  

void PersistentTable::persistentRequestLock(const Address& address, 
                                            MachineID locker, 
                                            AccessType type)
{

  // if (locker == m_chip_ptr->getID()  )
  // cout << "Chip " << m_chip_ptr->getID() << ": " << llocker 
  //      << " requesting lock for " << address << endl;

  // MachineID locker = (MachineID) persistent_randomize[llocker];
 
  assert(address == line_address(address));
  if (!m_map_ptr->exist(address)) {
    // Allocate if not present
    PersistentTableEntry entry;
    entry.m_starving.add(locker);
    if (type == AccessType_Write) {
      entry.m_request_to_write.add(locker);
    }
    m_map_ptr->add(address, entry);
  } else {
    PersistentTableEntry& entry = m_map_ptr->lookup(address);

    //
    // Make sure we're not already in the locked set
    //
    assert(!(entry.m_starving.isElement(locker)));

    entry.m_starving.add(locker);
    if (type == AccessType_Write) {
      entry.m_request_to_write.add(locker);
    }
    assert(entry.m_marked.isSubset(entry.m_starving));
  }
}

void PersistentTable::persistentRequestUnlock(const Address& address, 
                                              MachineID unlocker)
{
  // if (unlocker == m_chip_ptr->getID() )
  // cout << "Chip " << m_chip_ptr->getID() << ": " << uunlocker 
  //      << " requesting unlock for " << address << endl;

  // MachineID unlocker = (MachineID) persistent_randomize[uunlocker];

  assert(address == line_address(address));
  assert(m_map_ptr->exist(address));
  PersistentTableEntry& entry = m_map_ptr->lookup(address);

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
    m_map_ptr->erase(address);
  }
}

bool PersistentTable::okToIssueStarving(const Address& address, 
                                        MachineID machId) const
{
  assert(address == line_address(address));
  if (!m_map_ptr->exist(address)) {
    //
    // No entry present
    //
    return true; 
  } else if (m_map_ptr->lookup(address).m_starving.isElement(machId)) {
    //
    // We can't issue another lockdown until are previous unlock has occurred
    //
    return false; 
  } else {
    return (m_map_ptr->lookup(address).m_marked.isEmpty());
  }
}

MachineID PersistentTable::findSmallest(const Address& address) const
{
  assert(address == line_address(address));
  assert(m_map_ptr->exist(address));
  const PersistentTableEntry& entry = m_map_ptr->lookup(address);
  return entry.m_starving.smallestElement();
}

AccessType PersistentTable::typeOfSmallest(const Address& address) const
{
  assert(address == line_address(address));
  assert(m_map_ptr->exist(address));
  const PersistentTableEntry& entry = m_map_ptr->lookup(address);
  if (entry.m_request_to_write.isElement(entry.m_starving.smallestElement())) {
    return AccessType_Write;
  } else {
    return AccessType_Read;
  }
}

void PersistentTable::markEntries(const Address& address)
{
  assert(address == line_address(address));
  if (m_map_ptr->exist(address)) {
    PersistentTableEntry& entry = m_map_ptr->lookup(address);

    //
    // None should be marked
    //
    assert(entry.m_marked.isEmpty());  

    //
    // Mark all the nodes currently in the table
    //
    entry.m_marked = entry.m_starving; 
  }
}

bool PersistentTable::isLocked(const Address& address) const
{
  assert(address == line_address(address));
  // If an entry is present, it must be locked
  return (m_map_ptr->exist(address));
}

int PersistentTable::countStarvingForAddress(const Address& address) const
{
  if (m_map_ptr->exist(address)) {
    PersistentTableEntry& entry = m_map_ptr->lookup(address);
    return (entry.m_starving.count());
  }
  else {
    return 0;
  }
}

int PersistentTable::countReadStarvingForAddress(const Address& address) const
{
  if (m_map_ptr->exist(address)) {
    PersistentTableEntry& entry = m_map_ptr->lookup(address);
    return (entry.m_starving.count() - entry.m_request_to_write.count());
  }
  else {
    return 0;
  }
}

void PersistentTable::print(ostream& out) const
{
}

