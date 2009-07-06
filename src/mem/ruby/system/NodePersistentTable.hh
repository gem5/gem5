
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

/*
 * $Id: NodePersistentTable.hh 1.3 04/08/16 14:12:33-05:00 beckmann@c2-143.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef NodePersistentTable_H
#define NodePersistentTable_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/protocol/AccessType.hh"

class AbstractChip;

template <class KEY_TYPE, class VALUE_TYPE> class Map;
class Address;
class NodePersistentTableEntry;

class NodePersistentTable {
public:
  // Constructors
  NodePersistentTable(AbstractChip* chip_ptr, int version);

  // Destructor
  ~NodePersistentTable();

  // Public Methods
  void persistentRequestLock(const Address& address, NodeID locker, AccessType type);
  void persistentRequestUnlock(const Address& address, NodeID unlocker);
  bool okToIssueStarving(const Address& address) const;
  NodeID findSmallest(const Address& address) const;
  AccessType typeOfSmallest(const Address& address) const;
  void markEntries(const Address& address);
  bool isLocked(const Address& addr) const;
  int countStarvingForAddress(const Address& addr) const;
  int countReadStarvingForAddress(const Address& addr) const;

  static void printConfig(ostream& out) {}

  void print(ostream& out) const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  NodePersistentTable(const NodePersistentTable& obj);
  NodePersistentTable& operator=(const NodePersistentTable& obj);

  // Data Members (m_prefix)
  Map<Address, NodePersistentTableEntry>* m_map_ptr;
  AbstractChip* m_chip_ptr;
  int m_version;
};

// Output operator declaration
ostream& operator<<(ostream& out, const NodePersistentTable& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const NodePersistentTable& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //NodePersistentTable_H
