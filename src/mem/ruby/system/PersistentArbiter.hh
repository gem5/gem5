
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
 * PersistentArbiter.hh
 *
 * Description:
 *
 * Used for hierarchical distributed persistent request scheme
 *
 */

#ifndef PERSISTENTARBITER_H
#define PERSISTENTARBITER_H

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/slicc_interface/AbstractChip.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/protocol/AccessType.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/Address.hh"

struct ArbiterEntry {
  bool valid;
  Address address;
  AccessType type;
  NodeID localId;
};

class PersistentArbiter {
public:

  // Constructors
  PersistentArbiter(AbstractChip* chip_ptr);

  // Destructor
  ~PersistentArbiter();

  // Public Methods

  void addLocker(NodeID id, Address addr, AccessType type);
  void removeLocker(NodeID id);
  bool successorRequestPresent(Address addr, NodeID id);
  bool lockersExist();
  void advanceActiveLock();
  Address getActiveLockAddress();
  NodeID getArbiterId();
  bool isBusy();

  void setIssuedAddress(Address addr);
  bool isIssuedAddress(Address addr);


  Address getIssuedAddress() { return m_issued_address; }

  static void printConfig(ostream& out) {}
  void print(ostream& out) const;

  NodeID getActiveLocalId();

private:

  Address m_issued_address;
  AbstractChip* m_chip_ptr;
  int m_locker;
  bool m_busy;
  Vector<ArbiterEntry> m_entries;
};

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const PersistentArbiter& obj)
{
  obj.print(out);
  out << flush;
  return out;
}


#endif //PERFECTCACHEMEMORY_H
