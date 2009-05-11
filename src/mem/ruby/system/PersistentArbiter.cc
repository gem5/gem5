
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

#include "mem/ruby/system/PersistentArbiter.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/slicc_interface/AbstractChip.hh"
#include "mem/gems_common/util.hh"

PersistentArbiter::PersistentArbiter(AbstractChip* chip_ptr)
{
  m_chip_ptr = chip_ptr;

  // wastes entries, but who cares
  m_entries.setSize(RubyConfig::numberOfProcessors());

  for (int i = 0; i < m_entries.size(); i++) {
    m_entries[i].valid = false;
  }

  m_busy = false;
  m_locker = -1;

}

PersistentArbiter::~PersistentArbiter()
{
  m_chip_ptr = NULL;
}


void PersistentArbiter::addLocker(NodeID id, Address addr, AccessType type) {
  //cout << "Arbiter " << getArbiterId() << " adding locker " << id << " " << addr << endl;
  assert(m_entries[id].valid == false);
  m_entries[id].valid = true;
  m_entries[id].address = addr;
  m_entries[id].type = type;
  m_entries[id].localId = id;

}

void PersistentArbiter::removeLocker(NodeID id) {
  //cout << "Arbiter " << getArbiterId() << " removing locker " << id << " " << m_entries[id].address << endl;
  assert(m_entries[id].valid == true);
  m_entries[id].valid = false;

  if (!lockersExist()) {
    m_busy = false;
  }
}

bool PersistentArbiter::successorRequestPresent(Address addr, NodeID id) {
  for (int i = (id + 1); i < m_entries.size(); i++) {
    if (m_entries[i].address == addr && m_entries[i].valid) {
      //cout << "m_entries[" << id << ", address " << m_entries[id].address << " is equal to " << addr << endl;
      return true;
    }
  }
  return false;
}

bool PersistentArbiter::lockersExist() {
  for (int i = 0; i < m_entries.size(); i++) {
    if (m_entries[i].valid == true) {
      return true;
    }
  }
  //cout << "no lockers found" << endl;
  return false;
}

void PersistentArbiter::advanceActiveLock() {
  assert(lockersExist());

  //cout << "arbiter advancing lock from " << m_locker;
  m_busy = false;

  if (m_locker < (m_entries.size() - 1)) {
    for (int i = (m_locker+1); i < m_entries.size(); i++) {
      if (m_entries[i].valid == true) {
        m_locker = i;
        m_busy = true;
        //cout << " to " << m_locker << endl;
        return;
      }
    }
  }

  if (!m_busy) {
    for (int i = 0; i < m_entries.size(); i++) {
      if (m_entries[i].valid == true) {
        m_locker = i;
        m_busy = true;
        //cout << " to " << m_locker << endl;
        return;
      }
    }

    assert(m_busy)
  }
}

Address PersistentArbiter::getActiveLockAddress() {
  assert( m_entries[m_locker].valid = true );
  return m_entries[m_locker].address;
}


NodeID PersistentArbiter::getArbiterId() {
  return m_chip_ptr->getID()*RubyConfig::numberOfProcsPerChip();
}

bool PersistentArbiter::isBusy() {
  return m_busy;
}

NodeID PersistentArbiter::getActiveLocalId() {
  assert( m_entries[m_locker].valid = true );
  return m_entries[m_locker].localId;
}

void PersistentArbiter::setIssuedAddress(Address addr) {
  m_issued_address = addr;
}

bool PersistentArbiter::isIssuedAddress(Address addr) {
  return (m_issued_address == addr);
}

void PersistentArbiter::print(ostream& out) const {

  out << "[";
  for (int i = 0; i < m_entries.size(); i++) {
    if (m_entries[i].valid == true) {
      out << "( " << m_entries[i].localId  << ", " << m_entries[i].address << ") ";
    }
  }
  out << "]" << endl;

}
