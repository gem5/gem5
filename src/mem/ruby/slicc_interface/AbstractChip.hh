
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
 * $Id$
 *
 * Description: Common base class for a machine chip.
 *
 */

#ifndef ABSTRACT_CHIP_H
#define ABSTRACT_CHIP_H

#include "Global.hh"
#include "NodeID.hh"
#include "RubyConfig.hh"
#include "L1Cache_Entry.hh"
#include "Address.hh"
#include "Vector.hh"

class Network;
class Sequencer;
class StoreBuffer;
class ENTRY;
class MessageBuffer;
class CacheRecorder;
class TransactionInterfaceManager;

template<class ENTRY> class CacheMemory;

class AbstractChip {
public:
  // Constructors
  AbstractChip(NodeID chip_number, Network* net_ptr);

  // Destructor, prevent from being instantiated
  virtual ~AbstractChip() = 0;

  // Public Methods
  NodeID getID() const { return m_id; };
  Network* getNetwork() const { return m_net_ptr; };
  Sequencer* getSequencer(int index) const { return m_L1Cache_sequencer_vec[index]; };
  TransactionInterfaceManager* getTransactionInterfaceManager(int index) const { return m_L1Cache_xact_mgr_vec[index]; };
  void setTransactionInterfaceManager(TransactionInterfaceManager* manager, int index) { m_L1Cache_xact_mgr_vec[index] = manager; }

  // used when CHECK_COHERENCE is enabled.  See System::checkGlobalCoherence()
  virtual bool isBlockExclusive(const Address& addr) const { return false; }
  virtual bool isBlockShared(const Address& addr) const { return false; }

  // cache dump functions
  virtual void recordCacheContents(CacheRecorder& tr) const = 0;
  virtual void dumpCaches(ostream& out) const = 0;
  virtual void dumpCacheData(ostream& out) const = 0;

  virtual void printConfig(ostream& out) = 0;
  virtual void print(ostream& out) const = 0;

  // pulic data structures
  Vector < CacheMemory<L1Cache_Entry>* > m_L1Cache_L1DcacheMemory_vec;
  Vector < CacheMemory<L1Cache_Entry>* > m_L1Cache_L1IcacheMemory_vec;
  Vector < CacheMemory<L1Cache_Entry>* > m_L1Cache_cacheMemory_vec;
  Vector < CacheMemory<L1Cache_Entry>* > m_L1Cache_L2cacheMemory_vec;
  Vector < CacheMemory<L1Cache_Entry>* > m_L2Cache_L2cacheMemory_vec;

  // added so that the prefetcher and sequencer can access the L1 and L2 request queues
  Vector < MessageBuffer* > m_L1Cache_optionalQueue_vec;
  Vector < MessageBuffer* >m_L1Cache_mandatoryQueue_vec;

  // TSO storebuffer
  Vector < StoreBuffer* > m_L1Cache_storeBuffer_vec;

  // TM transaction manager
  Vector < TransactionInterfaceManager* >  m_L1Cache_xact_mgr_vec;

protected:

  // Data Members (m_ prefix)
  NodeID                 m_id;            // Chip id
  Network*               m_net_ptr;       // Points to the Network simulator
  Vector < Sequencer* >  m_L1Cache_sequencer_vec; // All chip should have a sequencer


};

// Output operator declaration
ostream& operator<<(ostream& out, const AbstractChip& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const AbstractChip& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //ABSTRACT_CHIP_H

