
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
 * Description:
 *
 */

#ifndef StoreBuffer_H
#define StoreBuffer_H

#include "Global.hh"
#include "Consumer.hh"
#include "Address.hh"
#include "AccessModeType.hh"
#include "CacheRequestType.hh"
#include "StoreCache.hh"

class CacheMsg;
class DataBlock;
class SubBlock;
class StoreBufferEntry;
class AbstractChip;

template <class TYPE> class Vector;

class StoreBuffer : public Consumer {
public:
  // Constructors
  StoreBuffer(AbstractChip* chip_ptr, int version);

  // Destructor
  ~StoreBuffer();

  // Public Methods
  void wakeup(); // Used only for deadlock detection
  void callBack(const Address& addr, DataBlock& data);
  void insertStore(const CacheMsg& request);
  void updateSubBlock(SubBlock& sub_block) const { m_store_cache.update(sub_block); }
  bool trySubBlock(const SubBlock& sub_block) const { assert(isReady()); return m_store_cache.check(sub_block); }
  void print(ostream& out) const;
  bool isEmpty() const { return (m_size == 0); }
  bool isReady() const;

  // Class methods
  static void printConfig(ostream& out);

private:
  // Private Methods
  void processHeadOfQueue();

  StoreBufferEntry& peek();
  void dequeue();
  void enqueue(const StoreBufferEntry& entry);
  StoreBufferEntry& getEntry(int index);

  // Private copy constructor and assignment operator
  StoreBuffer(const StoreBuffer& obj);
  StoreBuffer& operator=(const StoreBuffer& obj);

  // Data Members (m_ prefix)
  int m_version;

  Vector<StoreBufferEntry>* m_queue_ptr;
  int m_head;
  int m_tail;
  int m_size;

  StoreCache m_store_cache;

  AbstractChip* m_chip_ptr;
  bool m_pending;
  Address m_pending_address;
  bool m_seen_atomic;
  bool m_deadlock_check_scheduled;
};

// Output operator declaration
ostream& operator<<(ostream& out, const StoreBuffer& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const StoreBuffer& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //StoreBuffer_H
