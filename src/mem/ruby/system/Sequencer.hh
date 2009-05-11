
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
 * $Id: Sequencer.h 1.70 2006/09/27 14:56:41-05:00 bobba@s1-01.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "Global.hh"
#include "RubyConfig.hh"
#include "Consumer.hh"
#include "CacheRequestType.hh"
#include "AccessModeType.hh"
#include "GenericMachineType.hh"
#include "PrefetchBit.hh"
#include "Map.hh"
#include "packet.hh"

class DataBlock;
class AbstractChip;
class CacheMsg;
class Address;
class MachineID;

class Sequencer : public Consumer {
public:
  // Constructors
  Sequencer(AbstractChip* chip_ptr, int version);

  // Destructor
  ~Sequencer();

  // Public Methods
  void wakeup(); // Used only for deadlock detection

  static void printConfig(ostream& out);

  // returns total number of outstanding request (includes prefetches)
  int getNumberOutstanding();
  // return only total number of outstanding demand requests
  int getNumberOutstandingDemand();
  // return only total number of outstanding prefetch requests
  int getNumberOutstandingPrefetch();

  // remove load/store request from queue
  void removeLoadRequest(const Address & addr, int thread);
  void removeStoreRequest(const Address & addr, int thread);

  void printProgress(ostream& out) const;

  // returns a pointer to the request in the request tables
  CacheMsg & getReadRequest( const Address & addr, int thread );
  CacheMsg & getWriteRequest( const Address & addr, int thread );

  // called by Ruby when transaction completes
  void writeConflictCallback(const Address& address);
  void readConflictCallback(const Address& address);
  void writeConflictCallback(const Address& address, GenericMachineType respondingMach, int thread);
  void readConflictCallback(const Address& address, GenericMachineType respondingMach, int thread);

  void writeCallback(const Address& address, DataBlock& data);
  void readCallback(const Address& address, DataBlock& data);
  void writeCallback(const Address& address);
  void readCallback(const Address& address);
  void writeCallback(const Address& address, DataBlock& data, GenericMachineType respondingMach, PrefetchBit pf, int thread);
  void readCallback(const Address& address, DataBlock& data, GenericMachineType respondingMach, PrefetchBit pf, int thread);
  void writeCallback(const Address& address, DataBlock& data, GenericMachineType respondingMach, int thread);
  void readCallback(const Address& address, DataBlock& data, GenericMachineType respondingMach, int thread);

  // returns the thread ID of the request
  int getRequestThreadID(const Address & addr);
  // returns the physical address of the request
  Address getRequestPhysicalAddress(const Address & lineaddr);
  // returns whether a request is a prefetch request
  bool isPrefetchRequest(const Address & lineaddr);

  //notifies driver of debug print
  void printDebug();

  // called by Tester or Simics
  void makeRequest(const Packet* pkt, void* data);
  void makeRequest(const CacheMsg& request); // depricate this function
  bool doRequest(const CacheMsg& request);
  void issueRequest(const CacheMsg& request);
  bool isReady(const Packet* pkt) const;
  bool isReady(const CacheMsg& request) const; // depricate this function
  bool empty() const;
  void resetRequestTime(const Address& addr, int thread);
  Address getLogicalAddressOfRequest(Address address, int thread);
  AccessModeType getAccessModeOfRequest(Address address, int thread);
  //uint64 getSequenceNumberOfRequest(Address addr, int thread);

  void print(ostream& out) const;
  void checkCoherence(const Address& address);

  bool getRubyMemoryValue(const Address& addr, char* value, unsigned int size_in_bytes);
  bool setRubyMemoryValue(const Address& addr, char *value, unsigned int size_in_bytes);

  void removeRequest(const CacheMsg& request);
private:
  // Private Methods
  bool tryCacheAccess(const Address& addr, CacheRequestType type, const Address& pc, AccessModeType access_mode, int size, DataBlock*& data_ptr);
  void conflictCallback(const CacheMsg& request, GenericMachineType respondingMach, int thread);
  void hitCallback(const CacheMsg& request, DataBlock& data, GenericMachineType respondingMach, int thread);
  bool insertRequest(const CacheMsg& request);


  // Private copy constructor and assignment operator
  Sequencer(const Sequencer& obj);
  Sequencer& operator=(const Sequencer& obj);

  // Data Members (m_ prefix)
  AbstractChip* m_chip_ptr;

  // indicates what processor on the chip this sequencer is associated with
  int m_version;

  // One request table per SMT thread
  Map<Address, CacheMsg>** m_writeRequestTable_ptr;
  Map<Address, CacheMsg>** m_readRequestTable_ptr;
  // Global outstanding request count, across all request tables
  int m_outstanding_count;
  bool m_deadlock_check_scheduled;

};

// Output operator declaration
ostream& operator<<(ostream& out, const Sequencer& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Sequencer& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SEQUENCER_H

