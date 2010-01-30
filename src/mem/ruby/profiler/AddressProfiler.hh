
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
 * AddressProfiler.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef ADDRESSPROFILER_H
#define ADDRESSPROFILER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/protocol/AccessType.hh"

class AccessTraceForAddress;
class Set;
template <class KEY_TYPE, class VALUE_TYPE> class Map;

class AddressProfiler {
public:
  // Constructors
  AddressProfiler(int num_of_sequencers);

  // Destructor
  ~AddressProfiler();

  // Public Methods
  void printStats(ostream& out) const;
  void clearStats();

  void addTraceSample(Address data_addr, Address pc_addr, CacheRequestType type, AccessModeType access_mode, NodeID id, bool sharing_miss);
  void profileRetry(const Address& data_addr, AccessType type, int count);
  void profileGetX(const Address& datablock, const Address& PC, const Set& owner, const Set& sharers, NodeID requestor);
  void profileGetS(const Address& datablock, const Address& PC, const Set& owner, const Set& sharers, NodeID requestor);

  void print(ostream& out) const;

  //added by SS
  void setHotLines(bool hot_lines);
  void setAllInstructions(bool all_instructions);
private:
  // Private Methods

  // Private copy constructor and assignment operator
  AddressProfiler(const AddressProfiler& obj);
  AddressProfiler& operator=(const AddressProfiler& obj);

  // Data Members (m_ prefix)
  int64 m_sharing_miss_counter;

  Map<Address, AccessTraceForAddress>* m_dataAccessTrace;
  Map<Address, AccessTraceForAddress>* m_macroBlockAccessTrace;
  Map<Address, AccessTraceForAddress>* m_programCounterAccessTrace;
  Map<Address, AccessTraceForAddress>* m_retryProfileMap;
  Histogram m_retryProfileHisto;
  Histogram m_retryProfileHistoWrite;
  Histogram m_retryProfileHistoRead;
  Histogram m_getx_sharing_histogram;
  Histogram m_gets_sharing_histogram;
//added by SS
  bool m_hot_lines;
  bool m_all_instructions;

  int m_num_of_sequencers;
};

// Output operator declaration
ostream& operator<<(ostream& out, const AddressProfiler& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const AddressProfiler& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //ADDRESSPROFILER_H
