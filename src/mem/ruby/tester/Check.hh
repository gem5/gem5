
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

#ifndef CHECK_H
#define CHECK_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/protocol/TesterStatus.hh"
#include "mem/protocol/AccessModeType.hh"
class Sequencer;
class SubBlock;

const int CHECK_SIZE_BITS = 2;
const int CHECK_SIZE = (1<<CHECK_SIZE_BITS);

class Check {
public:
  // Constructors
  Check(const Address& address, const Address& pc);

  // Default Destructor
  //~Check();

  // Public Methods

  void initiate(); // Does Action or Check or nether
  void performCallback(NodeID proc, SubBlock& data);
  const Address& getAddress() { return m_address; }
  void changeAddress(const Address& address);

  void print(ostream& out) const;
private:
  // Private Methods
  void initiatePrefetch(Sequencer* targetSequencer_ptr);
  void initiatePrefetch();
  void initiateAction();
  void initiateCheck();

  Sequencer* initiatingSequencer() const;

  void pickValue();
  void pickInitiatingNode();

  // Using default copy constructor and assignment operator
  //  Check(const Check& obj);
  //  Check& operator=(const Check& obj);

  // Data Members (m_ prefix)
  TesterStatus m_status;
  uint8 m_value;
  int m_store_count;
  NodeID m_initiatingNode;
  Address m_address;
  Address m_pc;
  AccessModeType m_access_mode;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Check& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Check& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //CHECK_H
