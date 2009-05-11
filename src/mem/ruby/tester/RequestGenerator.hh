
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

#ifndef REQUESTGENERATOR_H
#define REQUESTGENERATOR_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/protocol/RequestGeneratorStatus.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Address.hh"

class Sequencer;
class SubBlock;
class SyntheticDriver;

class RequestGenerator : public Consumer {
public:
  // Constructors
  RequestGenerator(NodeID node, SyntheticDriver& driver);

  // Destructor
  ~RequestGenerator();

  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, SubBlock& data);

  void print(ostream& out) const;
private:
  // Private Methods
  int thinkTime() const;
  int waitTime() const;
  int holdTime() const;
  void initiateTest();
  void initiateSwap();
  void initiateRelease();
  void pickAddress();
  Sequencer* sequencer() const;

  // Private copy constructor and assignment operator
  RequestGenerator(const RequestGenerator& obj);
  RequestGenerator& operator=(const RequestGenerator& obj);

  // Data Members (m_ prefix)
  SyntheticDriver& m_driver;
  NodeID m_node;
  RequestGeneratorStatus m_status;
  int m_counter;
  Time m_last_transition;
  Address m_address;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RequestGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const RequestGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //REQUESTGENERATOR_H

