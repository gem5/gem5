
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

// This Deterministic Generator generates GETS request for all nodes in the system
// then Invalidates them with a GETX.  The GETS and GETX request are generated one
// at a time in round-robin fashion 0...1...2...etc.

#ifndef DETERMINVGENERATOR_H
#define DETERMINVGENERATOR_H

#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/protocol/DetermInvGeneratorStatus.hh"
#include "mem/ruby/tester/SpecifiedGenerator.hh"

class DeterministicDriver;

class DetermInvGenerator : public SpecifiedGenerator {
public:
  // Constructors
  DetermInvGenerator(NodeID node, DeterministicDriver& driver);

  // Destructor
  ~DetermInvGenerator();
  
  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, Address address);

  void print(ostream& out) const;
private:
  // Private Methods
  int thinkTime() const;
  int waitTime() const;
  int holdTime() const;
  void initiateLoad();
  void initiateStore();
  void pickLoadAddress();
  void pickStoreAddress();
  
  // copy constructor and assignment operator
  DetermInvGenerator(const DetermInvGenerator& obj);
  DetermInvGenerator& operator=(const DetermInvGenerator& obj);

  // Data Members (m_ prefix)
  DetermInvGeneratorStatus m_status;
  int m_counter;
  Address m_address;
  NodeID m_node;
  DeterministicDriver& m_driver;
  Time m_last_transition;

};

// Output operator declaration
ostream& operator<<(ostream& out, const DetermInvGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const DetermInvGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DETERMINVGENERATOR_H

