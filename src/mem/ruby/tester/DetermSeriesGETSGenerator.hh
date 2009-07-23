
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

// This Deterministic Generator generates a series of GETS requests for a given node.
// Sequentially goes through all nodes in the system
// This generator is used to tune the HW prefetcher
// The GETS requests are generated one at a time in round-robin fashion 0...1...2...etc.

#ifndef DETERMSERIESGETSGENERATOR_H
#define DETERMSERIESGETSGENERATOR_H

#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/protocol/DetermSeriesGETSGeneratorStatus.hh"
#include "mem/ruby/tester/SpecifiedGenerator.hh"

class DeterministicDriver;

class DetermSeriesGETSGenerator : public SpecifiedGenerator {
public:
  // Constructors
  DetermSeriesGETSGenerator(NodeID node, DeterministicDriver& driver);

  // Destructor
  ~DetermSeriesGETSGenerator();
  
  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, Address address);

  void print(ostream& out) const;
private:
  // Private Methods
  int thinkTime() const;
  int waitTime() const;
  void initiateLoad();
  void pickAddress();

  // copy constructor and assignment operator
  DetermSeriesGETSGenerator(const DetermSeriesGETSGenerator& obj);
  DetermSeriesGETSGenerator& operator=(const DetermSeriesGETSGenerator& obj);

  // Data Members (m_ prefix)
  DetermSeriesGETSGeneratorStatus m_status;
  int m_counter;
  Address m_address;
  NodeID m_node;
  DeterministicDriver& m_driver;
  Time m_last_transition;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DetermSeriesGETSGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const DetermSeriesGETSGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DETERMSeriesGETSGENERATOR_H

