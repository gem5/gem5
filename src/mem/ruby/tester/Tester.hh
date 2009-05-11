
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

#ifndef TESTER_H
#define TESTER_H

#include "Global.hh"
#include "Driver.hh"
#include "CheckTable.hh"
#include "CacheRequestType.hh"

class System;

class Tester : public Driver, public Consumer {
public:
  // Constructors
  Tester(System* sys_ptr);

  // Destructor
  ~Tester();

  // Public Methods

  void hitCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread);
  void wakeup();
  void printStats(ostream& out) const {}
  void clearStats() {}
  void printConfig(ostream& out) const {}

  void print(ostream& out) const;
private:
  // Private Methods

  void checkForDeadlock();

  // Private copy constructor and assignment operator
  Tester(const Tester& obj);
  Tester& operator=(const Tester& obj);

  // Data Members (m_ prefix)

  CheckTable m_checkTable;
  Vector<Time> m_last_progress_vector;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Tester& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Tester& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TESTER_H
