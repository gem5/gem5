/*
 * Copyright (c) 1999 Mark D. Hill and David A. Wood
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
 * Description: This implements a pseudo racey thread which drives ruby timing
 *              simulator with access to two shared arrays.
 *
 */

#ifndef RACEYPSEUDOTHREAD_H
#define RACEYPSEUDOTHREAD_H

#include "mem/ruby/tester/Global_Tester.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "Address_Tester.hh"
#include "mem/ruby/libruby.hh"

class RaceyDriver;

class RaceyPseudoThread : public Consumer {
private:
  // constants
  static const int PRIME1     = 103072243;
  static const int PRIME2     = 103995407;
  static const int M_ELEM     = 64;

  // m and sig array's starting address,
  // each signature should occupy a cacheline
  static const int SIG_ARR    = 0;
  static const int M_ARR      = 0x10000;
  static const int LINESIZE   = 64;

  // get address of a element from the m and sig arrays
  physical_address_t sig(unsigned index) {
    assert(index < M_ARR/64);
    return SIG_ARR + (index*64);
  };
  physical_address_t m(unsigned index) { return M_ARR + (index*64); };

public:
  // Constructors
  RaceyPseudoThread(NodeID node, RaceyDriver& driver);

  // Destructor
  ~RaceyPseudoThread();

  // Public Methods
  void performCallback(int proc, Address address, uint8_t * data);

  void wakeup();

  integer_t getInstructionCount() { return m_ic_counter; };

  unsigned getSignature() { assert(m_proc_id == 0); return m_final_sig; };

  void checkForDeadlock();

  // save and restore the thread state
  void saveCPUStates(string filename);
  void loadCPUStates(string filename);

  // reset IC to zero for next checkpoint
  void resetIC() { m_ic_counter = 0; };

  bool getInitializedState() { return m_initialized; };

  void print(ostream& out) const;
private:
  // Private Methods

  // mix two numbers
  unsigned mix (unsigned i, unsigned j) { return (i + j * PRIME2) % PRIME1; };

  // load or store the array
  void load_sig(unsigned index);
  void load_m(unsigned index);
  void store_sig(unsigned index, unsigned value);
  void store_m(unsigned index, unsigned value);

  // Private copy constructor and assignment operator
  RaceyPseudoThread(const RaceyPseudoThread& obj);
  RaceyPseudoThread& operator=(const RaceyPseudoThread& obj);

  // Data Members (m_ prefix)
  RaceyDriver& m_driver;
  NodeID m_proc_id;

  // are we done?
  bool m_done;

  // [committed] instruction counter
  int m_ic_counter;

  // last time we made progress
  Time m_last_progress;

  // value of the callback block
  bool     m_read;
  unsigned m_value;

  // final signature
  unsigned m_final_sig;

  // local variables for the pseudo thread
  int m_looper;
  unsigned m_num, m_index1, m_index2, m_stop;
  bool m_initialized;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RaceyPseudoThread& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const RaceyPseudoThread& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //RACEYPSEUDOTHREAD_H

