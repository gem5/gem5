/*
 * Copyright (c) 1999 by Mark Hill and David Wood for the Wisconsin
 * Multifacet Project.  ALL RIGHTS RESERVED.  
 *
 * ##HEADER##
 *
 * This software is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and the
 * inclusion of the above copyright notice.  This software or any
 * other copies thereof or any derivative works may not be provided or
 * otherwise made available to any other persons.  Title to and
 * ownership of the software is retained by Mark Hill and David Wood.
 * Any use of this software must include the above copyright notice.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS".  THE LICENSOR MAKES NO
 * WARRANTIES ABOUT ITS CORRECTNESS OR PERFORMANCE.
 * */

/*
 * Description: This implements a pseudo racey thread which drives ruby timing
 *              simulator with access to two shared arrays.
 *
 */

#ifndef RACEYPSEUDOTHREAD_H
#define RACEYPSEUDOTHREAD_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Address.hh"
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

