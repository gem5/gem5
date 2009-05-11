
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

#ifndef BARRIERGENERATOR_H
#define BARRIERGENERATOR_H

#include "Global.hh"
#include "Consumer.hh"
#include "NodeID.hh"
#include "Address.hh"

class Sequencer;
class SubBlock;
class SyntheticDriver;


enum BarrierGeneratorStatus {
  BarrierGeneratorStatus_FIRST,
  BarrierGeneratorStatus_Thinking = BarrierGeneratorStatus_FIRST,
  BarrierGeneratorStatus_Test_Pending,
  BarrierGeneratorStatus_Test_Waiting,
  BarrierGeneratorStatus_Before_Swap,
  BarrierGeneratorStatus_Swap_Pending,
  BarrierGeneratorStatus_Holding,
  BarrierGeneratorStatus_Release_Pending,
  BarrierGeneratorStatus_Release_Waiting,
  BarrierGeneratorStatus_StoreFlag_Waiting,
  BarrierGeneratorStatus_StoreFlag_Pending,
  BarrierGeneratorStatus_Done,
  BarrierGeneratorStatus_SpinFlag_Ready,
  BarrierGeneratorStatus_SpinFlag_Pending,
  BarrierGeneratorStatus_LoadBarrierCounter_Pending,
  BarrierGeneratorStatus_StoreBarrierCounter_Pending,
  BarrierGeneratorStatus_StoreBarrierCounter_Waiting,
  BarrierGeneratorStatus_NUM
};


// UNCOMMENT THIS FOR A SINGLE WORK QUEUE
// static int m_counter;

class BarrierGenerator : public Consumer {
public:
  // Constructors
  BarrierGenerator(NodeID node, SyntheticDriver& driver);

  // Destructor
  ~BarrierGenerator();

  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, SubBlock& data);

  void print(ostream& out) const;
private:
  // Private Methods
  int thinkTime() ;
  int waitTime() const;
  void initiateTest();
  void initiateSwap();
  void initiateRelease();
  void initiateLoadCtr();
  void initiateStoreCtr();
  void initiateLoadFlag();
  void initiateStoreFlag();
  Sequencer* sequencer() const;

  // Private copy constructor and assignment operator
  BarrierGenerator(const BarrierGenerator& obj);
  BarrierGenerator& operator=(const BarrierGenerator& obj);

  // Data Members (m_ prefix)
  SyntheticDriver& m_driver;
  NodeID m_node;
  BarrierGeneratorStatus m_status;
  int proc_counter;

  int m_counter;

  bool m_local_sense;
  bool m_barrier_done;

  Time m_last_transition;
  Address m_address;

  int m_total_think;
  int m_think_periods;
};

// Output operator declaration
ostream& operator<<(ostream& out, const BarrierGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const BarrierGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //REQUESTGENERATOR_H

