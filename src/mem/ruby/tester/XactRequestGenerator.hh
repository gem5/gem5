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
 * $Id: XactRequestGenerator.h 1.4 05/07/07 10:35:32-05:00 kmoore@s0-30.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef XACTREQUESTGENERATOR_H
#define XACTREQUESTGENERATOR_H

#include "global.hh"
#include "mem/ruby/tester/RequestGenerator.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Address.hh"
#include "TransactionManager.hh"

class Sequencer;
class SubBlock;
class SyntheticDriver;
class Instruction;
class TransactionManager;

#define MAX_ADDRESS 16777216
const int TESTER_MAX_DEPTH = 16;

enum XactRequestGeneratorStatus {
  XactRequestGeneratorStatus_Waiting,
  XactRequestGeneratorStatus_Ready,
  XactRequestGeneratorStatus_Blocked,
  XactRequestGeneratorStatus_Aborted,
  XactRequestGeneratorStatus_Done
};

class XactRequestGenerator : public RequestGenerator {
public:
  // Constructors
  XactRequestGenerator(NodeID node, SyntheticDriver& driver);

  // Destructor
  ~XactRequestGenerator();

  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, SubBlock& data);
  void abortTransaction();

  void print(ostream& out) const;

  // For dealing with NACKs/retries
  void notifySendNack(const Address & addr, uint64 remote_timestamp, const MachineID & remote_id);
  void notifyReceiveNack(const Address & addr, uint64 remote_timestamp, const MachineID & remote_id);
  void notifyReceiveNackFinal(const Address & addr);
private:
  // Private Methods
  int thinkTime() const;
  int waitTime() const;
  int holdTime() const;
  void initiateBeginTransaction();
  void initiateStore(Address a);
  void initiateCommit();
  void initiateInc(Address a);
  void initiateLoad(Address a);
  void initiateDone();
  void pickAddress();
  Sequencer* sequencer() const;
  TransactionManager* transactionManager() const;
  void execute();
  void scheduleEvent(int time);
  void checkCorrectness();

  // Private copy constructor and assignment operator
  XactRequestGenerator(const XactRequestGenerator& obj);
  XactRequestGenerator& operator=(const XactRequestGenerator& obj);

  void newTransaction(bool init);
  void printPcStack(int depth);

  // Data Members (m_ prefix)
  SyntheticDriver& m_driver;
  NodeID m_node;
  XactRequestGeneratorStatus m_status;
  int m_counter;
  int m_size;
  Time m_last_transition;
  Address m_address;

  Instruction *m_instructions;
  int m_pc;
  int pc_stack[TESTER_MAX_DEPTH];
  bool m_transaction;
  uint8 m_register;
  uint8 * testArray;
  //static uint8 dataArray[];
  bool m_eventPending;

  // for pending aborts
  bool m_abortPending;
};

// Output operator declaration
ostream& operator<<(ostream& out, const XactRequestGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const XactRequestGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //XACTREQUESTGENERATOR_H

