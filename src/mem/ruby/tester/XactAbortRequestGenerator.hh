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
 * $Id$
 *
 * Description:
 *
 */

#ifndef XACTABORTREQUESTGENERATOR_H
#define XACTABORTREQUESTGENERATOR_H

#ifdef XACT_MEM

#include "mem/ruby/tester/RequestGenerator.hh"
#include "global.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Address.hh"

class Sequencer;
class SubBlock;
class SyntheticDriver;
class Instruction;
class TransactionManager;

#define MAX_ADDRESS 16777216

enum XactAbortRequestGeneratorStatus {
  XactAbortRequestGeneratorStatus_Waiting,
  XactAbortRequestGeneratorStatus_Ready,
  XactAbortRequestGeneratorStatus_Blocked,
  XactAbortRequestGeneratorStatus_Aborted,
  XactAbortRequestGeneratorStatus_Done
};

class XactAbortRequestGenerator : public RequestGenerator {
public:
  // Constructors
  XactAbortRequestGenerator(NodeID node, SyntheticDriver& driver);

  // Destructor
  ~XactAbortRequestGenerator();

  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, SubBlock& data);
  void abortTransaction();

  void print(ostream& out) const;
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
  void pickAddress();
  Sequencer* sequencer() const;
  TransactionManager* transactionManager() const;
  void execute();

  void checkCorrectness();

  // Private copy constructor and assignment operator
  XactAbortRequestGenerator(const XactAbortRequestGenerator& obj);
  XactAbortRequestGenerator& operator=(const XactAbortRequestGenerator& obj);

  void newTransaction();

  // Data Members (m_ prefix)
  SyntheticDriver& m_driver;
  NodeID m_node;
  XactAbortRequestGeneratorStatus m_xact_status;
  int m_counter;
  Time m_last_transition;
  Address m_address;

  Instruction *m_instructions;
  int m_pc;
  uint8 m_register;
  static Vector<uint8> testArray;
  //static uint8 dataArray[];
};

// Output operator declaration
ostream& operator<<(ostream& out, const XactAbortRequestGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const XactAbortRequestGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //XACT_MEM

#endif //REQUESTGENERATOR_H

