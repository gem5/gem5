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
 */

#ifdef XACT_MEM

#include "XactAbortRequestGenerator.hh"
#include "LockStatus.hh"
#include "Sequencer.hh"
#include "System.hh"
#include "RubyConfig.hh"
#include "SubBlock.hh"
#include "SyntheticDriver.hh"
#include "Chip.hh"
#include "Instruction.hh"
#include "TransactionManager.hh"

//uint8 XactAbortRequestGenerator::testArray[MAX_ADDRESS];
//uint8 XactAbortRequestGenerator::dataArray[MAX_ADDRESS];
Vector<uint8> XactAbortRequestGenerator::testArray;

XactAbortRequestGenerator::XactAbortRequestGenerator(NodeID node, SyntheticDriver& driver) :
  RequestGenerator(node, driver), m_driver(driver)
{
  //DEBUG_EXPR(TESTER_COMP, MedPrio, "#### -- Creating XactAbortRequestGenerator\n");
  cout << "#### -- Creating XactAbortRequestGenerator " << node << endl;

  testArray.setSize(g_MEMORY_SIZE_BYTES);
  assert(testArray.size() == g_MEMORY_SIZE_BYTES);
  // Create instructions
  m_instructions = new Instruction[XACT_LENGTH];
  newTransaction();

  m_xact_status = XactAbortRequestGeneratorStatus_Ready;
  m_last_transition = 0;
  m_node = node;
  //pickAddress();
  m_counter = 0;
  m_register = 5;
  m_pc = 0;

  //for(int i=0; i<XACT_SIZE; ++i){
  //testArray[i] = 64;
  //}

  //testArray = new uint8[XACT_SIZE];
  //dataArray = new uint8[XACT_SIZE];
  g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

void XactAbortRequestGenerator::newTransaction(){
  int num_stores = 16;
  int num_loads = XACT_LENGTH - num_stores - 2;

  for(int i=0; i<XACT_LENGTH; ++i){
    if (i == 0){
      m_instructions[i].init(Opcode_BEGIN, Address(1));
    } else if (i == XACT_LENGTH - 1){
      m_instructions[i].init(Opcode_COMMIT, Address(1));
    } else if (i < num_loads) {
      physical_address_t address = i % XACT_SIZE;
      ASSERT(address < XACT_SIZE);

      int selectOpcode = random() % 2;
      Opcode op;
      switch(selectOpcode){
      case 0:
        op = Opcode_LD;
        break;
      case 1:
        op = Opcode_INC;
        break;
      };
      m_instructions[i].init(op, Address(address));
    } else {
      physical_address_t address = i - num_loads;
      ASSERT(address < XACT_SIZE);
      Opcode op = Opcode_ST;
      m_instructions[i].init(op, Address(address));
    }
  }
}

XactAbortRequestGenerator::~XactAbortRequestGenerator()
{
  delete m_instructions;
}

void XactAbortRequestGenerator::wakeup()
{
  assert(m_xact_status == XactAbortRequestGeneratorStatus_Ready || m_xact_status == XactAbortRequestGeneratorStatus_Aborted);
  m_xact_status = XactAbortRequestGeneratorStatus_Blocked;
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_xact_status);

  m_last_transition = g_eventQueue_ptr->getTime();
  execute();
}

void XactAbortRequestGenerator::execute(){
  assert(m_pc >= 0 && m_pc < XACT_LENGTH);
  Instruction current = m_instructions[m_pc];
  //cout << "  " << m_node << " executing pc: " << m_pc;
  switch (current.getOpcode()){
  case Opcode_BEGIN:
    //cout << "  -- begin.";
    initiateBeginTransaction();
    break;
  case Opcode_LD:
    //cout << "  -- load: " << current.getAddress();
    initiateLoad(current.getAddress());
    break;
  case Opcode_INC:
    //cout << "  -- inc.";
    initiateInc(current.getAddress());
    break;
  case Opcode_ST:
    //cout << "  -- store: " << current.getAddress();
    initiateStore(current.getAddress());
    break;
  case Opcode_COMMIT:
    //cout << "  -- commit.";
    initiateCommit();
    break;
  default:
    WARN_EXPR(current.getOpcode());
    ERROR_MSG("Invalid opcode");
  };
  //cout << endl;
}

void XactAbortRequestGenerator::performCallback(NodeID proc, SubBlock& data)
{
  assert(m_xact_status == XactAbortRequestGeneratorStatus_Waiting ||
         m_xact_status == XactAbortRequestGeneratorStatus_Aborted);
  assert(proc == m_node);

  Address address = data.getAddress();

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_xact_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  m_last_transition = g_eventQueue_ptr->getTime();

  //cout << "  " << m_node << " in performCallback, pc:" << m_pc
  //     << ", addr:" << address << endl;
  if(m_xact_status == XactAbortRequestGeneratorStatus_Aborted){
    cout << "  " << m_node << "  aborted, resetting pc." << endl;
    m_pc = 0;
    m_register = 5;
    m_xact_status = XactAbortRequestGeneratorStatus_Ready;
    g_eventQueue_ptr->scheduleEvent(this, waitTime());
  } else {
    m_xact_status = XactAbortRequestGeneratorStatus_Blocked;

    bool found;
    uint8 value;
    switch (m_instructions[m_pc].getOpcode()){
    case Opcode_BEGIN:
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_register = 5;
      m_xact_status = XactAbortRequestGeneratorStatus_Ready;
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
      m_pc++;
      break;
    case Opcode_LD:
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_register = data.getByte(0);
      //cout << "  " << m_node << " " << g_eventQueue_ptr->getTime() << " Callback--LD: " << (int) m_register << endl;
      m_xact_status = XactAbortRequestGeneratorStatus_Ready;
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
      m_pc++;
      break;
      //case Opcode_INC:  // We shouldn't get a callback for this!
      //m_driver.recordSwapLatency(g_eventQueue_ptr->getTime() - m_last_transition);

      //    break;
    case Opcode_ST:
      m_driver.recordReleaseLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      //data.setByte(address.getOffset(), m_register);
      data.setByte(0, m_register);
      //cout << "  " << m_node << " " << g_eventQueue_ptr->getTime() << " Callback--ST: " << (int) m_register << endl;

      //dataArray[address.getAddress()] = m_register;
      found = sequencer()->setRubyMemoryValue(address, (char *) (&m_register), 1);
      assert(found);
      found = sequencer()->getRubyMemoryValue(address, (char *) (&value), 1);
      assert(found);
      assert(value == m_register);

      m_xact_status = XactAbortRequestGeneratorStatus_Ready;
      g_eventQueue_ptr->scheduleEvent(this, thinkTime());
      m_pc++;
      break;
    case Opcode_COMMIT:
      m_counter++;
      cout << "  " << m_node << " callback--commit, counter is " << m_counter << " length is: " << g_tester_length << endl;
      // Check for correctness
      checkCorrectness();

      m_driver.recordReleaseLatency(g_eventQueue_ptr->getTime() - m_last_transition);

      if (m_counter < g_tester_length) {
        m_last_transition = g_eventQueue_ptr->getTime();
        //pickAddress(); // Necessary?

        // Create new random transaction
        newTransaction();
        m_pc = 0;

        m_xact_status = XactAbortRequestGeneratorStatus_Ready;
        g_eventQueue_ptr->scheduleEvent(this, thinkTime());
      } else {
        cout << "Ending" << endl;
        m_driver.reportDone();
        m_xact_status = XactAbortRequestGeneratorStatus_Done;
      }
      break;
    default:
      ERROR_MSG("Invalid Opcode");
    };
  }
}

int XactAbortRequestGenerator::thinkTime() const
{
  return g_think_time;
}

int XactAbortRequestGenerator::waitTime() const
{
  return g_wait_time;
}

int XactAbortRequestGenerator::holdTime() const
{
  return g_hold_time;
}

void XactAbortRequestGenerator::pickAddress()
{
  //m_address = m_driver.pickAddress(m_node);
}

void XactAbortRequestGenerator::initiateBeginTransaction()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Begin Transaction");
  cout << "### -- initiating Begin " << m_node << endl;
  m_xact_status = XactAbortRequestGeneratorStatus_Waiting;
  sequencer()->makeRequest(CacheMsg(Address(physical_address_t(0)), Address(physical_address_t(0)), CacheRequestType_BEGIN_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
  transactionManager()->beginTransaction();
}

void XactAbortRequestGenerator::initiateStore(Address addr)
{
  //DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Store");
  //cout << "### -- initiating Store " << m_node << endl;
  m_xact_status = XactAbortRequestGeneratorStatus_Waiting;
  ASSERT(transactionManager()->inTransaction(0));
  sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_ST_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
}

void XactAbortRequestGenerator::initiateCommit()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Commit ");
  cout << "### -- initiating Commit " << m_node << endl;

  m_xact_status = XactAbortRequestGeneratorStatus_Waiting;
  sequencer()->makeRequest(CacheMsg(Address(physical_address_t(0)), Address(physical_address_t(0)), CacheRequestType_COMMIT_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
  transactionManager()->commitTransaction();
}

void XactAbortRequestGenerator::initiateLoad(Address addr)
{
  //DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Load ");
  //cout << "### -- initiating Load " << m_node << endl;
  m_xact_status = XactAbortRequestGeneratorStatus_Waiting;
  ASSERT(transactionManager()->inTransaction(0));
  sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_LD_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
}

void XactAbortRequestGenerator::initiateInc(Address addr)
{
  //DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Load ");
  //cout << "### -- initiating Inc " << m_node << endl;
  m_register++;
  m_xact_status = XactAbortRequestGeneratorStatus_Ready;
  g_eventQueue_ptr->scheduleEvent(this, holdTime());
  m_pc++;
}

void XactAbortRequestGenerator::checkCorrectness(){
  // Execute the transaction on the test array
  int testPC = 0;
  bool done = false;
  for(int i=0; i<XACT_LENGTH && !done; ++i){
    Opcode op = m_instructions[i].getOpcode();
    Address addr = m_instructions[i].getAddress();
    ASSERT(addr.getAddress() < testArray.size());
    uint8 reg_val;
    switch(op){
    case Opcode_BEGIN:
      reg_val = 0;
      break; // do nothing
    case Opcode_LD:
      reg_val = testArray[addr.getAddress()];
      //cout << m_node << " LD: " << addr << ", " << (int) reg_val << endl;
      break;
    case Opcode_INC:
      reg_val++;
      //cout << m_node << " INC: " << (int) reg_val << endl;
      break;
    case Opcode_ST:
      testArray[addr.getAddress()] = reg_val;
      //cout << m_node << " ST: " << addr << ", " << (int) reg_val << endl;
      break;
    case Opcode_COMMIT:
      done = true;
      break;
    default:
      ERROR_MSG("Invalid Opcode.");
    };
  }

  bool success = true;
  uint8 ruby_value;
  bool found = false;
  for(int i=0; i<XACT_LENGTH && !done; ++i){
    Opcode op = m_instructions[i].getOpcode();
    Address addr = m_instructions[i].getAddress();

    uint8 reg_val;
    switch(op){
    case Opcode_BEGIN:
    case Opcode_INC:
      break; // do nothing
    case Opcode_LD:
    case Opcode_ST:
      found = sequencer()->getRubyMemoryValue(m_instructions[i].getAddress(), (char *) &ruby_value, 1);
      assert(found);

      if (ruby_value != testArray[i]){
        success = false;
        WARN_MSG("DATA MISMATCH!");
        WARN_EXPR((int) ruby_value);
        WARN_EXPR((int) testArray[i]);
        WARN_EXPR(i);
        assert(success);
      }
      break;
    case Opcode_COMMIT:
      done = true;
      break;
    default:
      ERROR_MSG("Invalid Opcode.");
    };
  }
  cout << m_node << " CORRECT!" << endl;
}

Sequencer* XactAbortRequestGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

TransactionManager* XactAbortRequestGenerator::transactionManager() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getTransactionManager(m_node%RubyConfig::numberOfProcsPerChip());
}

void XactAbortRequestGenerator::print(ostream& out) const
{
}

void XactAbortRequestGenerator::abortTransaction(){
  cout << "  " << m_node << "    *** ABORT! ***" << endl;
  //m_pc = 0;
  //m_register = 5;
  m_xact_status = XactAbortRequestGeneratorStatus_Aborted;
}

#endif //XACT_MEM
