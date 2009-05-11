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
 * $Id: XactRequestGenerator.C 1.7 05/09/22 16:49:19-05:00 xu@s0-29.cs.wisc.edu $
 *
 */

#include "mem/ruby/tester/XactRequestGenerator.hh"
#include "mem/protocol/LockStatus.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/tester/SyntheticDriver.hh"
#include "mem/protocol/Chip.hh"
#include "mem/ruby/tester/Instruction.hh"

XactRequestGenerator::XactRequestGenerator(NodeID node, SyntheticDriver& driver) :
  m_driver(driver), RequestGenerator(node, driver)
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, "#### -- Creating XactRequestGenerator\n");
  cout << "#### -- Creating XactRequestGenerator " << node << endl;

  assert(XACT_SIZE > 0);
  testArray = new uint8[g_MEMORY_SIZE_BYTES];;
  // Create instructions
  m_instructions = new Instruction[XACT_LENGTH];
  newTransaction(true);

  m_status = XactRequestGeneratorStatus_Ready;
  m_last_transition = 0;
  m_node = node;
  //pickAddress();
  m_counter = 0;
  m_register = 5;
  m_pc = 0;

  m_abortPending = false;
  g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

void XactRequestGenerator::newTransaction(bool init){
  // important: reset abort flag
  m_abortPending = false;

  int depth = 0;
  bool prev_ldst = false;
  m_size = (random() % (XACT_LENGTH-2)) + 2;
  cout << "XactRequestGenerator::newTransaction m_size=" << m_size << endl;
  ASSERT(m_size >= 2);
  if (!init)
    ASSERT(!transactionManager()->inTransaction(0));
  m_transaction = (random() % 2);

  if(m_transaction){
    cout << "  " << m_node << " new transaction " << endl;
  } else {
    cout << "  " << m_node << " new NON-transaction " << endl;
  }

  cout << "***INSTR STREAM ";
  for(int i=0; i<m_size; ++i){
    if (i == 0 && m_transaction){ // new xact must start with begin
      m_instructions[i].init(Opcode_BEGIN, Address(1));
      depth++;
      cout << "begin ";
      prev_ldst = false;
    } else if (i == m_size - 1){
      if(m_transaction) { // new xact must end with commit
        m_instructions[i].init(Opcode_COMMIT, Address(1));
        depth--;
        cout << "commit ";
      } else {
        m_instructions[i].init(Opcode_DONE, Address(1));
        cout << "done ";
      }
    } else {
      int selectAction;
      if (!m_transaction) { // non-xact: must choose op
        selectAction = 1;
      } else { // xact
        if (depth == m_size - i) { // must choose commit
          selectAction = 0;
        } else if (prev_ldst) { // only choose xact if intervenient ld/st
          if (m_size - i < depth + 3) { // can choose op or
                                        // commit (can't choose
                                        // begin)
            selectAction = (random() % 2);
          } else if (depth == 0) { // can choose begin or op (can't
                                   // choose commit)
            selectAction = (random() % 2);
            if (selectAction == 0) selectAction = 2;
          } else { // can choose begin, op, or commit
            selectAction = (random() % 3);
          }
        } else {
          selectAction = 1;
        }
      }

      physical_address_t address;
      int selectOpcode;
      switch (selectAction) {
      case 0: // commit
        m_instructions[i].init(Opcode_COMMIT, Address(1));
        depth--;
        cout << "commit ";
        prev_ldst = false;
        break;
      case 1:
        address = (random() % XACT_SIZE);
        //cout << "address: " << address << endl;
        //cout << "XACT_SIZE: " << XACT_SIZE << endl;

        //assert(address < XACT_SIZE);
        //physical_address_t address = 0;
        selectOpcode = random() % 3;
        Opcode op;
        switch(selectOpcode){
        case 0:
          op = Opcode_LD;
          cout << "ld ";
          break;
        case 1:
          op = Opcode_ST;
          cout << "st ";
          break;
        case 2:
          op = Opcode_INC;
          cout << "inc ";
          break;
        default:
          assert(false);
        };
        assert(op < Opcode_NUM_OPCODES);
        m_instructions[i].init(op, Address(address));
        prev_ldst = true;
        break;
      case 2:
        m_instructions[i].init(Opcode_BEGIN, Address(1));
        depth++;
        cout << "begin ";
        prev_ldst = false;
        break;
      default:
        assert(false);
      };
    }
  }
  cout << endl;
  if(m_transaction){
    ASSERT(m_instructions[0].getOpcode() == Opcode_BEGIN);
  }
}

XactRequestGenerator::~XactRequestGenerator()
{
  if(testArray){
    delete [] testArray;
    testArray = NULL;
  }
  delete m_instructions;
}

void XactRequestGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  assert(m_status == XactRequestGeneratorStatus_Ready || m_status == XactRequestGeneratorStatus_Aborted);
  m_status = XactRequestGeneratorStatus_Blocked;

  m_last_transition = g_eventQueue_ptr->getTime();
  execute();
}

void XactRequestGenerator::execute(){
  cout << "XactRequestGenerator::execute m_node=" << m_node << " m_pc=" << m_pc << " m_size=" << m_size << endl;
  assert(m_pc >= 0);
  assert(m_pc < m_size);
  assert(m_pc < XACT_LENGTH);

  Instruction current = m_instructions[m_pc];
  switch (current.getOpcode()){
  case Opcode_BEGIN:
    cout << "  -- begin.";
    initiateBeginTransaction();
    break;
  case Opcode_LD:
    cout << "  -- load: " << current.getAddress();
    initiateLoad(current.getAddress());
    break;
  case Opcode_INC:
    cout << "  -- inc.";
    initiateInc(current.getAddress());
    break;
  case Opcode_ST:
    cout << "  -- store: " << current.getAddress();
    initiateStore(current.getAddress());
    break;
  case Opcode_COMMIT:
    cout << "  -- commit.";
    initiateCommit();
    break;
  case Opcode_DONE:
    cout << " -- done.";
    initiateDone();
    break;
  default:
    WARN_EXPR(current.getOpcode());
    ERROR_MSG("Invalid opcode");
  };
  cout << endl;
}

void XactRequestGenerator::performCallback(NodeID proc, SubBlock& data)
{
  cout << "XactRequestGenerator::performCallback m_node=" << m_node << endl;
  assert(m_status == XactRequestGeneratorStatus_Waiting ||
         m_status == XactRequestGeneratorStatus_Aborted);
  assert(proc == m_node);

  Address address = data.getAddress();
  //assert(address == m_address);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  m_last_transition = g_eventQueue_ptr->getTime();

  cout << "  " << m_node << " in performCallback, pc:" << m_pc
       << ", addr:" << address << endl;


  int depth;
  if(m_status == XactRequestGeneratorStatus_Aborted){
    depth = transactionManager()->postAbortIndex(0);
    m_pc = pc_stack[depth];
    cout << "XactRequestGenerator::performCallback m_node=" << m_node << " setting m_pc=" << m_pc << endl;
    printPcStack(depth);
    m_register = 5;
    m_status = XactRequestGeneratorStatus_Ready;
    g_eventQueue_ptr->scheduleEvent(this, ABORT_RETRY_TIME);
  } else {
    m_status = XactRequestGeneratorStatus_Blocked;
    bool found, outermost;
    uint8 value;
    switch (m_instructions[m_pc].getOpcode()){
    case Opcode_BEGIN:
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_register = 5;
      m_status = XactRequestGeneratorStatus_Ready;
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
      depth = transactionManager()->getDepth(0);
      if (!transactionManager()->isSubsuming(0)) {
        pc_stack[depth - 1] = m_pc;
        cout << "XactRequestGenerator::performCallback m_node=" << m_node << " SETTING PC_STACK" << endl;
        printPcStack(depth);
      }
      m_pc++;
      break;
    case Opcode_LD:
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_register = data.getByte(0);
      //cout << "  " << m_node << " " << g_eventQueue_ptr->getTime() << " Callback--LD: " << (int) m_register << endl;
      m_status = XactRequestGeneratorStatus_Ready;
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

      m_status = XactRequestGeneratorStatus_Ready;
      g_eventQueue_ptr->scheduleEvent(this, thinkTime());
      m_pc++;
      break;
    case Opcode_COMMIT:
      outermost = transactionManager()->getDepth(0) == 1;
      if (outermost) { // about to commit outermost
        m_counter++;
      }

      cout << "  " << m_node << " callback--commit, counter is " << m_counter << " length is: " << g_tester_length << endl;
      // Check for correctness

      checkCorrectness();
      transactionManager()->commitTransaction();

      m_driver.recordReleaseLatency(g_eventQueue_ptr->getTime() - m_last_transition);

      if (outermost) {
        if (m_counter < g_tester_length) {
          m_last_transition = g_eventQueue_ptr->getTime();
          //pickAddress(); // Necessary?

          // Create new random transaction
          cout << "CREATING NEW RANDOM XACT SET" << endl;
          newTransaction(false);
          m_pc = 0;

          m_status = XactRequestGeneratorStatus_Ready;
          g_eventQueue_ptr->scheduleEvent(this, thinkTime());
        } else {
          cout << "  " << m_node << " Done." << endl;
          m_driver.reportDone();
          m_status = XactRequestGeneratorStatus_Done;
        }
      } else {
        m_status = XactRequestGeneratorStatus_Ready;
        g_eventQueue_ptr->scheduleEvent(this, thinkTime());
        m_pc++;
      }
      break;
    default:
      ERROR_MSG("Invalid Opcode");
    };
  }
}

int XactRequestGenerator::thinkTime() const
{
  return g_think_time;
}

int XactRequestGenerator::waitTime() const
{
  return g_wait_time;
}

int XactRequestGenerator::holdTime() const
{
  return g_hold_time;
}

void XactRequestGenerator::pickAddress()
{
  //m_address = m_driver.pickAddress(m_node);
}

void XactRequestGenerator::initiateBeginTransaction()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Begin Transaction");
  cout << "### -- initiating Begin " << m_node << endl;
  m_status = XactRequestGeneratorStatus_Waiting;
  sequencer()->makeRequest(CacheMsg(Address(physical_address_t(0)), Address(physical_address_t(0)), CacheRequestType_BEGIN_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
  transactionManager()->beginTransaction();
}

void XactRequestGenerator::initiateStore(Address addr)
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Store");
  DEBUG_MSG(TESTER_COMP, MedPrio, addr);
  cout << "### -- initiating Store " << m_node << endl;
  m_status = XactRequestGeneratorStatus_Waiting;
  if(m_transaction){
    sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_ST_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
  } else {
    sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_ST, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0),transactionManager()->inExposedAction(0), 0));
  }
}

void XactRequestGenerator::initiateCommit()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Commit ");
  cout << "### -- initiating Commit " << m_node << endl;

  m_status = XactRequestGeneratorStatus_Waiting;
  sequencer()->makeRequest(CacheMsg(Address(physical_address_t(0)), Address(physical_address_t(0)), CacheRequestType_COMMIT_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
}

void XactRequestGenerator::initiateLoad(Address addr)
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Load ");
  DEBUG_MSG(TESTER_COMP, MedPrio, addr);

  m_status = XactRequestGeneratorStatus_Waiting;
  if(m_transaction){
    cout << "### -- initiating Load XACT" << m_node << endl;
    sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_LD_XACT, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0 ));
  } else {
    cout << "### -- initiating Load " << m_node << endl;
    sequencer()->makeRequest(CacheMsg(addr, addr, CacheRequestType_LD, Address(m_pc), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false, Address(0), transactionManager()->getTransactionLevel(0), 0, 0 /* only 1 SMT thread */, transactionManager()->getTimestamp(0), transactionManager()->inExposedAction(0), 0));
  }
}

void XactRequestGenerator::initiateInc(Address addr)
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Inc ");
  DEBUG_MSG(TESTER_COMP, MedPrio, addr);
  cout << "### -- initiating Inc " << m_node << endl;
  m_register++;
  m_status = XactRequestGeneratorStatus_Ready;
  g_eventQueue_ptr->scheduleEvent(this, holdTime());
  m_pc++;
}

void XactRequestGenerator::initiateDone()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "### -- initiating Done ");
  cout << "### -- initiating Done " << m_node << endl;
  newTransaction(false);
  m_pc = 0;
  //m_register = 5;

  m_status = XactRequestGeneratorStatus_Ready;
  g_eventQueue_ptr->scheduleEvent(this, thinkTime());

}

void XactRequestGenerator::checkCorrectness(){
  // Execute the transaction on the test array
  int testPC = 0;
  bool done = false;
  for(int i=0; i<XACT_LENGTH && !done; ++i){
    Opcode op = m_instructions[i].getOpcode();
    Address addr = m_instructions[i].getAddress();

    uint8 reg_val;
    switch(op){
    case Opcode_BEGIN:
      reg_val = 0;
      break; // do nothing
    case Opcode_LD:
      //cout << "\tcheckCorrectness " << m_node << " LD: " << addr << " address = " << hex << addr.getAddress() << endl;
      ASSERT(addr.getAddress() < g_MEMORY_SIZE_BYTES);
      reg_val = testArray[addr.getAddress()];
      //cout << m_node << " LD: " << addr << ", " << (int) reg_val << endl;
      break;
    case Opcode_INC:
      reg_val++;
      //cout << m_node << " INC: " << (int) reg_val << endl;
      break;
    case Opcode_ST:
      //cout << "\tcheckCorrectness " << m_node << " ST: " << addr << " address = " << hex << addr.getAddress() << endl;
      ASSERT(addr.getAddress() < g_MEMORY_SIZE_BYTES);
      testArray[addr.getAddress()] = reg_val;
      //cout << m_node << " ST: " << addr << ", " << (int) reg_val << endl;
      break;
    case Opcode_DONE:
    case Opcode_COMMIT:
      done = true;
      break;
    default:
      ERROR_MSG("Invalid Opcode.");
    };
  }

  bool success = true;
  uint8 ruby_value;
  done = false;
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

Sequencer* XactRequestGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

TransactionManager* XactRequestGenerator::transactionManager() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getTransactionManager(m_node%RubyConfig::numberOfProcsPerChip());
}

void XactRequestGenerator::print(ostream& out) const
{
}

void XactRequestGenerator::abortTransaction(){
  cout << "  " << m_node << "    *** ABORT! ***" << endl;
  m_status = XactRequestGeneratorStatus_Aborted;
}

void XactRequestGenerator::printPcStack(int depth) {
  cout << "XactRequestGenerator::printPcStack m_node=" << m_node << " [";
  for (int i = 0; i < depth; i++) {
    cout << pc_stack[i] << ", ";
  }
  cout << "]" << endl;
}

void XactRequestGenerator::notifySendNack( const Address & physicalAddr, uint64 remote_timestamp, const MachineID & remote_id) {
  Address addr = physicalAddr;
  addr.makeLineAddress();
  TransactionManager * xact_mgr = transactionManager();
  bool isOlder = xact_mgr->isOlder(remote_timestamp, remote_id, addr );
  if(isOlder){
    bool inReadSet = xact_mgr->isInReadSetFilter(physicalAddr, 0);
    bool inWriteSet = xact_mgr->isInWriteSetFilter(physicalAddr, 0);
    // addr should be in perfect or Bloom filter
    ASSERT( inReadSet || inWriteSet );
    cout << "notifySendNack addr = " << addr << " setting POSSIBLE CYCLE " << " my_ts = " << xact_mgr->getTimestamp(0) << " remote_ts = " << remote_timestamp << " proc = " << m_node << endl;
    xact_mgr->setPossibleCycle(addr, L1CacheMachIDToProcessorNum(remote_id), 0 /*remote thread*/ , remote_timestamp, 0 /*our thread*/);
  }
  // otherwise don't set the proc possible cycle flag
}

void XactRequestGenerator::notifyReceiveNack( const Address & physicalAddr, uint64 remote_timestamp, const MachineID & remote_id ) {
  Address addr = physicalAddr;
  addr.makeLineAddress();
  // check whether the possible cycle is set, and whether remote_timestamp is older.
  // we only have 1 SMT thread
  TransactionManager * xact_mgr = transactionManager();
  int local_timestamp = xact_mgr->getTimestamp(0);
  bool possible_cycle = xact_mgr->possibleCycle(0);
  // calculate isOlder() only if possible_cycle is set. This is because isOlder assumes proc is in a transaction
  bool isOlder = false;
  if(possible_cycle){
    isOlder = xact_mgr->isOlder(remote_timestamp, remote_id, addr );
  }

  if(isOlder && possible_cycle){
    // set our pendingAbort flag
    cout << "notifyReceiveNack Setting Abort Pending Flag addr= " << addr << " ID = " << m_node << " possible_cycle = " << possible_cycle << " time = " << g_eventQueue_ptr->getTime() << endl;
    m_abortPending = true;
    assert(possible_cycle);
    assert(isOlder);
  }

  // profile this nack
  xact_mgr->profileNack(addr, IDToInt(L1CacheMachIDToProcessorNum(remote_id)), remote_timestamp, 0, 0);
}

void XactRequestGenerator::notifyReceiveNackFinal(const Address & physicalAddr) {
  Address addr = physicalAddr;
  addr.makeLineAddress();
  TransactionManager * xact_mgr = transactionManager();
  int local_timestamp = xact_mgr->getTimestamp(0);
  bool possible_cycle = xact_mgr->possibleCycle(0);

    // we should still have an active request
  if(m_abortPending){
    cout << "notifyReceiveNackFinal ABORTING addr= " << addr << " ID = " << m_node << " possible_cycle = " << possible_cycle << " time = " << g_eventQueue_ptr->getTime() << endl;
    assert(possible_cycle);

    // we abort
    // Step 1: remove the aborting request from sequencer, and mark it as "Trap" in our request table if needed
    //   Note: by marking request as "Trap" we can simulate HW abort delay
    switch (m_instructions[m_pc].getOpcode()){
    case Opcode_LD:
      sequencer()->readCallbackAbort(addr, 0);
      break;
    case Opcode_ST:
      sequencer()->writeCallbackAbort(addr, 0);
      break;
    default:
      cout << "Invalid Opcode = " << m_instructions[m_pc].getOpcode() << endl;
      ASSERT(0);
    };
    // Step 2: call the abort handler explicitly. If using software handler + trap aborts we wait until
    //         Simics transfers control to us again
    //         Note: it is impossible for this request to be a prefetch, so don't need that check
    xact_mgr->abortTransaction(0, 0, -1 /*dummy remote ID*/, addr);
    //reset the abort flag
    m_abortPending = false;
  }
  else{
    // retry the request
    // figure out whether to retry transactional load or store
    switch (m_instructions[m_pc].getOpcode()){
    case Opcode_LD:
      cout << "RETRYING LOAD " << addr << " of proc = " << m_node << " transactionLevel = " << xact_mgr->getTransactionLevel(0) << " time = " << g_eventQueue_ptr->getTime() << endl;
      sequencer()->issueInterProcLoadRetryRequest(addr, 0);
      break;
    case Opcode_ST:
      cout << "RETRYING STORE " << addr << " of proc = " << m_node << " transactionLevel = " << xact_mgr->getTransactionLevel(0) << " time = " << g_eventQueue_ptr->getTime() << endl;
      sequencer()->issueInterProcStoreRetryRequest(addr, 0);
      break;
    default:
      cout << "Invalid Opcode = " << m_instructions[m_pc].getOpcode() << endl;
      ASSERT(0);
    };
  }
}
