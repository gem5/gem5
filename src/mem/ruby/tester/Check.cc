
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
 */

#include "Check.hh"
#include "Sequencer.hh"
#include "System.hh"
#include "SubBlock.hh"
#include "Chip.hh"

Check::Check(const Address& address, const Address& pc)
{
  m_status = TesterStatus_Idle;

  pickValue();
  pickInitiatingNode();
  changeAddress(address);
  m_pc = pc;
  m_access_mode = AccessModeType(random() % AccessModeType_NUM);
  m_store_count = 0;
}

void Check::initiate()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating");
  DEBUG_EXPR(TESTER_COMP, MedPrio, *this);

  // current CMP protocol doesn't support prefetches
  if (!Protocol::m_CMP && (random() & 0xf) == 0) { // 1 in 16 chance
    initiatePrefetch(); // Prefetch from random processor
  }

  if(m_status == TesterStatus_Idle) {
    initiateAction();
  } else if(m_status == TesterStatus_Ready) {
    initiateCheck();
  } else {
    // Pending - do nothing
    DEBUG_MSG(TESTER_COMP, MedPrio, "initiating action/check - failed: action/check is pending\n");
  }
}

void Check::initiatePrefetch(Sequencer* targetSequencer_ptr)
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating prefetch");

  CacheRequestType type;
  if ((random() & 0x7) != 0) { // 1 in 8 chance
    if ((random() & 0x1) == 0) { // 50% chance
      type = CacheRequestType_LD;
    } else {
      type = CacheRequestType_IFETCH;
    }
  } else {
    type = CacheRequestType_ST;
  }
  assert(targetSequencer_ptr != NULL);
  CacheMsg request(m_address, m_address, type, m_pc, m_access_mode, 0, PrefetchBit_Yes, 0, Address(0), 0 /* only 1 SMT thread */);
  if (targetSequencer_ptr->isReady(request)) {
    targetSequencer_ptr->makeRequest(request);
  }
}

void Check::initiatePrefetch()
{
  // Any sequencer can issue a prefetch for this address
  Sequencer* targetSequencer_ptr = g_system_ptr->getChip(random() % RubyConfig::numberOfChips())->getSequencer(random() % RubyConfig::numberOfProcsPerChip());
  assert(targetSequencer_ptr != NULL);
  initiatePrefetch(targetSequencer_ptr);
}

void Check::initiateAction()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Action");
  assert(m_status == TesterStatus_Idle);

  CacheRequestType type = CacheRequestType_ST;
  if ((random() & 0x1) == 0) { // 50% chance
    type = CacheRequestType_ATOMIC;
  }

  CacheMsg request(Address(m_address.getAddress()+m_store_count), Address(m_address.getAddress()+m_store_count), type, m_pc, m_access_mode, 1, PrefetchBit_No, 0, Address(0), 0 /* only 1 SMT thread */);
  Sequencer* sequencer_ptr = initiatingSequencer();
  if (sequencer_ptr->isReady(request) == false) {
    DEBUG_MSG(TESTER_COMP, MedPrio, "failed to initiate action - sequencer not ready\n");
  } else {
    DEBUG_MSG(TESTER_COMP, MedPrio, "initiating action - successful\n");
    DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);
    m_status = TesterStatus_Action_Pending;
    sequencer_ptr->makeRequest(request);
  }
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);
}

void Check::initiateCheck()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Check");
  assert(m_status == TesterStatus_Ready);

  CacheRequestType type = CacheRequestType_LD;
  if ((random() & 0x1) == 0) { // 50% chance
    type = CacheRequestType_IFETCH;
  }

  CacheMsg request(m_address, m_address, type, m_pc, m_access_mode, CHECK_SIZE, PrefetchBit_No, 0, Address(0), 0 /* only 1 SMT thread */);
  Sequencer* sequencer_ptr = initiatingSequencer();
  if (sequencer_ptr->isReady(request) == false) {
    DEBUG_MSG(TESTER_COMP, MedPrio, "failed to initiate check - sequencer not ready\n");
  } else {
    DEBUG_MSG(TESTER_COMP, MedPrio, "initiating check - successful\n");
    DEBUG_MSG(TESTER_COMP, MedPrio, m_status);
    m_status = TesterStatus_Check_Pending;
    sequencer_ptr->makeRequest(request);
  }
  DEBUG_MSG(TESTER_COMP, MedPrio, m_status);
}

void Check::performCallback(NodeID proc, SubBlock& data)
{
  Address address = data.getAddress();
  //  assert(getAddress() == address);  // This isn't exactly right since we now have multi-byte checks
  assert(getAddress().getLineAddress() == address.getLineAddress());

  DEBUG_MSG(TESTER_COMP, MedPrio, "Callback");
  DEBUG_EXPR(TESTER_COMP, MedPrio, *this);

  if (m_status == TesterStatus_Action_Pending) {
    DEBUG_MSG(TESTER_COMP, MedPrio, "Action callback");
    // Perform store
    data.setByte(0, m_value+m_store_count); // We store one byte at a time
    m_store_count++;

    if (m_store_count == CHECK_SIZE) {
      m_status = TesterStatus_Ready;
    } else {
      m_status = TesterStatus_Idle;
    }
  } else if (m_status == TesterStatus_Check_Pending) {
    DEBUG_MSG(TESTER_COMP, MedPrio, "Check callback");
    // Perform load/check
    for(int byte_number=0; byte_number<CHECK_SIZE; byte_number++) {
      if (uint8(m_value+byte_number) != data.getByte(byte_number) && (DATA_BLOCK == true)) {
        WARN_EXPR(proc);
        WARN_EXPR(address);
        WARN_EXPR(data);
        WARN_EXPR(byte_number);
        WARN_EXPR((int)m_value+byte_number);
        WARN_EXPR((int)data.getByte(byte_number));
        WARN_EXPR(*this);
        WARN_EXPR(g_eventQueue_ptr->getTime());
        ERROR_MSG("Action/check failure");
      }
    }
    DEBUG_MSG(TESTER_COMP, HighPrio, "Action/check success:");
    DEBUG_EXPR(TESTER_COMP, HighPrio, *this);
    DEBUG_EXPR(TESTER_COMP, MedPrio, data);

    m_status = TesterStatus_Idle;
    pickValue();

  } else {
    WARN_EXPR(*this);
    WARN_EXPR(proc);
    WARN_EXPR(data);
    WARN_EXPR(m_status);
    WARN_EXPR(g_eventQueue_ptr->getTime());
    ERROR_MSG("Unexpected TesterStatus");
  }

  DEBUG_EXPR(TESTER_COMP, MedPrio, proc);
  DEBUG_EXPR(TESTER_COMP, MedPrio, data);
  DEBUG_EXPR(TESTER_COMP, MedPrio, getAddress().getLineAddress());
  DEBUG_MSG(TESTER_COMP, MedPrio, "Callback done");
  DEBUG_EXPR(TESTER_COMP, MedPrio, *this);
}

void Check::changeAddress(const Address& address)
{
  assert((m_status == TesterStatus_Idle) || (m_status == TesterStatus_Ready));
  m_status = TesterStatus_Idle;
  m_address = address;
  m_store_count = 0;
}

Sequencer* Check::initiatingSequencer() const
{
  return g_system_ptr->getChip(m_initiatingNode/RubyConfig::numberOfProcsPerChip())->getSequencer(m_initiatingNode%RubyConfig::numberOfProcsPerChip());
}

void Check::pickValue()
{
  assert(m_status == TesterStatus_Idle);
  m_status = TesterStatus_Idle;
  //  DEBUG_MSG(TESTER_COMP, MedPrio, m_status);
  DEBUG_MSG(TESTER_COMP, MedPrio, *this);
  m_value = random() & 0xff; // One byte
  //  DEBUG_MSG(TESTER_COMP, MedPrio, m_value);
  DEBUG_MSG(TESTER_COMP, MedPrio, *this);
  m_store_count = 0;
}

void Check::pickInitiatingNode()
{
  assert((m_status == TesterStatus_Idle) || (m_status == TesterStatus_Ready));
  m_status = TesterStatus_Idle;
  DEBUG_MSG(TESTER_COMP, MedPrio, m_status);
  m_initiatingNode = (random() % RubyConfig::numberOfProcessors());
  DEBUG_MSG(TESTER_COMP, MedPrio, m_initiatingNode);
  m_store_count = 0;
}

void Check::print(ostream& out) const
{
  out << "["
      << m_address << ", value: "
      << (int) m_value << ", status: "
      << m_status << ", initiating node: "
      << m_initiatingNode << ", store_count: "
      << m_store_count
      << "]" << flush;
}
