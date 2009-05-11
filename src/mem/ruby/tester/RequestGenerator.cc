
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

#include "mem/ruby/tester/RequestGenerator.hh"
#include "mem/protocol/RequestGeneratorStatus.hh"
#include "mem/protocol/LockStatus.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/tester/SyntheticDriver.hh"
#include "mem/protocol/Chip.hh"

RequestGenerator::RequestGenerator(NodeID node, SyntheticDriver& driver) :
  m_driver(driver)
{
  m_status = RequestGeneratorStatus_Thinking;
  m_last_transition = 0;
  m_node = node;
  pickAddress();
  m_counter = 0;

  //g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

RequestGenerator::~RequestGenerator()
{
}

void RequestGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  if (m_status == RequestGeneratorStatus_Thinking) {
    m_status = RequestGeneratorStatus_Test_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateTest();  // Test
  } else if (m_status == RequestGeneratorStatus_Holding) {
    m_status = RequestGeneratorStatus_Release_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateRelease();  // Release
  } else if (m_status == RequestGeneratorStatus_Before_Swap) {
    m_status = RequestGeneratorStatus_Swap_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateSwap();
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

void RequestGenerator::performCallback(NodeID proc, SubBlock& data)
{
  Address address = data.getAddress();
  assert(proc == m_node);
  assert(address == m_address);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  if (m_status == RequestGeneratorStatus_Test_Pending) {
    //    m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    if (data.readByte() == LockStatus_Locked) {
      // Locked - keep spinning
      m_status = RequestGeneratorStatus_Thinking;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      // Unlocked - try the swap
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_status = RequestGeneratorStatus_Before_Swap;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    }
  } else if (m_status == RequestGeneratorStatus_Swap_Pending) {
    m_driver.recordSwapLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    if (data.readByte() == LockStatus_Locked) {
      // We failed to aquire the lock
      m_status = RequestGeneratorStatus_Thinking;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      // We acquired the lock
      data.writeByte(LockStatus_Locked);
      m_status = RequestGeneratorStatus_Holding;
      m_last_transition = g_eventQueue_ptr->getTime();
      DEBUG_MSG(TESTER_COMP, HighPrio, "Acquired");
      DEBUG_EXPR(TESTER_COMP, HighPrio, proc);
      DEBUG_EXPR(TESTER_COMP, HighPrio, g_eventQueue_ptr->getTime());
      g_eventQueue_ptr->scheduleEvent(this, holdTime());
    }
  } else if (m_status == RequestGeneratorStatus_Release_Pending) {
    m_driver.recordReleaseLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    // We're releasing the lock
    data.writeByte(LockStatus_Unlocked);

    m_counter++;
    if (m_counter < g_tester_length) {
      m_status = RequestGeneratorStatus_Thinking;
      m_last_transition = g_eventQueue_ptr->getTime();
      pickAddress();
      g_eventQueue_ptr->scheduleEvent(this, thinkTime());
    } else {
      m_driver.reportDone();
      m_status = RequestGeneratorStatus_Done;
      m_last_transition = g_eventQueue_ptr->getTime();
    }
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int RequestGenerator::thinkTime() const
{
  return g_think_time;
}

int RequestGenerator::waitTime() const
{
  return g_wait_time;
}

int RequestGenerator::holdTime() const
{
  return g_hold_time;
}

void RequestGenerator::pickAddress()
{
  assert(m_status == RequestGeneratorStatus_Thinking);
  m_address = m_driver.pickAddress(m_node);
}

void RequestGenerator::initiateTest()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Test");

  Addr data_addr = m_address.getAddress();
  Request request(0, data_addr, 1, Flags<unsigned int>(), 1, 0, 0);
  MemCmd::Command command;
  command = MemCmd::ReadReq;

  Packet pkt(&request, command, 0); // TODO -- make dest a real NodeID

  sequencer()->makeRequest(&pkt);
}

void RequestGenerator::initiateSwap()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Swap");

  Addr data_addr = m_address.getAddress();
  Request request(0, data_addr, 1, Flags<unsigned int>(), 2, 0, 0);
  MemCmd::Command command;
  command = MemCmd::SwapReq;

  Packet pkt(&request, command, 0); // TODO -- make dest a real NodeID

  sequencer()->makeRequest(&pkt);
}

void RequestGenerator::initiateRelease()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Release");

  Addr data_addr = m_address.getAddress();
  Request request(0, data_addr, 1, Flags<unsigned int>(), 3, 0, 0);
  MemCmd::Command command;
  command = MemCmd::WriteReq;

  Packet pkt(&request, command, 0); // TODO -- make dest a real NodeID

  sequencer()->makeRequest(&pkt);
}

Sequencer* RequestGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

void RequestGenerator::print(ostream& out) const
{
  out << "[RequestGenerator]" << endl;
}

