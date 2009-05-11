
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

// This Deterministic Generator generates GETX requests for all nodes in the system
// The GETX requests are generated one at a time in round-robin fashion 0...1...2...etc.

#include "mem/ruby/tester/DetermGETXGenerator.hh"
#include "mem/protocol/DetermGETXGeneratorStatus.hh"
#include "mem/protocol/LockStatus.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/protocol/Chip.hh"
#include "mem/packet.hh"

DetermGETXGenerator::DetermGETXGenerator(NodeID node, DeterministicDriver& driver) :
  m_driver(driver)
{
  m_status = DetermGETXGeneratorStatus_Thinking;
  m_last_transition = 0;
  m_node = node;
  m_address = Address(9999);  // initialize to null value
  m_counter = 0;

  // don't know exactly when this node needs to request so just guess randomly
  g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

DetermGETXGenerator::~DetermGETXGenerator()
{
}

void DetermGETXGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  // determine if this node is next for the GETX round robin request
  if (m_status == DetermGETXGeneratorStatus_Thinking) {
    if (m_driver.isStoreReady(m_node)) {
      pickAddress();
      m_status = DetermGETXGeneratorStatus_Store_Pending;  // Store Pending
      m_last_transition = g_eventQueue_ptr->getTime();
      initiateStore();  // GETX
    } else { // I'll check again later
      g_eventQueue_ptr->scheduleEvent(this, thinkTime());
    }
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }

}

void DetermGETXGenerator::performCallback(NodeID proc, SubBlock& data)
{
  Address address = data.getAddress();
  assert(proc == m_node);
  assert(address == m_address);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  if (m_status == DetermGETXGeneratorStatus_Store_Pending) {
    m_driver.recordStoreLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    data.writeByte(m_node);
    m_driver.storeCompleted(m_node, data.getAddress());  // advance the store queue

    m_counter++;
    if (m_counter < g_tester_length) {
      m_status = DetermGETXGeneratorStatus_Thinking;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      m_driver.reportDone();
      m_status = DetermGETXGeneratorStatus_Done;
      m_last_transition = g_eventQueue_ptr->getTime();
    }

  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int DetermGETXGenerator::thinkTime() const
{
  return g_think_time;
}

int DetermGETXGenerator::waitTime() const
{
  return g_wait_time;
}

void DetermGETXGenerator::pickAddress()
{
  assert(m_status == DetermGETXGeneratorStatus_Thinking);

  m_address = m_driver.getNextStoreAddr(m_node);
}

void DetermGETXGenerator::initiateStore()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Store");

  Addr data_addr = m_address.getAddress();
  Request request(0, data_addr, 1, Flags<unsigned int>(), 3, 0, 0);
  MemCmd::Command command;
  command = MemCmd::WriteReq;

  Packet pkt(&request, command, 0); // TODO -- make dest a real NodeID

  sequencer()->makeRequest(&pkt);
}

Sequencer* DetermGETXGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

void DetermGETXGenerator::print(ostream& out) const
{
}

