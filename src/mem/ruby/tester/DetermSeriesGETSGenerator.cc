
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

#include "DetermSeriesGETSGenerator.hh"
#include "DetermSeriesGETSGeneratorStatus.hh"
#include "LockStatus.hh"
#include "Sequencer.hh"
#include "System.hh"
#include "RubyConfig.hh"
#include "SubBlock.hh"
#include "DeterministicDriver.hh"
#include "Chip.hh"

DetermSeriesGETSGenerator::DetermSeriesGETSGenerator(NodeID node, DeterministicDriver& driver) :
  m_driver(driver)
{
  m_status = DetermSeriesGETSGeneratorStatus_Thinking;
  m_last_transition = 0;
  m_node = node;
  m_address = Address(9999);  // initialize to null value
  m_counter = 0;

  // don't know exactly when this node needs to request so just guess randomly
  g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

DetermSeriesGETSGenerator::~DetermSeriesGETSGenerator()
{
}

void DetermSeriesGETSGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  // determine if this node is next for the SeriesGETS round robin request
  if (m_status == DetermSeriesGETSGeneratorStatus_Thinking) {
    if (m_driver.isLoadReady(m_node)) {
      pickAddress();
      m_status = DetermSeriesGETSGeneratorStatus_Load_Pending;  // Load Pending
      m_last_transition = g_eventQueue_ptr->getTime();
      initiateLoad();  // SeriesGETS
    } else { // I'll check again later
      g_eventQueue_ptr->scheduleEvent(this, thinkTime());
    }
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }

}

void DetermSeriesGETSGenerator::performCallback(NodeID proc, SubBlock& data)
{
  Address address = data.getAddress();
  assert(proc == m_node);
  assert(address == m_address);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  if (m_status == DetermSeriesGETSGeneratorStatus_Load_Pending) {
    m_driver.recordLoadLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    data.writeByte(m_node);
    m_driver.loadCompleted(m_node, data.getAddress());  // advance the load queue

    m_counter++;
    // do we still have more requests to complete before the next proc starts?
    if (m_counter < g_tester_length*g_NUM_COMPLETIONS_BEFORE_PASS) {
      m_status = DetermSeriesGETSGeneratorStatus_Thinking;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      m_driver.reportDone();
      m_status = DetermSeriesGETSGeneratorStatus_Done;
      m_last_transition = g_eventQueue_ptr->getTime();
    }

  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int DetermSeriesGETSGenerator::thinkTime() const
{
  return g_think_time;
}

int DetermSeriesGETSGenerator::waitTime() const
{
  return g_wait_time;
}

void DetermSeriesGETSGenerator::pickAddress()
{
  assert(m_status == DetermSeriesGETSGeneratorStatus_Thinking);

  m_address = m_driver.getNextLoadAddr(m_node);
}

void DetermSeriesGETSGenerator::initiateLoad()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Load");
  sequencer()->makeRequest(CacheMsg(m_address, m_address, CacheRequestType_IFETCH, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, Address(0), 0 /* only 1 SMT thread */));
}

Sequencer* DetermSeriesGETSGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

void DetermSeriesGETSGenerator::print(ostream& out) const
{
}

