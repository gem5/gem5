
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

#include "mem/ruby/tester/DetermSeriesGETSGenerator.hh"
#include "mem/protocol/DetermSeriesGETSGeneratorStatus.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"

DetermSeriesGETSGenerator::DetermSeriesGETSGenerator(NodeID node, DeterministicDriver& driver) :
  m_driver(driver)
{
  m_status = DetermSeriesGETSGeneratorStatus_Thinking;
  m_last_transition = 0;
  m_node = node;
  m_address = Address(9999);  // initialize to null value
  m_counter = 0;


  // don't know exactly when this node needs to request so just guess randomly
  m_driver.eventQueue->scheduleEvent(this, 1+(random() % 200));
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
      m_last_transition = m_driver.eventQueue->getTime();
      initiateLoad();  // SeriesGETS
    } else { // I'll check again later
      m_driver.eventQueue->scheduleEvent(this, thinkTime());
    }
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }

}

void DetermSeriesGETSGenerator::performCallback(NodeID proc, Address address)
{
  assert(proc == m_node);
  assert(address == m_address);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);

  if (m_status == DetermSeriesGETSGeneratorStatus_Load_Pending) {
    m_driver.recordLoadLatency(m_driver.eventQueue->getTime() - m_last_transition);
    //data.writeByte(m_node);
    m_driver.loadCompleted(m_node, address);  // advance the load queue

    m_counter++;
    // do we still have more requests to complete before the next proc starts?
    if (m_counter < m_driver.m_tester_length*m_driver.m_numCompletionsPerNode) {
      m_status = DetermSeriesGETSGeneratorStatus_Thinking;
      m_last_transition = m_driver.eventQueue->getTime();
      m_driver.eventQueue->scheduleEvent(this, waitTime());
    } else {
      m_driver.reportDone();
      m_status = DetermSeriesGETSGeneratorStatus_Done;
      m_last_transition = m_driver.eventQueue->getTime();
    }

  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int DetermSeriesGETSGenerator::thinkTime() const
{
  return m_driver.m_think_time;
}

int DetermSeriesGETSGenerator::waitTime() const
{
  return m_driver.m_wait_time;
}

void DetermSeriesGETSGenerator::pickAddress()
{
  assert(m_status == DetermSeriesGETSGeneratorStatus_Thinking);

  m_address = m_driver.getNextLoadAddr(m_node);
}

void DetermSeriesGETSGenerator::initiateLoad()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Load");
  //sequencer()->makeRequest(CacheMsg(m_address, m_address, CacheRequestType_IFETCH, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, Address(0), 0 /* only 1 SMT thread */));

  uint8_t *read_data = new uint8_t[64];

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_node);

  int64_t request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m_address.getAddress(), read_data, 64, 0, RubyRequestType_LD, RubyAccessMode_Supervisor));

  //delete [] read_data;

  ASSERT(m_driver.requests.find(request_id) == m_driver.requests.end());
  m_driver.requests.insert(make_pair(request_id, make_pair(m_node, m_address)));
}

void DetermSeriesGETSGenerator::print(ostream& out) const
{
}

