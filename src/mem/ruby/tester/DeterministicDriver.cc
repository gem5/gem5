
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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/tester/SpecifiedGenerator.hh"
#include "mem/ruby/tester/DetermGETXGenerator.hh"
#include "mem/ruby/tester/DetermInvGenerator.hh"
#include "mem/ruby/tester/DetermSeriesGETSGenerator.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/protocol/Chip.hh"
#include "mem/packet.hh"

DeterministicDriver::DeterministicDriver(RubySystem* sys_ptr)
{
  m_finish_time = 0;
  m_last_issue = -11;
  m_done_counter = 0;
  m_loads_completed = 0;
  m_stores_completed = 0;

  m_numCompletionsPerNode = g_NUM_COMPLETIONS_BEFORE_PASS;

  m_last_progress_vector.setSize(RubyConfig::numberOfProcessors());
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    m_last_progress_vector[i] = 0;
  }

  m_load_vector.setSize(g_deterministic_addrs);
  for (int i=0; i<m_load_vector.size(); i++) {
    m_load_vector[i] = -1;  // No processor last held it
  }

  m_store_vector.setSize(g_deterministic_addrs);
  for (int i=0; i<m_store_vector.size(); i++) {
    m_store_vector[i] = -1;  // No processor last held it
  }

  m_generator_vector.setSize(RubyConfig::numberOfProcessors());

  SpecifiedGeneratorType generator = string_to_SpecifiedGeneratorType(g_SpecifiedGenerator);

  for (int i=0; i<m_generator_vector.size(); i++) {
    switch (generator) {
    case SpecifiedGeneratorType_DetermGETXGenerator:
      m_generator_vector[i] = new DetermGETXGenerator(i, *this);
      break;
    case SpecifiedGeneratorType_DetermSeriesGETSGenerator:
      m_generator_vector[i] = new DetermSeriesGETSGenerator(i, *this);
      break;
    case SpecifiedGeneratorType_DetermInvGenerator:
      m_generator_vector[i] = new DetermInvGenerator(i, *this);
      break;
    default:
      ERROR_MSG("Unexpected specified generator type");
    }
  }

  // add the tester consumer to the global event queue
  g_eventQueue_ptr->scheduleEvent(this, 1);
}

DeterministicDriver::~DeterministicDriver()
{
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    delete m_generator_vector[i];
  }
}

void
DeterministicDriver::hitCallback(Packet * pkt)
{
  NodeID proc = pkt->req->contextId();
  SubBlock data(Address(pkt->getAddr()), pkt->req->getSize());
  if (pkt->hasData()) {
    for (int i = 0; i < pkt->req->getSize(); i++) {
      data.setByte(i, *(pkt->getPtr<uint8>()+i));
    }
  }
  m_generator_vector[proc]->performCallback(proc, data);
  m_last_progress_vector[proc] = g_eventQueue_ptr->getTime();
}

bool DeterministicDriver::isStoreReady(NodeID node)
{
  return isAddrReady(node, m_store_vector);
}

bool DeterministicDriver::isStoreReady(NodeID node, Address addr)
{
  return isAddrReady(node, m_store_vector, addr);
}

bool DeterministicDriver::isLoadReady(NodeID node)
{
  return isAddrReady(node, m_load_vector);
}

bool DeterministicDriver::isLoadReady(NodeID node, Address addr)
{
  return isAddrReady(node, m_load_vector, addr);
}

// searches for any address in the addr_vector
bool DeterministicDriver::isAddrReady(NodeID node, Vector<NodeID> addr_vector)
{
  for (int i=0; i<addr_vector.size(); i++) {
    if (((addr_vector[i]+1)%RubyConfig::numberOfProcessors() == node) &&
        (m_loads_completed+m_stores_completed >= m_numCompletionsPerNode*node) && // is this node next
        (g_eventQueue_ptr->getTime() >= m_last_issue + 10)) { // controll rate of requests
      return true;
    }
  }
  return false;
}

// test for a particular addr
bool DeterministicDriver::isAddrReady(NodeID node, Vector<NodeID> addr_vector, Address addr)
{
  int addr_number = addr.getAddress()/RubyConfig::dataBlockBytes();

  ASSERT ((addr_number >= 0) && (addr_number < addr_vector.size()));

  if (((addr_vector[addr_number]+1)%RubyConfig::numberOfProcessors() == node) &&
      (m_loads_completed+m_stores_completed >= m_numCompletionsPerNode*node) && // is this node next
      (g_eventQueue_ptr->getTime() >= m_last_issue + 10)) { // controll rate of requests
    return true;
  } else {
    return false;
  }
}

void DeterministicDriver::loadCompleted(NodeID node, Address addr)
{
  m_loads_completed++;
  setNextAddr(node, addr, m_load_vector);
}

void DeterministicDriver::storeCompleted(NodeID node, Address addr)
{
  m_stores_completed++;
  setNextAddr(node, addr, m_store_vector);
}

void DeterministicDriver::setNextAddr(NodeID node, Address addr, Vector<NodeID>& addr_vector)
{
  // mark the addr vector that this proc was the last to use the particular address
  int addr_number = addr.getAddress()/RubyConfig::dataBlockBytes();
  addr_vector[addr_number] = node;
}

Address DeterministicDriver::getNextLoadAddr(NodeID node)
{
  return getNextAddr(node, m_load_vector);
}

Address DeterministicDriver::getNextStoreAddr(NodeID node)
{
  return getNextAddr(node, m_store_vector);
}

Address DeterministicDriver::getNextAddr(NodeID node, Vector<NodeID> addr_vector)
{

  // This method deterministically picks the next addr the node should acquirer
  // The addrs cycle through according to NodeID 0->1->...->lastID->0...

  Address addr;

  // should only be called if we know a addr is ready for the node
  ASSERT(isAddrReady(node, addr_vector));

  for (int addr_number=0; addr_number<addr_vector.size(); addr_number++) {
  //for (int addr_number=addr_vector.size()-1; addr_number>0; addr_number--) {

    // is this node next in line for the addr
    if (((addr_vector[addr_number]+1)%RubyConfig::numberOfProcessors()) == node) {

      // One addr per cache line
      addr.setAddress(addr_number * RubyConfig::dataBlockBytes());
    }
  }

  m_last_issue = g_eventQueue_ptr->getTime();

  return addr;
}


void DeterministicDriver::reportDone()
{
  m_done_counter++;
  if ((m_done_counter == RubyConfig::numberOfProcessors())) {
      //|| (m_done_counter == g_tester_length)) {
    m_finish_time = g_eventQueue_ptr->getTime();
  }
}

void DeterministicDriver::recordLoadLatency(Time time)
{
  m_load_latency.add(time);
}

void DeterministicDriver::recordStoreLatency(Time time)
{
  m_store_latency.add(time);
}

void DeterministicDriver::wakeup()
{
  //  checkForDeadlock();
  if (m_done_counter < RubyConfig::numberOfProcessors()) {
    g_eventQueue_ptr->scheduleEvent(this, g_DEADLOCK_THRESHOLD);
  }
}

void DeterministicDriver::checkForDeadlock()
{
  int size = m_last_progress_vector.size();
  Time current_time = g_eventQueue_ptr->getTime();
  for (int processor=0; processor<size; processor++) {
    if ((current_time - m_last_progress_vector[processor]) > g_DEADLOCK_THRESHOLD) {
      WARN_EXPR(processor);
#ifndef NDEBUG
      Sequencer* seq_ptr = g_system_ptr->getChip(processor/RubyConfig::numberOfProcsPerChip())->getSequencer(processor%RubyConfig::numberOfProcsPerChip());
#endif
      assert(seq_ptr != NULL);
      //     if (seq_ptr->isRequestPending()) {
      //       WARN_EXPR(seq_ptr->pendingAddress());
      //      }
      WARN_EXPR(current_time);
      WARN_EXPR(m_last_progress_vector[processor]);
      WARN_EXPR(current_time - m_last_progress_vector[processor]);
      ERROR_MSG("Deadlock detected.");
    }
  }
}

void DeterministicDriver::printStats(ostream& out) const
{
  out << endl;
  out << "DeterministicDriver Stats" << endl;
  out << "---------------------" << endl;

  out << "finish_time: " << m_finish_time << endl;
  out << "load_latency: " << m_load_latency << endl;
  out << "store_latency: " << m_store_latency << endl;
}

void DeterministicDriver::print(ostream& out) const
{
}
