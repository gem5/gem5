
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

#include <cassert>

#include "base/misc.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
//#include "DMAGenerator.hh"
#include "mem/ruby/tester/DetermGETXGenerator.hh"

using namespace std;

#define DATA_BLOCK_BYTES 64

DeterministicDriver::DeterministicDriver(string generator_type, int num_completions, int num_procs, Time g_think_time, Time g_wait_time, int g_tester_length)
{
  eventQueue = new RubyEventQueue; 
  m_finish_time = 0;
  m_last_issue = -11;
  m_done_counter = 0;
  m_loads_completed = 0;
  m_stores_completed = 0;
  
  m_numCompletionsPerNode = num_completions;
  m_num_procs = num_procs;
  m_think_time = g_think_time;
  m_wait_time = g_wait_time;
  m_tester_length = g_tester_length;
  
  m_last_progress_vector.resize(num_procs);
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    m_last_progress_vector[i] = 0;
  }

  m_load_vector.resize(10);
  for (int i=0; i<m_load_vector.size(); i++) {
    m_load_vector[i] = -1;  // No processor last held it
  }

  m_store_vector.resize(10);
  for (int i=0; i<m_store_vector.size(); i++) {
    m_store_vector[i] = -1;  // No processor last held it
  }

  m_generator_vector.resize(num_procs);

  int generator = string_to_SpecifiedGeneratorType(generator_type);

  for (int i=0; i<m_generator_vector.size(); i++) {
    switch (generator) {
    case SpecifiedGeneratorType_DetermGETXGenerator:
      m_generator_vector[i] = new DetermGETXGenerator(i, this);
      break;
    case SpecifiedGeneratorType_DetermInvGenerator:
      m_generator_vector[i] = new DetermInvGenerator(i, *this);
      break;
    case SpecifiedGeneratorType_DetermSeriesGETSGenerator:
      m_generator_vector[i] = new DetermSeriesGETSGenerator(i, *this);
      break;
    default:
      fatal("Unexpected specified generator type");
    }
  }

  //m_dma_generator = new DMAGenerator(0, this);
}


void DeterministicDriver::go()
{
  // tick both queues until everyone is done
  while (m_done_counter != m_num_procs) {
    libruby_tick(1);
    eventQueue->triggerEvents(eventQueue->getTime() + 1);
  }
}


DeterministicDriver::~DeterministicDriver()
{
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    delete m_generator_vector[i];
  }
}

//void DeterministicDriver::dmaHitCallback()
//{
//  m_dma_generator->performCallback();
//}

void DeterministicDriver::wakeup() {
  assert(0);
  // this shouldn't be called as we are not scheduling the driver ever
}

void DeterministicDriver::hitCallback(int64_t request_id)
{
  assert(requests.find(request_id) != requests.end());
  int proc = requests[request_id].first;
  Address address = requests[request_id].second;

  m_generator_vector[proc]->performCallback(proc, address);
  
  m_last_progress_vector[proc] = eventQueue->getTime();

  requests.erase(request_id);
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
bool DeterministicDriver::isAddrReady(NodeID node, vector<NodeID> addr_vector)
{
  for (int i=0; i<addr_vector.size(); i++) {
    if (((addr_vector[i]+1)%m_num_procs == node) &&
        (m_loads_completed+m_stores_completed >= m_numCompletionsPerNode*node) && // is this node next
        (eventQueue->getTime() >= m_last_issue + 10)) { // controll rate of requests 
      return true;
    }
  }

  return false;
}

// test for a particular addr
bool DeterministicDriver::isAddrReady(NodeID node, vector<NodeID> addr_vector, Address addr)
{
  int addr_number = addr.getAddress()/DATA_BLOCK_BYTES;

  assert((addr_number >= 0) && (addr_number < addr_vector.size()));

  if (((addr_vector[addr_number]+1)%m_num_procs == node) &&
      (m_loads_completed+m_stores_completed >= m_numCompletionsPerNode*node) && // is this node next
      (eventQueue->getTime() >= m_last_issue + 10)) { // controll rate of requests                   
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

void DeterministicDriver::setNextAddr(NodeID node, Address addr, vector<NodeID>& addr_vector)
{
  // mark the addr vector that this proc was the last to use the particular address
  int addr_number = addr.getAddress()/DATA_BLOCK_BYTES;
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

Address DeterministicDriver::getNextAddr(NodeID node, vector<NodeID> addr_vector)
{

  // This method deterministically picks the next addr the node should acquirer
  // The addrs cycle through according to NodeID 0->1->...->lastID->0...
  
  Address addr;
  
  // should only be called if we know a addr is ready for the node
  assert(isAddrReady(node, addr_vector));

  for (int addr_number=0; addr_number<addr_vector.size(); addr_number++) {

    // is this node next in line for the addr
    // POLINA: LOOK HERE!
    // if ((addr_vector[addr_number] != 1) && ((addr_vector[addr_number]+1)%m_num_procs) == node) {
    if (((addr_vector[addr_number]+1)%m_num_procs) == node) {

      // One addr per cache line
      addr.setAddress(addr_number * DATA_BLOCK_BYTES);
    }
  }

  m_last_issue = eventQueue->getTime();

  return addr;
}


void DeterministicDriver::reportDone()
{
  m_done_counter++;
  if ((m_done_counter == m_num_procs)) {
    m_finish_time = eventQueue->getTime();
    //m_dma_generator->stop();
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
