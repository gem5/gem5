
/*
    Copyright (C) 1999-2008 by Mark D. Hill and David A. Wood for the
    Wisconsin Multifacet Project.  Contact: gems@cs.wisc.edu
    http://www.cs.wisc.edu/gems/

    --------------------------------------------------------------------

    This file is part of the Ruby Multiprocessor Memory System Simulator, 
    a component of the Multifacet GEMS (General Execution-driven 
    Multiprocessor Simulator) software toolset originally developed at 
    the University of Wisconsin-Madison.

    Ruby was originally developed primarily by Milo Martin and Daniel
    Sorin with contributions from Ross Dickson, Carl Mauer, and Manoj
    Plakal.

    Substantial further development of Multifacet GEMS at the
    University of Wisconsin was performed by Alaa Alameldeen, Brad
    Beckmann, Jayaram Bobba, Ross Dickson, Dan Gibson, Pacia Harper,
    Derek Hower, Milo Martin, Michael Marty, Carl Mauer, Michelle Moravan,
    Kevin Moore, Andrew Phelps, Manoj Plakal, Daniel Sorin, Haris Volos, 
    Min Xu, and Luke Yen.
    --------------------------------------------------------------------

    If your use of this software contributes to a published paper, we
    request that you (1) cite our summary paper that appears on our
    website (http://www.cs.wisc.edu/gems/) and (2) e-mail a citation
    for your published paper to gems@cs.wisc.edu.

    If you redistribute derivatives of this software, we request that
    you notify us and either (1) ask people to register with us at our
    website (http://www.cs.wisc.edu/gems/) or (2) collect registration
    information and periodically send it to us.

    --------------------------------------------------------------------

    Multifacet GEMS is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General Public
    License as published by the Free Software Foundation.

    Multifacet GEMS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Multifacet GEMS; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
    02111-1307, USA

    The GNU General Public License is contained in the file LICENSE.

### END HEADER ###
*/

/*
 * $Id$
 *
 */

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
//#include "DMAGenerator.hh"
#include "mem/ruby/tester/DetermGETXGenerator.hh"

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
  
  m_last_progress_vector.setSize(num_procs);
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    m_last_progress_vector[i] = 0;
  }

  m_load_vector.setSize(10);
  for (int i=0; i<m_load_vector.size(); i++) {
    m_load_vector[i] = -1;  // No processor last held it
  }

  m_store_vector.setSize(10);
  for (int i=0; i<m_store_vector.size(); i++) {
    m_store_vector[i] = -1;  // No processor last held it
  }

  m_generator_vector.setSize(num_procs);

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
      ERROR_MSG("Unexpected specified generator type");
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
  ASSERT(requests.find(request_id) != requests.end());
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
  int addr_number = addr.getAddress()/DATA_BLOCK_BYTES;

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
    if (((addr_vector[i]+1)%m_num_procs == node) &&
        (m_loads_completed+m_stores_completed >= m_numCompletionsPerNode*node) && // is this node next
        (eventQueue->getTime() >= m_last_issue + 10)) { // controll rate of requests 
      return true;
    }
  }

  return false;
}

// test for a particular addr
bool DeterministicDriver::isAddrReady(NodeID node, Vector<NodeID> addr_vector, Address addr)
{
  int addr_number = addr.getAddress()/DATA_BLOCK_BYTES;

  ASSERT ((addr_number >= 0) && (addr_number < addr_vector.size()));

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

void DeterministicDriver::setNextAddr(NodeID node, Address addr, Vector<NodeID>& addr_vector)
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

Address DeterministicDriver::getNextAddr(NodeID node, Vector<NodeID> addr_vector)
{

  // This method deterministically picks the next addr the node should acquirer
  // The addrs cycle through according to NodeID 0->1->...->lastID->0...
  
  Address addr;
  
  // should only be called if we know a addr is ready for the node
  ASSERT(isAddrReady(node, addr_vector));

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
