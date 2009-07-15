
/*
    Copyright (C) 1999-2005 by Mark D. Hill and David A. Wood for the
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
    Beckmann, Ross Dickson, Pacia Harper, Milo Martin, Michael Marty,
    Carl Mauer, Kevin Moore, Manoj Plakal, Daniel Sorin, Min Xu, and
    Luke Yen.

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
#include "mem/ruby/tester/RaceyDriver.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/tester/RaceyPseudoThread.hh"

RaceyDriver::RaceyDriver(int num_procs, int tester_length)
{
  m_finish_time = 0;
  m_done_counter = 0;
  m_wakeup_thread0 = false;
  m_num_procs = num_procs;
  m_tester_length = tester_length;
  eventQueue = new RubyEventQueue;

  // racey at least need two processors
  assert(m_num_procs >= 2);

  // init all racey pseudo threads
  m_racey_pseudo_threads.setSize(m_num_procs);
  for (int i=0; i<m_racey_pseudo_threads.size(); i++) {
    m_racey_pseudo_threads[i] = new RaceyPseudoThread(i, *this);
  }

  // add this driver to the global event queue, for deadlock detection
  eventQueue->scheduleEvent(this, g_DEADLOCK_THRESHOLD);
}

RaceyDriver::~RaceyDriver()
{
  for (int i=0; i<m_racey_pseudo_threads.size(); i++) {
    delete m_racey_pseudo_threads[i];
  }
}

void RaceyDriver::go() {
  // tick both queues until everyone is done
  while (m_done_counter != m_num_procs) {
    libruby_tick(1);
    eventQueue->triggerEvents(eventQueue->getTime() + 1);
  }
}


void RaceyDriver::hitCallback(int64_t request_id)
{
  ASSERT(requests.find(request_id) != requests.end());
  int proc = requests[request_id].first;
  Address address = requests[request_id].second.address;
  uint8_t * data = new uint8_t[4];
  for (int i = 0; i < 4; i++) {
    data[i] = requests[request_id].second.data[i];
  }  
  requests[request_id].second.data;
  m_racey_pseudo_threads[proc]->performCallback(proc, address, data);
  requests.erase(request_id);
}

integer_t RaceyDriver::getInstructionCount(int procID) const
{
 // return m_racey_pseudo_threads[procID]->getInstructionCounter();
 assert(0);
}

int RaceyDriver::runningThreads()
{
  return m_num_procs - m_done_counter;
}

// used to wake up thread 0 whenever other thread finishes
void RaceyDriver::registerThread0Wakeup()
{
  m_wakeup_thread0 = true;
}

void RaceyDriver::joinThread()
{
  m_done_counter++;
  if (m_done_counter == m_num_procs) {
   m_finish_time = eventQueue->getTime();
  }

  if(m_wakeup_thread0) {
    eventQueue->scheduleEvent(m_racey_pseudo_threads[0], 1);
    m_wakeup_thread0 = false;
  }
}

void RaceyDriver::wakeup()
{
  // check for deadlock
  for(int i = 0 ; i < m_racey_pseudo_threads.size(); i++) {
    m_racey_pseudo_threads[i]->checkForDeadlock();
  }

  // schedule next wakeup
  if (m_done_counter < m_num_procs) {
    eventQueue->scheduleEvent(this, g_DEADLOCK_THRESHOLD);
  }
}

void RaceyDriver::printStats(ostream& out) const
{
  assert(m_done_counter == m_num_procs);
  out << endl;
  out << "RaceyDriver Stats" << endl;
  out << "---------------------" << endl;

  out << "execution signature: " << m_racey_pseudo_threads[0]->getSignature() << endl;
  out << "finish_time: " << m_finish_time << endl;
}

void RaceyDriver::print(ostream& out) const
{
}
