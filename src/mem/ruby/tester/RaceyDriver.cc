
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
  m_racey_pseudo_threads.resize(m_num_procs);
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
  assert(requests.find(request_id) != requests.end());
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
