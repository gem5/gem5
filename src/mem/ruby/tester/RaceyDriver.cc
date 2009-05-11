
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

#include "Global.hh"
#include "System.hh"
#include "RaceyDriver.hh"
#include "EventQueue.hh"
#include "RaceyPseudoThread.hh"
#include "SubBlock.hh"

RaceyDriver::RaceyDriver()
{
  if (g_SIMICS) {
    ERROR_MSG("g_SIMICS should not be defined.");
  }

  // debug transition?
  if(false) {
    assert(g_debug_ptr);
    g_debug_ptr->setDebugTime(1);
  }

  m_finish_time = 0;
  m_done_counter = 0;
  m_wakeup_thread0 = false;

  // racey at least need two processors
  assert(RubyConfig::numberOfProcessors() >= 2);

  // init all racey pseudo threads
  m_racey_pseudo_threads.setSize(RubyConfig::numberOfProcessors());
  for (int i=0; i<m_racey_pseudo_threads.size(); i++) {
    m_racey_pseudo_threads[i] = new RaceyPseudoThread(i, *this);
  }

  // add this driver to the global event queue, for deadlock detection
  g_eventQueue_ptr->scheduleEvent(this, g_DEADLOCK_THRESHOLD);
}

RaceyDriver::~RaceyDriver()
{
  for (int i=0; i<m_racey_pseudo_threads.size(); i++) {
    delete m_racey_pseudo_threads[i];
  }
}

void RaceyDriver::hitCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread)
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, data);
  m_racey_pseudo_threads[proc]->performCallback(proc, data);
}

integer_t RaceyDriver::getInstructionCount(int procID) const
{
  return m_racey_pseudo_threads[procID]->getInstructionCounter();
}

int RaceyDriver::runningThreads()
{
  return RubyConfig::numberOfProcessors() - m_done_counter;
}

// used to wake up thread 0 whenever other thread finishes
void RaceyDriver::registerThread0Wakeup()
{
  m_wakeup_thread0 = true;
}

void RaceyDriver::joinThread()
{
  m_done_counter++;
  if (m_done_counter == RubyConfig::numberOfProcessors()) {
    m_finish_time = g_eventQueue_ptr->getTime();
  }

  if(m_wakeup_thread0) {
    g_eventQueue_ptr->scheduleEvent(m_racey_pseudo_threads[0], 1);
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
  if (m_done_counter < RubyConfig::numberOfProcessors()) {
    g_eventQueue_ptr->scheduleEvent(this, g_DEADLOCK_THRESHOLD);
  }
}

void RaceyDriver::printStats(ostream& out) const
{
  assert(m_done_counter == RubyConfig::numberOfProcessors());
  out << endl;
  out << "RaceyDriver Stats" << endl;
  out << "---------------------" << endl;

  out << "execution signature: " << m_racey_pseudo_threads[0]->getSignature() << endl;
  out << "finish_time: " << m_finish_time << endl;
}

void RaceyDriver::print(ostream& out) const
{
}
