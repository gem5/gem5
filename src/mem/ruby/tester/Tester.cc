
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
#include "mem/ruby/tester/Tester.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/tester/Check.hh"
#include "mem/protocol/Chip.hh"

Tester::Tester(RubySystem* sys_ptr)
{
  g_callback_counter = 0;

  // add the tester consumer to the global event queue
  g_eventQueue_ptr->scheduleEvent(this, 1);

  m_last_progress_vector.setSize(RubyConfig::numberOfProcessors());
  for (int i=0; i<m_last_progress_vector.size(); i++) {
    m_last_progress_vector[i] = 0;
  }
}

Tester::~Tester()
{
}

void Tester::hitCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread)
{
  // Mark that we made progress
  m_last_progress_vector[proc] = g_eventQueue_ptr->getTime();
  g_callback_counter++;

  // This tells us our store has 'completed' or for a load gives us
  // back the data to make the check
  DEBUG_EXPR(TESTER_COMP, MedPrio, proc);
  DEBUG_EXPR(TESTER_COMP, MedPrio, data);
  Check* check_ptr = m_checkTable.getCheck(data.getAddress());
  assert(check_ptr != NULL);
  check_ptr->performCallback(proc, data);

}

void Tester::wakeup()
{
  if (g_callback_counter < g_tester_length) {
    // Try to perform an action or check
    Check* check_ptr = m_checkTable.getRandomCheck();
    assert(check_ptr != NULL);
    check_ptr->initiate();

    checkForDeadlock();

    g_eventQueue_ptr->scheduleEvent(this, 2);
  }
}

void Tester::checkForDeadlock()
{
  int size = m_last_progress_vector.size();
  Time current_time = g_eventQueue_ptr->getTime();
  for (int processor=0; processor<size; processor++) {
    if ((current_time - m_last_progress_vector[processor]) > g_DEADLOCK_THRESHOLD) {
      WARN_EXPR(current_time);
      WARN_EXPR(m_last_progress_vector[processor]);
      WARN_EXPR(current_time - m_last_progress_vector[processor]);
      WARN_EXPR(processor);
      Sequencer* seq_ptr = g_system_ptr->getChip(processor/RubyConfig::numberOfProcsPerChip())->getSequencer(processor%RubyConfig::numberOfProcsPerChip());
      assert(seq_ptr != NULL);
      WARN_EXPR(*seq_ptr);
      ERROR_MSG("Deadlock detected.");
    }
  }
}

void Tester::print(ostream& out) const
{
  out << "[Tester]" << endl;
}

