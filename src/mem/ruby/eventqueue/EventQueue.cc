
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
 */

#include "EventQueue.hh"
#include "RubyConfig.hh"
#include "Consumer.hh"
#include "Profiler.hh"
#include "System.hh"
#include "PrioHeap.hh"
#include "EventQueueNode.hh"

// Class public method definitions

EventQueue::EventQueue()
{
  m_prio_heap_ptr = NULL;
  init();
}

EventQueue::~EventQueue()
{
  delete m_prio_heap_ptr;
}

void EventQueue::init()
{
  m_globalTime = 1;
  m_timeOfLastRecovery = 1;
  m_prio_heap_ptr = new PrioHeap<EventQueueNode>;
  m_prio_heap_ptr->init();
}

bool EventQueue::isEmpty() const
{
  return (m_prio_heap_ptr->size() == 0);
}

void EventQueue::scheduleEventAbsolute(Consumer* consumer, Time timeAbs)
{
  // Check to see if this is a redundant wakeup
  //  Time time = timeDelta + m_globalTime;
  ASSERT(consumer != NULL);
  if (consumer->getLastScheduledWakeup() != timeAbs) {
    // This wakeup is not redundant
    EventQueueNode thisNode;
    thisNode.m_consumer_ptr = consumer;
    assert(timeAbs > m_globalTime);
    thisNode.m_time = timeAbs;
    m_prio_heap_ptr->insert(thisNode);
    consumer->setLastScheduledWakeup(timeAbs);
  }
}

void EventQueue::triggerEvents(Time t)
{
  EventQueueNode thisNode;

  while(m_prio_heap_ptr->size() > 0 && m_prio_heap_ptr->peekMin().m_time <= t) {
    m_globalTime = m_prio_heap_ptr->peekMin().m_time;
    thisNode = m_prio_heap_ptr->extractMin();
    assert(thisNode.m_consumer_ptr != NULL);
    DEBUG_EXPR(EVENTQUEUE_COMP,MedPrio,*(thisNode.m_consumer_ptr));
    DEBUG_EXPR(EVENTQUEUE_COMP,MedPrio,thisNode.m_time);
    thisNode.m_consumer_ptr->triggerWakeup();
  }
  m_globalTime = t;
}

void EventQueue::triggerAllEvents()
{
  // FIXME - avoid repeated code
  EventQueueNode thisNode;

  while(m_prio_heap_ptr->size() > 0) {
    m_globalTime = m_prio_heap_ptr->peekMin().m_time;
    thisNode = m_prio_heap_ptr->extractMin();
    assert(thisNode.m_consumer_ptr != NULL);
    DEBUG_EXPR(EVENTQUEUE_COMP,MedPrio,*(thisNode.m_consumer_ptr));
    DEBUG_EXPR(EVENTQUEUE_COMP,MedPrio,thisNode.m_time);
    thisNode.m_consumer_ptr->triggerWakeup();
  }
}

// Class private method definitions

void
EventQueue::print(ostream& out) const
{
  out << "[Event Queue: " << *m_prio_heap_ptr << "]";
}
