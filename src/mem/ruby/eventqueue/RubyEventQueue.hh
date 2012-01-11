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
 * The RubyEventQueue class implements an event queue which
 * can be trigger events, allowing our simulation to be event driven.
 *
 * Currently, the only event we support is a Consumer being signaled
 * by calling the consumer's wakeup() routine.  Adding the event to
 * the queue does not require a virtual function call, though calling
 * wakeup() is a virtual function call.
 *
 * The method triggerEvents() is called with a global time.  All
 * events which are before or at this time are triggered in timestamp
 * order.  No ordering is enforced for events scheduled to occur at
 * the same time.  Events scheduled to wakeup the same consumer at the
 * same time are combined into a single event.
 *
 * The method scheduleConsumerWakeup() is called with a global time
 * and a consumer pointer.  The event queue will call the wakeup()
 * method of the consumer at the appropriate time.
 *
 * This implementation of RubyEventQueue uses a dynamically sized array
 * managed as a heap.  The algorithms used has O(lg n) for insert and
 * O(lg n) for extract minimum element. (Based on chapter 7 of Cormen,
 * Leiserson, and Rivest.)  The array is dynamically sized and is
 * automatically doubled in size when necessary.
 *
 */

#ifndef __MEM_RUBY_EVENTQUEUE_RUBYEVENTQUEUE_HH__
#define __MEM_RUBY_EVENTQUEUE_RUBYEVENTQUEUE_HH__

#include <iostream>

#include "mem/ruby/common/TypeDefines.hh"
#include "sim/eventq.hh"

class Consumer;
class RubyEventQueueNode;

class RubyEventQueue : public EventManager
{
  public:
    RubyEventQueue(EventQueue* eventq, Tick _clock);
    ~RubyEventQueue();

    Time getTime() const { return curTick()/m_clock; }
    Tick getClock() const { return m_clock; }
    void scheduleEvent(Consumer* consumer, Time timeDelta);
    void scheduleEventAbsolute(Consumer* consumer, Time timeAbs);
    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    RubyEventQueue(const RubyEventQueue& obj);
    RubyEventQueue& operator=(const RubyEventQueue& obj);

    // Data Members (m_ prefix)
    Tick m_clock;
};

inline std::ostream&
operator<<(std::ostream& out, const RubyEventQueue& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_EVENTQUEUE_RUBYEVENTQUEUE_HH__
