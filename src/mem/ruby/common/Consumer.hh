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
 * This is the virtual base class of all classes that can be the
 * targets of wakeup events.  There is only two methods, wakeup() and
 * print() and no data members.
 */

#ifndef __MEM_RUBY_COMMON_CONSUMER_HH__
#define __MEM_RUBY_COMMON_CONSUMER_HH__

#include <iostream>
#include <set>

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"

class MessageBuffer;

class Consumer
{
  public:
    Consumer()
        : m_last_scheduled_wakeup(0), m_last_wakeup(0)
    {
    }

    virtual
    ~Consumer()
    { }

    void
    triggerWakeup(RubyEventQueue *eventQueue)
    {
        Time time = eventQueue->getTime();
        if (m_last_wakeup != time) {
            wakeup();
            m_last_wakeup = time;
        }
    }

    virtual void wakeup() = 0;
    virtual void print(std::ostream& out) const = 0;
    virtual void storeEventInfo(int info) {}

    const Time&
    getLastScheduledWakeup() const
    {
        return m_last_scheduled_wakeup;
    }

    void
    setLastScheduledWakeup(const Time& time)
    {
        m_last_scheduled_wakeup = time;
    }

    bool
    alreadyScheduled(Time time)
    {
        return m_scheduled_wakeups.find(time) != m_scheduled_wakeups.end();
    }

    void
    insertScheduledWakeupTime(Time time)
    {
        m_scheduled_wakeups.insert(time);
    }

    void
    removeScheduledWakeupTime(Time time)
    {
        assert(alreadyScheduled(time));
        m_scheduled_wakeups.erase(time);
    }

  private:
    Time m_last_scheduled_wakeup;
    std::set<Time> m_scheduled_wakeups;
    Time m_last_wakeup;
};

inline std::ostream&
operator<<(std::ostream& out, const Consumer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_COMMON_CONSUMER_HH__
