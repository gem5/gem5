/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_CORE_SCHED_EVENT_HH__
#define __SYSTEMC_CORE_SCHED_EVENT_HH__

#include <functional>
#include <list>

#include "base/types.hh"

namespace sc_gem5
{

class ScEvent;

typedef std::list<ScEvent *> ScEvents;

class ScEvent
{
  private:
    std::function<void()> work;
    Tick _when;
    ScEvents *_events;
    ScEvents::iterator _it;

    friend class Scheduler;

    void
    schedule(ScEvents &events, Tick w)
    {
        when(w);
        assert(!scheduled());
        _events = &events;
        _events->push_back(this);
        _it = _events->end();
        _it--;
    }

    void
    deschedule()
    {
        assert(scheduled());
        _events->erase(_it);
        _events = nullptr;
    }
  public:
    ScEvent(std::function<void()> work) :
        work(work), _when(MaxTick), _events(nullptr)
    {}

    ~ScEvent();

    bool scheduled() { return _events != nullptr; }
    ScEvents *scheduledOn() { return _events; }

    void when(Tick w) { _when = w; }
    Tick when() { return _when; }

    void run() { deschedule(); work(); }
};

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SCHED_EVENT_HH__
