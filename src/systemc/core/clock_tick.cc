/*
 * Copyright 2018 Google, Inc.
 * Copyright 2026 Google LLC
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

#include <string>

#include "systemc/core/clock_tick_ops.hh"
#include "systemc/core/process_types.hh"
#include "systemc/core/sched_event.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_clock.hh"
#include "systemc/ext/core/sc_module.hh" // for sc_gen_unique_name

namespace sc_gem5
{

class ClockTick : public ScEvent
{
  private:
    ::sc_core::sc_time _period;
    std::string name;
    Process *p;
    ProcessMemberFuncWrapper<::sc_core::sc_clock> funcWrapper;

  public:
    ClockTick(::sc_core::sc_clock *clock, bool to, ::sc_core::sc_time _period)
        : ScEvent([this]() { tick(); }),
          _period(_period),
          name(clock->name()),
          p(nullptr),
          funcWrapper(clock, to ? &::sc_core::sc_clock::tickUp
                                : &::sc_core::sc_clock::tickDown)
    {
        name += std::string(to ? "_posedge_action" : "_negedge_action");
        name = ::sc_core::sc_gen_unique_name(name.c_str());
    }

    void
    createProcess()
    {
        p = new Method(name.c_str(), &funcWrapper, true);
        p->dontInitialize(true);
        scheduler().reg(p);
    }

    ~ClockTick()
    {
        if (scheduled()) {
            scheduler().deschedule(this);
        }
        if (p) {
            p->popListNode();
        }
    }

    void
    tick()
    {
        scheduler().schedule(this, _period);
        p->ready();
    }
};

ClockTick *
newClockTick(::sc_core::sc_clock *clock, bool to,
             const ::sc_core::sc_time &period)
{
    return new ClockTick(clock, to, period);
}

void
destroyClockTick(ClockTick *tick)
{
    delete tick;
}

void
createClockTickProcess(ClockTick *tick)
{
    tick->createProcess();
}

void
scheduleClockTick(ClockTick *tick, const ::sc_core::sc_time &delay)
{
    scheduler().schedule(tick, delay);
}

void
descheduleClockTick(ClockTick *tick)
{
    scheduler().deschedule(tick);
}

bool
isClockTickScheduled(ClockTick *tick)
{
    return tick->scheduled();
}

} // namespace sc_gem5
