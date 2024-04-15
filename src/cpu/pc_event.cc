/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "cpu/pc_event.hh"

#include <algorithm>
#include <string>
#include <utility>

#include "base/debug.hh"
#include "base/trace.hh"
#include "debug/PCEvent.hh"
#include "sim/cur_tick.hh"
#include "sim/system.hh"

namespace gem5
{

PCEventQueue::PCEventQueue() {}

PCEventQueue::~PCEventQueue() {}

bool
PCEventQueue::remove(PCEvent *event)
{
    int removed = 0;
    range_t range = equal_range(event);
    iterator i = range.first;
    while (i != range.second && i != pcMap.end()) {
        if (*i == event) {
            DPRINTF(PCEvent, "PC based event removed at %#x: %s\n",
                    event->pc(), event->descr());
            i = pcMap.erase(i);
            ++removed;
        } else {
            i++;
        }
    }

    return removed > 0;
}

bool
PCEventQueue::schedule(PCEvent *event)
{
    pcMap.push_back(event);
    std::sort(pcMap.begin(), pcMap.end(), MapCompare());

    DPRINTF(PCEvent, "PC based event scheduled for %#x: %s\n", event->pc(),
            event->descr());

    return true;
}

bool
PCEventQueue::doService(Addr pc, ThreadContext *tc)
{
    // Using the raw PC address will fail to break on Alpha PALcode addresses,
    // but that is a rare use case.
    int serviced = 0;
    range_t range = equal_range(pc);
    for (iterator i = range.first; i != range.second; ++i) {
        DPRINTF(PCEvent, "PC based event serviced at %#x: %s\n", (*i)->pc(),
                (*i)->descr());

        (*i)->process(tc);
        ++serviced;
    }

    return serviced > 0;
}

void
PCEventQueue::dump() const
{
    const_iterator i = pcMap.begin();
    const_iterator e = pcMap.end();

    for (; i != e; ++i)
        cprintf("%d: event at %#x: %s\n", curTick(), (*i)->pc(),
                (*i)->descr());
}

PCEventQueue::range_t
PCEventQueue::equal_range(Addr pc)
{
    return std::equal_range(pcMap.begin(), pcMap.end(), pc, MapCompare());
}

BreakPCEvent::BreakPCEvent(PCEventScope *s, const std::string &desc, Addr addr,
                           bool del)
    : PCEvent(s, desc, addr), remove(del)
{}

void
BreakPCEvent::process(ThreadContext *tc)
{
    StringWrap name("break_event");
    DPRINTFN("break event %s triggered\n", descr());
    debug::breakpoint();
    if (remove)
        delete this;
}

PanicPCEvent::PanicPCEvent(PCEventScope *s, const std::string &desc, Addr pc)
    : PCEvent(s, desc, pc)
{}

void
PanicPCEvent::process(ThreadContext *tc)
{
    StringWrap name("panic_event");
    panic(descr());
}

} // namespace gem5
