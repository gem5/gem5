/*
 * Copyright (c) 2002-2003 The Regents of The University of Michigan
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

#include <algorithm>
#include <map>
#include <string>
#include <utility>

#include "sim/debug.hh"
#include "cpu/exec_context.hh"
#include "cpu/pc_event.hh"
#include "base/trace.hh"
#include "sim/universe.hh"

using namespace std;

PCEventQueue::PCEventQueue()
{}

PCEventQueue::~PCEventQueue()
{}

bool
PCEventQueue::remove(PCEvent *event)
{
    int removed = 0;
    range_t range = equal_range(event);
    for (iterator i = range.first; i != range.second; ++i) {
        if (*i == event) {
            DPRINTF(PCEvent, "PC based event removed at %#x: %s\n",
                    event->pc(), event->descr());
            pc_map.erase(i);
            ++removed;
        }
    }

    return removed > 0;
}

bool
PCEventQueue::schedule(PCEvent *event)
{
    pc_map.push_back(event);
    sort(pc_map.begin(), pc_map.end(), MapCompare());

    DPRINTF(PCEvent, "PC based event scheduled for %#x: %s\n",
            event->pc(), event->descr());

    return true;
}

bool
PCEventQueue::doService(ExecContext *xc)
{
    Addr pc = xc->regs.pc;
    int serviced = 0;
    range_t range = equal_range(pc);
    for (iterator i = range.first; i != range.second; ++i) {
        // Make sure that the pc wasn't changed as the side effect of
        // another event.  This for example, prevents two invocations
        // of the SkipFuncEvent.  Maybe we should have separate PC
        // event queues for each processor?
        if (pc != xc->regs.pc)
            continue;

        DPRINTF(PCEvent, "PC based event serviced at %#x: %s\n",
                (*i)->pc(), (*i)->descr());

        (*i)->process(xc);
        ++serviced;
    }

    return serviced > 0;
}

void
PCEventQueue::dump() const
{
    const_iterator i = pc_map.begin();
    const_iterator e = pc_map.end();

    for (; i != e; ++i)
        cprintf("%d: event at %#x: %s\n", curTick, (*i)->pc(),
                (*i)->descr());
}

PCEventQueue::range_t
PCEventQueue::equal_range(Addr pc)
{
    return std::equal_range(pc_map.begin(), pc_map.end(), pc, MapCompare());
}

BreakPCEvent::BreakPCEvent(PCEventQueue *q, const std::string &desc, bool del)
    : PCEvent(q, desc), remove(del)
{
}

void
BreakPCEvent::process(ExecContext *xc)
{
    debug_break();
    if (remove)
        delete this;
}

#ifdef FULL_SYSTEM
extern "C"
void
sched_break_pc_sys(System *sys, Addr addr)
{
    PCEvent *event = new BreakPCEvent(&sys->pcEventQueue, "debug break", true);
    event->schedule(addr);
}

extern "C"
void
sched_break_pc(Addr addr)
{
     for (vector<System *>::iterator sysi = System::systemList.begin();
          sysi != System::systemList.end(); ++sysi) {
         sched_break_pc_sys(*sysi, addr);
    }

}
#endif
