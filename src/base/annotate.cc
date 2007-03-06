/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 */

#include "base/annotate.hh"
#include "base/callback.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"



class AnnotateDumpCallback : public Callback
{
  public:
    virtual void process();
};

void
AnnotateDumpCallback::process()
{
    Annotate::annotations.dump();
}

namespace Annotate {


Annotate annotations;

Annotate::Annotate()
{
    registerExitCallback(new AnnotateDumpCallback);
}

void
Annotate::add(System *sys, Addr stack, uint32_t sm, uint32_t st,
        uint32_t wm, uint32_t ws)
{
    AnnotateData *an;

    an = new AnnotateData;
    an->time = curTick;

    std::map<System*, std::string>::iterator i = nameCache.find(sys);
    if (i == nameCache.end()) {
        nameCache[sys] = sys->name();
    }

    an->system = nameCache[sys];
    an->stack = stack;
    an->stateMachine = sm;
    an->curState = st;
    an->waitMachine = wm;
    an->waitState = ws;

    data.push_back(an);
    if (an->waitMachine)
        DPRINTF(Annotate, "Annotating: %s(%#llX) %d:%d waiting on %d:%d\n",
                an->system, an->stack, an->stateMachine, an->curState,
                an->waitMachine, an->waitState);
    else
        DPRINTF(Annotate, "Annotating: %s(%#llX) %d:%d beginning\n", an->system,
                an->stack, an->stateMachine, an->curState);

    DPRINTF(Annotate, "Now %d events on list\n", data.size());

}

void
Annotate::dump()
{

    std::list<AnnotateData*>::iterator i;

    i = data.begin();

    if (i == data.end())
        return;

    std::ostream *os = simout.create("annotate.dat");

    AnnotateData *an;

    while (i != data.end()) {
        DPRINTF(Annotate, "Writing\n", data.size());
        an = *i;
        ccprintf(*os, "%d %s(%#llX) %d %d %d %d\n", an->time, an->system,
                an->stack, an->stateMachine, an->curState, an->waitMachine,
                an->waitState);
        i++;
    }
}

} //namespace Annotate
