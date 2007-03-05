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
 * Authors: Nathan Binkert
 */

%module event

%{
#include "python/swig/pyevent.hh"

#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/simulate.hh"
%}

%include "stdint.i"
%include "std_string.i"
%include "sim/host.hh"

void create(PyObject *object, Tick when);

class Event;
class CountedDrainEvent : public Event {
  public:
    void setCount(int _count);
};

CountedDrainEvent *createCountedDrain();
void cleanupCountedDrain(Event *drain_event);

%immutable curTick;
Tick curTick;

// minimal definition of SimExitEvent interface to wrap
class SimLoopExitEvent {
  public:
    std::string getCause();
    int getCode();
    SimLoopExitEvent(EventQueue *q, Tick _when, Tick _repeat,
                     const std::string &_cause, int c = 0);
};

%exception simulate {
    $action
    if (!result) {
        return NULL;
    }
}
SimLoopExitEvent *simulate(Tick num_cycles = MaxTick);
void exitSimLoop(const std::string &message, int exit_code);

Tick curTick;

%wrapper %{
// fix up module name to reflect the fact that it's inside the m5 package
#undef SWIG_name
#define SWIG_name "m5.internal._event"
%}
