/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

%module(package="m5.internal") event

%{
#include "base/types.hh"
#include "python/swig/pyevent.hh"
#include "sim/eventq_impl.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/simulate.hh"
%}

%import "python/swig/serialize.i"

#pragma SWIG nowarn=350,351

%extend EventQueue {
    void
    schedule(Event *event, Tick when)
    {
        // Any python event that are scheduled must have their
        // internal object's refcount incremented so that the object
        // sticks around while it is in the event queue.
        PythonEvent *pyevent = dynamic_cast<PythonEvent *>(event);
        if (pyevent)
            pyevent->incref();
        $self->schedule(event, when);
    }

    void
    deschedule(Event *event)
    {
        $self->deschedule(event);

        // Now that we're removing the python object from the event
        // queue, we need to decrement its reference count.
        PythonEvent *pyevent = dynamic_cast<PythonEvent *>(event);
        if (pyevent)
            pyevent->decref();
    }
}

%typemap(out) PythonEvent* {
   result->object = $result = SWIG_NewPointerObj(SWIG_as_voidptr(result), SWIGTYPE_p_PythonEvent, SWIG_POINTER_NEW);
}

%ignore EventQueue::schedule;
%ignore EventQueue::deschedule;

%include <std_string.i>
%include <stdint.i>

%include "base/types.hh"
%include "sim/eventq.hh"

// This must follow eventq.hh
%include "python/swig/pyevent.hh"

// minimal definition of SimExitEvent interface to wrap
class GlobalSimLoopExitEvent
{
  public:
    std::string getCause();
    int getCode();
    GlobalSimLoopExitEvent(Tick when, const std::string &_cause, int c,
                           Tick _repeat = 0);
};

%exception simulate {
    $action
    if (!result) {
        return NULL;
    }
}

GlobalSimLoopExitEvent *simulate(Tick num_cycles = MaxTick);
void exitSimLoop(const std::string &message, int exit_code);
void curEventQueue( EventQueue *);
EventQueue *getEventQueue(uint32_t index);
