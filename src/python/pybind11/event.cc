/*
 * Copyright (c) 2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 *          Andreas Sandberg
 */

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "sim/eventq.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/simulate.hh"

namespace py = pybind11;

/**
 * PyBind wrapper for Events
 *
 * We need to wrap the Event class with some Python glue code to
 * enable method overrides in Python and memory management. Unlike its
 * C++ cousin, PyEvents need to override __call__ instead of
 * Event::process().
 *
 * Memory management is mostly done using reference counting in
 * Python. However, PyBind can't keep track of the reference the event
 * queue holds to a scheduled event. We therefore need to inhibit
 * deletion and hand over ownership to the event queue in case a
 * scheduled event loses all of its Python references.
 */
class PyEvent : public Event
{
  public:
    struct Deleter {
        void operator()(PyEvent *ev) {
            assert(!ev->isAutoDelete());
            if (ev->scheduled()) {
                // The event is scheduled, give ownership to the event
                // queue.
                ev->setFlags(Event::AutoDelete);
            } else {
                // The event isn't scheduled, hence Python owns it and
                // we need to free it here.
                delete ev;
            }
        }
    };

    PyEvent(Event::Priority priority)
        : Event(priority)
    { }

    void process() override {
        if (isAutoDelete()) {
            // Ownership of the event was handed over to the event queue
            // because the last revference in Python land was GCed. We
            // need to claim the object again since we're creating a new
            // Python reference.
            clearFlags(AutoDelete);
        }

        // Call the Python implementation as __call__. This provides a
        // slightly more Python-friendly interface.
        PYBIND11_OVERLOAD_PURE_NAME(void, PyEvent, "__call__", process);
    }
};

void
pybind_init_event(py::module &m_native)
{
    py::module m = m_native.def_submodule("event");

    m.def("simulate", &simulate,
          py::arg("ticks") = MaxTick);
    m.def("exitSimLoop", &exitSimLoop);
    m.def("getEventQueue", []() { return curEventQueue(); },
          py::return_value_policy::reference);
    m.def("setEventQueue", [](EventQueue *q) { return curEventQueue(q); });
    m.def("getEventQueue", &getEventQueue,
          py::return_value_policy::reference);

    py::class_<EventQueue>(m, "EventQueue")
        .def("name",  [](EventQueue *eq) { return eq->name(); })
        .def("dump", &EventQueue::dump)
        .def("schedule", [](EventQueue *eq, PyEvent *e, Tick t) {
                eq->schedule(e, t);
            }, py::arg("event"), py::arg("when"))
        .def("deschedule", [](EventQueue *eq, PyEvent *e) {
                eq->deschedule(e);
            }, py::arg("event"))
        .def("reschedule", [](EventQueue *eq, PyEvent *e, Tick t, bool alw) {
                eq->reschedule(e, t, alw);
            }, py::arg("event"), py::arg("tick"), py::arg("always") = false)
        ;

    // TODO: Ownership of global exit events has always been a bit
    // questionable. We currently assume they are owned by the C++
    // word. This is what the old SWIG code did, but that will result
    // in memory leaks.
    py::class_<GlobalSimLoopExitEvent,
               std::unique_ptr<GlobalSimLoopExitEvent, py::nodelete>>(
               m, "GlobalSimLoopExitEvent")
        .def("getCause", &GlobalSimLoopExitEvent::getCause)
        .def("getCode", &GlobalSimLoopExitEvent::getCode)
        ;

    // TODO: We currently export a wrapper class and not the Event
    // base class. This wil be problematic if we ever return an event
    // from C++.
    py::class_<PyEvent, std::unique_ptr<PyEvent, PyEvent::Deleter>>
        c_event(m, "Event");
    c_event
        .def(py::init<Event::Priority>(),
             py::arg("priority") = (int)Event::Default_Pri)
        .def("name", &Event::name)
        .def("dump", &Event::dump)
        .def("scheduled", &Event::scheduled)
        .def("squash", &Event::squash)
        .def("squashed", &Event::squashed)
        .def("isExitEvent", &Event::isExitEvent)
        .def("isAutoDelete", &Event::isAutoDelete)
        .def("when", &Event::when)
        .def("priority", &Event::priority)
        ;

#define PRIO(n) c_event.attr(# n) = py::cast((int)Event::n)
    PRIO(Minimum_Pri);
    PRIO(Minimum_Pri);
    PRIO(Debug_Enable_Pri);
    PRIO(Debug_Break_Pri);
    PRIO(CPU_Switch_Pri);
    PRIO(Delayed_Writeback_Pri);
    PRIO(Default_Pri);
    PRIO(DVFS_Update_Pri);
    PRIO(Serialize_Pri);
    PRIO(CPU_Tick_Pri);
    PRIO(Stat_Event_Pri);
    PRIO(Progress_Event_Pri);
    PRIO(Sim_Exit_Pri);
    PRIO(Maximum_Pri);
}
