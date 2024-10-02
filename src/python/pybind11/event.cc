/*
 * Copyright (c) 2017, 2021 Arm Limited
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
 */

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "base/logging.hh"
#include "sim/eventq.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/simulate.hh"

namespace py = pybind11;

namespace gem5
{

/**
 * PyBind wrapper for Events
 *
 * We need to wrap the Event class with some Python glue code to
 * enable method overrides in Python and memory management. Unlike its
 * C++ cousin, PyEvents need to override __call__ instead of
 * Event::process().
 *
 * Memory management is done using reference counting in Python.
 */
class PyEvent : public Event
{
  public:
    PyEvent(Event::Priority priority)
        : Event(priority, Event::Managed)
    {
    }

    void process() override {
        // Call the Python implementation as __call__. This provides a
        // slightly more Python-friendly interface.
        PYBIND11_OVERLOAD_PURE_NAME(void, PyEvent, "__call__", process);
    }

  protected:
    void acquireImpl() override {
        py::object obj = py::cast(this);

        if (obj) {
            obj.inc_ref();
        } else {
            panic("Failed to get PyBind object to increase ref count\n");
        }
    }

    void releaseImpl() override {
        py::object obj = py::cast(this);

        if (obj) {
            obj.dec_ref();
        } else {
            panic("Failed to get PyBind object to decrease ref count\n");
        }
    }
};

void
pybind_init_event(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("event");

    m.def("simulate", &simulate,
          py::arg("ticks") = MaxTick);
    m.def("setMaxTick", &set_max_tick, py::arg("tick"));
    m.def("getMaxTick", &get_max_tick, py::return_value_policy::copy);
    m.def("terminateEventQueueThreads", &terminateEventQueueThreads);
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
        .def("deschedule", &EventQueue::deschedule,
             py::arg("event"))
        .def("reschedule", &EventQueue::reschedule,
             py::arg("event"), py::arg("tick"), py::arg("always") = false)
        ;

    // TODO: Ownership of global exit events has always been a bit
    // questionable. We currently assume they are owned by the C++
    // world. This is what the old SWIG code did, but that will result
    // in memory leaks.
    py::class_<GlobalSimLoopExitEvent,
               std::unique_ptr<GlobalSimLoopExitEvent, py::nodelete>>(
               m, "GlobalSimLoopExitEvent")
        .def("getCause", &GlobalSimLoopExitEvent::getCause)
        .def("getCode", &GlobalSimLoopExitEvent::getCode)
        ;

    py::class_<ExitEvent,
                std::unique_ptr<ExitEvent, py::nodelete>>(
                m, "ExitEvent")
          .def("description", &ExitEvent::description)
          .def("reenter_simloop", &ExitEvent::reenter_simloop)
          ;

    // Event base class. These should never be returned directly to
    // Python since they don't have a well-defined life cycle. Python
    // events should be derived from PyEvent instead.
    py::class_<Event> c_event(
        m, "Event");
    c_event
        .def("name", &Event::name)
        .def("dump", &Event::dump)
        .def("scheduled", &Event::scheduled)
        .def("squash", &Event::squash)
        .def("squashed", &Event::squashed)
        .def("isExitEvent", &Event::isExitEvent)
        .def("when", &Event::when)
        .def("priority", &Event::priority)
        ;

    py::class_<PyEvent, Event>(m, "PyEvent")
        .def(py::init<Event::Priority>(),
             py::arg("priority") = (int)Event::Default_Pri)
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

} // namespace gem5
