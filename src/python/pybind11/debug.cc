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
 * Copyright (c) 2010 The Hewlett-Packard Development Company
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

#include <map>
#include <vector>

#include "base/debug.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "sim/debug.hh"

namespace py = pybind11;

namespace Debug {
extern int allFlagsVersion;
}

static void
output(const char *filename)
{
    OutputStream *file_stream = simout.find(filename);

    if (!file_stream)
        file_stream = simout.create(filename);

    Trace::setDebugLogger(new Trace::OstreamLogger(*file_stream->stream()));
}

static void
ignore(const char *expr)
{
    ObjectMatch ignore(expr);

    Trace::getDebugLogger()->addIgnore(ignore);
}

void
pybind_init_debug(py::module &m_native)
{
    py::module m_debug = m_native.def_submodule("debug");

    m_debug
        .def("getAllFlagsVersion", []() { return Debug::allFlagsVersion; })
        .def("allFlags", &Debug::allFlags, py::return_value_policy::reference)
        .def("findFlag", &Debug::findFlag)
        .def("setDebugFlag", &setDebugFlag)
        .def("clearDebugFlag", &clearDebugFlag)
        .def("dumpDebugFlags", &dumpDebugFlags)

        .def("schedBreak", &schedBreak)
        .def("setRemoteGDBPort", &setRemoteGDBPort)
        ;

    py::class_<Debug::Flag> c_flag(m_debug, "Flag");
    c_flag
        .def("name", &Debug::Flag::name)
        .def("desc", &Debug::Flag::desc)
        .def("kids", &Debug::Flag::kids)
        .def("enable", &Debug::Flag::enable)
        .def("disable", &Debug::Flag::disable)
        .def("sync", &Debug::Flag::sync)
        ;

    py::class_<Debug::SimpleFlag>(m_debug, "SimpleFlag", c_flag);
    py::class_<Debug::CompoundFlag>(m_debug, "CompoundFlag", c_flag);


    py::module m_trace = m_native.def_submodule("trace");
    m_trace
        .def("output", &output)
        .def("ignore", &ignore)
        .def("enable", &Trace::enable)
        .def("disable", &Trace::disable)
        ;
}
