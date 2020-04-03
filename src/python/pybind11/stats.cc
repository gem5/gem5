/*
 * Copyright (c) 2017-2019 ARM Limited
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

#include "config/use_hdf5.hh"

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "base/statistics.hh"
#include "base/stats/text.hh"
#if USE_HDF5
#include "base/stats/hdf5.hh"
#endif
#include "sim/stat_control.hh"
#include "sim/stat_register.hh"


namespace py = pybind11;

namespace Stats {

void
pythonDump()
{
    py::module m = py::module::import("m5.stats");
    m.attr("dump")();
}

void
pythonReset()
{
    py::module m = py::module::import("m5.stats");
    m.attr("reset")();
}

}

void
pybind_init_stats(py::module &m_native)
{
    py::module m = m_native.def_submodule("stats");

    m
        .def("initSimStats", &Stats::initSimStats)
        .def("initText", &Stats::initText, py::return_value_policy::reference)
#if USE_HDF5
        .def("initHDF5", &Stats::initHDF5)
#endif
        .def("registerPythonStatsHandlers",
             &Stats::registerPythonStatsHandlers)
        .def("schedStatEvent", &Stats::schedStatEvent)
        .def("periodicStatDump", &Stats::periodicStatDump)
        .def("updateEvents", &Stats::updateEvents)
        .def("processResetQueue", &Stats::processResetQueue)
        .def("processDumpQueue", &Stats::processDumpQueue)
        .def("enable", &Stats::enable)
        .def("enabled", &Stats::enabled)
        .def("statsList", &Stats::statsList)
        ;

    py::class_<Stats::Output>(m, "Output")
        .def("begin", &Stats::Output::begin)
        .def("end", &Stats::Output::end)
        .def("valid", &Stats::Output::valid)
        .def("beginGroup", &Stats::Output::beginGroup)
        .def("endGroup", &Stats::Output::endGroup)
        ;

    py::class_<Stats::Info, std::unique_ptr<Stats::Info, py::nodelete>>(
        m, "Info")
        .def_readwrite("name", &Stats::Info::name)
        .def_readonly("desc", &Stats::Info::desc)
        .def_readonly("id", &Stats::Info::id)
        .def_property_readonly("flags", [](const Stats::Info &info) {
                return (Stats::FlagsType)info.flags;
            })
        .def("check", &Stats::Info::check)
        .def("baseCheck", &Stats::Info::baseCheck)
        .def("enable", &Stats::Info::enable)
        .def("prepare", &Stats::Info::prepare)
        .def("reset", &Stats::Info::reset)
        .def("zero", &Stats::Info::zero)
        .def("visit", &Stats::Info::visit)
        ;

    py::class_<Stats::Group, std::unique_ptr<Stats::Group, py::nodelete>>(
        m, "Group")
        .def("regStats", &Stats::Group::regStats)
        .def("resetStats", &Stats::Group::resetStats)
        .def("preDumpStats", &Stats::Group::preDumpStats)
        .def("getStats", &Stats::Group::getStats)
        .def("getStatGroups", &Stats::Group::getStatGroups)
        .def("addStatGroup", &Stats::Group::addStatGroup)
        .def("resolveStat", &Stats::Group::resolveStat)
        ;
}
