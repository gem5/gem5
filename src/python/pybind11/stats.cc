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

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "base/statistics.hh"
#include "base/stats/text.hh"
#include "config/have_hdf5.hh"

#if HAVE_HDF5
#include "base/stats/hdf5.hh"

#endif
#include "sim/stat_control.hh"
#include "sim/stat_register.hh"

namespace py = pybind11;

namespace gem5
{

static const py::object
cast_stat_info(const statistics::Info *info)
{
    /* PyBind11 gets confused by the InfoProxy magic, so we need to
     * explicitly cast to the right wrapper type. */

#define TRY_CAST(T)                                                           \
    do {                                                                      \
        auto _stat = dynamic_cast<const T *>(info);                           \
        if (_stat)                                                            \
            return py::cast(_stat);                                           \
    } while (0)

    TRY_CAST(statistics::ScalarInfo);
    /* FormulaInfo is a subclass of VectorInfo. Therefore, a cast to
     * FormulaInfo must be attempted before a cast to VectorInfo. Otherwise
     * instances of ForumlaInfo will be cast to VectorInfo.
     */
    TRY_CAST(statistics::FormulaInfo);
    TRY_CAST(statistics::VectorInfo);
    TRY_CAST(statistics::DistInfo);

    return py::cast(info);

#undef TRY_CAST
}

namespace statistics
{

void
pythonDump()
{
    py::module_ m = py::module_::import("m5.stats");
    m.attr("dump")();
}

void
pythonReset()
{
    py::module_ m = py::module_::import("m5.stats");
    m.attr("reset")();
}

} // namespace statistics

void
pybind_init_stats(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("stats");

    m.def("initSimStats", &statistics::initSimStats)
        .def("initText", &statistics::initText,
             py::return_value_policy::reference)
#if HAVE_HDF5
        .def("initHDF5", &statistics::initHDF5)
#endif
        .def("registerPythonStatsHandlers",
             &statistics::registerPythonStatsHandlers)
        .def("schedStatEvent", &statistics::schedStatEvent)
        .def("periodicStatDump", &statistics::periodicStatDump)
        .def("updateEvents", &statistics::updateEvents)
        .def("processResetQueue", &statistics::processResetQueue)
        .def("processDumpQueue", &statistics::processDumpQueue)
        .def("enable", &statistics::enable)
        .def("enabled", &statistics::enabled)
        .def("statsList", &statistics::statsList);

    py::class_<statistics::Output>(m, "Output")
        .def("begin", &statistics::Output::begin)
        .def("end", &statistics::Output::end)
        .def("valid", &statistics::Output::valid)
        .def("beginGroup", &statistics::Output::beginGroup)
        .def("endGroup", &statistics::Output::endGroup);

    py::class_<statistics::Info,
               std::unique_ptr<statistics::Info, py::nodelete>>(m, "Info")
        .def_readwrite("name", &statistics::Info::name)
        .def_property_readonly("unit",
                               [](const statistics::Info &info) {
                                   return info.unit->getUnitString();
                               })
        .def_readonly("desc", &statistics::Info::desc)
        .def_readonly("id", &statistics::Info::id)
        .def_property_readonly("flags", [](const statistics::Info &info) {
                return (statistics::FlagsType)info.flags;
            })
        .def_property_readonly("is_nozero", [](const statistics::Info &info) {
                return info.flags.isSet(statistics::nozero);
            })
        .def("check", &statistics::Info::check)
        .def("baseCheck", &statistics::Info::baseCheck)
        .def("enable", &statistics::Info::enable)
        .def("prepare", &statistics::Info::prepare)
        .def("reset", &statistics::Info::reset)
        .def("zero", &statistics::Info::zero)
        .def("visit", &statistics::Info::visit);

    py::class_<statistics::ScalarInfo, statistics::Info,
               std::unique_ptr<statistics::ScalarInfo, py::nodelete>>(
        m, "ScalarInfo")
        .def_property_readonly(
            "value",
            [](const statistics::ScalarInfo &info) { return info.value(); })
        .def_property_readonly(
            "result",
            [](const statistics::ScalarInfo &info) { return info.result(); })
        .def_property_readonly(
            "total",
            [](const statistics::ScalarInfo &info) { return info.total(); });

    py::class_<statistics::VectorInfo, statistics::Info,
               std::unique_ptr<statistics::VectorInfo, py::nodelete>>(
        m, "VectorInfo")
        .def_readwrite("subnames", &statistics::VectorInfo::subnames)
        .def_readwrite("subdescs", &statistics::VectorInfo::subdescs)
        .def_property_readonly(
            "size",
            [](const statistics::VectorInfo &info) { return info.size(); })
        .def_property_readonly(
            "value",
            [](const statistics::VectorInfo &info) { return info.value(); })
        .def_property_readonly(
            "result",
            [](const statistics::VectorInfo &info) { return info.result(); })
        .def_property_readonly(
            "total",
            [](const statistics::VectorInfo &info) { return info.total(); });

    py::class_<statistics::FormulaInfo, statistics::VectorInfo,
               std::unique_ptr<statistics::FormulaInfo, py::nodelete>>(
        m, "FormulaInfo")
        .def_property_readonly("str", [](const statistics::FormulaInfo &info) {
            return info.str();
        });

    py::class_<statistics::DistInfo, statistics::Info,
               std::unique_ptr<statistics::DistInfo, py::nodelete>>(m,
                                                                    "DistInfo")
        .def_property_readonly(
            "min_val",
            [](const statistics::DistInfo &info) { return info.data.min_val; })
        .def_property_readonly(
            "max_val",
            [](const statistics::DistInfo &info) { return info.data.max_val; })
        .def_property_readonly("bucket_size",
                               [](const statistics::DistInfo &info) {
                                   return info.data.bucket_size;
                               })
        .def_property_readonly(
            "values",
            [](const statistics::DistInfo &info) { return info.data.cvec; })
        .def_property_readonly("overflow",
                               [](const statistics::DistInfo &info) {
                                   return info.data.overflow;
                               })
        .def_property_readonly("underflow",
                               [](const statistics::DistInfo &info) {
                                   return info.data.underflow;
                               })
        .def_property_readonly(
            "sum",
            [](const statistics::DistInfo &info) { return info.data.sum; })
        .def_property_readonly(
            "logs",
            [](const statistics::DistInfo &info) { return info.data.logs; })
        .def_property_readonly("squares",
                               [](const statistics::DistInfo &info) {
                                   return info.data.squares;
                               });

    py::class_<statistics::Group,
               std::unique_ptr<statistics::Group, py::nodelete>>(m, "Group")
        .def("regStats", &statistics::Group::regStats)
        .def("resetStats", &statistics::Group::resetStats)
        .def("preDumpStats", &statistics::Group::preDumpStats)
        .def("getStats",
             [](const statistics::Group &self) -> std::vector<py::object> {
                 auto stats = self.getStats();
                 std::vector<py::object> py_stats;
                 py_stats.reserve(stats.size());
                 std::transform(stats.begin(), stats.end(),
                                std::back_inserter(py_stats), cast_stat_info);
                 return py_stats;
             })
        .def("getStatGroups", &statistics::Group::getStatGroups)
        .def("addStatGroup", &statistics::Group::addStatGroup)
        .def("resolveStat",
             [](const statistics::Group &self,
                const std::string &name) -> py::object {
                 const statistics::Info *stat = self.resolveStat(name);
                 if (!stat)
                     throw pybind11::key_error("Unknown stat name");

                 return cast_stat_info(stat);
             });
}

} // namespace gem5
