/*
 * Copyright (c) 2017, 2019, 2021 ARM Limited
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
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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

#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <ctime>

#include "base/addr_range.hh"
#include "base/inet.hh"
#include "base/loader/elf_object.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/socket.hh"
#include "base/temperature.hh"
#include "base/types.hh"
#include "sim/core.hh"
#include "sim/cur_tick.hh"
#include "sim/drain.hh"
#include "sim/serialize.hh"
#include "sim/sim_object.hh"
#include "cpu/probes/pc_count_pair.hh"

namespace py = pybind11;

namespace gem5
{

/** Resolve a SimObject name using the Pybind configuration */
class PybindSimObjectResolver : public SimObjectResolver
{
    SimObject *resolveSimObject(const std::string &name);
};

PybindSimObjectResolver pybindSimObjectResolver;

SimObject *
PybindSimObjectResolver::resolveSimObject(const std::string &name)
{
    // TODO
    py::module_ m = py::module_::import("m5.SimObject");
    auto f = m.attr("resolveSimObject");

    return f(name).cast<SimObject *>();
}

extern const char *compileDate;
extern const char *gem5Version;

static void
init_drain(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("drain");

    py::enum_<DrainState>(m, "DrainState")
        .value("Running", DrainState::Running)
        .value("Draining", DrainState::Draining)
        .value("Drained", DrainState::Drained)
        ;

    py::class_<Drainable, std::unique_ptr<Drainable, py::nodelete>>(
        m, "Drainable")
        .def("drainState", &Drainable::drainState)
        .def("notifyFork", &Drainable::notifyFork)
        ;

    // The drain manager is a singleton with a private
    // destructor. Disable deallocation from the Python binding.
    py::class_<DrainManager, std::unique_ptr<DrainManager, py::nodelete>>(
        m, "DrainManager")
        .def("tryDrain", &DrainManager::tryDrain)
        .def("resume", &DrainManager::resume)
        .def("preCheckpointRestore", &DrainManager::preCheckpointRestore)
        .def("isDrained", &DrainManager::isDrained)
        .def("state", &DrainManager::state)
        .def("signalDrainDone", &DrainManager::signalDrainDone)
        .def_static("instance", &DrainManager::instance,
                    py::return_value_policy::reference)
        ;
}

static void
init_serialize(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("serialize");

    py::class_<Serializable, std::unique_ptr<Serializable, py::nodelete>>(
        m, "Serializable")
        ;

    py::class_<CheckpointIn>(m, "CheckpointIn")
        ;
}

static void
init_range(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("range");

    py::class_<AddrRange>(m, "AddrRange")
        .def(py::init<>())
        .def(py::init<Addr &, Addr &>())
        .def(py::init<Addr, Addr, const std::vector<Addr> &, uint8_t>())
        .def(py::init<const std::vector<AddrRange> &>())
        .def(py::init<Addr, Addr, uint8_t, uint8_t, uint8_t, uint8_t>())

        .def("__str__", &AddrRange::to_string)

        .def("interleaved", &AddrRange::interleaved)
        .def("granularity", &AddrRange::granularity)
        .def("stripes", &AddrRange::stripes)
        .def("size", &AddrRange::size)
        .def("valid", &AddrRange::valid)
        .def("start", &AddrRange::start)
        .def("end", &AddrRange::end)
        .def("mergesWith", &AddrRange::mergesWith)
        .def("intersects", &AddrRange::intersects)
        .def("isSubset", &AddrRange::isSubset)
        .def("exclude", static_cast<AddrRangeList (AddrRange::*)(
                    const AddrRangeList &) const>(&AddrRange::exclude))
        ;

    m.def("RangeEx", &RangeEx);
    m.def("RangeIn", &RangeIn);
    m.def("RangeSize", &RangeSize);
}

static void
init_pc(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("pc");
    py::class_<PcCountPair>(m, "PcCountPair")
        .def(py::init<>())
        .def(py::init<Addr, int>())
        .def("__eq__", [](const PcCountPair& self, py::object other) {
            py::int_ pyPC = other.attr("get_pc")();
            py::int_ pyCount = other.attr("get_count")();
            uint64_t cPC = pyPC.cast<uint64_t>();
            int cCount = pyCount.cast<int>();
            return (self.getPC() == cPC && self.getCount() == cCount);
        })
        .def("__hash__", [](const PcCountPair& self){
            py::int_ pyPC = py::cast(self.getPC());
            py::int_ pyCount = py::cast(self.getCount());
            return py::hash(py::make_tuple(pyPC, pyCount));
        })
        .def("__str__", &PcCountPair::to_string)
        .def("get_pc", &PcCountPair::getPC)
        .def("get_count", &PcCountPair::getCount)
        ;
}

static void
init_net(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("net");

    py::class_<networking::EthAddr>(m, "EthAddr")
        .def(py::init<>())
        .def(py::init<const std::string &>())
        ;

    py::class_<networking::IpAddress>(m, "IpAddress")
        .def(py::init<>())
        .def(py::init<uint32_t>())
        ;

    py::class_<networking::IpNetmask, networking::IpAddress>(m, "IpNetmask")
        .def(py::init<>())
        .def(py::init<uint32_t, uint8_t>())
        ;

    py::class_<networking::IpWithPort, networking::IpAddress>(m, "IpWithPort")
        .def(py::init<>())
        .def(py::init<uint32_t, uint16_t>())
        ;
}

static void
init_loader(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("loader");

    m.def("setInterpDir", &loader::setInterpDir);
}

static void
init_socket(py::module_ &m_native)
{
    py::module_ m_socket = m_native.def_submodule("socket");
    m_socket
        .def("listenSocketEmptyConfig", &listenSocketEmptyConfig)
        .def("listenSocketInetConfig", &listenSocketInetConfig)
        .def("listenSocketUnixFileConfig", &listenSocketUnixFileConfig)
        .def("listenSocketUnixAbstractConfig",
                &listenSocketUnixAbstractConfig);

    py::class_<ListenSocketConfig>(m_socket, "ListenSocketConfig");
}

void
pybind_init_core(py::module_ &m_native)
{
    py::module_ m_core = m_native.def_submodule("core");

    py::class_<Cycles>(m_core, "Cycles")
        .def(py::init<>())
        .def(py::init<uint64_t>())
        .def("__int__", &Cycles::operator uint64_t)
        .def("__add__", &Cycles::operator+)
        .def("__sub__", &Cycles::operator-)
        ;

    py::class_<Temperature>(m_core, "Temperature")
        .def(py::init<>())
        .def(py::init<double>())
        .def_static("from_celsius", &Temperature::fromCelsius)
        .def_static("from_kelvin", &Temperature::fromKelvin)
        .def_static("from_fahrenheit", &Temperature::fromFahrenheit)
        .def("celsius", &Temperature::toCelsius)
        .def("kelvin", &Temperature::toKelvin)
        .def("fahrenheit", &Temperature::toFahrenheit)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(py::self < py::self)
        .def(py::self <= py::self)
        .def(py::self > py::self)
        .def(py::self >= py::self)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def(float() * py::self)
        .def(py::self / float())
        .def("__str__", [](const Temperature &t) {
                std::stringstream s;
                s << t;
                return s.str();
            })
        .def("__repr__", [](const Temperature &t) {
                std::stringstream s;
                s << "Temperature(" << t.toKelvin() << ")";
                return s.str();
            })
        ;

    py::class_<tm>(m_core, "tm")
        .def_static("gmtime", [](std::time_t t) { return *std::gmtime(&t); })
        .def_readwrite("tm_sec", &tm::tm_sec)
        .def_readwrite("tm_min", &tm::tm_min)
        .def_readwrite("tm_hour", &tm::tm_hour)
        .def_readwrite("tm_mday", &tm::tm_mday)
        .def_readwrite("tm_mon", &tm::tm_mon)
        .def_readwrite("tm_wday", &tm::tm_wday)
        .def_readwrite("tm_yday", &tm::tm_yday)
        .def_readwrite("tm_isdst", &tm::tm_isdst)
        ;

    py::enum_<Logger::LogLevel>(m_core, "LogLevel")
        .value("PANIC", Logger::PANIC)
        .value("FATAL", Logger::FATAL)
        .value("WARN", Logger::WARN)
        .value("INFO", Logger::INFO)
        .value("HACK", Logger::HACK)
        ;

    m_core
        .def("setLogLevel", &Logger::setLevel)
        .def("setOutputDir", &setOutputDir)
        .def("doExitCleanup", &doExitCleanup)

        .def("disableAllListeners", &ListenSocket::disableAll)
        .def("listenersDisabled", &ListenSocket::allDisabled)
        .def("listenersLoopbackOnly", &ListenSocket::loopbackOnly)
        .def("seedRandom", [](uint64_t seed) { random_mt.init(seed); })


        .def("fixClockFrequency", &fixClockFrequency)
        .def("clockFrequencyFixed", &clockFrequencyFixed)

        .def("setClockFrequency", &setClockFrequency)
        .def("getClockFrequency", &getClockFrequency)
        .def("curTick", curTick)
        ;

    /* TODO: These should be read-only */
    m_core.attr("compileDate") = py::cast(compileDate);
    m_core.attr("gem5Version") = py::cast(gem5Version);

    m_core.attr("TRACING_ON") = py::cast(TRACING_ON);

    m_core.attr("MaxTick") = py::cast(MaxTick);

    /*
     * Serialization helpers
     */
    m_core
        .def("serializeAll", &SimObject::serializeAll)
        .def("getCheckpoint", [](const std::string &cpt_dir) {
            SimObject::setSimObjectResolver(&pybindSimObjectResolver);
            return new CheckpointIn(cpt_dir);
        })

        ;


    init_drain(m_native);
    init_serialize(m_native);
    init_range(m_native);
    init_net(m_native);
    init_loader(m_native);
    init_pc(m_native);
    init_socket(m_native);
}

} // namespace gem5
