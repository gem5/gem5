/*
 * Copyright 2019 Google, Inc.
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

#include "systemc/core/python.hh"
#include "systemc/ext/core/sc_time.hh"

namespace
{

::sc_gem5::PythonInitFunc installScTime([](pybind11::module_ &systemc) {
    pybind11::class_<sc_core::sc_time> sc_time(systemc, "sc_time");
    sc_time
        // Constructors (omitting nonstandard and deprecated)
        .def(pybind11::init<>())
        .def(pybind11::init<double, sc_core::sc_time_unit>())
        .def(pybind11::init<const sc_core::sc_time &>())

        // Converters from sc_time.
        .def("value", &sc_core::sc_time::value)
        .def("to_double", &sc_core::sc_time::to_double)
        .def("to_seconds", &sc_core::sc_time::to_seconds)
        .def("to_string", &sc_core::sc_time::to_string)
        .def("__str__", &sc_core::sc_time::to_string)

        // Converters into sc_time.
        .def_static("from_value", &sc_core::sc_time::from_value)
        .def_static("from_seconds", &sc_core::sc_time::from_seconds)
        .def_static("from_string", &sc_core::sc_time::from_string)

        // Operators.
        .def(pybind11::self == pybind11::self)
        .def(pybind11::self != pybind11::self)
        .def(pybind11::self < pybind11::self)
        .def(pybind11::self <= pybind11::self)
        .def(pybind11::self > pybind11::self)
        .def(pybind11::self >= pybind11::self)
        .def(pybind11::self += pybind11::self)
        .def(pybind11::self -= pybind11::self)
        .def(pybind11::self *= double())
        .def(pybind11::self /= double())
        .def(pybind11::self + pybind11::self)
        .def(pybind11::self - pybind11::self)
        .def(pybind11::self * double())
        .def(pybind11::self / double())
        ;

    pybind11::enum_<sc_core::sc_time_unit>(sc_time, "sc_time_unit")
        .value("SC_FS", sc_core::SC_FS)
        .value("SC_PS", sc_core::SC_PS)
        .value("SC_NS", sc_core::SC_NS)
        .value("SC_US", sc_core::SC_US)
        .value("SC_MS", sc_core::SC_MS)
        .value("SC_SEC", sc_core::SC_SEC)
        .export_values()
        ;
});

} // anonymous namespace
