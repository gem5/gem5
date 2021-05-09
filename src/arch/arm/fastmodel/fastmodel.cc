/*
 * Copyright (c) 2020 ARM Limited
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

#include "python/pybind11/pybind.hh"
#include "scx/scx.h"
#include "sim/init.hh"

namespace gem5
{

namespace
{

void
arm_fast_model_pybind(pybind11::module_ &m_internal)
{
    auto arm_fast_model = m_internal.def_submodule("arm_fast_model");
    arm_fast_model
        .def("scx_initialize", [](std::string id) {
                scx::scx_initialize(id);
             })

        // Loading of applications or raw data.
        .def("scx_load_application", &scx::scx_load_application)
        .def("scx_load_application_all", &scx::scx_load_application_all)
        .def("scx_load_data", &scx::scx_load_data)
        .def("scx_load_data_all", &scx::scx_load_data_all)

        // Only expose the string based versions of these functions. Exposing
        // specializations of the templated versions is likely overkill,
        // especially since there are other preferred methods for setting up
        // the parameters of a component.
        .def("scx_set_parameter",
             static_cast<bool (*)(const std::string &, const std::string &)>(
                 &scx::scx_set_parameter))
        .def("scx_get_parameter",
             static_cast<bool (*)(const std::string &, std::string &)>(
                 &scx::scx_get_parameter))
        .def("scx_get_parameter_list", &scx::scx_get_parameter_list)

        .def("scx_set_cpi_file", &scx::scx_set_cpi_file)

        // These might be used internally by the gem5 fast model wrapper, and
        // may not be worth exposing.
        .def("scx_cpulimit", &scx::scx_cpulimit)
        .def("scx_timelimit", &scx::scx_timelimit)
        .def("scx_simlimit", &scx::scx_simlimit)

        .def("scx_parse_and_configure",
             [](int argc, std::vector<char *> argv,
                const char *trailer=NULL, bool sig_handler=true) {
                 scx::scx_parse_and_configure(argc, argv.data(),
                                              trailer, sig_handler);
             },
             pybind11::arg("argc"),
             pybind11::arg("argv"),
             pybind11::arg("trailer") = NULL,
             pybind11::arg("sig_handler") = true)

        // CADI stuff.
        .def("scx_start_cadi_server", &scx::scx_start_cadi_server,
             pybind11::arg("start") = true,
             pybind11::arg("run") = true,
             pybind11::arg("debug") = false)
        .def("scx_enable_cadi_log", &scx::scx_enable_cadi_log,
             pybind11::arg("log") = true)
        .def("scx_print_port_number", &scx::scx_print_port_number,
             pybind11::arg("print") = true)

        .def("scx_print_statistics", &scx::scx_print_statistics,
             pybind11::arg("print") = true)
        .def("scx_load_plugin", &scx::scx_load_plugin)
        .def("scx_sync", &scx::scx_sync)
        .def("scx_set_min_sync_latency",
             static_cast<void (*)(double)>(&scx::scx_set_min_sync_latency))
        .def("scx_set_min_sync_latency",
             static_cast<void (*)(sg::ticks_t)>(
                 &scx::scx_set_min_sync_latency))
        .def("scx_get_min_sync_latency",
             static_cast<double (*)()>(&scx::scx_get_min_sync_latency))
        .def("scx_get_min_sync_latency",
             static_cast<sg::ticks_t (*)(sg::Tag<sg::ticks_t> *)>(
                 &scx::scx_get_min_sync_latency))
        ;
}
EmbeddedPyBind embed_("arm_fast_model", &arm_fast_model_pybind);

} // anonymous namespace
} // namespace gem5
