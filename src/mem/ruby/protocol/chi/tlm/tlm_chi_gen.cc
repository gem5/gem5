/*
 * Copyright (c) 2024 Arm Limited
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

#include <ARM/TLM/arm_chi.h>

#include "mem/ruby/protocol/chi/tlm/generator.hh"
// This is required by Transaction::expect
#include "pybind11/functional.h"
#include "python/pybind11/pybind.hh"
#include "sim/init.hh"

namespace py = pybind11;

using namespace ARM::CHI;

namespace gem5 {

namespace {

void
tlm_chi_generator_pybind(pybind11::module_ &m_tlm_chi)
{
    auto tlm_chi_gen = m_tlm_chi.def_submodule("tlm_chi_gen");

    using Action = tlm::chi::TlmGenerator::Transaction::Action;
    using Expectation = tlm::chi::TlmGenerator::Transaction::Expectation;
    using Assertion = tlm::chi::TlmGenerator::Transaction::Assertion;
    using Callback = Action::Callback;
    py::class_<tlm::chi::TlmGenerator::Transaction>(tlm_chi_gen, "Transaction")
        .def(py::init<Payload*, Phase&>())
        .def("EXPECT_STR",
            [] (tlm::chi::TlmGenerator::Transaction &self,
                std::string name,
                Callback cb)
            {
                self.addCallback(std::make_unique<Expectation>(name, cb));
            })
        .def("EXPECT",
            [] (tlm::chi::TlmGenerator::Transaction &self,
                py::function cb)
            {
                self.addCallback(std::make_unique<Expectation>(
                    cb.attr("__name__").cast<std::string>(),
                    cb.cast<Callback>()));
            })
        .def("ASSERT",
            [] (tlm::chi::TlmGenerator::Transaction &self,
                std::string name,
                Callback cb)
            {
                self.addCallback(std::make_unique<Assertion>(name, cb));
            })
        .def("DO",
            [] (tlm::chi::TlmGenerator::Transaction &self,
                Callback cb)
            {
                self.addCallback(std::make_unique<Action>(cb, false));
            })
        .def("DO_WAIT",
            [] (tlm::chi::TlmGenerator::Transaction &self,
                Callback cb)
            {
                self.addCallback(std::make_unique<Action>(cb, true));
            })
        .def("inject", &tlm::chi::TlmGenerator::Transaction::inject)
        .def_property("phase",
            &tlm::chi::TlmGenerator::Transaction::phase,
            &tlm::chi::TlmGenerator::Transaction::phase)
        ;
}

EmbeddedPyBind embed_("tlm_chi_gen", &tlm_chi_generator_pybind, "tlm_chi");

} // namespace
} // namespace gem5
