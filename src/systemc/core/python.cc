/*
 * Copyright 2018 Google, Inc.
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

#include "systemc/core/python.hh"

#include <vector>

#include "python/pybind11/pybind.hh"
#include "sim/init.hh"

namespace sc_gem5
{

namespace
{

PythonInitFunc *&
firstInitFunc()
{
    static PythonInitFunc *first = nullptr;
    return first;
}

bool python_initialized = false;

void
systemc_pybind(pybind11::module_ &m_internal)
{
    pybind11::module_ m = m_internal.def_submodule("systemc");
    for (auto ptr = firstInitFunc(); ptr; ptr = ptr->next)
        ptr->callback(m);

    python_initialized = true;
}

gem5::EmbeddedPyBind embed_("systemc", &systemc_pybind);

} // anonymous namespace

PythonInitFunc::PythonInitFunc(Callback run)
    : callback(run), next(firstInitFunc())
{
    firstInitFunc() = this;

    // If the python was already initialized, run the callback immediately.
    if (python_initialized) {
        auto systemc_module = pybind11::module_::import("_m5.systemc");
        callback(systemc_module);
    }
}

} // namespace sc_gem5
