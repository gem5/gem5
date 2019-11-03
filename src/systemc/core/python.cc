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
 *
 * Authors: Gabe Black
 */

#include "systemc/core/python.hh"

#include <vector>

#include "python/pybind11/pybind.hh"
#include "sim/init.hh"

namespace sc_gem5
{

namespace
{

PythonReadyFunc *&
firstReadyFunc()
{
    static PythonReadyFunc *first = nullptr;
    return first;
}

PythonInitFunc *&
firstInitFunc()
{
    static PythonInitFunc *first = nullptr;
    return first;
}

void
python_ready(pybind11::args args)
{
    for (auto ptr = firstReadyFunc(); ptr; ptr = ptr->next)
        ptr->run();
}

void
systemc_pybind(pybind11::module &m_internal)
{
    pybind11::module m = m_internal.def_submodule("systemc");
    m.def("python_ready", &python_ready);
    for (auto ptr = firstInitFunc(); ptr; ptr = ptr->next)
        ptr->run(m);
}
EmbeddedPyBind embed_("systemc", &systemc_pybind);

} // anonymous namespace

PythonReadyFunc::PythonReadyFunc() : next(firstReadyFunc())
{
    firstReadyFunc() = this;
}

PythonInitFunc::PythonInitFunc() : next(firstInitFunc())
{
    firstInitFunc() = this;
}

} // namespace sc_gem5
