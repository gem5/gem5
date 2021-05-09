/*
 * Copyright 2020 Google, Inc.
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

#include "base/loader/object_file.hh"
#include "python/pybind11/pybind.hh"
#include "sim/init.hh"

namespace py = pybind11;

namespace gem5
{

namespace
{

void
objectfile_pybind(py::module_ &m_internal)
{
    py::module_ m = m_internal.def_submodule("object_file");

    py::class_<loader::ObjectFile>(m, "ObjectFile")
        .def("get_arch", [](const loader::ObjectFile &obj) {
                return loader::archToString(obj.getArch());
                }, py::return_value_policy::reference)
        .def("get_op_sys", [](const loader::ObjectFile &obj) {
                return loader::opSysToString(obj.getOpSys());
                }, py::return_value_policy::reference)
        .def("entry_point", &loader::ObjectFile::entryPoint)
        .def("get_interpreter", &loader::ObjectFile::getInterpreter);

    m.def("create", [](const std::string &fname) {
            return loader::createObjectFile(fname); });
}
EmbeddedPyBind embed_("object_file", &objectfile_pybind);

} // anonymous namespace
} // namespace gem5
