/*
 * Copyright (c) 2025 Arm Limited
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

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "mem/port_proxy.hh"

namespace py = pybind11;

namespace gem5
{

void
pybind_init_port(py::module_ &m_native)
{
    py::module_ m = m_native.def_submodule("port");

    py::class_<gem5::PortProxy>(m, "PyPort")
        .def(
            "read",
            [](gem5::PortProxy &self, Addr phys_addr, uint64_t size) {
                std::vector<uint8_t> buffer(size);
                if (!self.tryReadBlob(phys_addr, buffer.data(), size)) {
                    throw std::runtime_error(csprintf(
                        "Failed to read from address: %#x\n", phys_addr));
                }
                return py::bytes(reinterpret_cast<const char *>(buffer.data()),
                                 size);
            },
            py::arg("addr"), py::arg("size"),
            "Read size bytes from addr and return as Python bytes")
        .def(
            "write",
            [](gem5::PortProxy &self, Addr phys_addr, py::buffer src_buf) {
                py::buffer_info info = src_buf.request();

                if (!self.tryWriteBlob(
                        phys_addr, reinterpret_cast<const uint8_t *>(info.ptr),
                        info.size)) {
                    throw std::runtime_error(csprintf(
                        "Failed to write to address: %#x\n", phys_addr));
                }
            },
            py::arg("addr"), py::arg("data"),
            "Write from any 1D byte-like buffer into memory at addr");
}

} // namespace gem5
