/*
 * Copyright (c) 2019 ARM Limited
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

#include <pybind11/embed.h>

#include <cstdlib>
#include <iostream>

namespace py = pybind11;

/*
 * This wrapper program runs python scripts using the python interpretter which
 * will be built into gem5. Its first argument is the script to run, and then
 * all subsequent arguments are passed to the python script as its argv.
 */

int
main(int argc, const char **argv)
{
    py::scoped_interpreter guard;

    // Embedded python doesn't set up sys.argv, so we'll do that ourselves.
    py::list py_argv;
    auto sys = py::module::import("sys");
    if (py::hasattr(sys, "argv")) {
        // sys.argv already exists, so grab that.
        py_argv = sys.attr("argv");
    } else {
        // sys.argv doesn't exist, so create it.
        sys.add_object("argv", py_argv);
    }

    auto importer = py::module_::import("importer");
    importer.attr("install")();

    // Clear out argv just in case it has something in it.
    py_argv.attr("clear")();

    if (argc < 2) {
        std::cerr << "Usage: gem5py SCRIPT [arg] ..." << std::endl;
        std::exit(1);
    }

    // Fill it with our argvs.
    for (int i = 1; i < argc; i++)
        py_argv.append(argv[i]);

    // Actually call the script.
    py::eval_file(argv[1]);

    return 0;
}
