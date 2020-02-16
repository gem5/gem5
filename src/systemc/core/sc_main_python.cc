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

// This should be on top since it is including python headers
#include "systemc/core/python.hh"

#include <cstring>
#include <string>

#include "base/fiber.hh"
#include "base/logging.hh"
#include "systemc/core/sc_main_fiber.hh"

namespace
{

// This wrapper adapts the python version of sc_main to the c++ version.
void
sc_main(pybind11::args args)
{
    panic_if(::sc_gem5::scMainFiber.called(),
            "sc_main called more than once.");

    int argc = args.size();
    char **argv = new char *[argc];

    // Initialize all the argvs to NULL so we can delete [] them
    // unconditionally.
    for (int idx = 0; idx < argc; idx++)
        argv[idx] = NULL;

    // Attempt to convert all the arguments to strings. If that fails, clean
    // up after ourselves. Also don't count this as a call to sc_main since
    // we never got to the c++ version of that function.
    try {
        for (int idx = 0; idx < argc; idx++) {
            std::string arg = args[idx].cast<std::string>();
            argv[idx] = new char[arg.length() + 1];
            strcpy(argv[idx], arg.c_str());
        }
    } catch (...) {
        // If that didn't work for some reason (probably a conversion error)
        // blow away argv and argc and pass on the exception.
        for (int idx = 0; idx < argc; idx++)
            delete [] argv[idx];
        delete [] argv;
        argc = 0;
        throw;
    }

    ::sc_gem5::scMainFiber.setArgs(argc, argv);
    ::sc_gem5::scMainFiber.run();
}

int
sc_main_result_code()
{
    return ::sc_gem5::scMainFiber.resultInt();
}

std::string
sc_main_result_str()
{
    return ::sc_gem5::scMainFiber.resultStr();
}

// Make our sc_main wrapper available in the internal _m5 python module under
// the systemc submodule.

struct InstallScMain : public ::sc_gem5::PythonInitFunc
{
    void
    run(pybind11::module &systemc) override
    {
        systemc.def("sc_main", &sc_main);
        systemc.def("sc_main_result_code", &sc_main_result_code);
        systemc.def("sc_main_result_str", &sc_main_result_str);
    }
} installScMain;

} // anonymous namespace
