/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#include <Python.h>

#include <iostream>

#include "pybind11/embed.h"
#include "pybind11/pybind11.h"

#include "python/embedded.hh"
#include "sim/init_signals.hh"

using namespace gem5;

namespace py = pybind11;

// main() is now pretty stripped down and just sets up python and then
// calls EmbeddedPython::initAll which loads the various embedded python
// modules into the python environment and then starts things running by
// running python's m5.main().
int
main(int argc, char **argv)
{
    // Initialize gem5 special signal handling.
    initSignals();

#if PY_VERSION_HEX < 0x03080000
    // Convert argv[0] to a wchar_t string, using python's locale and cleanup
    // functions.
    std::unique_ptr<wchar_t[], decltype(&PyMem_RawFree)> program(
        Py_DecodeLocale(argv[0], nullptr), &PyMem_RawFree);

    // This can help python find libraries at run time relative to this binary.
    // It's probably not necessary, but is mostly harmless and might be useful.
    Py_SetProgramName(program.get());
#else
    // Preinitialize Python for Python 3.8+
    // This ensures that the locale configuration takes effect
    PyStatus status;

    PyConfig config;
    PyConfig_InitPythonConfig(&config);

    /* Set the program name. Implicitly preinitialize Python. */
    status = PyConfig_SetBytesString(&config, &config.program_name, argv[0]);
    if (PyStatus_Exception(status)) {
        PyConfig_Clear(&config);
        Py_ExitStatusException(status);
        return 1;
    }
#endif

    py::scoped_interpreter guard(true, argc, argv);

    auto importer = py::module_::import("importer");
    importer.attr("install")();

    try {
        py::module_::import("m5").attr("main")();
    } catch (py::error_already_set &e) {
        if (e.matches(PyExc_SystemExit))
            return e.value().attr("code").cast<int>();

        std::cerr << e.what();
        return 1;
    }

    return 0;
}
