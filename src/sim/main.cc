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
 *
 * Authors: Nathan Binkert
 */

#include <Python.h>

#include "sim/init.hh"
#include "sim/init_signals.hh"

// main() is now pretty stripped down and just sets up python and then
// calls initM5Python which loads the various embedded python modules
// into the python environment and then starts things running by
// calling m5Main.
int
main(int argc, char **argv)
{
    int ret;

    // Initialize m5 special signal handling.
    initSignals();

#if PY_MAJOR_VERSION >= 3
    std::unique_ptr<wchar_t[], decltype(&PyMem_RawFree)> program(
        Py_DecodeLocale(argv[0], NULL),
        &PyMem_RawFree);
    Py_SetProgramName(program.get());
#else
    Py_SetProgramName(argv[0]);
#endif

    // Register native modules with Python's init system before
    // initializing the interpreter.
    registerNativeModules();

    // initialize embedded Python interpreter
    Py_Initialize();

    // Initialize the embedded m5 python library
    ret = EmbeddedPython::initAll();

    if (ret == 0) {
        // start m5
        ret = m5Main(argc, argv);
    }

    // clean up Python intepreter.
    Py_Finalize();

    return ret;
}
