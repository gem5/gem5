/*
 * Copyright 2021 Google, Inc.
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

#ifndef __PYTHON_PYBIND_INIT_HH__
#define __PYTHON_PYBIND_INIT_HH__

#include <Python.h>

#include "pybind11/pybind11.h"

namespace gem5
{

struct PybindModuleInit
{
    PybindModuleInit(const char *name, PyObject *(*func)())
    {
        if (Py_IsInitialized()) {
            // If python is running, initialize and inject the module.
            PyImport_AddModule(name);
            PyObject *sys_modules = PyImport_GetModuleDict();
            PyDict_SetItemString(sys_modules, name, func());
        } else {
            // If not, tell python how to build importer itself later.
            PyImport_AppendInittab(name, func);
        }
    }
};

} // namespace gem5

#define GEM5_PYBIND_MODULE_INIT(name, func)                                   \
    namespace                                                                 \
    {                                                                         \
                                                                              \
    ::PyObject *                                                              \
    initializer()                                                             \
    {                                                                         \
        static ::pybind11::module_::module_def mod_def;                       \
        static auto m = ::pybind11::module_::create_extension_module(         \
            #name, nullptr, &mod_def);                                        \
        func(m);                                                              \
        m.inc_ref();                                                          \
        return m.ptr();                                                       \
    }                                                                         \
                                                                              \
    ::gem5::PybindModuleInit modInit(#name, initializer);                     \
                                                                              \
    } // anonymous namespace

#endif // __PYTHON_PYBIND_INIT_HH__
