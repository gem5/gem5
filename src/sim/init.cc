/*
 * Copyright (c) 2012, 2017 ARM Limited
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
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
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

#include <marshal.h>
#include <zlib.h>

#include <iostream>
#include <list>
#include <string>

#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/types.hh"
#include "config/have_protobuf.hh"
#include "python/pybind11/pybind.hh"
#include "sim/async.hh"
#include "sim/core.hh"

#if HAVE_PROTOBUF
#include <google/protobuf/stubs/common.h>

#endif

using namespace std;
namespace py = pybind11;

// The python library is totally messed up with respect to constness,
// so make a simple macro to make life a little easier
#define PyCC(x) (const_cast<char *>(x))

EmbeddedPython *EmbeddedPython::importer = NULL;
PyObject *EmbeddedPython::importerModule = NULL;
EmbeddedPython::EmbeddedPython(const char *filename, const char *abspath,
    const char *modpath, const unsigned char *code, int zlen, int len)
    : filename(filename), abspath(abspath), modpath(modpath), code(code),
      zlen(zlen), len(len)
{
    // if we've added the importer keep track of it because we need it
    // to bootstrap.
    if (string(modpath) == string("importer"))
        importer = this;
    else
        getList().push_back(this);
}

list<EmbeddedPython *> &
EmbeddedPython::getList()
{
    static list<EmbeddedPython *> the_list;
    return the_list;
}

/*
 * Uncompress and unmarshal the code object stored in the
 * EmbeddedPython
 */
PyObject *
EmbeddedPython::getCode() const
{
    Bytef marshalled[len];
    uLongf unzlen = len;
    int ret = uncompress(marshalled, &unzlen, (const Bytef *)code, zlen);
    if (ret != Z_OK)
        panic("Could not uncompress code: %s\n", zError(ret));
    assert(unzlen == (uLongf)len);

    return PyMarshal_ReadObjectFromString((char *)marshalled, len);
}

bool
EmbeddedPython::addModule() const
{
    PyObject *code = getCode();
    PyObject *result = PyObject_CallMethod(importerModule, PyCC("add_module"),
        PyCC("sssO"), filename, abspath, modpath, code);
    if (!result) {
        PyErr_Print();
        return false;
    }

    Py_DECREF(result);
    return true;
}

/*
 * Load and initialize all of the python parts of M5.
 */
int
EmbeddedPython::initAll()
{
    // Load the importer module
    PyObject *code = importer->getCode();
    importerModule = PyImport_ExecCodeModule(PyCC("importer"), code);
    if (!importerModule) {
        PyErr_Print();
        return 1;
    }

    // Load the rest of the embedded python files into the embedded
    // python importer
    list<EmbeddedPython *>::iterator i = getList().begin();
    list<EmbeddedPython *>::iterator end = getList().end();
    for (; i != end; ++i)
        if (!(*i)->addModule())
            return 1;

    return 0;
}

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module &),
                               const char *_base)
    : initFunc(init_func), registered(false), name(_name), base(_base)
{
    getMap()[_name] = this;
}

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module &))
    : initFunc(init_func), registered(false), name(_name), base("")
{
    getMap()[_name] = this;
}

void
EmbeddedPyBind::init(py::module &m)
{
    if (!registered) {
        initFunc(m);
        registered = true;
    } else {
        cprintf("Warning: %s already registered.\n", name);
    }
}

bool
EmbeddedPyBind::depsReady() const
{
    return base.empty() || getMap()[base]->registered;
}

std::map<std::string, EmbeddedPyBind *> &
EmbeddedPyBind::getMap()
{
    static std::map<std::string, EmbeddedPyBind *> objs;
    return objs;
}

void
EmbeddedPyBind::initAll()
{
    std::list<EmbeddedPyBind *> pending;

    py::module m_m5 = py::module("_m5");
    m_m5.attr("__package__") = py::cast("_m5");

    pybind_init_core(m_m5);
    pybind_init_debug(m_m5);

    pybind_init_event(m_m5);
    pybind_init_pyobject(m_m5);
    pybind_init_stats(m_m5);

    for (auto &kv : getMap()) {
        auto &obj = kv.second;
        if (obj->base.empty()) {
            obj->init(m_m5);
        } else {
            pending.push_back(obj);
        }
    }

    while (!pending.empty()) {
        for (auto it = pending.begin(); it != pending.end(); ) {
            EmbeddedPyBind &obj = **it;
            if (obj.depsReady()) {
                obj.init(m_m5);
                it = pending.erase(it);
            } else {
                ++it;
            }
        }
    }
}

int
initM5Python()
{
    EmbeddedPyBind::initAll();
    return EmbeddedPython::initAll();
}

/*
 * Make the commands array weak so that they can be overridden (used
 * by unit tests to specify a different python main function.
 */
const char * __attribute__((weak)) m5MainCommands[] = {
    "import m5",
    "m5.main()",
    0 // sentinel is required
};

/*
 * Start up the M5 simulator.  This mostly vectors into the python
 * main function.
 */
int
m5Main(int argc, char **argv)
{
#if HAVE_PROTOBUF
    // Verify that the version of the protobuf library that we linked
    // against is compatible with the version of the headers we
    // compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;
#endif

    PySys_SetArgv(argc, argv);

    // We have to set things up in the special __main__ module
    PyObject *module = PyImport_AddModule(PyCC("__main__"));
    if (module == NULL)
        panic("Could not import __main__");
    PyObject *dict = PyModule_GetDict(module);

    // import the main m5 module
    PyObject *result;
    const char **command = m5MainCommands;

    // evaluate each command in the m5MainCommands array (basically a
    // bunch of python statements.
    while (*command) {
        result = PyRun_String(*command, Py_file_input, dict, dict);
        if (!result) {
            PyErr_Print();
            return 1;
        }
        Py_DECREF(result);

        command++;
    }

#if HAVE_PROTOBUF
    google::protobuf::ShutdownProtobufLibrary();
#endif

    return 0;
}

PyMODINIT_FUNC
initm5(void)
{
    initM5Python();
    PyImport_ImportModule(PyCC("m5"));
}
