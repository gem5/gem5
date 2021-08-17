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
 */

#include "pybind11/embed.h"

#include "sim/init.hh"

#include <zlib.h>

#include <list>
#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/have_protobuf.hh"
#include "python/pybind11/pybind.hh"

#if HAVE_PROTOBUF
#include <google/protobuf/stubs/common.h>

#endif

namespace py = pybind11;

namespace gem5
{

EmbeddedPython::EmbeddedPython(const char *abspath, const char *modpath,
        const unsigned char *code, int zlen, int len)
    : abspath(abspath), modpath(modpath), code(code), zlen(zlen), len(len)
{
    getList().push_back(this);
}

std::list<EmbeddedPython *> &
EmbeddedPython::getList()
{
    static std::list<EmbeddedPython *> the_list;
    return the_list;
}

/*
 * Uncompress and unmarshal the code object stored in the
 * EmbeddedPython
 */
py::object
EmbeddedPython::getCode() const
{
    Bytef marshalled[len];
    uLongf unzlen = len;
    int ret = uncompress(marshalled, &unzlen, (const Bytef *)code, zlen);
    if (ret != Z_OK)
        panic("Could not uncompress code: %s\n", zError(ret));
    assert(unzlen == (uLongf)len);

    auto marshal = py::module_::import("marshal");
    return marshal.attr("loads")(py::bytes((char *)marshalled, len));
}

bool
EmbeddedPython::addModule() const
{
    auto importer = py::module_::import("importer");
    importer.attr("add_module")(abspath, modpath, getCode());
    return true;
}

/*
 * Load and initialize all of the python parts of M5.
 */
int
EmbeddedPython::initAll()
{
    // Load the embedded python files into the embedded python importer.
    for (auto *embedded: getList()) {
        if (!embedded->addModule())
            return 1;
    }
    return 0;
}

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module_ &),
                               const char *_base)
    : initFunc(init_func), registered(false), name(_name), base(_base)
{
    getMap()[_name] = this;
}

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module_ &))
    : initFunc(init_func), registered(false), name(_name), base("")
{
    getMap()[_name] = this;
}

void
EmbeddedPyBind::init(py::module_ &m)
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
EmbeddedPyBind::initAll(py::module_ &_m5)
{
    std::list<EmbeddedPyBind *> pending;

    pybind_init_core(_m5);
    pybind_init_debug(_m5);

    pybind_init_event(_m5);
    pybind_init_stats(_m5);

    for (auto &kv : EmbeddedPyBind::getMap()) {
        auto &obj = kv.second;
        if (obj->base.empty()) {
            obj->init(_m5);
        } else {
            pending.push_back(obj);
        }
    }

    while (!pending.empty()) {
        for (auto it = pending.begin(); it != pending.end(); ) {
            EmbeddedPyBind &obj = **it;
            if (obj.depsReady()) {
                obj.init(_m5);
                it = pending.erase(it);
            } else {
                ++it;
            }
        }
    }
}

PYBIND11_EMBEDDED_MODULE(_m5, _m5)
{
    EmbeddedPyBind::initAll(_m5);
}

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
    // Clear out argv just in case it has something in it.
    py_argv.attr("clear")();

    // Fill it with our argvs.
    for (int i = 0; i < argc; i++)
        py_argv.append(argv[i]);

    try {
        py::module_::import("m5").attr("main")();
    } catch (py::error_already_set &e) {
        if (e.matches(PyExc_SystemExit))
            return e.value().attr("code").cast<int>();

        std::cerr << e.what();
        return 1;
    }

#if HAVE_PROTOBUF
    google::protobuf::ShutdownProtobufLibrary();
#endif

    return 0;
}

} // namespace gem5
