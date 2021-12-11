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

#include <map>
#include <string>

#include "base/cprintf.hh"
#include "python/pybind11/pybind.hh"
#include "python/pybind_init.hh"

namespace py = pybind11;

namespace gem5
{

pybind11::module_ *EmbeddedPyBind::mod = nullptr;

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module_ &),
                               const char *_base) :
    initFunc(init_func), name(_name), base(_base)
{
    init();
}

EmbeddedPyBind::EmbeddedPyBind(const char *_name,
                               void (*init_func)(py::module_ &)) :
    EmbeddedPyBind(_name, init_func, "")
{}

void
EmbeddedPyBind::init()
{
    // If this module is already registered, complain and stop.
    if (registered) {
        cprintf("Warning: %s already registered.\n", name);
        return;
    }

    auto &ready = getReady();
    auto &pending = getPending();

    // If we're not ready for this module yet, defer intialization.
    if (!mod || (!base.empty() && ready.find(base) == ready.end())) {
        pending.insert({std::string(base), this});
        return;
    }

    // We must be ready, so set this module up.
    initFunc(*mod);
    ready[name] = this;
    registered = true;

    // Find any other modules that were waiting for this one and init them.
    initPending(name);
}

void
EmbeddedPyBind::initPending(const std::string &finished)
{
    auto &pending = getPending();

    auto range = pending.equal_range(finished);
    std::list<std::pair<std::string, EmbeddedPyBind *>> todo(
        range.first, range.second);
    pending.erase(range.first, range.second);

    for (auto &entry: todo)
        entry.second->init();
}

std::map<std::string, EmbeddedPyBind *> &
EmbeddedPyBind::getReady()
{
    static std::map<std::string, EmbeddedPyBind *> ready;
    return ready;
}

std::multimap<std::string, EmbeddedPyBind *> &
EmbeddedPyBind::getPending()
{
    static std::multimap<std::string, EmbeddedPyBind *> pending;
    return pending;
}

void
EmbeddedPyBind::initAll(py::module_ &_m5)
{
    pybind_init_core(_m5);
    pybind_init_debug(_m5);

    pybind_init_event(_m5);
    pybind_init_stats(_m5);

    mod = &_m5;

    // Init all the modules that were waiting on the _m5 module itself.
    initPending("");
}

GEM5_PYBIND_MODULE_INIT(_m5, EmbeddedPyBind::initAll)

} // namespace gem5
