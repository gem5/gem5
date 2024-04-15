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

#include "systemc/core/module.hh"

#include <cassert>

#include "base/logging.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_export.hh"
#include "systemc/ext/core/sc_port.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

namespace
{

std::list<Module *> _modules;
Module *_new_module;

} // anonymous namespace

UniqueNameGen globalNameGen;

Module::Module(const char *name)
    : _name(name),
      _sc_mod(nullptr),
      _obj(nullptr),
      _ended(false),
      _deprecatedConstructor(false),
      bindingIndex(0)
{
    using namespace gem5;
    panic_if(_new_module, "Previous module not finished.\n");
    _new_module = this;
}

Module::~Module()
{
    // Aborted module construction?
    if (_new_module == this)
        _new_module = nullptr;

    // Attempt to pop now in case we're at the top of the stack, so that
    // a stale pointer to us isn't left floating around for somebody to trip
    // on.
    pop();

    allModules.remove(this);
}

void
Module::finish(Object *this_obj)
{
    assert(!_obj);
    _obj = this_obj;
    _modules.push_back(this);
    pushParentModule(this);
    try {
        _new_module = nullptr;
        // This is called from the constructor of this_obj, so it can't use
        // dynamic cast.
        sc_mod(static_cast<::sc_core::sc_module *>(this_obj->sc_obj()));
        allModules.emplace_back(this);
    } catch (...) {
        popParentModule();
        throw;
    }
}

void
Module::pop()
{
    if (_modules.empty() || _modules.back() != this)
        return;

    using namespace gem5;
    panic_if(_new_module, "Pop with unfinished module.\n");

    _modules.pop_back();
    popParentModule();
}

void
Module::bindPorts(std::vector<const ::sc_core::sc_bind_proxy *> &proxies)
{
    using namespace gem5;
    panic_if(proxies.size() > ports.size(),
             "Trying to bind %d interfaces/ports to %d ports.\n",
             proxies.size(), ports.size());

    auto proxyIt = proxies.begin();
    auto portIt = ports.begin();
    portIt += bindingIndex;
    for (; proxyIt != proxies.end(); proxyIt++, portIt++) {
        auto proxy = *proxyIt;
        auto port = *portIt;
        if (proxy->interface())
            port->vbind(*proxy->interface());
        else
            port->vbind(*proxy->port());
    }
    bindingIndex += proxies.size();
}

void
Module::beforeEndOfElaboration()
{
    pushParentModule(this);
    try {
        _sc_mod->before_end_of_elaboration();
        for (auto e : exports)
            e->before_end_of_elaboration();
    } catch (...) {
        popParentModule();
        throw;
    }
    popParentModule();
}

void
Module::endOfElaboration()
{
    if (_deprecatedConstructor && !_ended) {
        std::string msg = gem5::csprintf("module '%s'", name());
        SC_REPORT_WARNING(sc_core::SC_ID_END_MODULE_NOT_CALLED_, msg.c_str());
    }
    pushParentModule(this);
    try {
        _sc_mod->end_of_elaboration();
        for (auto e : exports)
            e->end_of_elaboration();
    } catch (...) {
        popParentModule();
        throw;
    }
    popParentModule();
}

void
Module::startOfSimulation()
{
    pushParentModule(this);
    try {
        _sc_mod->start_of_simulation();
        for (auto e : exports)
            e->start_of_simulation();
    } catch (...) {
        popParentModule();
        throw;
    }
    popParentModule();
}

void
Module::endOfSimulation()
{
    pushParentModule(this);
    try {
        _sc_mod->end_of_simulation();
        for (auto e : exports)
            e->end_of_simulation();
    } catch (...) {
        popParentModule();
        throw;
    }
    popParentModule();
}

Module *
currentModule()
{
    if (_modules.empty())
        return nullptr;
    return _modules.back();
}

Module *
newModuleChecked()
{
    if (!_new_module)
        SC_REPORT_ERROR(sc_core::SC_ID_MODULE_NAME_STACK_EMPTY_, "");
    return _new_module;
}

Module *
newModule()
{
    return _new_module;
}

std::list<Module *> allModules;

} // namespace sc_gem5
