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
 *
 * Authors: Gabe Black
 */

#include "systemc/core/module.hh"

#include <cassert>

#include "base/logging.hh"
#include "systemc/ext/core/sc_port.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

namespace
{

std::list<Module *> _modules;
Module *_new_module;

Module *_callbackModule = nullptr;

} // anonymous namespace

Module::Module(const char *name) : _name(name), _sc_mod(nullptr), _obj(nullptr)
{
    panic_if(_new_module, "Previous module not finished.\n");
    _new_module = this;
}

Module::~Module()
{
    if (_new_module == this) {
        // Aborted module construction?
        _new_module = nullptr;
    }
    allModules.remove(this);
}

void
Module::finish(Object *this_obj)
{
    assert(!_obj);
    _obj = this_obj;
    _modules.push_back(this);
    _new_module = nullptr;
    // This is called from the constructor of this_obj, so it can't use
    // dynamic cast.
    sc_mod(static_cast<::sc_core::sc_module *>(this_obj->sc_obj()));
    allModules.emplace_back(this);
}

void
Module::pop()
{
    panic_if(!_modules.size(), "Popping from empty module list.\n");
    panic_if(_modules.back() != this,
            "Popping module which isn't at the end of the module list.\n");
    panic_if(_new_module, "Pop with unfinished module.\n");
    _modules.pop_back();
}

void
Module::bindPorts(std::vector<const ::sc_core::sc_bind_proxy *> &proxies)
{
    panic_if(proxies.size() > ports.size(),
            "Trying to bind %d interfaces/ports to %d ports.\n",
            proxies.size(), ports.size());

    auto proxyIt = proxies.begin();
    auto portIt = ports.begin();
    for (; proxyIt != proxies.end(); proxyIt++, portIt++) {
        auto proxy = *proxyIt;
        auto port = *portIt;
        if (proxy->interface())
            port->vbind(*proxy->interface());
        else
            port->vbind(*proxy->port());
    }
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
    if (!_new_module) {
        SC_REPORT_ERROR("(E533) module name stack is empty: "
                "did you forget to add a sc_module_name parameter to "
                "your module constructor?", nullptr);
    }
    return _new_module;
}

Module *
newModule()
{
    return _new_module;
}

void callbackModule(Module *m) { _callbackModule = m; }
Module *callbackModule() { return _callbackModule; }

std::list<Module *> allModules;

} // namespace sc_gem5
