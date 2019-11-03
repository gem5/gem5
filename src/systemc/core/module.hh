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

#ifndef __SYSTEMC_CORE_MODULE_HH__
#define __SYSTEMC_CORE_MODULE_HH__

#include <cassert>
#include <list>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "systemc/core/object.hh"
#include "systemc/ext/core/sc_module.hh"

namespace sc_core
{

class sc_port_base;
class sc_export_base;

} // namespace sc_core

namespace sc_gem5
{

class UniqueNameGen
{
  private:
    std::map<std::string, int> counts;
    std::string buf;

  public:
    const char *
    gen(std::string seed)
    {
        std::ostringstream os;
        os << seed << "_" << counts[seed]++;
        buf = os.str();
        return buf.c_str();
    }
};

extern UniqueNameGen globalNameGen;

class Module
{
  private:
    const char *_name;
    sc_core::sc_module *_sc_mod;
    Object *_obj;
    bool _ended;
    bool _deprecatedConstructor;

    UniqueNameGen nameGen;

  public:
    Module(const char *name);
    ~Module();

    static Module *
    fromScModule(::sc_core::sc_module *mod)
    {
        return mod->_gem5_module;
    }

    void finish(Object *this_obj);

    const char *name() const { return _name; }
    void endModule() { _ended = true; }
    void deprecatedConstructor() { _deprecatedConstructor = true; }

    sc_core::sc_module *
    sc_mod() const
    {
        assert(_sc_mod);
        return _sc_mod;
    }

    void
    sc_mod(sc_core::sc_module *sc_mod)
    {
        assert(!_sc_mod);
        _sc_mod = sc_mod;
    }

    Object *
    obj()
    {
        assert(_obj);
        return _obj;
    }

    void pop();

    const char *uniqueName(const char *seed) { return nameGen.gen(seed); }

    void bindPorts(std::vector<const ::sc_core::sc_bind_proxy *> &proxies);

    std::vector<::sc_core::sc_port_base *> ports;
    std::vector<::sc_core::sc_export_base *> exports;

    int bindingIndex;

    void beforeEndOfElaboration();
    void endOfElaboration();
    void startOfSimulation();
    void endOfSimulation();
};

Module *currentModule();
Module *newModuleChecked();
Module *newModule();

static inline Module *
pickParentModule()
{
    ::sc_core::sc_object *obj = pickParentObj();
    auto mod = dynamic_cast<::sc_core::sc_module *>(obj);
    if (!mod)
        return nullptr;
    return Module::fromScModule(mod);
}
static inline void
pushParentModule(Module *m)
{
    pushParentObj(m->obj()->sc_obj());
}
static inline void
popParentModule()
{
    assert(pickParentModule());
    popParentObj();
}

extern std::list<Module *> allModules;

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_MODULE_HH__
