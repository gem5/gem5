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

#include "base/logging.hh"
#include "systemc/core/process.hh"
#include "systemc/core/process_types.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_spawn.hh"

namespace sc_gem5
{

Process *
spawnWork(ProcessFuncWrapper *func, const char *name,
          const ::sc_core::sc_spawn_options *opts)
{
    bool method = false;
    bool dontInitialize = false;
    if (opts) {
        if (opts->_spawnMethod)
            method = true;
        if (opts->_dontInitialize)
            dontInitialize = true;
        if (opts->_stackSize != -1)
            warn("Ignoring request to set stack size.\n");
    }

    if (!name || name[0] == '\0') {
        if (method)
            name = ::sc_core::sc_gen_unique_name("method_p");
        else
            name = ::sc_core::sc_gen_unique_name("thread_p");
    }

    Process *proc;
    if (method)
        proc = new Method(name, func);
    else
        proc = new Thread(name, func);

    proc->dontInitialize(dontInitialize);

    if (opts) {
        for (auto e: opts->_events)
            newStaticSensitivityEvent(proc, e);

        for (auto p: opts->_ports)
            newStaticSensitivityPort(proc, p);

        for (auto e: opts->_exports)
            newStaticSensitivityExport(proc, e);

        for (auto i: opts->_interfaces)
            newStaticSensitivityInterface(proc, i);

        for (auto f: opts->_finders)
            newStaticSensitivityFinder(proc, f);
    }

    if (opts && opts->_dontInitialize &&
            opts->_events.empty() && opts->_ports.empty() &&
            opts->_exports.empty() && opts->_interfaces.empty() &&
            opts->_finders.empty()) {
        SC_REPORT_WARNING(
                "(W558) disable() or dont_initialize() called on process "
                "with no static sensitivity, it will be orphaned",
                proc->name());
    }

    scheduler.reg(proc);

    return proc;
}

} // namespace sc_gem5

namespace sc_core
{

sc_spawn_options::sc_spawn_options() :
    _spawnMethod(false), _dontInitialize(false), _stackSize(-1)
{}


void
sc_spawn_options::spawn_method()
{
    _spawnMethod = true;
}

void
sc_spawn_options::dont_initialize()
{
    _dontInitialize = true;
}

void
sc_spawn_options::set_stack_size(int ss)
{
    _stackSize = ss;
}


void
sc_spawn_options::set_sensitivity(const sc_event *e)
{
    _events.push_back(e);
}

void
sc_spawn_options::set_sensitivity(sc_port_base *p)
{
    _ports.push_back(p);
}

void
sc_spawn_options::set_sensitivity(sc_export_base *e)
{
    _exports.push_back(e);
}

void
sc_spawn_options::set_sensitivity(sc_interface *i)
{
    _interfaces.push_back(i);
}

void
sc_spawn_options::set_sensitivity(sc_event_finder *f)
{
    _finders.push_back(f);
}


void
sc_spawn_options::reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_options::async_reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_warn_unimpl(const char *func)
{
    warn("%s not implemented.\n", func);
}

} // namespace sc_core

namespace sc_unnamed
{

ImplementationDefined _1;
ImplementationDefined _2;
ImplementationDefined _3;
ImplementationDefined _4;
ImplementationDefined _5;
ImplementationDefined _6;
ImplementationDefined _7;
ImplementationDefined _8;
ImplementationDefined _9;

} // namespace sc_unnamed
