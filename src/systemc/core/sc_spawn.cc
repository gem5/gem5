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

#include "base/logging.hh"
#include "systemc/core/process.hh"
#include "systemc/core/process_types.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_in.hh"
#include "systemc/ext/channel/sc_inout.hh"
#include "systemc/ext/channel/sc_out.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/messages.hh"
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
            warn_once("Ignoring request to set stack size.\n");
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

        for (auto p: opts->_in_resets)
            newReset(p.target, proc, p.sync, p.value);

        for (auto p: opts->_inout_resets)
            newReset(p.target, proc, p.sync, p.value);

        for (auto p: opts->_out_resets)
            newReset(p.target, proc, p.sync, p.value);

        for (auto i: opts->_if_resets)
            newReset(i.target, proc, i.sync, i.value);
    }

    if (opts && opts->_dontInitialize &&
            opts->_events.empty() && opts->_ports.empty() &&
            opts->_exports.empty() && opts->_interfaces.empty() &&
            opts->_finders.empty()) {
        SC_REPORT_WARNING(sc_core::SC_ID_DISABLE_WILL_ORPHAN_PROCESS_,
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
sc_spawn_options::reset_signal_is(const sc_in<bool> &port, bool value)
{
    _in_resets.emplace_back(&port, value, true);
}

void
sc_spawn_options::reset_signal_is(const sc_inout<bool> &port, bool value)
{
    _inout_resets.emplace_back(&port, value, true);
}

void
sc_spawn_options::reset_signal_is(const sc_out<bool> &port, bool value)
{
    _out_resets.emplace_back(&port, value, true);
}

void
sc_spawn_options::reset_signal_is(
        const sc_signal_in_if<bool> &iface, bool value)
{
    _if_resets.emplace_back(&iface, value, true);
}


void
sc_spawn_options::async_reset_signal_is(const sc_in<bool> &port, bool value)
{
    _in_resets.emplace_back(&port, value, false);
}

void
sc_spawn_options::async_reset_signal_is(const sc_inout<bool> &port, bool value)
{
    _inout_resets.emplace_back(&port, value, false);
}

void
sc_spawn_options::async_reset_signal_is(const sc_out<bool> &port, bool value)
{
    _out_resets.emplace_back(&port, value, false);
}

void
sc_spawn_options::async_reset_signal_is(
        const sc_signal_in_if<bool> &iface, bool value)
{
    _if_resets.emplace_back(&iface, value, false);
}

} // namespace sc_core
