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

#include <memory>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "systemc/core/event.hh"
#include "systemc/core/kernel.hh"
#include "systemc/core/module.hh"
#include "systemc/core/object.hh"
#include "systemc/core/port.hh"
#include "systemc/core/process_types.hh"
#include "systemc/core/sensitivity.hh"
#include "systemc/ext/channel/sc_in.hh"
#include "systemc/ext/channel/sc_inout.hh"
#include "systemc/ext/channel/sc_out.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/ext/dt/bit/sc_logic.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

Process *
newMethodProcess(const char *name, ProcessFuncWrapper *func)
{
    Method *p = new Method(name, func);
    if (::sc_core::sc_is_running()) {
        std::string name = p->name();
        delete p;
        SC_REPORT_ERROR(sc_core::SC_ID_MODULE_METHOD_AFTER_START_,
                name.c_str());
        return nullptr;
    }
    scheduler.reg(p);
    return p;
}

Process *
newThreadProcess(const char *name, ProcessFuncWrapper *func)
{
    Thread *p = new Thread(name, func);
    if (::sc_core::sc_is_running()) {
        std::string name = p->name();
        delete p;
        SC_REPORT_ERROR(sc_core::SC_ID_MODULE_THREAD_AFTER_START_,
                name.c_str());
        return nullptr;
    }
    scheduler.reg(p);
    return p;
}

Process *
newCThreadProcess(const char *name, ProcessFuncWrapper *func)
{
    CThread *p = new CThread(name, func);
    if (::sc_core::sc_is_running()) {
        std::string name = p->name();
        delete p;
        SC_REPORT_ERROR(sc_core::SC_ID_MODULE_CTHREAD_AFTER_START_,
                name.c_str());
        return nullptr;
    }
    scheduler.reg(p);
    p->dontInitialize(true);
    return p;
}

} // namespace sc_gem5

namespace sc_core
{

sc_bind_proxy::sc_bind_proxy() : _interface(nullptr), _port(nullptr) {}

sc_bind_proxy::sc_bind_proxy(sc_interface &_interface) :
    _interface(&_interface), _port(nullptr)
{}

sc_bind_proxy::sc_bind_proxy(sc_port_base &_port) :
    _interface(nullptr), _port(&_port)
{}

const sc_bind_proxy SC_BIND_PROXY_NIL;

::Port &
sc_module::gem5_getPort(const std::string &if_name, int idx)
{
    fatal("%s does not have any port named %s\n", name(), if_name);
}

sc_module::~sc_module() { delete _gem5_module; }

void
sc_module::operator () (const sc_bind_proxy &p001,
                        const sc_bind_proxy &p002,
                        const sc_bind_proxy &p003,
                        const sc_bind_proxy &p004,
                        const sc_bind_proxy &p005,
                        const sc_bind_proxy &p006,
                        const sc_bind_proxy &p007,
                        const sc_bind_proxy &p008,
                        const sc_bind_proxy &p009,
                        const sc_bind_proxy &p010,
                        const sc_bind_proxy &p011,
                        const sc_bind_proxy &p012,
                        const sc_bind_proxy &p013,
                        const sc_bind_proxy &p014,
                        const sc_bind_proxy &p015,
                        const sc_bind_proxy &p016,
                        const sc_bind_proxy &p017,
                        const sc_bind_proxy &p018,
                        const sc_bind_proxy &p019,
                        const sc_bind_proxy &p020,
                        const sc_bind_proxy &p021,
                        const sc_bind_proxy &p022,
                        const sc_bind_proxy &p023,
                        const sc_bind_proxy &p024,
                        const sc_bind_proxy &p025,
                        const sc_bind_proxy &p026,
                        const sc_bind_proxy &p027,
                        const sc_bind_proxy &p028,
                        const sc_bind_proxy &p029,
                        const sc_bind_proxy &p030,
                        const sc_bind_proxy &p031,
                        const sc_bind_proxy &p032,
                        const sc_bind_proxy &p033,
                        const sc_bind_proxy &p034,
                        const sc_bind_proxy &p035,
                        const sc_bind_proxy &p036,
                        const sc_bind_proxy &p037,
                        const sc_bind_proxy &p038,
                        const sc_bind_proxy &p039,
                        const sc_bind_proxy &p040,
                        const sc_bind_proxy &p041,
                        const sc_bind_proxy &p042,
                        const sc_bind_proxy &p043,
                        const sc_bind_proxy &p044,
                        const sc_bind_proxy &p045,
                        const sc_bind_proxy &p046,
                        const sc_bind_proxy &p047,
                        const sc_bind_proxy &p048,
                        const sc_bind_proxy &p049,
                        const sc_bind_proxy &p050,
                        const sc_bind_proxy &p051,
                        const sc_bind_proxy &p052,
                        const sc_bind_proxy &p053,
                        const sc_bind_proxy &p054,
                        const sc_bind_proxy &p055,
                        const sc_bind_proxy &p056,
                        const sc_bind_proxy &p057,
                        const sc_bind_proxy &p058,
                        const sc_bind_proxy &p059,
                        const sc_bind_proxy &p060,
                        const sc_bind_proxy &p061,
                        const sc_bind_proxy &p062,
                        const sc_bind_proxy &p063,
                        const sc_bind_proxy &p064)
{
    std::vector<const ::sc_core::sc_bind_proxy *> proxies;
    auto insert = [&proxies](const ::sc_core::sc_bind_proxy &p) -> bool {
        if (!p.port() && !p.interface())
            return false;
        proxies.push_back(&p);
        return true;
    };
    insert(p001) && insert(p002) && insert(p003) && insert(p004) &&
    insert(p005) && insert(p006) && insert(p007) && insert(p008) &&
    insert(p009) && insert(p010) && insert(p011) && insert(p012) &&
    insert(p013) && insert(p014) && insert(p015) && insert(p016) &&
    insert(p017) && insert(p018) && insert(p019) && insert(p020) &&
    insert(p021) && insert(p022) && insert(p023) && insert(p024) &&
    insert(p025) && insert(p026) && insert(p027) && insert(p028) &&
    insert(p029) && insert(p030) && insert(p031) && insert(p032) &&
    insert(p033) && insert(p034) && insert(p035) && insert(p036) &&
    insert(p037) && insert(p038) && insert(p039) && insert(p040) &&
    insert(p041) && insert(p042) && insert(p043) && insert(p044) &&
    insert(p045) && insert(p046) && insert(p047) && insert(p048) &&
    insert(p049) && insert(p050) && insert(p051) && insert(p052) &&
    insert(p053) && insert(p054) && insert(p055) && insert(p056) &&
    insert(p057) && insert(p058) && insert(p059) && insert(p060) &&
    insert(p061) && insert(p062) && insert(p063) && insert(p064);
    _gem5_module->bindPorts(proxies);
}

sc_module &
sc_module::operator << (sc_interface &iface)
{
    (*this)(iface);
    return *this;
}

sc_module &
sc_module::operator << (sc_port_base &pb)
{
    (*this)(pb);
    return *this;
}

sc_module &
sc_module::operator , (sc_interface &iface)
{
    (*this)(iface);
    return *this;
}

sc_module &
sc_module::operator , (sc_port_base &pb)
{
    (*this)(pb);
    return *this;
}

const std::vector<sc_object *> &
sc_module::get_child_objects() const
{
    return _gem5_module->obj()->get_child_objects();
}

const std::vector<sc_event *> &
sc_module::get_child_events() const
{
    return _gem5_module->obj()->get_child_events();
}

sc_module::sc_module() :
    sc_object(sc_gem5::newModuleChecked()->name()),
    _gem5_module(sc_gem5::currentModule())
{
    if (sc_is_running())
        SC_REPORT_ERROR(SC_ID_INSERT_MODULE_, "simulation running");
    if (::sc_gem5::scheduler.elaborationDone())
        SC_REPORT_ERROR(SC_ID_INSERT_MODULE_, "elaboration done");
}

sc_module::sc_module(const sc_module_name &) : sc_module() {}
sc_module::sc_module(const char *_name) : sc_module(sc_module_name(_name))
{
    _gem5_module->deprecatedConstructor();
    SC_REPORT_WARNING(SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, _name);
}
sc_module::sc_module(const std::string &_name) :
    sc_module(sc_module_name(_name.c_str()))
{
    _gem5_module->deprecatedConstructor();
    SC_REPORT_WARNING(SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, _name.c_str());
}

void
sc_module::end_module()
{
    _gem5_module->endModule();
}

void
sc_module::reset_signal_is(const sc_in<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), true, val);
}

void
sc_module::reset_signal_is(const sc_inout<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), true, val);
}

void
sc_module::reset_signal_is(const sc_out<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), true, val);
}

void
sc_module::reset_signal_is(const sc_signal_in_if<bool> &signal, bool val)
{
    ::sc_gem5::newReset(&signal, ::sc_gem5::Process::newest(), true, val);
}


void
sc_module::async_reset_signal_is(const sc_in<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), false, val);
}

void
sc_module::async_reset_signal_is(const sc_inout<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), false, val);
}

void
sc_module::async_reset_signal_is(const sc_out<bool> &port, bool val)
{
    ::sc_gem5::newReset(&port, ::sc_gem5::Process::newest(), false, val);
}

void
sc_module::async_reset_signal_is(const sc_signal_in_if<bool> &signal, bool val)
{
    ::sc_gem5::newReset(&signal, ::sc_gem5::Process::newest(), false, val);
}


void
sc_module::dont_initialize()
{
    ::sc_gem5::Process *p = ::sc_gem5::Process::newest();
    if (p->procKind() == SC_CTHREAD_PROC_)
        SC_REPORT_WARNING(SC_ID_DONT_INITIALIZE_, "");
    p->dontInitialize(true);
}

void
sc_module::set_stack_size(size_t size)
{
    ::sc_gem5::Process::newest()->setStackSize(size);
}


void sc_module::next_trigger() { ::sc_core::next_trigger(); }

void
sc_module::next_trigger(const sc_event &e)
{
    ::sc_core::next_trigger(e);
}

void
sc_module::next_trigger(const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(eol);
}

void
sc_module::next_trigger(const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(eal);
}

void
sc_module::next_trigger(const sc_time &t)
{
    ::sc_core::next_trigger(t);
}

void
sc_module::next_trigger(double d, sc_time_unit u)
{
    ::sc_core::next_trigger(d, u);
}

void
sc_module::next_trigger(const sc_time &t, const sc_event &e)
{
    ::sc_core::next_trigger(t, e);
}

void
sc_module::next_trigger(double d, sc_time_unit u, const sc_event &e)
{
    ::sc_core::next_trigger(d, u, e);
}

void
sc_module::next_trigger(const sc_time &t, const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(t, eol);
}

void
sc_module::next_trigger(double d, sc_time_unit u, const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(d, u, eol);
}

void
sc_module::next_trigger(const sc_time &t, const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(t, eal);
}

void
sc_module::next_trigger(double d, sc_time_unit u, const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(d, u, eal);
}


bool
sc_module::timed_out()
{
    return ::sc_core::timed_out();
}


void
sc_module::wait()
{
    ::sc_core::wait();
}

void
sc_module::wait(int i)
{
    ::sc_core::wait(i);
}

void
sc_module::wait(const sc_event &e)
{
    ::sc_core::wait(e);
}

void
sc_module::wait(const sc_event_or_list &eol)
{
    ::sc_core::wait(eol);
}

void
sc_module::wait(const sc_event_and_list &eal)
{
    ::sc_core::wait(eal);
}

void
sc_module::wait(const sc_time &t)
{
    ::sc_core::wait(t);
}

void
sc_module::wait(double d, sc_time_unit u)
{
    ::sc_core::wait(d, u);
}

void
sc_module::wait(const sc_time &t, const sc_event &e)
{
    ::sc_core::wait(t, e);
}

void
sc_module::wait(double d, sc_time_unit u, const sc_event &e)
{
    ::sc_core::wait(d, u, e);
}

void
sc_module::wait(const sc_time &t, const sc_event_or_list &eol)
{
    ::sc_core::wait(t, eol);
}

void
sc_module::wait(double d, sc_time_unit u, const sc_event_or_list &eol)
{
    ::sc_core::wait(d, u, eol);
}

void
sc_module::wait(const sc_time &t, const sc_event_and_list &eal)
{
    ::sc_core::wait(t, eal);
}

void
sc_module::wait(double d, sc_time_unit u, const sc_event_and_list &eal)
{
    ::sc_core::wait(d, u, eal);
}


void
sc_module::halt()
{
    ::sc_core::halt();
}

void
sc_module::at_posedge(const sc_signal_in_if<bool> &s)
{
    ::sc_core::at_posedge(s);
}

void
sc_module::at_posedge(const sc_signal_in_if<sc_dt::sc_logic> &s)
{
    ::sc_core::at_posedge(s);
}

void
sc_module::at_negedge(const sc_signal_in_if<bool> &s)
{
    ::sc_core::at_negedge(s);
}

void
sc_module::at_negedge(const sc_signal_in_if<sc_dt::sc_logic> &s)
{
    ::sc_core::at_negedge(s);
}


void
next_trigger()
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->cancelTimeout();
    p->clearDynamic();
}

void
next_trigger(const sc_event &e)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEvent(p, &e);
}

void
next_trigger(const sc_event_or_list &eol)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEventOrList(p, &eol);
}

void
next_trigger(const sc_event_and_list &eal)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEventAndList(p, &eal);
}

void
next_trigger(const sc_time &t)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->setTimeout(t);
    p->clearDynamic();
}

void
next_trigger(double d, sc_time_unit u)
{
    next_trigger(sc_time(d, u));
}

void
next_trigger(const sc_time &t, const sc_event &e)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEvent(p, &e);
}

void
next_trigger(double d, sc_time_unit u, const sc_event &e)
{
    next_trigger(sc_time(d, u), e);
}

void
next_trigger(const sc_time &t, const sc_event_or_list &eol)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEventOrList(p, &eol);
}

void
next_trigger(double d, sc_time_unit u, const sc_event_or_list &eol)
{
    next_trigger(sc_time(d, u), eol);
}

void
next_trigger(const sc_time &t, const sc_event_and_list &eal)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEventAndList(p, &eal);
}

void
next_trigger(double d, sc_time_unit u, const sc_event_and_list &eal)
{
    next_trigger(sc_time(d, u), eal);
}

bool
timed_out()
{
    ::sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (!p)
        return false;
    else
        return p->timedOut();
}


namespace
{

bool
waitErrorCheck(sc_gem5::Process *p)
{
    if (p->procKind() == SC_METHOD_PROC_) {
        SC_REPORT_ERROR(SC_ID_WAIT_NOT_ALLOWED_,
                "\n        in SC_METHODs use next_trigger() instead");
        return true;
    }
    return false;
}

} // anonymous namespace

void
wait()
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->cancelTimeout();
    p->clearDynamic();
    sc_gem5::scheduler.yield();
}

void
wait(int n)
{
    if (n <= 0) {
        std::string msg = csprintf("n = %d", n);
        SC_REPORT_ERROR(SC_ID_WAIT_N_INVALID_, msg.c_str());
    }
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    p->waitCount(n - 1);
    wait();
}

void
wait(const sc_event &e)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEvent(p, &e);
    sc_gem5::scheduler.yield();
}

void
wait(const sc_event_or_list &eol)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEventOrList(p, &eol);
    sc_gem5::scheduler.yield();
}

void
wait(const sc_event_and_list &eal)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->cancelTimeout();
    ::sc_gem5::newDynamicSensitivityEventAndList(p, &eal);
    sc_gem5::scheduler.yield();
}

void
wait(const sc_time &t)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->setTimeout(t);
    p->clearDynamic();
    sc_gem5::scheduler.yield();
}

void
wait(double d, sc_time_unit u)
{
    wait(sc_time(d, u));
}

void
wait(const sc_time &t, const sc_event &e)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEvent(p, &e);
    sc_gem5::scheduler.yield();
}

void
wait(double d, sc_time_unit u, const sc_event &e)
{
    wait(sc_time(d, u), e);
}

void
wait(const sc_time &t, const sc_event_or_list &eol)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEventOrList(p, &eol);
    sc_gem5::scheduler.yield();
}

void
wait(double d, sc_time_unit u, const sc_event_or_list &eol)
{
    wait(sc_time(d, u), eol);
}

void
wait(const sc_time &t, const sc_event_and_list &eal)
{
    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (waitErrorCheck(p))
        return;
    p->setTimeout(t);
    ::sc_gem5::newDynamicSensitivityEventAndList(p, &eal);
    sc_gem5::scheduler.yield();
}

void
wait(double d, sc_time_unit u, const sc_event_and_list &eal)
{
    wait(sc_time(d, u), eal);
}

void
halt()
{
    ::sc_core::wait();
    throw ::sc_gem5::ScHalt();
}

void
at_posedge(const sc_signal_in_if<bool> &s)
{
    while (s.read())
        wait();
    while (!s.read())
        wait();
}

void
at_posedge(const sc_signal_in_if<sc_dt::sc_logic> &s)
{
    while (s.read() == sc_dt::Log_1)
        wait();
    while (s.read() == sc_dt::Log_0)
        wait();
}

void
at_negedge(const sc_signal_in_if<bool> &s)
{
    while (!s.read())
        wait();
    while (s.read())
        wait();
}

void
at_negedge(const sc_signal_in_if<sc_dt::sc_logic> &s)
{
    while (s.read() == sc_dt::Log_0)
        wait();
    while (s.read() == sc_dt::Log_1)
        wait();
}

const char *
sc_gen_unique_name(const char *seed)
{
    if (!seed || seed[0] == '\0') {
        SC_REPORT_ERROR(SC_ID_GEN_UNIQUE_NAME_, "");
        seed = "unnamed";
    }

    auto mod = sc_gem5::pickParentModule();
    if (mod)
        return mod->uniqueName(seed);

    sc_gem5::Process *p = sc_gem5::scheduler.current();
    if (p)
        return p->uniqueName(seed);

    return ::sc_gem5::globalNameGen.gen(seed);
}

bool
sc_hierarchical_name_exists(const char *name)
{
    return sc_gem5::findEvent(name) != sc_gem5::allEvents.end() ||
        ::sc_gem5::findObject(name, sc_gem5::allObjects);
}

bool
sc_start_of_simulation_invoked()
{
    return ::sc_gem5::kernel->startOfSimulationComplete();
}

bool
sc_end_of_simulation_invoked()
{
    return ::sc_gem5::kernel->endOfSimulationComplete();
}

sc_module *
sc_module_sc_new(sc_module *mod)
{
    static std::vector<std::unique_ptr<sc_module> > modules;
    modules.emplace_back(mod);
    return mod;
}

} // namespace sc_core
