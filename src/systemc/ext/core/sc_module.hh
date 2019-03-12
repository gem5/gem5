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

#ifndef __SYSTEMC_CORE_EXT_SC_MODULE_HH__
#define __SYSTEMC_CORE_EXT_SC_MODULE_HH__

#include <string>
#include <vector>

#include "sc_object.hh"
#include "sc_process_handle.hh"
#include "sc_sensitive.hh"
#include "sc_time.hh"

namespace sc_dt
{

class sc_logic;

} // namespace sc_dt

namespace sc_gem5
{

class Kernel;
class Module;
class Process;
struct ProcessFuncWrapper;

Process *newMethodProcess(const char *name, ProcessFuncWrapper *func);
Process *newThreadProcess(const char *name, ProcessFuncWrapper *func);
Process *newCThreadProcess(const char *name, ProcessFuncWrapper *func);

} // namespace sc_gem5

// Gem5 prototype
class Port;

namespace sc_core
{

template <class T>
class sc_in;
template <class T>
class sc_out;
template <class T>
class sc_inout;
template <class T>
class sc_signal_in_if;

class sc_event;
class sc_event_and_list;
class sc_event_or_list;
class sc_module_name;

class sc_bind_proxy
{
  private:
    sc_interface *_interface;
    sc_port_base *_port;

  public:
    sc_bind_proxy();
    sc_bind_proxy(sc_interface &_interface);
    sc_bind_proxy(sc_port_base &_port);

    sc_interface *interface() const { return _interface; }
    sc_port_base *port() const { return _port; }
};

extern const sc_bind_proxy SC_BIND_PROXY_NIL;

class sc_module : public sc_object
{
  public:
    // Gem5 specific extensions
    virtual ::Port &gem5_getPort(const std::string &if_name, int idx=-1);

  public:
    friend class ::sc_gem5::Kernel;
    friend class ::sc_gem5::Module;

    virtual ~sc_module();

    virtual const char *kind() const { return "sc_module"; }

    void operator () (const sc_bind_proxy &p001,
                      const sc_bind_proxy &p002 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p003 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p004 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p005 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p006 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p007 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p008 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p009 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p010 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p011 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p012 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p013 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p014 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p015 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p016 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p017 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p018 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p019 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p020 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p021 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p022 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p023 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p024 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p025 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p026 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p027 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p028 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p029 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p030 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p031 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p032 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p033 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p034 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p035 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p036 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p037 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p038 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p039 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p040 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p041 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p042 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p043 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p044 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p045 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p046 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p047 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p048 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p049 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p050 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p051 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p052 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p053 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p054 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p055 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p056 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p057 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p058 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p059 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p060 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p061 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p062 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p063 = SC_BIND_PROXY_NIL,
                      const sc_bind_proxy &p064 = SC_BIND_PROXY_NIL);

    // Deprecated
    sc_module &operator << (sc_interface &);
    sc_module &operator << (sc_port_base &);
    sc_module &operator , (sc_interface &);
    sc_module &operator , (sc_port_base &);

    virtual const std::vector<sc_object *> &get_child_objects() const;
    virtual const std::vector<sc_event *> &get_child_events() const;

  protected:
    sc_module(const sc_module_name &);
    sc_module();

    // Deprecated
    sc_module(const char *);
    sc_module(const std::string &);

    /* Deprecated, but used in the regression tests. */
    void end_module();

    void reset_signal_is(const sc_in<bool> &, bool);
    void reset_signal_is(const sc_inout<bool> &, bool);
    void reset_signal_is(const sc_out<bool> &, bool);
    void reset_signal_is(const sc_signal_in_if<bool> &, bool);

    void async_reset_signal_is(const sc_in<bool> &, bool);
    void async_reset_signal_is(const sc_inout<bool> &, bool);
    void async_reset_signal_is(const sc_out<bool> &, bool);
    void async_reset_signal_is(const sc_signal_in_if<bool> &, bool);

    sc_sensitive sensitive;

    void dont_initialize();
    void set_stack_size(size_t);

    void next_trigger();
    void next_trigger(const sc_event &);
    void next_trigger(const sc_event_or_list &);
    void next_trigger(const sc_event_and_list &);
    void next_trigger(const sc_time &);
    void next_trigger(double, sc_time_unit);
    void next_trigger(const sc_time &, const sc_event &);
    void next_trigger(double, sc_time_unit, const sc_event &);
    void next_trigger(const sc_time &, const sc_event_or_list &);
    void next_trigger(double, sc_time_unit, const sc_event_or_list &);
    void next_trigger(const sc_time &, const sc_event_and_list &);
    void next_trigger(double, sc_time_unit, const sc_event_and_list &);

    // Nonstandard
    bool timed_out();

    void wait();
    void wait(int);
    void wait(const sc_event &);
    void wait(const sc_event_or_list &);
    void wait(const sc_event_and_list &);
    void wait(const sc_time &);
    void wait(double, sc_time_unit);
    void wait(const sc_time &, const sc_event &);
    void wait(double, sc_time_unit, const sc_event &);
    void wait(const sc_time &, const sc_event_or_list &);
    void wait(double, sc_time_unit, const sc_event_or_list &);
    void wait(const sc_time &, const sc_event_and_list &);
    void wait(double, sc_time_unit, const sc_event_and_list &);

    // Nonstandard
    void halt();
    void at_posedge(const sc_signal_in_if<bool> &);
    void at_posedge(const sc_signal_in_if<sc_dt::sc_logic> &);
    void at_negedge(const sc_signal_in_if<bool> &);
    void at_negedge(const sc_signal_in_if<sc_dt::sc_logic> &);

    virtual void before_end_of_elaboration() {}
    virtual void end_of_elaboration() {}
    virtual void start_of_simulation() {}
    virtual void end_of_simulation() {}

  private:
    sc_gem5::Module *_gem5_module;

    // Disabled
    sc_module(const sc_module &) : sc_object() {};
    sc_module &operator = (const sc_module &) { return *this; }
};

void next_trigger();
void next_trigger(const sc_event &);
void next_trigger(const sc_event_or_list &);
void next_trigger(const sc_event_and_list &);
void next_trigger(const sc_time &);
void next_trigger(double, sc_time_unit);
void next_trigger(const sc_time &, const sc_event &);
void next_trigger(double, sc_time_unit, const sc_event &);
void next_trigger(const sc_time &, const sc_event_or_list &);
void next_trigger(double, sc_time_unit, const sc_event_or_list &);
void next_trigger(const sc_time &, const sc_event_and_list &);
void next_trigger(double, sc_time_unit, const sc_event_and_list &);

void wait();
void wait(int);
void wait(const sc_event &);
void wait(const sc_event_or_list &);
void wait(const sc_event_and_list &);
void wait(const sc_time &);
void wait(double, sc_time_unit);
void wait(const sc_time &, const sc_event &);
void wait(double, sc_time_unit, const sc_event &);
void wait(const sc_time &, const sc_event_or_list &);
void wait(double, sc_time_unit, const sc_event_or_list &);
void wait(const sc_time &, const sc_event_and_list &);
void wait(double, sc_time_unit, const sc_event_and_list &);

// Nonstandard
bool timed_out();

#define SC_MODULE(name) struct name : ::sc_core::sc_module

#define SC_CTOR(name) \
    typedef name SC_CURRENT_USER_MODULE; \
    name(::sc_core::sc_module_name)

#define SC_HAS_PROCESS(name) typedef name SC_CURRENT_USER_MODULE

#define SC_METHOD(name) \
    { \
        ::sc_gem5::Process *p = \
            ::sc_gem5::newMethodProcess( \
                #name, new ::sc_gem5::ProcessMemberFuncWrapper< \
                    SC_CURRENT_USER_MODULE>(this, \
                        &SC_CURRENT_USER_MODULE::name)); \
        if (p) \
            this->sensitive << p; \
    }
#define SC_THREAD(name) \
    { \
        ::sc_gem5::Process *p = \
            ::sc_gem5::newThreadProcess( \
                #name, new ::sc_gem5::ProcessMemberFuncWrapper< \
                    SC_CURRENT_USER_MODULE>(this, \
                        &SC_CURRENT_USER_MODULE::name)); \
        if (p) \
            this->sensitive << p; \
    }
#define SC_CTHREAD(name, clk) \
    { \
        ::sc_gem5::Process *p = \
            ::sc_gem5::newCThreadProcess( \
                #name, new ::sc_gem5::ProcessMemberFuncWrapper< \
                    SC_CURRENT_USER_MODULE>(this, \
                        &SC_CURRENT_USER_MODULE::name)); \
        if (p) \
            this->sensitive(p, clk); \
    }

// Nonstandard
// Documentation for this is very scarce, but it looks like it's supposed to
// stop the currently executing cthread, or if a cthread isn't running report
// an error.
void halt();
void at_posedge(const sc_signal_in_if<bool> &);
void at_posedge(const sc_signal_in_if<sc_dt::sc_logic> &);
void at_negedge(const sc_signal_in_if<bool> &);
void at_negedge(const sc_signal_in_if<sc_dt::sc_logic> &);

const char *sc_gen_unique_name(const char *);

// Nonstandard
bool sc_hierarchical_name_exists(const char *name);

typedef sc_module sc_behavior;
typedef sc_module sc_channel;

bool sc_start_of_simulation_invoked();
bool sc_end_of_simulation_invoked();

// Nonstandard
// Allocates a module of type x and records a pointer to it so that it gets
// destructed automatically at the end of the simulation.
sc_module *sc_module_sc_new(sc_module *);
#define SC_NEW(x) ::sc_core::sc_module_sc_new(new x);

// Nonstandard
#define SC_WAIT() \
    ::sc_core::sc_set_location(__FILE__, __LINE__); \
    ::sc_core::wait(); \
    ::sc_core::sc_set_location(NULL, 0)

// Nonstandard
#define SC_WAITN(n) \
    ::sc_core::sc_set_location(__FILE__, __LINE__); \
    ::sc_core::wait(n); \
    ::sc_core::sc_set_location(NULL, 0)

// Nonstandard
#define SC_WAIT_UNTIL(expr) \
    do { SC_WAIT(); } while (!(expr))

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_MODULE_HH__
