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

#ifndef __SYSTEMC_SC_MODULE_HH__
#define __SYSTEMC_SC_MODULE_HH__

#include <vector>

#include "sc_object.hh"
#include "sc_sensitive.hh"
#include "sc_time.hh"

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
  public:
    sc_bind_proxy(const sc_interface &interface);
    sc_bind_proxy(const sc_port_base &port);
};

extern const sc_bind_proxy SC_BIND_PROXY_NIL;

class sc_module : public sc_object
{
  public:
    virtual ~sc_module();

    virtual const char *kind() const;

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

    virtual const std::vector<sc_object *> &get_child_objects() const;
    virtual const std::vector<sc_event *> &get_child_events() const;

  protected:
    sc_module(const sc_module_name &);
    sc_module();

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

    virtual void before_end_of_elaboration() {}
    virtual void end_of_elaboration() {}
    virtual void start_of_simulation() {}
    virtual void end_of_simulation() {}

  private:
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

#define SC_MODULE(name) struct name : ::sc_core::sc_module

#define SC_CTOR(name) \
    typedef name SC_CURRENT_USER_MODULE; \
    name(::sc_core::sc_module_name)

#define SC_HAS_PROCESS(name) typedef name SC_CURRENT_USER_MODULE

#define SC_METHOD(name) /* Implementation defined */
#define SC_THREAD(name) /* Implementation defined */
#define SC_CTHREAD(name, clk) /* Implementation defined */

const char *sc_gen_unique_name(const char *);

typedef sc_module sc_behavior;
typedef sc_module sc_channel;

bool sc_start_of_simulation_invoked();
bool sc_end_of_simulation_invoked();

} // namespace sc_core

#endif  //__SYSTEMC_SC_MODULE_HH__
