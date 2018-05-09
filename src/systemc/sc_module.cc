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

#include "systemc/sc_module.hh"

#include "base/logging.hh"

namespace sc_core
{

sc_bind_proxy::sc_bind_proxy(const sc_interface &interface)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_bind_proxy::sc_bind_proxy(const sc_port_base &port)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const sc_bind_proxy SC_BIND_PROXY_NUL(*(const sc_port_base *)nullptr);

sc_module::~sc_module()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const char *
sc_module::kind() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

const sc_bind_proxy SC_BIND_PROXY_NIL(*(const sc_port_base *)nullptr);

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
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const std::vector<sc_object *> &
sc_module::get_child_objects() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_object *> *)nullptr;
}

const std::vector<sc_event *> &
sc_module::get_child_events() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_event *> *)nullptr;
}

sc_module::sc_module(const sc_module_name &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_module::sc_module()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_module::async_reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::async_reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::async_reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::async_reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_module::dont_initialize()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::set_stack_size(size_t)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_module::next_trigger()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_time &, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(double, sc_time_unit, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_time &, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(double, sc_time_unit, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(const sc_time &, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::next_trigger(double, sc_time_unit, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_module::wait()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(int)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_time &, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(double, sc_time_unit, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_time &, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(double, sc_time_unit, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(const sc_time &, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_module::wait(double, sc_time_unit, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
next_trigger()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_time &, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(double, sc_time_unit, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_time &, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(double, sc_time_unit, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(const sc_time &, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
next_trigger(double, sc_time_unit, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
wait()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(int)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_time &, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(double, sc_time_unit, const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_time &, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(double, sc_time_unit, const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(const sc_time &, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
wait(double, sc_time_unit, const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const char *
sc_gen_unique_name(const char *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_start_of_simulation_invoked()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_end_of_simulation_invoked()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

} // namespace sc_core
