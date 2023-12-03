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

#include "systemc/core/channel.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_prim.hh"

namespace sc_gem5
{

uint64_t
getChangeStamp()
{
    return scheduler.changeStamp();
}

} // namespace sc_gem5

namespace sc_core
{

sc_prim_channel::sc_prim_channel() : _gem5_channel(nullptr)
{
    if (sc_is_running()) {
        SC_REPORT_ERROR(SC_ID_INSERT_PRIM_CHANNEL_, "simulation running");
    }
    if (::sc_gem5::scheduler.elaborationDone()) {
        SC_REPORT_ERROR(SC_ID_INSERT_PRIM_CHANNEL_, "elaboration done");
    }
    _gem5_channel = new sc_gem5::Channel(this);
}

sc_prim_channel::sc_prim_channel(const char *_name)
    : sc_object(_name), _gem5_channel(nullptr)
{
    if (sc_is_running()) {
        SC_REPORT_ERROR(SC_ID_INSERT_PRIM_CHANNEL_, "simulation running");
    }
    if (::sc_gem5::scheduler.elaborationDone()) {
        SC_REPORT_ERROR(SC_ID_INSERT_PRIM_CHANNEL_, "elaboration done");
    }
    _gem5_channel = new sc_gem5::Channel(this);
}

sc_prim_channel::~sc_prim_channel() { delete _gem5_channel; }

void
sc_prim_channel::request_update()
{
    _gem5_channel->requestUpdate();
}

void
sc_prim_channel::async_request_update()
{
    _gem5_channel->asyncRequestUpdate();
}

void
sc_prim_channel::next_trigger()
{
    ::sc_core::next_trigger();
}

void
sc_prim_channel::next_trigger(const sc_event &e)
{
    ::sc_core::next_trigger(e);
}

void
sc_prim_channel::next_trigger(const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(eol);
}

void
sc_prim_channel::next_trigger(const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(eal);
}

void
sc_prim_channel::next_trigger(const sc_time &t)
{
    ::sc_core::next_trigger(t);
}

void
sc_prim_channel::next_trigger(double d, sc_time_unit u)
{
    ::sc_core::next_trigger(d, u);
}

void
sc_prim_channel::next_trigger(const sc_time &t, const sc_event &e)
{
    ::sc_core::next_trigger(t, e);
}

void
sc_prim_channel::next_trigger(double d, sc_time_unit u, const sc_event &e)
{
    ::sc_core::next_trigger(d, u, e);
}

void
sc_prim_channel::next_trigger(const sc_time &t, const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(t, eol);
}

void
sc_prim_channel::next_trigger(double d, sc_time_unit u,
                              const sc_event_or_list &eol)
{
    ::sc_core::next_trigger(d, u, eol);
}

void
sc_prim_channel::next_trigger(const sc_time &t, const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(t, eal);
}

void
sc_prim_channel::next_trigger(double d, sc_time_unit u,
                              const sc_event_and_list &eal)
{
    ::sc_core::next_trigger(d, u, eal);
}

bool
sc_prim_channel::timed_out()
{
    return ::sc_core::timed_out();
}

void
sc_prim_channel::wait()
{
    ::sc_core::wait();
}

void
sc_prim_channel::wait(int i)
{
    ::sc_core::wait(i);
}

void
sc_prim_channel::wait(const sc_event &e)
{
    ::sc_core::wait(e);
}

void
sc_prim_channel::wait(const sc_event_or_list &eol)
{
    ::sc_core::wait(eol);
}

void
sc_prim_channel::wait(const sc_event_and_list &eal)
{
    ::sc_core::wait(eal);
}

void
sc_prim_channel::wait(const sc_time &t)
{
    ::sc_core::wait(t);
}

void
sc_prim_channel::wait(double d, sc_time_unit u)
{
    ::sc_core::wait(d, u);
}

void
sc_prim_channel::wait(const sc_time &t, const sc_event &e)
{
    ::sc_core::wait(t, e);
}

void
sc_prim_channel::wait(double d, sc_time_unit u, const sc_event &e)
{
    ::sc_core::wait(d, u, e);
}

void
sc_prim_channel::wait(const sc_time &t, const sc_event_or_list &eol)
{
    ::sc_core::wait(t, eol);
}

void
sc_prim_channel::wait(double d, sc_time_unit u, const sc_event_or_list &eol)
{
    ::sc_core::wait(d, u, eol);
}

void
sc_prim_channel::wait(const sc_time &t, const sc_event_and_list &eal)
{
    ::sc_core::wait(t, eal);
}

void
sc_prim_channel::wait(double d, sc_time_unit u, const sc_event_and_list &eal)
{
    ::sc_core::wait(d, u, eal);
}

} // namespace sc_core
