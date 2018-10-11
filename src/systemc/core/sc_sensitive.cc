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

#include "systemc/core/process.hh"
#include "systemc/ext/channel/sc_in.hh"
#include "systemc/ext/channel/sc_inout.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_sensitive.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

namespace
{

void
checkIfRunning()
{
    if (sc_is_running())
        SC_REPORT_ERROR(SC_ID_MAKE_SENSITIVE_, "simulation running");
}

} // anonymous namespace

sc_sensitive::sc_sensitive() : currentProcess(nullptr) {}

sc_sensitive &
sc_sensitive::operator << (const sc_event &e)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityEvent(currentProcess, &e);
    return *this;
}

sc_sensitive &
sc_sensitive::operator << (const sc_interface &i)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityInterface(currentProcess, &i);
    return *this;
}

sc_sensitive &
sc_sensitive::operator << (const sc_port_base &b)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityPort(currentProcess, &b);
    return *this;
}

sc_sensitive &
sc_sensitive::operator << (sc_event_finder &f)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(currentProcess, &f);
    return *this;
}

sc_sensitive &
sc_sensitive::operator << (::sc_gem5::Process *p)
{
    currentProcess = p;
    return *this;
}


void
sc_sensitive::operator () (::sc_gem5::Process *p,
                           const sc_signal_in_if<bool> &i)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityEvent(p, &i.posedge_event());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p,
                           const sc_signal_in_if<sc_dt::sc_logic> &i)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityEvent(p, &i.posedge_event());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p, const sc_in<bool> &port)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(p, &port.pos());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p,
                           const sc_in<sc_dt::sc_logic> &port)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(p, &port.pos());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p, const sc_inout<bool> &port)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(p, &port.pos());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p,
                           const sc_inout<sc_dt::sc_logic> &port)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(p, &port.pos());
}

void
sc_sensitive::operator () (::sc_gem5::Process *p, sc_event_finder &f)
{
    checkIfRunning();
    sc_gem5::newStaticSensitivityFinder(p, &f);
}

} // namespace sc_core
