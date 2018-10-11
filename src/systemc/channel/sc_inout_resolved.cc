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

#include "base/cprintf.hh"
#include "systemc/ext/channel/messages.hh"
#include "systemc/ext/channel/sc_inout_resolved.hh"
#include "systemc/ext/channel/sc_signal_resolved.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

sc_inout_resolved::sc_inout_resolved() : sc_inout<sc_dt::sc_logic>() {}

sc_inout_resolved::sc_inout_resolved(const char *name) :
        sc_inout<sc_dt::sc_logic>(name)
{}

sc_inout_resolved::~sc_inout_resolved() {}

void
sc_inout_resolved::end_of_elaboration()
{
    sc_inout<sc_dt::sc_logic>::end_of_elaboration();
    if (!dynamic_cast<sc_signal_resolved *>(get_interface())) {
        std::string msg = csprintf("port '%s' (%s)", name(), kind());
        SC_REPORT_ERROR(SC_ID_RESOLVED_PORT_NOT_BOUND_, msg.c_str());
    }
}

sc_inout_resolved &
sc_inout_resolved::operator = (const sc_dt::sc_logic &l)
{
    (*this)->write(l);
    return *this;
}

sc_inout_resolved &
sc_inout_resolved::operator = (const sc_signal_in_if<sc_dt::sc_logic> &i)
{
    (*this)->write(i.read());
    return *this;
}

sc_inout_resolved &
sc_inout_resolved::operator = (
        const sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &p)
{
    (*this)->write(p->read());
    return *this;
}

sc_inout_resolved &
sc_inout_resolved::operator = (
        const sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &p)
{
    (*this)->write(p->read());
    return *this;
}

sc_inout_resolved &
sc_inout_resolved::operator = (const sc_inout_resolved &p)
{
    (*this)->write(p->read());
    return *this;
}

} // namespace sc_core
