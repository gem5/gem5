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
#include "systemc/ext/channel/sc_event_queue.hh"

namespace sc_core
{

sc_event_queue::sc_event_queue(sc_module_name name) :
        sc_interface(), sc_event_queue_if(), sc_module(name)
{}

sc_event_queue::~sc_event_queue() {}

const char *sc_event_queue::kind() const { return "sc_event_queue"; }

void
sc_event_queue::notify(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_event_queue::notify(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_event_queue::cancel_all()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const sc_event &
sc_event_queue::default_event() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_event *)nullptr;
}

} // namespace sc_core
