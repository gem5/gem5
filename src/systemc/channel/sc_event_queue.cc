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

#include "systemc/ext/channel/sc_event_queue.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_time.hh"

namespace sc_core
{

sc_event_queue::sc_event_queue(sc_module_name name) :
        sc_interface(), sc_event_queue_if(), sc_module(name)
{
    SC_METHOD(_trigger);
    dont_initialize();
    sensitive << _defaultEvent;
}

sc_event_queue::~sc_event_queue() {}

void
sc_event_queue::notify(double d, sc_time_unit tu)
{
    notify(sc_time(d, tu));
}

void
sc_event_queue::notify(const sc_time &t)
{
    _times.push(sc_time_stamp() + t);
    _defaultEvent.notify(_times.top() - sc_time_stamp());
}

void
sc_event_queue::cancel_all()
{
    _defaultEvent.cancel();
    _times = std::priority_queue<
        sc_time, std::vector<sc_time>, std::greater<sc_time> >();
}

const sc_event &
sc_event_queue::default_event() const
{
    return _defaultEvent;
}

void
sc_event_queue::_trigger()
{
    _times.pop();
    if (!_times.empty())
        _defaultEvent.notify(_times.top() - sc_time_stamp());
}

} // namespace sc_core
