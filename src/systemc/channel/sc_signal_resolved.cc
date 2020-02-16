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

#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_signal_resolved.hh"
#include "systemc/ext/core/sc_module.hh" // for sc_gen_unique_name

namespace sc_core
{

sc_signal_resolved::sc_signal_resolved() : sc_interface(),
        sc_signal<sc_dt::sc_logic, SC_MANY_WRITERS>(
                sc_gen_unique_name("signal_resolved"))
{}

sc_signal_resolved::sc_signal_resolved(const char *name) :
        sc_interface(), sc_signal<sc_dt::sc_logic, SC_MANY_WRITERS>(name)
{}

sc_signal_resolved::~sc_signal_resolved() {}
void sc_signal_resolved::register_port(sc_port_base &, const char *) {}

void
sc_signal_resolved::write(const sc_dt::sc_logic &l)
{
    ::sc_gem5::Process *p = ::sc_gem5::scheduler.current();

    auto it = inputs.find(p);
    if (it == inputs.end()) {
        inputs.emplace(p, l);
        request_update();
    } else if (it->second != l) {
        it->second = l;
        request_update();
    }
}

sc_signal_resolved &
sc_signal_resolved::operator = (const sc_dt::sc_logic &l)
{
    write(l);
    return *this;
}

sc_signal_resolved &
sc_signal_resolved::operator = (const sc_signal_resolved &r)
{
    write(r.read());
    return *this;
}

void
sc_signal_resolved::update()
{
    using sc_dt::Log_0;
    using sc_dt::Log_1;
    using sc_dt::Log_Z;
    using sc_dt::Log_X;
    static sc_dt::sc_logic_value_t merge_table[4][4] = {
        { Log_0, Log_X, Log_0, Log_X },
        { Log_X, Log_1, Log_1, Log_X },
        { Log_0, Log_1, Log_Z, Log_X },
        { Log_X, Log_X, Log_X, Log_X }
    };

    // Resolve the inputs, and give the result to the underlying signal class.
    m_new_val = Log_Z;
    for (auto &input: inputs)
        m_new_val = merge_table[m_new_val.value()][input.second.value()];

    // Ask the signal to update it's value.
    sc_signal<sc_dt::sc_logic, SC_MANY_WRITERS>::update();
}

} // namespace sc_core
