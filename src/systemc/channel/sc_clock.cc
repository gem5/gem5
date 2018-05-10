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
#include "systemc/ext/channel/sc_clock.hh"
#include "systemc/ext/core/sc_module.hh" // for sc_gen_unique_name

namespace sc_core
{

sc_clock::sc_clock() :
        sc_interface(), sc_signal<bool>(sc_gen_unique_name("clock"))
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_clock::sc_clock(const char *name) : sc_interface(), sc_signal<bool>(name)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_clock::sc_clock(const char *name, const sc_time &period,
                   double duty_cycle, const sc_time &start_time,
                   bool posedge_first)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_clock::sc_clock(const char *name, double period_v, sc_time_unit period_tu,
                   double duty_cycle)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_clock::sc_clock(const char *name, double period_v, sc_time_unit period_tu,
                   double duty_cycle, double start_time_v,
                   sc_time_unit start_time_tu, bool posedge_first)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_clock::~sc_clock() {}

void
sc_clock::write(const bool &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const sc_time &
sc_clock::period() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_time *)nullptr;
}

double
sc_clock::duty_cycle() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0.0;
}

const sc_time &
sc_clock::start_time() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_time *)nullptr;
}

bool
sc_clock::posedge_first() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

const char *sc_clock::kind() const { return "sc_clock"; }

void sc_clock::before_end_of_elaboration() {}

} // namespace sc_core
