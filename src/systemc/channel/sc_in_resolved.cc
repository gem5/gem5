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

#include "base/cprintf.hh"
#include "systemc/ext/channel/sc_in_resolved.hh"
#include "systemc/ext/channel/sc_signal_resolved.hh"

namespace sc_core
{

sc_in_resolved::sc_in_resolved() : sc_in<sc_dt::sc_logic>() {}

sc_in_resolved::sc_in_resolved(const char *name) : sc_in<sc_dt::sc_logic>(name)
{}

sc_in_resolved::~sc_in_resolved() {}

void
sc_in_resolved::end_of_elaboration()
{
    sc_in<sc_dt::sc_logic>::end_of_elaboration();
    if (!dynamic_cast<sc_signal_resolved *>(get_interface())) {
        std::string msg = gem5::csprintf("port '%s' (%s)", name(), kind());
        SC_REPORT_ERROR("(E117) resolved port not bound to resolved signal",
                        msg.c_str());
    }
}

} // namespace sc_core
