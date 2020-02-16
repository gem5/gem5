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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_OUT_RESOLVED_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_OUT_RESOLVED_HH__

#include "sc_inout_resolved.hh"
#include "sc_out.hh"
#include "sc_signal_in_if.hh"
#include "sc_signal_inout_if.hh"

namespace sc_dt
{

class sc_logic;

} // namespace sc_dt

namespace sc_core
{

class sc_out_resolved : public sc_inout_resolved
{
  public:
    sc_out_resolved();
    explicit sc_out_resolved(const char *name);
    virtual ~sc_out_resolved();

    sc_out_resolved &operator = (const sc_dt::sc_logic &);
    sc_out_resolved &operator = (const sc_signal_in_if<sc_dt::sc_logic> &);
    sc_out_resolved &operator = (
            const sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &);
    sc_out_resolved &operator = (
            const sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &);
    sc_out_resolved &operator = (const sc_out_resolved &);

    virtual const char *kind() const { return "sc_out_resolved"; }

  private:
    // Disabled
    sc_out_resolved(const sc_out_resolved &) : sc_inout_resolved() {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_OUT_RESOLVED_HH__
