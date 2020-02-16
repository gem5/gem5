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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RESOLVED_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RESOLVED_HH__

#include <map>

#include "sc_signal.hh"
#include "sc_signal_inout_if.hh"

namespace sc_dt
{

class sc_logic;

};

namespace sc_gem5
{

class Process;

} // namespace sc_gem5

namespace sc_core
{

class sc_port_base;

class sc_signal_resolved : public sc_signal<sc_dt::sc_logic, SC_MANY_WRITERS>
{
  public:
    sc_signal_resolved();
    explicit sc_signal_resolved(const char *name);
    virtual ~sc_signal_resolved();

    virtual void register_port(sc_port_base &, const char *);

    virtual void write(const sc_dt::sc_logic &);
    sc_signal_resolved &operator = (const sc_dt::sc_logic &);
    sc_signal_resolved &operator = (const sc_signal_resolved &);

    virtual const char *kind() const { return "sc_signal_resolved"; }

  protected:
    virtual void update();

  private:
    // Disabled
    sc_signal_resolved(const sc_signal_resolved &) :
            sc_interface(), sc_signal<sc_dt::sc_logic, SC_MANY_WRITERS>()
    {}

    std::map<::sc_gem5::Process *, sc_dt::sc_logic> inputs;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RESOLVED_HH__
