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

#ifndef __SYSTEMC_EXT_CORE_SC_SENSITIVE_HH__
#define __SYSTEMC_EXT_CORE_SC_SENSITIVE_HH__

namespace sc_gem5
{

class Process;

} // namespace sc_gem5

namespace sc_dt
{

class sc_logic;

} // namespace sc_dt

namespace sc_core
{

class sc_event;
class sc_event_finder;
class sc_interface;
class sc_module;
class sc_port_base;

template <class T>
class sc_signal_in_if;

template <class T>
class sc_in;

template <class T>
class sc_inout;

class sc_sensitive
{
  public:
    sc_sensitive &operator << (const sc_event &);
    sc_sensitive &operator << (const sc_interface &);
    sc_sensitive &operator << (const sc_port_base &);
    sc_sensitive &operator << (sc_event_finder &);

    sc_sensitive &operator << (::sc_gem5::Process *p);

    // Nonstandard.
    void operator () (::sc_gem5::Process *p, const sc_signal_in_if<bool> &);
    void operator () (::sc_gem5::Process *p,
                      const sc_signal_in_if<sc_dt::sc_logic> &);
    void operator () (::sc_gem5::Process *p, const sc_in<bool> &);
    void operator () (::sc_gem5::Process *p, const sc_in<sc_dt::sc_logic> &);
    void operator () (::sc_gem5::Process *p, const sc_inout<bool> &);
    void operator () (::sc_gem5::Process *p,
                      const sc_inout<sc_dt::sc_logic> &);
    void operator () (::sc_gem5::Process *p, sc_event_finder &);

  private:
    friend class sc_module;

    // Install all the static events which may not have been ready at
    // construction time, like the default_event of the peer of an unbound
    // port.
    void finalize();

    sc_sensitive();

    ::sc_gem5::Process *currentProcess;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_SENSITIVE_HH__
