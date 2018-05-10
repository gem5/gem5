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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RV_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RV_HH__

#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "sc_signal.hh"
#include "warn_unimpl.hh"

namespace sc_dt
{

template <int W>
class sc_lv;

};

namespace sc_core
{

class sc_port_base;

template <int W>
class sc_signal_rv : public sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS>
{
  public:
    sc_signal_rv() : sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS>(
            sc_gen_unique_name("signal_rv"))
    {}
    sc_signal_rv(const char *name) :
            sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS>(name)
    {}
    virtual ~sc_signal_rv() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    write(const sc_dt::sc_lv<W> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_signal_rv<W> &
    operator = (const sc_dt::sc_lv<W> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }
    sc_signal_rv<W> &
    operator = (const sc_signal_rv<W> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }

    virtual const char *kind() const { return "sc_signal_rv"; }

  protected:
    virtual void
    update()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

  private:
    // Disabled
    sc_signal_rv(const sc_signal_rv<W> &) :
            sc_signal<sc_dt::sc_lv<W>, SC_MANY_WRITERS>()
    {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_RV_HH__
