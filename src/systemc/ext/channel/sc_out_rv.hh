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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_OUT_RV_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_OUT_RV_HH__

#include "../core/sc_port.hh"
#include "sc_inout_rv.hh"
#include "sc_signal_in_if.hh"
#include "sc_signal_inout_if.hh"

namespace sc_dt
{

template <int W>
class sc_lv;

} // namespace sc_dt

namespace sc_core
{

template <int W>
class sc_out_rv : public sc_inout_rv<W>
{
  public:
    sc_out_rv() : sc_inout_rv<W>() {}
    explicit sc_out_rv(const char *name) : sc_inout_rv<W>(name) {}
    virtual ~sc_out_rv() {};

    sc_out_rv<W> &
    operator = (const sc_dt::sc_lv<W> &l)
    {
        (*this)->write(l);
        return *this;
    }
    sc_out_rv<W> &
    operator = (const sc_signal_in_if<sc_dt::sc_lv<W>> &i)
    {
        (*this)->write(i.read());
        return *this;
    }
    sc_out_rv<W> &
    operator = (const sc_port<sc_signal_in_if<sc_dt::sc_lv<W>>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_out_rv<W> &
    operator = (const sc_port<sc_signal_inout_if<sc_dt::sc_lv<W>>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_out_rv<W> &
    operator = (const sc_out_rv<W> &p)
    {
        (*this)->write(p->read());
        return *this;
    }

    virtual const char *kind() const { return "sc_out_rv"; }

  private:
    // Disabled
    sc_out_rv(const sc_out_rv<W> &) : sc_inout_rv<W>() {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_OUT_RV_HH__
