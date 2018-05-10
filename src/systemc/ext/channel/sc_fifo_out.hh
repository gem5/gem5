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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_FIFO_OUT_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_FIFO_OUT_HH__

#include "../core/sc_port.hh"
#include "sc_fifo_out_if.hh"
#include "warn_unimpl.hh"

namespace sc_core
{

class sc_event;
class sc_event_finder;

template <class T>
class sc_fifo_out : public sc_port<sc_fifo_out_if<T>, 0>
{
  public:
    sc_fifo_out() : sc_port<sc_fifo_out_if<T>, 0>() {}
    explicit sc_fifo_out(const char *name) :
            sc_port<sc_fifo_out_if<T>, 0>(name)
    {}
    virtual ~sc_fifo_out() {}

    void
    write(const T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    bool
    nb_write(const T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    const sc_event &
    data_read_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    sc_event_finder &
    data_read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }
    int
    num_free() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return 0;
    }
    virtual const char *kind() const { return "sc_fifo_out"; }

  private:
    // Disabled
    sc_fifo_out(const sc_fifo_out<T> &) : sc_port<sc_fifo_out_if<T>, 0>() {}
    sc_fifo_out<T> &operator = (const sc_fifo_out<T> &) { return *this; }
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_FIFO_OUT_HH__
