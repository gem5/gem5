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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_INOUT_IF_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_INOUT_IF_HH__

#include "../core/sc_interface.hh"
#include "sc_signal_in_if.hh"

namespace sc_core
{

enum sc_writer_policy
{
    SC_ONE_WRITER,
    SC_MANY_WRITERS
};

template <class T>
class sc_signal_write_if : virtual public sc_interface
{
  public:
    virtual sc_writer_policy
    get_writer_policy() const
    {
        return SC_ONE_WRITER;
    }
    virtual void write(const T &) = 0;

  protected:
    sc_signal_write_if() : sc_interface() {}

  private:
    // Disabled
    sc_signal_write_if(const sc_signal_write_if<T> &) : sc_interface() {}
    sc_signal_write_if<T> &
    operator = (const sc_signal_write_if<T> &)
    {
        return *this;
    }
};

template <class T>
class sc_signal_inout_if : public sc_signal_in_if<T>,
                           public sc_signal_write_if<T>
{
  protected:
    sc_signal_inout_if() : sc_signal_in_if<T>(), sc_signal_write_if<T>() {}

  private:
    // Disabled
    sc_signal_inout_if(const sc_signal_inout_if<T> &) :
        sc_signal_in_if<T>(), sc_signal_write_if<T>()
    {}
    sc_signal_inout_if<T> &
    operator = (const sc_signal_inout_if<T> &)
    {
        return *this;
    }
};

// Deprecated.
template <class T>
using sc_signal_out_if = sc_signal_inout_if<T>;

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_INOUT_IF_HH__
