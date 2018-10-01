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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_OUT_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_OUT_HH__

#include "../core/sc_port.hh"
#include "sc_inout.hh"

namespace sc_core
{

template <class T>
class sc_out : public sc_inout<T>
{
  public:
    sc_out() : sc_inout<T>() {}
    explicit sc_out(const char *name) : sc_inout<T>(name) {}
    virtual ~sc_out() {}

    // Deprecated binding constructors.
    explicit sc_out(const sc_signal_inout_if<T> &interface) :
        sc_inout<T>(interface)
    {}
    sc_out(const char *name, const sc_signal_inout_if<T> &interface) :
        sc_inout<T>(name, interface)
    {}
    explicit sc_out(sc_port_b<sc_signal_inout_if<T> > &parent) :
        sc_inout<T>(parent)
    {}
    sc_out(const char *name, sc_port_b<sc_signal_inout_if<T> > &parent) :
        sc_inout<T>(name, parent)
    {}
    explicit sc_out(sc_out<T> &parent) : sc_inout<T>(parent) {}
    sc_out(const char *name, sc_out<T> &parent) : sc_inout<T>(name, parent) {}

    sc_out<T> &
    operator = (const T &t)
    {
        sc_inout<T>::operator = (t);
        return *this;
    }
    sc_out<T> &
    operator = (const sc_signal_in_if<T> &c)
    {
        sc_inout<T>::operator = (c);
        return *this;
    }
    sc_out<T> &
    operator = (const sc_port<sc_signal_in_if<T>, 1> &c)
    {
        sc_inout<T>::operator = (c);
        return *this;
    }
    sc_out<T> &
    operator = (const sc_port<sc_signal_inout_if<T>, 1> &c)
    {
        sc_inout<T>::operator = (c);
        return *this;
    }
    sc_out<T> &
    operator = (const sc_out<T> &c)
    {
        sc_inout<T>::operator = (c);
        return *this;
    }

    virtual const char *kind() const { return "sc_out"; }

  private:
    // Disabled
    sc_out(const sc_out<T> &) : sc_inout<T>() {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_OUT_HH__
