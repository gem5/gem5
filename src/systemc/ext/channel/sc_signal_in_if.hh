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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_IN_IF_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_IN_IF_HH__

#include "../core/sc_interface.hh"

namespace sc_dt
{

class sc_logic;

};

namespace sc_gem5
{

class Reset;

} // namespace sc_gem5

namespace sc_core
{

class sc_event;

template <class T>
class sc_signal_in_if : virtual public sc_interface
{
  public:
    virtual const T &read() const = 0;
    virtual const sc_event &value_changed_event() const = 0;
    virtual bool event() const = 0;

  protected:
    sc_signal_in_if() : sc_interface() {}

  private:
    // Disabled
    sc_signal_in_if(const sc_signal_in_if<T> &) : sc_interface() {}
    sc_signal_in_if<T> &
    operator = (const sc_signal_in_if<T> &)
    {
        return *this;
    }
};

template <>
class sc_signal_in_if<bool> : virtual public sc_interface
{
  public:
    virtual const bool &read() const = 0;

    virtual const sc_event &value_changed_event() const = 0;
    virtual const sc_event &posedge_event() const = 0;
    virtual const sc_event &negedge_event() const = 0;

    virtual bool event() const = 0;
    virtual bool posedge() const = 0;
    virtual bool negedge() const = 0;

  protected:
    sc_signal_in_if() : sc_interface() {}

  private:
    friend class sc_gem5::Reset;
    virtual bool
    _addReset(sc_gem5::Reset *reset) const
    {
        return false;
    }

    // Disabled
    sc_signal_in_if(const sc_signal_in_if<bool> &) : sc_interface() {}
    sc_signal_in_if<bool> &
    operator = (const sc_signal_in_if<bool> &)
    {
        return *this;
    }
};

template <>
class sc_signal_in_if<sc_dt::sc_logic> : virtual public sc_interface
{
  public:
    virtual const sc_dt::sc_logic &read() const = 0;

    virtual const sc_event &value_changed_event() const = 0;
    virtual const sc_event &posedge_event() const = 0;
    virtual const sc_event &negedge_event() const = 0;

    virtual bool event() const = 0;
    virtual bool posedge() const = 0;
    virtual bool negedge() const = 0;

  protected:
    sc_signal_in_if() : sc_interface() {}

  private:
    // Disabled
    sc_signal_in_if(const sc_signal_in_if<sc_dt::sc_logic> &) :
            sc_interface()
    {}
    sc_signal_in_if<sc_dt::sc_logic> &
    operator = (const sc_signal_in_if<sc_dt::sc_logic> &)
    {
        return *this;
    }
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_IN_IF_HH__
