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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_IN_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_IN_HH__

#include <string>

#include "../core/sc_port.hh"
#include "sc_signal_in_if.hh"
#include "sc_signal_inout_if.hh"
#include "warn_unimpl.hh"

namespace sc_core
{

class sc_event;
class sc_event_finder;
class sc_trace_file;

template <class T>
class sc_in : public sc_port<sc_signal_in_if<T>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<T>, 1>() {}
    explicit sc_in(const char *name) : sc_port<sc_signal_in_if<T>, 1>(name) {}
    virtual ~sc_in() {}

    virtual void
    bind(const sc_signal_in_if<T> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (const sc_signal_in_if<T> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_in_if<T>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_in_if<T>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<T>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_inout_if<T>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    end_of_elaboration()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    const T &
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }
    operator const T& () const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }

    const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    sc_event_finder &
    value_changed() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }

    virtual const char *kind() const { return "sc_in"; }

  private:
    // Disabled
    sc_in(const sc_in<T> &) : sc_port<sc_signal_in_if<T>, 1>() {}
    sc_in<T> &operator = (const sc_in<T> &) { return *this; }
};

template <class T>
inline void
sc_trace(sc_trace_file *, const sc_in<T> &, const std::string &)
{
    sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
}

template <>
class sc_in<bool> : public sc_port<sc_signal_in_if<bool>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<bool>, 1>() {}
    explicit sc_in(const char *name) :
        sc_port<sc_signal_in_if<bool>, 1>(name) {}
    virtual ~sc_in() {}

    virtual void
    bind(const sc_signal_in_if<bool> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (const sc_signal_in_if<bool> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_in_if<bool>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_in_if<bool>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<bool>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_inout_if<bool>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    end_of_elaboration()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    const bool &
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const bool *)nullptr;
    }
    operator const bool& () const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const bool *)nullptr;
    }

    const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    posedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    negedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }

    bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    bool
    posedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    bool
    negedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }

    sc_event_finder &
    value_changed() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }
    sc_event_finder &
    pos() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }
    sc_event_finder &
    neg() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }

    virtual const char *kind() const { return "sc_in"; }

  private:
    // Disabled
    sc_in(const sc_in<bool> &) : sc_port<sc_signal_in_if<bool>, 1>() {}
    sc_in<bool> &operator = (const sc_in<bool> &) { return *this; }
};

template <>
inline void
sc_trace<bool>(sc_trace_file *, const sc_in<bool> &, const std::string &)
{
    sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
}

template <>
class sc_in<sc_dt::sc_logic> :
    public sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>() {}
    explicit sc_in(const char *name) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(name)
    {}
    virtual ~sc_in() {}

    virtual void
    bind(const sc_signal_in_if<sc_dt::sc_logic> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (const sc_signal_in_if<sc_dt::sc_logic> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    void
    operator () (sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    end_of_elaboration()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    const sc_dt::sc_logic &
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_dt::sc_logic *)nullptr;
    }
    operator const sc_dt::sc_logic& () const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_dt::sc_logic *)nullptr;
    }

    const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    posedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    const sc_event &
    negedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }

    bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    bool
    posedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    bool
    negedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }

    sc_event_finder &
    value_changed() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }
    sc_event_finder &
    pos() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }
    sc_event_finder &
    neg() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event_finder *)nullptr;
    }

    virtual const char *kind() const { return "sc_in"; }

  private:
    // Disabled
    sc_in(const sc_in<sc_dt::sc_logic> &) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>()
    {}
    sc_in<sc_dt::sc_logic> &
    operator = (const sc_in<sc_dt::sc_logic> &)
    {
        return *this;
    }
};

template <>
inline void
sc_trace<sc_dt::sc_logic>(
        sc_trace_file *, const sc_in<sc_dt::sc_logic> &, const std::string &)
{
    sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_IN_HH__
