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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SIGNAL_HH__

#include <iostream>
#include <string>
#include <vector>

#include "../core/sc_event.hh"
#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "../core/sc_prim.hh"
#include "../dt/bit/sc_logic.hh"
#include "sc_signal_inout_if.hh"
#include "warn_unimpl.hh" // for warn_unimpl

namespace sc_core
{

class sc_port_base;
class sc_trace_file;

// Nonstandard
// Despite having a warning "FOR INTERNAL USE ONLY!" in all caps above this
// class definition in the Accellera implementation, it appears in their
// examples and test programs, and so we need to have it here as well.
struct sc_trace_params
{
    sc_trace_file *tf;
    std::string name;

    sc_trace_params(sc_trace_file *tf, const std::string &name) :
        tf(tf), name(name)
    {}
};
typedef std::vector<sc_trace_params *> sc_trace_params_vec;

template <class T, sc_writer_policy WRITER_POLICY=SC_ONE_WRITER>
class sc_signal : public sc_signal_inout_if<T>,
                  public sc_prim_channel
{
  public:
    sc_signal() : sc_signal_inout_if<T>(),
                  sc_prim_channel(sc_gen_unique_name("signal")),
                  m_cur_val(T()), m_new_val(T()), _changeStamp(~0ULL)
    {}
    explicit sc_signal(const char *name) :
        sc_signal_inout_if<T>(), sc_prim_channel(name),
        m_cur_val(T()), m_new_val(T()), _changeStamp(~0ULL)
    {}
    explicit sc_signal(const char *name, const T &initial_value) :
        sc_signal_inout_if<T>(), sc_prim_channel(name),
        m_cur_val(initial_value), m_new_val(initial_value), _changeStamp(~0ULL)
    {}
    virtual ~sc_signal() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const T &read() const { return m_cur_val; }
    operator const T&() const { return read(); }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const T &t)
    {
        m_new_val = t;
        bool changed = !(m_cur_val == m_new_val);
        //TODO check whether this write follows the write policy.
        if (changed)
            request_update();
    }
    sc_signal<T, WRITER_POLICY> &
    operator = (const T &t)
    {
        write(t);
        return *this;
    }
    sc_signal<T, WRITER_POLICY> &
    operator = (const sc_signal<T, WRITER_POLICY> &s)
    {
        write(s.read());
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        return value_changed_event();
    }
    virtual const sc_event &
    value_changed_event() const
    {
        return _valueChangedEvent;
    }
    virtual bool
    event() const
    {
        return _changeStamp == ::sc_gem5::getChangeStamp();
    }

    virtual void print(std::ostream &os=std::cout) const { os << m_cur_val; }
    virtual void
    dump(std::ostream &os=std::cout) const
    {
        os << "     name = " << name() << ::std::endl;
        os << "    value = " << m_cur_val << ::std::endl;
        os << "new value = " << m_new_val << ::std::endl;
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        if (m_new_val == m_cur_val)
            return;

        m_cur_val = m_new_val;
        _signalChange();
        _changeStamp = ::sc_gem5::getChangeStamp();
        _valueChangedEvent.notify(SC_ZERO_TIME);
    }

    void
    _signalChange()
    {
        _changeStamp = ::sc_gem5::getChangeStamp();
        _valueChangedEvent.notify(SC_ZERO_TIME);
    }

    // These members which store the current and future value of the signal
    // are not specified in the standard but are referred to directly by one
    // of the tests.
    T m_cur_val;
    T m_new_val;

  private:
    sc_event _valueChangedEvent;
    uint64_t _changeStamp;

    // Disabled
    sc_signal(const sc_signal<T, WRITER_POLICY> &) :
            sc_signal_inout_if<T>(), sc_prim_channel("")
    {}
};

template <class T, sc_writer_policy WRITER_POLICY>
inline std::ostream &
operator << (std::ostream &os, const sc_signal<T, WRITER_POLICY> &s)
{
    os << s.read();
    return os;
}

template <sc_writer_policy WRITER_POLICY>
class sc_signal<bool, WRITER_POLICY> :
    public sc_signal_inout_if<bool>, public sc_prim_channel
{
  public:
    sc_signal() : sc_signal_inout_if<bool>(),
                  sc_prim_channel(sc_gen_unique_name("signal")),
                  m_cur_val(bool()), m_new_val(bool()),
                  _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    explicit sc_signal(const char *name) :
        sc_signal_inout_if<bool>(), sc_prim_channel(name),
        m_cur_val(bool()), m_new_val(bool()),
        _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    explicit sc_signal(const char *name, const bool &initial_value) :
        sc_signal_inout_if<bool>(), sc_prim_channel(name),
        m_cur_val(initial_value), m_new_val(initial_value),
        _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    virtual ~sc_signal() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const bool &read() const { return m_cur_val; }
    operator const bool &() const { return read(); }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const bool &b)
    {
        m_new_val = b;
        bool changed = !(m_cur_val == m_new_val);
        //TODO check whether this write follows the write policy.
        if (changed)
            request_update();
    }
    sc_signal<bool, WRITER_POLICY> &
    operator = (const bool &b)
    {
        write(b);
        return *this;
    }
    sc_signal<bool, WRITER_POLICY> &
    operator = (const sc_signal<bool, WRITER_POLICY> &s)
    {
        write(s.read());
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        return value_changed_event();
    }

    virtual const sc_event &
    value_changed_event() const
    {
        return _valueChangedEvent;
    }
    virtual const sc_event &
    posedge_event() const
    {
        return _posedgeEvent;
    }
    virtual const sc_event &
    negedge_event() const
    {
        return _negedgeEvent;
    }

    virtual bool
    event() const
    {
        return _changeStamp == ::sc_gem5::getChangeStamp();
    }
    virtual bool
    posedge() const
    {
        return _posStamp == ::sc_gem5::getChangeStamp();
    }
    virtual bool
    negedge() const
    {
        return _negStamp == ::sc_gem5::getChangeStamp();
    }

    virtual void print(std::ostream &os=std::cout) const { os << m_cur_val; }
    virtual void
    dump(std::ostream &os=std::cout) const
    {
        os << "     name = " << name() << ::std::endl;
        os << "    value = " << m_cur_val << ::std::endl;
        os << "new value = " << m_new_val << ::std::endl;
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        if (m_new_val == m_cur_val)
            return;

        m_cur_val = m_new_val;
        _signalChange();
        if (m_cur_val) {
            _posStamp = ::sc_gem5::getChangeStamp();
            _posedgeEvent.notify(SC_ZERO_TIME);
        } else {
            _negStamp = ::sc_gem5::getChangeStamp();
            _negedgeEvent.notify(SC_ZERO_TIME);
        }
    }

    void
    _signalChange()
    {
        _changeStamp = ::sc_gem5::getChangeStamp();
        _valueChangedEvent.notify(SC_ZERO_TIME);
    }

    bool m_cur_val;
    bool m_new_val;

  private:
    sc_event _valueChangedEvent;
    sc_event _posedgeEvent;
    sc_event _negedgeEvent;

    uint64_t _changeStamp;
    uint64_t _posStamp;
    uint64_t _negStamp;

    // Disabled
    sc_signal(const sc_signal<bool, WRITER_POLICY> &) :
            sc_signal_inout_if<bool>(), sc_prim_channel("")
    {}
};

template <sc_writer_policy WRITER_POLICY>
class sc_signal<sc_dt::sc_logic, WRITER_POLICY> :
    public sc_signal_inout_if<sc_dt::sc_logic>, public sc_prim_channel
{
  public:
    sc_signal() : sc_signal_inout_if<sc_dt::sc_logic>(),
                  sc_prim_channel(sc_gen_unique_name("signal")),
                  m_cur_val(sc_dt::sc_logic()), m_new_val(sc_dt::sc_logic()),
                  _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    explicit sc_signal(const char *name) :
        sc_signal_inout_if<sc_dt::sc_logic>(), sc_prim_channel(name),
        m_cur_val(sc_dt::sc_logic()), m_new_val(sc_dt::sc_logic()),
        _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    explicit sc_signal(const char *name,
            const sc_dt::sc_logic &initial_value) :
        sc_signal_inout_if<sc_dt::sc_logic>(), sc_prim_channel(name),
        m_cur_val(initial_value), m_new_val(initial_value),
        _changeStamp(~0ULL), _posStamp(~0ULL), _negStamp(~0ULL)
    {}
    virtual ~sc_signal() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const sc_dt::sc_logic &read() const { return m_cur_val; }
    operator const sc_dt::sc_logic &() const { return read(); }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const sc_dt::sc_logic &l)
    {
        m_new_val = l;
        bool changed = !(m_cur_val == m_new_val);
        //TODO check whether this write follows the write policy.
        if (changed)
            request_update();
    }
    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_dt::sc_logic &l)
    {
        write(l);
        return *this;
    }
    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &s)
    {
        write(s.read());
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        return value_changed_event();
    }

    virtual const sc_event &
    value_changed_event() const
    {
        return _valueChangedEvent;
    }
    virtual const sc_event &
    posedge_event() const
    {
        return _posedgeEvent;
    }
    virtual const sc_event &
    negedge_event() const
    {
        return _negedgeEvent;
    }

    virtual bool
    event() const
    {
        return _changeStamp == ::sc_gem5::getChangeStamp();
    }
    virtual bool
    posedge() const
    {
        return _posStamp == ::sc_gem5::getChangeStamp();
    }
    virtual bool
    negedge() const
    {
        return _negStamp == ::sc_gem5::getChangeStamp();
    }

    virtual void print(std::ostream &os=std::cout) const { os << m_cur_val; }
    virtual void
    dump(std::ostream &os=std::cout) const
    {
        os << "     name = " << name() << ::std::endl;
        os << "    value = " << m_cur_val << ::std::endl;
        os << "new value = " << m_new_val << ::std::endl;
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        if (m_new_val == m_cur_val)
            return;

        m_cur_val = m_new_val;
        _signalChange();
        if (m_cur_val == sc_dt::SC_LOGIC_1) {
            _posStamp = ::sc_gem5::getChangeStamp();
            _posedgeEvent.notify(SC_ZERO_TIME);
        } else if (m_cur_val == sc_dt::SC_LOGIC_0) {
            _negStamp = ::sc_gem5::getChangeStamp();
            _negedgeEvent.notify(SC_ZERO_TIME);
        }
    }

    void
    _signalChange()
    {
        _changeStamp = ::sc_gem5::getChangeStamp();
        _valueChangedEvent.notify(SC_ZERO_TIME);
    }

    sc_dt::sc_logic m_cur_val;
    sc_dt::sc_logic m_new_val;

  private:
    sc_event _valueChangedEvent;
    sc_event _posedgeEvent;
    sc_event _negedgeEvent;

    uint64_t _changeStamp;
    uint64_t _posStamp;
    uint64_t _negStamp;

    // Disabled
    sc_signal(const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &) :
            sc_signal_inout_if<sc_dt::sc_logic>(), sc_prim_channel("")
    {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_HH__
