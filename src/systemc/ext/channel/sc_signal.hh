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

namespace sc_core
{

class sc_port_base;

} // namespace sc_core

namespace sc_gem5
{

class Process;
class Reset;

class ScSignalBase : public sc_core::sc_prim_channel
{
  public:
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    ScSignalBase(const char *_name);
    virtual ~ScSignalBase();

    const sc_core::sc_event &defaultEvent() const;
    const sc_core::sc_event &valueChangedEvent() const;

    bool event() const;

    void _signalChange();

    virtual sc_core::sc_writer_policy get_writer_policy() const = 0;

    InternalScEvent _valueChangedEvent;
    uint64_t _changeStamp;
    sc_core::sc_port_base *_gem5WriterPort;
};

class ScSignalBaseBinary : public ScSignalBase
{
  protected:
    ScSignalBaseBinary(const char *_name);

    mutable std::vector<sc_gem5::Reset *> _resets;
    void _signalReset(sc_gem5::Reset *reset);
    void _signalReset();

    const sc_core::sc_event &posedgeEvent() const;
    const sc_core::sc_event &negedgeEvent() const;

    bool posedge() const;
    bool negedge() const;

    InternalScEvent _posedgeEvent;
    InternalScEvent _negedgeEvent;

    uint64_t _posStamp;
    uint64_t _negStamp;

    void _signalPosedge();
    void _signalNegedge();
};

template <class T>
class ScSignalBasePicker : public ScSignalBase
{
  protected:
    ScSignalBasePicker(const char *_name) : ScSignalBase(_name) {}
};

template <>
class ScSignalBasePicker<bool> : public ScSignalBaseBinary
{
  protected:
    ScSignalBasePicker(const char *_name) : ScSignalBaseBinary(_name) {}
};

template <>
class ScSignalBasePicker<sc_dt::sc_logic> : public ScSignalBaseBinary
{
  protected:
    ScSignalBasePicker(const char *_name) : ScSignalBaseBinary(_name) {}
};

template <sc_core::sc_writer_policy WRITER_POLICY>
class WriteChecker;

template <>
class WriteChecker<sc_core::SC_ONE_WRITER>
{
  public:
    WriteChecker(ScSignalBase *_sig);

    void checkPort(sc_core::sc_port_base &port,
            std::string iface_type_name, std::string out_name);
    void checkWriter();

  private:
    ScSignalBase *sig;
    sc_core::sc_port_base *firstPort;
    Process *proc;
    uint64_t writeStamp;
};

template <>
class WriteChecker<sc_core::SC_MANY_WRITERS>
{
  public:
    WriteChecker(ScSignalBase *_sig);

    void checkPort(sc_core::sc_port_base &port,
            std::string iface_type_name, std::string out_name);
    void checkWriter();

  private:
    ScSignalBase *sig;
    Process *proc;
    uint64_t writeStamp;
};

template <class T, sc_core::sc_writer_policy WRITER_POLICY>
class ScSignalBaseT :
    public ScSignalBasePicker<T>, public sc_core::sc_signal_inout_if<T>
{
  public:
    ScSignalBaseT(const char *_name) :
        ScSignalBasePicker<T>(_name), m_cur_val(T()), m_new_val(T()),
        _checker(this)
    {}
    ScSignalBaseT(const char *_name, const T &initial_value) :
        ScSignalBasePicker<T>(_name), m_cur_val(initial_value),
        m_new_val(initial_value), _checker(this)
    {}
    virtual ~ScSignalBaseT() {}

    virtual void
    register_port(sc_core::sc_port_base &port, const char *iface_type_name)
    {
#       if !defined(SC_NO_WRITE_CHECK)
        {
            _checker.checkPort(port, iface_type_name,
                typeid(sc_core::sc_signal_inout_if<T>).name());
        }
#       endif
    }

    virtual const T &read() const { return m_cur_val; }
    operator const T&() const { return read(); }

    virtual void
    write(const T &t)
    {
#       if !defined(SC_NO_WRITE_CHECK)
        {
            _checker.checkWriter();
        }
#       endif
        m_new_val = t;
        bool changed = !(m_cur_val == m_new_val);
        if (changed)
            this->request_update();
    }

    virtual const sc_core::sc_event &
    default_event() const
    {
        return ScSignalBase::defaultEvent();
    }

    virtual const sc_core::sc_event &
    value_changed_event() const
    {
        return ScSignalBase::valueChangedEvent();
    }

    virtual void print(std::ostream &os=std::cout) const { os << m_cur_val; }
    virtual void
    dump(std::ostream &os=std::cout) const
    {
        os << "     name = " << this->name() << ::std::endl;
        os << "    value = " << m_cur_val << ::std::endl;
        os << "new value = " << m_new_val << ::std::endl;
    }

    virtual bool event() const { return ScSignalBase::event(); }

    virtual sc_core::sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }

  protected:
    // These members which store the current and future value of the signal
    // are not specified in the standard but are referred to directly by one
    // of the tests.
    T m_cur_val;
    T m_new_val;

    WriteChecker<WRITER_POLICY> _checker;
};

template <typename T, sc_core::sc_writer_policy WRITER_POLICY>
class ScSignalBinary : public ScSignalBaseT<T, WRITER_POLICY>
{
  public:
    ScSignalBinary(const char *_name) : ScSignalBaseT<T, WRITER_POLICY>(_name)
    {}
    ScSignalBinary(const char *_name, const T& initial_value) :
        ScSignalBaseT<T, WRITER_POLICY>(_name, initial_value)
    {}

    const sc_core::sc_event &
    posedge_event() const
    {
        return ScSignalBaseBinary::posedgeEvent();
    }
    const sc_core::sc_event &
    negedge_event() const
    {
        return ScSignalBaseBinary::negedgeEvent();
    }

    bool posedge() const { return ScSignalBaseBinary::posedge(); }
    bool negedge() const { return ScSignalBaseBinary::negedge(); }
};

} // namespace sc_gem5

namespace sc_core
{

template <class T, sc_writer_policy WRITER_POLICY=SC_ONE_WRITER>
class sc_signal : public sc_gem5::ScSignalBaseT<T, WRITER_POLICY>
{
  public:
    sc_signal() : sc_gem5::ScSignalBaseT<T, WRITER_POLICY>(
            sc_gen_unique_name("signal"))
    {}
    explicit sc_signal(const char *name) :
        sc_gem5::ScSignalBaseT<T, WRITER_POLICY>(name)
    {}
    explicit sc_signal(const char *name, const T &initial_value) :
        sc_gem5::ScSignalBaseT<T, WRITER_POLICY>(name, initial_value)
    {}
    virtual ~sc_signal() {}

    sc_signal<T, WRITER_POLICY> &
    operator = (const T &t)
    {
        this->write(t);
        return *this;
    }
    sc_signal<T, WRITER_POLICY> &
    operator = (const sc_signal<T, WRITER_POLICY> &s)
    {
        this->write(s.read());
        return *this;
    }

  protected:
    virtual void
    update()
    {
        if (this->m_new_val == this->m_cur_val)
            return;

        this->m_cur_val = this->m_new_val;
        this->_signalChange();
    }

  private:
    // Disabled
    sc_signal(const sc_signal<T, WRITER_POLICY> &);
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
    public sc_gem5::ScSignalBinary<bool, WRITER_POLICY>
{
  public:
    sc_signal() :
        sc_gem5::ScSignalBinary<bool, WRITER_POLICY>(
                sc_gen_unique_name("signal"))
    {}
    explicit sc_signal(const char *name) :
        sc_gem5::ScSignalBinary<bool, WRITER_POLICY>(name)
    {}
    explicit sc_signal(const char *name, const bool &initial_value) :
        sc_gem5::ScSignalBinary<bool, WRITER_POLICY>(name, initial_value)
    {}
    virtual ~sc_signal() {}

    sc_signal<bool, WRITER_POLICY> &
    operator = (const bool &b)
    {
        this->write(b);
        return *this;
    }
    sc_signal<bool, WRITER_POLICY> &
    operator = (const sc_signal<bool, WRITER_POLICY> &s)
    {
        this->write(s.read());
        return *this;
    }

  protected:
    virtual void
    update()
    {
        if (this->m_new_val == this->m_cur_val)
            return;

        this->m_cur_val = this->m_new_val;
        this->_signalChange();
    }

    void
    _signalChange()
    {
        sc_gem5::ScSignalBinary<bool, WRITER_POLICY>::_signalChange();
        this->_signalReset();
        if (this->m_cur_val)
            this->_signalPosedge();
        else
            this->_signalNegedge();
    }

  private:
    bool
    _addReset(sc_gem5::Reset *reset) const
    {
        this->_resets.push_back(reset);
        return true;
    }

    // Disabled
    sc_signal(const sc_signal<bool, WRITER_POLICY> &);
};

template <sc_writer_policy WRITER_POLICY>
class sc_signal<sc_dt::sc_logic, WRITER_POLICY> :
    public sc_gem5::ScSignalBinary<sc_dt::sc_logic, WRITER_POLICY>
{
  public:
    sc_signal() :
        sc_gem5::ScSignalBinary<sc_dt::sc_logic, WRITER_POLICY>(
                sc_gen_unique_name("signal"))
    {}
    explicit sc_signal(const char *name) :
        sc_gem5::ScSignalBinary<sc_dt::sc_logic, WRITER_POLICY>(name)
    {}
    explicit sc_signal(const char *name,
            const sc_dt::sc_logic &initial_value) :
        sc_gem5::ScSignalBinary<sc_dt::sc_logic, WRITER_POLICY>(
                name, initial_value)
    {}
    virtual ~sc_signal() {}

    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_dt::sc_logic &l)
    {
        this->write(l);
        return *this;
    }
    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &s)
    {
        this->write(s.read());
        return *this;
    }

  protected:
    virtual void
    update()
    {
        if (this->m_new_val == this->m_cur_val)
            return;

        this->m_cur_val = this->m_new_val;
        this->_signalChange();
    }

    void
    _signalChange()
    {
        sc_gem5::ScSignalBinary<sc_dt::sc_logic, WRITER_POLICY>::
            _signalChange();
        if (this->m_cur_val == sc_dt::SC_LOGIC_1)
            this->_signalPosedge();
        else if (this->m_cur_val == sc_dt::SC_LOGIC_0)
            this->_signalNegedge();
    }

  private:
    // Disabled
    sc_signal(const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &);
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_HH__
