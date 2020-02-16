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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_INOUT_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_INOUT_HH__

#include <string>

#include "../core/sc_event.hh"
#include "../core/sc_main.hh"
#include "../core/sc_port.hh"
#include "../dt/bit/sc_logic.hh"
#include "../utils/sc_trace_file.hh"
#include "sc_signal_inout_if.hh"

namespace sc_dt
{

class sc_logic;

} // namespace sc_dt

namespace sc_core
{

class sc_event;
class sc_trace_file;

template <class T>
class sc_inout : public sc_port<sc_signal_inout_if<T>, 1>
{
  public:
    sc_inout() : sc_port<sc_signal_inout_if<T>, 1>(), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    explicit sc_inout(const char *name) :
        sc_port<sc_signal_inout_if<T>, 1>(name), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    virtual ~sc_inout() { delete initValue; }

    // Deprecated binding constructors.
    explicit sc_inout(const sc_signal_inout_if<T> &interface) :
        sc_port<sc_signal_inout_if<T>, 1>(interface), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    sc_inout(const char *name, const sc_signal_inout_if<T> &interface) :
        sc_port<sc_signal_inout_if<T>, 1>(name, interface), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    explicit sc_inout(sc_port_b<sc_signal_inout_if<T> > &parent) :
        sc_port<sc_signal_inout_if<T>, 1>(parent), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    sc_inout(const char *name, sc_port_b<sc_signal_inout_if<T> > &parent) :
        sc_port<sc_signal_inout_if<T>, 1>(name, parent), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    explicit sc_inout(sc_port<sc_signal_inout_if<T>, 1> &parent) :
        sc_port<sc_signal_inout_if<T>, 1>(parent), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}
    sc_inout(const char *name, sc_port<sc_signal_inout_if<T>, 1> &parent) :
        sc_port<sc_signal_inout_if<T>, 1>(name, parent), initValue(nullptr),
        _valueChangedFinder(*this, &sc_signal_inout_if<T>::value_changed_event)
    {}

    void
    initialize(const T &t)
    {
        if (this->size()) {
            (*this)->write(t);
        } else {
            if (!initValue)
                initValue = new T;
            *initValue = t;
        }
    }
    void initialize(const sc_signal_in_if<T> &i) { initialize(i.read()); }

    virtual void
    end_of_elaboration()
    {
        if (initValue) {
            write(*initValue);
            delete initValue;
            initValue = nullptr;
        }

        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const T &read() const { return (*this)->read(); }
    operator const T& () const { return (*this)->read(); }

    void write(const T &t) { (*this)->write(t); }
    sc_inout<T> &
    operator = (const T &t)
    {
        (*this)->write(t);
        return *this;
    }
    sc_inout<T> &
    operator = (const sc_signal_in_if<T> &i)
    {
        (*this)->write(i.read());
        return *this;
    }
    sc_inout<T> &
    operator = (const sc_port<sc_signal_in_if<T>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<T> &
    operator = (const sc_port<sc_signal_inout_if<T>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<T> &
    operator = (const sc_inout<T> &p)
    {
        (*this)->write(p->read());
        return *this;
    }

    const sc_event &default_event() const { return (*this)->default_event(); }
    const sc_event &
    value_changed_event() const
    {
        return (*this)->value_changed_event();
    }
    bool event() const { return (*this)->event(); }
    sc_event_finder &value_changed() const { return _valueChangedFinder; }

    virtual const char *kind() const { return "sc_inout"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    T *initValue;
    mutable sc_event_finder_t<sc_signal_inout_if<T> > _valueChangedFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_inout(const sc_inout<T> &);
};

template <class T>
inline void
sc_trace(sc_trace_file *tf, const sc_inout<T> &i, const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

template <>
class sc_inout<bool> : public sc_port<sc_signal_inout_if<bool>, 1>
{
  public:
    sc_inout() : sc_port<sc_signal_inout_if<bool>, 1>(), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    explicit sc_inout(const char *name) :
            sc_port<sc_signal_inout_if<bool>, 1>(name), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    virtual ~sc_inout() { delete initValue; }

    // Deprecated binding constructors.
    explicit sc_inout(const sc_signal_inout_if<bool> &interface) :
        sc_port<sc_signal_inout_if<bool>, 1>(interface), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    sc_inout(const char *name, const sc_signal_inout_if<bool> &interface) :
        sc_port<sc_signal_inout_if<bool>, 1>(name, interface),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    explicit sc_inout(sc_port_b<sc_signal_inout_if<bool> > &parent) :
        sc_port<sc_signal_inout_if<bool>, 1>(parent), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    sc_inout(const char *name, sc_port_b<sc_signal_inout_if<bool> > &parent) :
        sc_port<sc_signal_inout_if<bool>, 1>(name, parent), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    explicit sc_inout(sc_port<sc_signal_inout_if<bool>, 1> &parent) :
        sc_port<sc_signal_inout_if<bool>, 1>(parent), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}
    sc_inout(const char *name, sc_port<sc_signal_inout_if<bool>, 1> &parent) :
        sc_port<sc_signal_inout_if<bool>, 1>(name, parent), initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<bool>::negedge_event)
    {}

    void
    initialize(const bool &b)
    {
        if (this->size()) {
            (*this)->write(b);
        } else {
            if (!initValue)
                initValue = new bool;
            *initValue = b;
        }
    }
    void initialize(const sc_signal_in_if<bool> &i) { initialize(i.read()); }

    virtual void
    end_of_elaboration()
    {
        if (initValue) {
            write(*initValue);
            delete initValue;
            initValue = nullptr;
        }

        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const bool &read() const { return (*this)->read(); }
    operator const bool& () const { return (*this)->read(); }

    void write(const bool &b) { (*this)->write(b); }
    sc_inout<bool> &
    operator = (const bool &b)
    {
        (*this)->write(b);
        return *this;
    }
    sc_inout<bool> &
    operator = (const sc_signal_in_if<bool> &i)
    {
        (*this)->write(i.read());
        return *this;
    }
    sc_inout<bool> &
    operator = (const sc_port<sc_signal_in_if<bool>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<bool> &
    operator = (const sc_port<sc_signal_inout_if<bool>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<bool> &
    operator = (const sc_inout<bool> &p)
    {
        (*this)->write(p->read());
        return *this;
    }

    const sc_event &default_event() const { return (*this)->default_event(); }
    const sc_event &
    value_changed_event() const
    {
        return (*this)->value_changed_event();
    }
    const sc_event &posedge_event() const { return (*this)->posedge_event(); }
    const sc_event &negedge_event() const { return (*this)->negedge_event(); }
    bool event() const { return (*this)->event(); }
    bool posedge() const { return (*this)->posedge(); }
    bool negedge() const { return (*this)->negedge(); }

    sc_event_finder &value_changed() const { return _valueChangedFinder; }
    sc_event_finder &pos() const { return _posFinder; }
    sc_event_finder &neg() const { return _negFinder; }

    virtual const char *kind() const { return "sc_inout"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    bool *initValue;
    mutable sc_event_finder_t<sc_signal_inout_if<bool> > _valueChangedFinder;
    mutable sc_event_finder_t<sc_signal_inout_if<bool> > _posFinder;
    mutable sc_event_finder_t<sc_signal_inout_if<bool> > _negFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_inout(const sc_inout<bool> &);
};

template <>
inline void sc_trace<bool>(
        sc_trace_file *tf, const sc_inout<bool> &i, const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

template <>
class sc_inout<sc_dt::sc_logic> :
        public sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>
{
  public:
    sc_inout() : sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_inout(const char *name) :
            sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(name),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    virtual ~sc_inout() { delete initValue; }

    // Deprecated binding constructors.
    explicit sc_inout(const sc_signal_inout_if<sc_dt::sc_logic> &interface) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(interface),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_inout(const char *name,
            const sc_signal_inout_if<sc_dt::sc_logic> &interface) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(name, interface),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_inout(
            sc_port_b<sc_signal_inout_if<sc_dt::sc_logic> > &parent) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(parent),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_inout(const char *name,
            sc_port_b<sc_signal_inout_if<sc_dt::sc_logic> > &parent) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(name, parent),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_inout(
            sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &parent) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(parent),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_inout(const char *name,
            sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &parent) :
        sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1>(name, parent),
        initValue(nullptr),
        _valueChangedFinder(*this,
                &sc_signal_inout_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_inout_if<sc_dt::sc_logic>::negedge_event)
    {}

    void
    initialize(const sc_dt::sc_logic &l)
    {
        if (this->size()) {
            (*this)->write(l);
        } else {
            if (!initValue)
                initValue = new sc_dt::sc_logic;
            *initValue = l;
        }
    }
    void
    initialize(const sc_signal_in_if<sc_dt::sc_logic> &i)
    {
        initialize(i.read());
    }

    virtual void
    end_of_elaboration()
    {
        if (initValue) {
            write(*initValue);
            delete initValue;
            initValue = nullptr;
        }

        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const sc_dt::sc_logic &read() const { return (*this)->read(); }
    operator const sc_dt::sc_logic& () const { return (*this)->read(); }

    void write(const sc_dt::sc_logic &l) { (*this)->write(l); }
    sc_inout<sc_dt::sc_logic> &
    operator = (const sc_dt::sc_logic &l)
    {
        (*this)->write(l);
        return *this;
    }
    sc_inout<sc_dt::sc_logic> &
    operator = (const sc_signal_in_if<sc_dt::sc_logic> &i)
    {
        (*this)->write(i.read());
        return *this;
    }
    sc_inout<sc_dt::sc_logic> &
    operator = (const sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<sc_dt::sc_logic> &
    operator = (const sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &p)
    {
        (*this)->write(p->read());
        return *this;
    }
    sc_inout<sc_dt::sc_logic> &
    operator = (const sc_inout<sc_dt::sc_logic> &p)
    {
        (*this)->write(p->read());
        return *this;
    }

    const sc_event &default_event() const { return (*this)->default_event(); }
    const sc_event &
    value_changed_event() const
    {
        return (*this)->value_changed_event();
    }
    const sc_event &posedge_event() const { return (*this)->posedge_event(); }
    const sc_event &negedge_event() const { return (*this)->negedge_event(); }
    bool event() const { return (*this)->event(); }
    bool posedge() const { return (*this)->posedge(); }
    bool negedge() const { return (*this)->negedge(); }

    sc_event_finder &value_changed() const { return _valueChangedFinder; }
    sc_event_finder &pos() const { return _posFinder; }
    sc_event_finder &neg() const { return _negFinder; }

    virtual const char *kind() const { return "sc_inout"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    sc_dt::sc_logic *initValue;
    mutable sc_event_finder_t<
        sc_signal_inout_if<sc_dt::sc_logic> > _valueChangedFinder;
    mutable sc_event_finder_t<sc_signal_inout_if<sc_dt::sc_logic> > _posFinder;
    mutable sc_event_finder_t<sc_signal_inout_if<sc_dt::sc_logic> > _negFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_inout(const sc_inout<sc_dt::sc_logic> &);
};

template <>
inline void
sc_trace<sc_dt::sc_logic>(sc_trace_file *tf,
        const sc_inout<sc_dt::sc_logic> &i, const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_INOUT_HH__
