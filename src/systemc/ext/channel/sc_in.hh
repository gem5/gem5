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

#include "../core/sc_event.hh"
#include "../core/sc_main.hh"
#include "../core/sc_port.hh"
#include "../utils/sc_trace_file.hh"
#include "sc_signal_in_if.hh"
#include "sc_signal_inout_if.hh"

namespace sc_core
{

class sc_event;
class sc_trace_file;

template <class T>
class sc_in : public sc_port<sc_signal_in_if<T>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<T>, 1>(),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    explicit sc_in(const char *name) : sc_port<sc_signal_in_if<T>, 1>(name),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    virtual ~sc_in() {}

    // Deprecated binding constructors.
    explicit sc_in(const sc_signal_in_if<T> &interface) :
        sc_port<sc_signal_in_if<T>, 1>(interface),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    sc_in(const char *name, const sc_signal_in_if<T> &interface) :
        sc_port<sc_signal_in_if<T>, 1>(name, interface),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    explicit sc_in(sc_port_b<sc_signal_in_if<T> > &parent) :
        sc_port<sc_signal_in_if<T>, 1>(parent),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    sc_in(const char *name, sc_port_b<sc_signal_in_if<T> > &parent) :
        sc_port<sc_signal_in_if<T>, 1>(name, parent),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    explicit sc_in(sc_port<sc_signal_in_if<T>, 1> &parent) :
        sc_port<sc_signal_in_if<T>, 1>(parent),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}
    sc_in(const char *name, sc_port<sc_signal_in_if<T>, 1> &parent) :
        sc_port<sc_signal_in_if<T>, 1>(name, parent),
        _valueChangedFinder(*this, &sc_signal_in_if<T>::value_changed_event)
    {}

    using sc_port<sc_signal_in_if<T>, 1>::bind;
    virtual void
    bind(const sc_signal_in_if<T> &i)
    {
        sc_port<sc_signal_in_if<T>, 1>::bind(
                const_cast<sc_signal_in_if<T> &>(i));
    }
    void operator () (const sc_signal_in_if<T> &i) { bind(i); }

    virtual void
    bind(sc_port<sc_signal_in_if<T>, 1> &i)
    {
        sc_port<sc_signal_in_if<T>, 1>::bind(i);
    }
    void
    operator () (sc_port<sc_signal_in_if<T>, 1> &p)
    {
        bind(p);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<T>, 1> &p)
    {
        sc_port_base::bind(p);
    }
    void
    operator () (sc_port<sc_signal_inout_if<T>, 1> &p)
    {
        bind(p);
    }

    virtual void
    end_of_elaboration()
    {
        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const T &read() const { return (*this)->read(); }
    operator const T& () const { return (*this)->read(); }

    const sc_event &default_event() const { return (*this)->default_event(); }
    const sc_event &
    value_changed_event() const
    {
        return (*this)->value_changed_event();
    }
    bool event() const { return (*this)->event(); }
    sc_event_finder &value_changed() const { return _valueChangedFinder; }

    virtual const char *kind() const { return "sc_in"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    mutable sc_event_finder_t<sc_signal_in_if<T> > _valueChangedFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_in(const sc_in<T> &);
    sc_in<T> &operator = (const sc_in<T> &);
};

template <class T>
inline void
sc_trace(sc_trace_file *tf, const sc_in<T> &i, const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

template <>
class sc_in<bool> : public sc_port<sc_signal_in_if<bool>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<bool>, 1>(),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    explicit sc_in(const char *name) :
        sc_port<sc_signal_in_if<bool>, 1>(name),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    virtual ~sc_in() {}

    // Deprecated binding constructors.
    explicit sc_in(const sc_signal_in_if<bool> &interface) :
        sc_port<sc_signal_in_if<bool>, 1>(interface),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    sc_in(const char *name, const sc_signal_in_if<bool> &interface) :
        sc_port<sc_signal_in_if<bool>, 1>(name, interface),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    explicit sc_in(sc_port_b<sc_signal_in_if<bool> > &parent) :
        sc_port<sc_signal_in_if<bool>, 1>(parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    sc_in(const char *name, sc_port_b<sc_signal_in_if<bool> > &parent) :
        sc_port<sc_signal_in_if<bool>, 1>(name, parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    explicit sc_in(sc_port<sc_signal_in_if<bool>, 1> &parent) :
        sc_port<sc_signal_in_if<bool>, 1>(parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}
    sc_in(const char *name, sc_port<sc_signal_in_if<bool>, 1> &parent) :
        sc_port<sc_signal_in_if<bool>, 1>(name, parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<bool>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<bool>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<bool>::negedge_event)
    {}

    using sc_port<sc_signal_in_if<bool>, 1>::bind;

    virtual void
    bind(const sc_signal_in_if<bool> &i)
    {
        sc_port<sc_signal_in_if<bool>, 1>::bind(
                const_cast<sc_signal_in_if<bool> &>(i));
    }
    void operator () (const sc_signal_in_if<bool> &i) { bind(i); }

    virtual void
    bind(sc_port<sc_signal_in_if<bool>, 1> &p)
    {
        sc_port<sc_signal_in_if<bool>, 1>::bind(p);
    }
    void
    operator () (sc_port<sc_signal_in_if<bool>, 1> &p)
    {
        bind(p);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<bool>, 1> &p)
    {
        sc_port_base::bind(p);
    }
    void
    operator () (sc_port<sc_signal_inout_if<bool>, 1> &p)
    {
        bind(p);
    }

    virtual void
    end_of_elaboration()
    {
        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const bool &read() const { return (*this)->read(); }
    operator const bool& () const { return (*this)->read(); }

    const sc_event &default_event() const { return (*this)->default_event(); }
    const sc_event &
    value_changed_event() const
    {
        return (*this)->value_changed_event();
    }
    const sc_event &
    posedge_event() const
    {
        return (*this)->posedge_event();
    }
    const sc_event &
    negedge_event() const
    {
        return (*this)->negedge_event();
    }

    bool event() const { return (*this)->event(); }
    bool posedge() const { return (*this)->posedge(); }
    bool negedge() const { return (*this)->negedge(); }

    sc_event_finder &value_changed() const { return _valueChangedFinder; }
    sc_event_finder &pos() const { return _posFinder; }
    sc_event_finder &neg() const { return _negFinder; }

    virtual const char *kind() const { return "sc_in"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    mutable sc_event_finder_t<sc_signal_in_if<bool> > _valueChangedFinder;
    mutable sc_event_finder_t<sc_signal_in_if<bool> > _posFinder;
    mutable sc_event_finder_t<sc_signal_in_if<bool> > _negFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_in(const sc_in<bool> &);
    sc_in<bool> &operator = (const sc_in<bool> &);
};

template <>
inline void
sc_trace<bool>(sc_trace_file *tf, const sc_in<bool> &i,
        const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

template <>
class sc_in<sc_dt::sc_logic> :
    public sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>
{
  public:
    sc_in() : sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_in(const char *name) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(name),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    virtual ~sc_in() {}

    // Deprecated binding constructors.
    explicit sc_in(const sc_signal_in_if<sc_dt::sc_logic> &interface) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(interface),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_in(const char *name,
            const sc_signal_in_if<sc_dt::sc_logic> &interface) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(name, interface),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_in(sc_port_b<sc_signal_in_if<sc_dt::sc_logic> > &parent) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_in(const char *name,
            sc_port_b<sc_signal_in_if<sc_dt::sc_logic> > &parent) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(name, parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    explicit sc_in(sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &parent) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}
    sc_in(const char *name,
            sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &parent) :
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>(name, parent),
        _valueChangedFinder(*this,
                &sc_signal_in_if<sc_dt::sc_logic>::value_changed_event),
        _posFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::posedge_event),
        _negFinder(*this, &sc_signal_in_if<sc_dt::sc_logic>::negedge_event)
    {}

    using sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>::bind;

    virtual void
    bind(const sc_signal_in_if<sc_dt::sc_logic> &i)
    {
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>::bind(
                const_cast<sc_signal_in_if<sc_dt::sc_logic> &>(i));
    }
    void
    operator () (const sc_signal_in_if<sc_dt::sc_logic> &i) { bind(i); }

    virtual void
    bind(sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &i)
    {
        sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1>::bind(i);
    }
    void
    operator () (sc_port<sc_signal_in_if<sc_dt::sc_logic>, 1> &p)
    {
        bind(p);
    }

    virtual void
    bind(sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &p)
    {
        sc_port_base::bind(p);
    }
    void
    operator () (sc_port<sc_signal_inout_if<sc_dt::sc_logic>, 1> &p)
    {
        bind(p);
    }

    virtual void
    end_of_elaboration()
    {
        for (auto params: traceParamsVec)
            sc_trace(params->tf, (*this)->read(), params->name);

        traceParamsVec.clear();
    }

    const sc_dt::sc_logic &read() const { return (*this)->read(); }
    operator const sc_dt::sc_logic& () const { return (*this)->read(); }

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

    virtual const char *kind() const { return "sc_in"; }

    void
    add_trace(sc_trace_file *tf, const std::string &name) const
    {
        traceParamsVec.push_back(new sc_trace_params(tf, name));
    }

  private:
    mutable sc_event_finder_t<sc_signal_in_if<sc_dt::sc_logic> >
        _valueChangedFinder;
    mutable sc_event_finder_t<sc_signal_in_if<sc_dt::sc_logic> > _posFinder;
    mutable sc_event_finder_t<sc_signal_in_if<sc_dt::sc_logic> > _negFinder;

    mutable sc_trace_params_vec traceParamsVec;

    // Disabled
    sc_in(const sc_in<sc_dt::sc_logic> &);
    sc_in<sc_dt::sc_logic> &operator = (const sc_in<sc_dt::sc_logic> &);
};

template <>
inline void
sc_trace<sc_dt::sc_logic>(sc_trace_file *tf, const sc_in<sc_dt::sc_logic> &i,
        const std::string &name)
{
    if (::sc_core::sc_get_status() < ::sc_core::SC_START_OF_SIMULATION)
        i.add_trace(tf, name);
    else
        sc_trace(tf, i->read(), name);
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_IN_HH__
