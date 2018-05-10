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

#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "../core/sc_prim.hh"
#include "sc_signal_inout_if.hh"
#include "warn_unimpl.hh" // for warn_unimpl

namespace sc_core
{

class sc_port_base;

template <class T, sc_writer_policy WRITER_POLICY=SC_ONE_WRITER>
class sc_signal : public sc_signal_inout_if<T>,
                  public sc_prim_channel
{
  public:
    sc_signal() : sc_signal_inout_if<T>(),
                  sc_prim_channel(sc_gen_unique_name("signal"))
    {}
    explicit sc_signal(const char *name) : sc_signal_inout_if<T>(),
                                           sc_prim_channel(name)
    {}
    virtual ~sc_signal() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const T&
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }
    operator const T&() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const T *)nullptr;
    }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const T&)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_signal<T, WRITER_POLICY> &
    operator = (const T&)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }
    sc_signal<T, WRITER_POLICY> &
    operator = (const sc_signal<T, WRITER_POLICY> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }

    virtual void
    print(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual void
    dump(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

  private:
    // Disabled
    sc_signal(const sc_signal<T, WRITER_POLICY> &) :
            sc_signal_inout_if<T>(), sc_prim_channel("")
    {}
};

template <class T, sc_writer_policy WRITER_POLICY>
inline std::ostream &
operator << (std::ostream &os, const sc_signal<T, WRITER_POLICY> &)
{
    sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    return os;
}

template <sc_writer_policy WRITER_POLICY>
class sc_signal<bool, WRITER_POLICY> :
    public sc_signal_inout_if<bool>, public sc_prim_channel
{
  public:
    sc_signal()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    explicit sc_signal(const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual ~sc_signal()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const bool &
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const bool *)nullptr;
    }
    operator const bool &() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const bool *)nullptr;
    }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const bool &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_signal<bool, WRITER_POLICY> &
    operator = (const bool &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }
    sc_signal<bool, WRITER_POLICY> &
    operator = (const sc_signal<bool, WRITER_POLICY> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }

    virtual const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual const sc_event &
    posedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual const sc_event &
    negedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }

    virtual bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    virtual bool
    posedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    virtual bool
    negedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }

    virtual void
    print(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual void
    dump(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

  private:
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
    sc_signal()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    explicit sc_signal(const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual ~sc_signal()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual const sc_dt::sc_logic &
    read() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_dt::sc_logic *)nullptr;
    }
    operator const sc_dt::sc_logic &() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_dt::sc_logic *)nullptr;
    }

    virtual sc_writer_policy
    get_writer_policy() const
    {
        return WRITER_POLICY;
    }
    virtual void
    write(const sc_dt::sc_logic &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_dt::sc_logic &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }
    sc_signal<sc_dt::sc_logic, WRITER_POLICY> &
    operator = (const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }

    virtual const sc_event &
    default_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }

    virtual const sc_event &
    value_changed_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual const sc_event &
    posedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }
    virtual const sc_event &
    negedge_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(sc_event *)nullptr;
    }

    virtual bool
    event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    virtual bool
    posedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    virtual bool
    negedge() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }

    virtual void
    print(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual void
    dump(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual const char *kind() const { return "sc_signal"; }

  protected:
    virtual void
    update()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

  private:
    // Disabled
    sc_signal(const sc_signal<sc_dt::sc_logic, WRITER_POLICY> &) :
            sc_signal_inout_if<sc_dt::sc_logic>(), sc_prim_channel("")
    {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SIGNAL_HH__
