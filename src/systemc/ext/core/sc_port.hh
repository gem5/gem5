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

#ifndef __SYSTEMC_EXT_CORE_SC_PORT_HH__
#define __SYSTEMC_EXT_CORE_SC_PORT_HH__

#include <typeinfo>
#include <vector>

#include "../channel/messages.hh"
#include "../utils/sc_report_handler.hh"
#include "sc_module.hh" // for sc_gen_unique_name
#include "sc_object.hh"

namespace sc_gem5
{

class Port;

};

namespace sc_core
{

class sc_interface;
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

enum sc_port_policy
{
    SC_ONE_OR_MORE_BOUND, // Default
    SC_ZERO_OR_MORE_BOUND,
    SC_ALL_BOUND
};

class sc_port_base : public sc_object
{
  public:
    sc_port_base(const char *name, int n, sc_port_policy p);
    virtual ~sc_port_base();

    void warn_port_constructor() const;

    int maxSize() const;
    int size() const;

    const char *kind() const { return "sc_port_base"; }

  protected:
    // Implementation defined, but depended on by the tests.
    void bind(sc_interface &);
    void bind(sc_port_base &);

    friend class ::sc_gem5::Module;

    // Implementation defined, but depended on by the tests.
    virtual int vbind(sc_interface &) = 0;
    virtual int vbind(sc_port_base &) = 0;

    virtual void before_end_of_elaboration() = 0;
    virtual void end_of_elaboration() = 0;
    virtual void start_of_simulation() = 0;
    virtual void end_of_simulation() = 0;

    void report_error(const char *id, const char *add_msg) const;

  private:
    friend class ::sc_gem5::Port;
    friend class ::sc_gem5::Kernel;

    virtual sc_interface *_gem5Interface(int n) const = 0;
    virtual void _gem5AddInterface(sc_interface *i) = 0;

    ::sc_gem5::Port *_gem5Port;
    virtual const char *_ifTypeName() const = 0;
    virtual sc_port_policy _portPolicy() const = 0;
};

template <class IF>
class sc_port_b : public sc_port_base
{
  public:
#pragma GCC diagnostic push
/**
 * The following warning is disabled because the bind methods are overloaded
 * in the derived class and the base class. In GCC v13+ this
 * 'overloaded-virtual' warning is strict enough to trigger here (though the
 * code is correct).
 * Please check section 9.3 of SystemC 2.3.1 release note for more details.
 */
#if defined(__GNUC__) && (__GNUC__ >= 13)
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#endif
    void operator () (IF &i) { bind(i); }
    void operator () (sc_port_b<IF> &p) { bind(p); }

    virtual void bind(IF &i) { sc_port_base::bind(i); }
    virtual void bind(sc_port_b<IF> &p) { sc_port_base::bind(p); }
#pragma GCC diagnostic pop

    IF *
    operator -> ()
    {
        if (_interfaces.empty()) {
            report_error(SC_ID_GET_IF_, "port is not bound");
            sc_abort();
        }
        return _interfaces[0];
    }
    const IF *
    operator -> () const
    {
        if (_interfaces.empty()) {
            report_error(SC_ID_GET_IF_, "port is not bound");
            sc_abort();
        }
        return _interfaces[0];
    }

    IF *
    operator [] (int n)
    {
        if (n < 0 || n >= size()) {
            report_error(SC_ID_GET_IF_, "index out of range");
            return NULL;
        }
        return _interfaces[n];
    }
    const IF *
    operator [] (int n) const
    {
        if (n < 0 || n >= size()) {
            report_error(SC_ID_GET_IF_, "index out of range");
            return NULL;
        }
        return _interfaces[n];
    }

    sc_interface *
    get_interface()
    {
        if (_interfaces.empty())
            return NULL;
        return _interfaces[0];
    }
    const sc_interface *
    get_interface() const
    {
        if (_interfaces.empty())
            return NULL;
        return _interfaces[0];
    }

  protected:
    void before_end_of_elaboration() override {}
    void end_of_elaboration() override {}
    void start_of_simulation() override {}
    void end_of_simulation() override {}

    explicit sc_port_b(int n, sc_port_policy p) :
            sc_port_base(sc_gen_unique_name("port"), n, p)
    {}
    sc_port_b(const char *name, int n, sc_port_policy p) :
            sc_port_base(name, n, p)
    {}
    virtual ~sc_port_b() {}

    // Implementation defined, but depended on by the tests.
    int
    vbind(sc_interface &i) override
    {
        IF *interface = dynamic_cast<IF *>(&i);
        if (!interface)
            return 2;
        sc_port_base::bind(*interface);
        return 0;
    }
    int
    vbind(sc_port_base &pb) override
    {
        sc_port_b<IF> *p = dynamic_cast<sc_port_b<IF> *>(&pb);
        if (!p)
            return 2;
        sc_port_base::bind(*p);
        return 0;
    }

  private:
    std::vector<IF *> _interfaces;

    sc_interface *
    _gem5Interface(int n) const override
    {
        if (n < 0 || n >= size()) {
            report_error(SC_ID_GET_IF_, "index out of range");
            return NULL;
        }
        return _interfaces[n];
    }
    void
    _gem5AddInterface(sc_interface *iface) override
    {
        IF *interface = dynamic_cast<IF *>(iface);
        sc_assert(interface);
        for (unsigned i = 0; i < _interfaces.size(); i++) {
            if (interface == _interfaces[i]) {
                report_error(SC_ID_BIND_IF_TO_PORT_,
                        "interface already bound to port");
            }
        }
        _interfaces.push_back(interface);
    }

    const char *_ifTypeName() const override { return typeid(IF).name(); }

    // Disabled
    sc_port_b() {}
    sc_port_b(const sc_port_b<IF> &) {}
    sc_port_b<IF> &operator = (const sc_port_b<IF> &) { return *this; }
};

template <class IF, int N=1, sc_port_policy P=SC_ONE_OR_MORE_BOUND>
class sc_port : public sc_port_b<IF>
{
  public:
    sc_port() : sc_port_b<IF>(N, P) {}
    explicit sc_port(const char *name) : sc_port_b<IF>(name, N, P) {}
    virtual ~sc_port() {}

    // Deprecated binding constructors.
    explicit sc_port(const IF &interface) : sc_port_b<IF>(N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(const_cast<IF &>(interface));
    }
    sc_port(const char *name, const IF &interface) : sc_port_b<IF>(name, N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(const_cast<IF &>(interface));
    }
    explicit sc_port(sc_port_b<IF> &parent) : sc_port_b<IF>(N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(parent);
    }
    sc_port(const char *name, sc_port_b<IF> &parent) :
        sc_port_b<IF>(name, N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(parent);
    }
    explicit sc_port(sc_port<IF, N, P> &parent) : sc_port_b<IF>(N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(parent);
    }
    sc_port(const char *name, sc_port<IF, N, P> &parent) :
        sc_port_b<IF>(name, N, P)
    {
        this->warn_port_constructor();
        sc_port_b<IF>::bind(parent);
    }

    virtual const char *kind() const override { return "sc_port"; }

  private:
    // Disabled
    sc_port(const sc_port<IF, N, P> &) {}
    sc_port<IF, N, P> &operator = (const sc_port<IF, N, P> &) { return *this; }

    virtual sc_port_policy _portPolicy() const override { return P; }
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_PORT_HH__
