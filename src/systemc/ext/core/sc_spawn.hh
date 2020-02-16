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

#ifndef __SYSTEMC_EXT_CORE_SC_SPAWN_HH__
#define __SYSTEMC_EXT_CORE_SC_SPAWN_HH__

#include <functional>
#include <vector>

#include "sc_join.hh"
#include "sc_process_handle.hh"

namespace sc_core
{

class sc_spawn_options;

} // namespace sc_core

namespace sc_gem5
{

class Process;

template <typename T>
struct ProcessObjFuncWrapper : public ProcessFuncWrapper
{
    T t;

    ProcessObjFuncWrapper(T t) : t(t) {}

    void call() override { t(); }
};

template <typename T, typename R>
struct ProcessObjRetFuncWrapper : public ProcessFuncWrapper
{
    T t;
    R *r;

    ProcessObjRetFuncWrapper(T t, R *r) : t(t), r(r) {}

    void call() override { *r = t(); }
};

Process *spawnWork(ProcessFuncWrapper *func, const char *name,
                   const ::sc_core::sc_spawn_options *);

} // namespace sc_gem5

namespace sc_core
{

template <class T>
class sc_in;
template <class T>
class sc_inout;
template <class T>
class sc_out;
template <class T>
class sc_signal_in_if;

class sc_event;
class sc_event_finder;
class sc_export_base;
class sc_interface;
class sc_port_base;

class sc_spawn_options
{
  public:
    friend ::sc_gem5::Process *::sc_gem5::spawnWork(
            ::sc_gem5::ProcessFuncWrapper *, const char *,
            const sc_spawn_options *);

    sc_spawn_options();

    void spawn_method();
    void dont_initialize();
    void set_stack_size(int);

    void set_sensitivity(const sc_event *);
    void set_sensitivity(sc_port_base *);
    void set_sensitivity(sc_export_base *);
    void set_sensitivity(sc_interface *);
    void set_sensitivity(sc_event_finder *);

    void reset_signal_is(const sc_in<bool> &, bool);
    void reset_signal_is(const sc_inout<bool> &, bool);
    void reset_signal_is(const sc_out<bool> &, bool);
    void reset_signal_is(const sc_signal_in_if<bool> &, bool);

    void async_reset_signal_is(const sc_in<bool> &, bool);
    void async_reset_signal_is(const sc_inout<bool> &, bool);
    void async_reset_signal_is(const sc_out<bool> &, bool);
    void async_reset_signal_is(const sc_signal_in_if<bool> &, bool);

  private:
    bool _spawnMethod;
    bool _dontInitialize;
    int _stackSize;
    std::vector<const sc_event *> _events;
    std::vector<sc_port_base *> _ports;
    std::vector<sc_export_base *> _exports;
    std::vector<sc_interface *> _interfaces;
    std::vector<sc_event_finder *> _finders;

    template <typename T>
    struct Reset
    {
        Reset(T *t, bool v, bool s) : target(t), value(v), sync(s) {}

        T *target;
        bool value;
        bool sync;
    };

    std::vector<Reset<const sc_in<bool> > > _in_resets;
    std::vector<Reset<const sc_inout<bool> > > _inout_resets;
    std::vector<Reset<const sc_out<bool> > > _out_resets;
    std::vector<Reset<const sc_signal_in_if<bool> > > _if_resets;

    // Disabled
    sc_spawn_options(const sc_spawn_options &) {}
    sc_spawn_options &operator = (const sc_spawn_options &) { return *this; }
};

template <typename T>
sc_process_handle
sc_spawn(T object, const char *name_p=nullptr,
         const sc_spawn_options *opt_p=nullptr)
{
    auto func = new ::sc_gem5::ProcessObjFuncWrapper<T>(object);
    ::sc_gem5::Process *p = spawnWork(func, name_p, opt_p);
    return sc_process_handle() = p;
}

template <typename T>
sc_process_handle
sc_spawn(typename T::result_type *r_p, T object, const char *name_p=nullptr,
         const sc_spawn_options *opt_p=nullptr)
{
    auto func = new ::sc_gem5::ProcessObjRetFuncWrapper<
        T, typename T::result_type>(object, r_p);
    ::sc_gem5::Process *p = spawnWork(func, name_p, opt_p);
    return sc_process_handle() = p;
}

#define SC_FORK \
{ \
    ::sc_core::sc_process_handle forkees[] = {

#define SC_JOIN \
    }; \
    ::sc_core::sc_join join; \
    for (int i = 0; i < sizeof(forkees) / sizeof(forkees[0]); i++) \
        join.add_process(forkees[i]); \
    join.wait(); \
}

// Non-standard
#define SC_CJOIN \
    }; \
    ::sc_core::sc_join join; \
    for (int i = 0; i < sizeof(forkees) / sizeof(forkees[0]); i++) \
        join.add_process(forkees[i]); \
    join.wait_clocked(); \
}

// This avoids boost introduces a dependency on c++11. If that's a problem,
// we could imitate Accellera and pick which one to use on the fly.

template <typename F, typename... Args>
auto sc_bind(F &&f, Args && ...args) ->
    decltype(std::bind(std::forward<F>(f), std::forward<Args>(args)...))
{
    return std::bind(std::forward<F>(f), std::forward<Args>(args)...);
}

template <typename T>
auto sc_ref(T &&v) -> decltype(std::ref(std::forward<T>(v)))
{
    return std::ref(std::forward<T>(v));
}

template <typename T>
auto sc_cref(T &&v) -> decltype(std::cref(std::forward<T>(v)))
{
    return std::cref(std::forward<T>(v));
}

} // namespace sc_core

using sc_core::sc_bind;
using sc_core::sc_ref;
using sc_core::sc_cref;

namespace sc_unnamed
{

using namespace std::placeholders;

} // namespace sc_unnamed

#endif  //__SYSTEMC_EXT_CORE_SC_SPAWN_HH__
