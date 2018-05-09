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

#ifndef __SYSTEMC_EXT_CORE_SC_SPAWN_HH__
#define __SYSTEMC_EXT_CORE_SC_SPAWN_HH__

#include "sc_process_handle.hh"

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
    // Disabled
    sc_spawn_options(const sc_spawn_options &) {}
    sc_spawn_options &operator = (const sc_spawn_options &) { return *this; }
};

void sc_spawn_warn_unimpl(const char *func);

template <typename T>
sc_process_handle
sc_spawn(T object, const char *name_p=nullptr,
         const sc_spawn_options *opt_p=nullptr)
{
    sc_spawn_warn_unimpl(__PRETTY_FUNCTION__);
    return sc_process_handle();
}

template <typename T>
sc_process_handle
sc_spawn(typename T::result_type *r_p, T object, const char *name_p=nullptr,
         const sc_spawn_options *opt_p=nullptr)
{
    sc_spawn_warn_unimpl(__PRETTY_FUNCTION__);
    return sc_process_handle();
}

#define sc_bind boost::bind
#define sc_ref(r) boost::ref(r)
#define sc_cref(r) boost::cref(r)

#define SC_FORK /* Implementation defined */
#define SC_JOIN /* Implementation defined */

} // namespace sc_core

namespace sc_unnamed
{

typedef int ImplementationDefined;
extern ImplementationDefined _1;
extern ImplementationDefined _2;
extern ImplementationDefined _3;
extern ImplementationDefined _4;
extern ImplementationDefined _5;
extern ImplementationDefined _6;
extern ImplementationDefined _7;
extern ImplementationDefined _8;
extern ImplementationDefined _9;

} // namespace sc_unnamed

#endif  //__SYSTEMC_EXT_CORE_SC_SPAWN_HH__
