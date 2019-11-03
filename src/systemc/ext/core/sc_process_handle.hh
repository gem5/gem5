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

#ifndef __SYSTEMC_EXT_CORE_SC_PROCESS_HANDLE_HH__
#define __SYSTEMC_EXT_CORE_SC_PROCESS_HANDLE_HH__

#include <exception>
#include <vector>

#include "../utils/sc_report_handler.hh"
#include "messages.hh"
#include "sc_object.hh"

namespace sc_gem5
{

class Process;

struct ProcessFuncWrapper
{
    virtual void call() = 0;
    virtual ~ProcessFuncWrapper() {}
};

template <typename T>
struct ProcessMemberFuncWrapper : public ProcessFuncWrapper
{
    typedef void (T::*TFunc)();
    T *t;
    TFunc func;

    ProcessMemberFuncWrapper(T *t, TFunc func) : t(t), func(func) {}

    void call() override { (t->*func)(); }
};

struct ExceptionWrapperBase
{
    virtual void throw_it() = 0;
};

template <typename T>
struct ExceptionWrapper : public ExceptionWrapperBase
{
    const T &t;
    ExceptionWrapper(const T &t) : t(t) {}

    void throw_it() { throw t; }
};

void throw_it_wrapper(Process *p, ExceptionWrapperBase &exc, bool inc_kids);

} // namespace sc_gem5

namespace sc_core
{

class sc_event;

enum sc_curr_proc_kind
{
    SC_NO_PROC_,
    SC_METHOD_PROC_,
    SC_THREAD_PROC_,
    SC_CTHREAD_PROC_
};

enum sc_descendent_inclusion_info
{
    SC_NO_DESCENDANTS,
    SC_INCLUDE_DESCENDANTS
};

class sc_unwind_exception : public std::exception
{
  public:
    virtual const char *what() const throw();
    virtual bool is_reset() const;

    // Nonstandard.
    // These should be protected, but I think this is to enable catch by
    // value.
  public:
    sc_unwind_exception(const sc_unwind_exception &);
    virtual ~sc_unwind_exception() throw();

  protected:
    bool _isReset;
    sc_unwind_exception();
};

// Deprecated
// An incomplete version of sc_process_b to satisfy the tests.
class sc_process_b : public sc_object
{
  public:
    sc_process_b(const char *name) : sc_object(name), file(nullptr), lineno(0)
    {}
    sc_process_b() : sc_object(), file(nullptr), lineno(0) {}

    const char *file;
    int lineno;
};

// Nonstandard
void sc_set_location(const char *file, int lineno);

// Deprecated
sc_process_b *sc_get_curr_process_handle();
static inline sc_process_b *
sc_get_current_process_b()
{
    return sc_get_curr_process_handle();
}

// Deprecated/nonstandard
struct sc_curr_proc_info
{
    sc_process_b *process_handle;
    sc_curr_proc_kind kind;
    sc_curr_proc_info() : process_handle(NULL), kind(SC_NO_PROC_) {}
};
typedef const sc_curr_proc_info *sc_curr_proc_handle;

class sc_process_handle
{
  private:
    ::sc_gem5::Process *_gem5_process;

  public:
    sc_process_handle();
    sc_process_handle(const sc_process_handle &);
    explicit sc_process_handle(sc_object *);
    ~sc_process_handle();

    // These non-standard operators provide access to the data structure which
    // actually tracks the process within gem5. By making them operators, we
    // can minimize the symbols added to the class namespace.
    operator ::sc_gem5::Process * () const { return _gem5_process; }
    sc_process_handle &
    operator = (::sc_gem5::Process *p)
    {
        _gem5_process = p;
        return *this;
    }

    bool valid() const;

    sc_process_handle &operator = (const sc_process_handle &);
    bool operator == (const sc_process_handle &) const;
    bool operator != (const sc_process_handle &) const;
    bool operator < (const sc_process_handle &) const;
    void swap(sc_process_handle &);

    const char *name() const;
    sc_curr_proc_kind proc_kind() const;
    const std::vector<sc_object *> &get_child_objects() const;
    const std::vector<sc_event *> &get_child_events() const;
    sc_object *get_parent_object() const;
    sc_object *get_process_object() const;
    bool dynamic() const;
    bool terminated() const;
    const sc_event &terminated_event() const;

    void suspend(sc_descendent_inclusion_info include_descendants=
                 SC_NO_DESCENDANTS);
    void resume(sc_descendent_inclusion_info include_descendants=
                SC_NO_DESCENDANTS);
    void disable(sc_descendent_inclusion_info include_descendants=
                 SC_NO_DESCENDANTS);
    void enable(sc_descendent_inclusion_info include_descendants=
                SC_NO_DESCENDANTS);
    void kill(sc_descendent_inclusion_info include_descendants=
              SC_NO_DESCENDANTS);
    void reset(sc_descendent_inclusion_info include_descendants=
               SC_NO_DESCENDANTS);
    bool is_unwinding();
    const sc_event &reset_event() const;

    void sync_reset_on(sc_descendent_inclusion_info include_descendants=
                       SC_NO_DESCENDANTS);
    void sync_reset_off(sc_descendent_inclusion_info include_descendants=
                        SC_NO_DESCENDANTS);

    template <typename T>
    void
    throw_it(const T &user_defined_exception,
             sc_descendent_inclusion_info include_descendants=
             SC_NO_DESCENDANTS)
    {
        if (!_gem5_process) {
            SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "throw_it()");
            return;
        }
        ::sc_gem5::ExceptionWrapper<T> exc(user_defined_exception);
        ::sc_gem5::throw_it_wrapper(_gem5_process, exc,
                include_descendants == SC_INCLUDE_DESCENDANTS);
    }
};

sc_process_handle sc_get_current_process_handle();
bool sc_is_unwinding();

// Nonstandard
// See Accellera's kernel/sim_context.cpp for an explanation of what this is
// supposed to do. It essentially selects what happens during certain
// undefined situations.
extern bool sc_allow_process_control_corners;

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_PROCESS_HANDLE_HH__
