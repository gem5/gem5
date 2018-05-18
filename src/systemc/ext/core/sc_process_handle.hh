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

namespace sc_gem5
{

class Process;

} // namespace sc_gem5

namespace sc_core
{

class sc_event;
class sc_object;

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

  protected:
    sc_unwind_exception();
    sc_unwind_exception(const sc_unwind_exception &);
    virtual ~sc_unwind_exception() throw();
};

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
    bool swap(sc_process_handle &);

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

    void warn_unimpl(const char *func);
    template <typename T>
    void throw_it(const T &user_defined_exception,
                  sc_descendent_inclusion_info include_descendants=
                  SC_NO_DESCENDANTS)
    {
        warn_unimpl(__PRETTY_FUNCTION__);
    }
};

sc_process_handle sc_get_current_process_handle();
bool sc_is_unwinding();

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_PROCESS_HANDLE_HH__
