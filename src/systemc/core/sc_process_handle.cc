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

#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_process_handle.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

const char *
sc_unwind_exception::what() const throw()
{
    return _isReset ? "RESET" : "KILL";
}

bool
sc_unwind_exception::is_reset() const
{
    return _isReset;
}

sc_unwind_exception::sc_unwind_exception() : _isReset(false) {}

sc_unwind_exception::sc_unwind_exception(const sc_unwind_exception &e)
    : _isReset(e._isReset)
{}

sc_unwind_exception::~sc_unwind_exception() throw() {}

void
sc_set_location(const char *file, int lineno)
{
    sc_process_b *current = ::sc_gem5::scheduler.current();
    if (!current)
        return;
    current->file = file;
    current->lineno = lineno;
}

sc_process_b *
sc_get_curr_process_handle()
{
    return ::sc_gem5::scheduler.current();
}

sc_process_handle::sc_process_handle() : _gem5_process(nullptr) {}

sc_process_handle::sc_process_handle(const sc_process_handle &handle)
    : _gem5_process(handle._gem5_process)
{
    if (_gem5_process)
        _gem5_process->incref();
}

sc_process_handle::sc_process_handle(sc_object *obj)
    : _gem5_process(dynamic_cast<::sc_gem5::Process *>(obj))
{
    if (_gem5_process)
        _gem5_process->incref();
}

sc_process_handle::~sc_process_handle()
{
    if (_gem5_process)
        _gem5_process->decref();
}

bool
sc_process_handle::valid() const
{
    return _gem5_process != nullptr;
}

sc_process_handle &
sc_process_handle::operator=(const sc_process_handle &handle)
{
    if (_gem5_process)
        _gem5_process->decref();
    _gem5_process = handle._gem5_process;
    if (_gem5_process)
        _gem5_process->incref();
    return *this;
}

bool
sc_process_handle::operator==(const sc_process_handle &handle) const
{
    return _gem5_process && handle._gem5_process &&
           (_gem5_process == handle._gem5_process);
}

bool
sc_process_handle::operator!=(const sc_process_handle &handle) const
{
    return !(handle == *this);
}

bool
sc_process_handle::operator<(const sc_process_handle &other) const
{
    return _gem5_process < other._gem5_process;
}

void
sc_process_handle::swap(sc_process_handle &handle)
{
    ::sc_gem5::Process *temp = handle._gem5_process;
    handle._gem5_process = _gem5_process;
    _gem5_process = temp;
}

const char *
sc_process_handle::name() const
{
    return _gem5_process ? _gem5_process->name() : "";
}

sc_curr_proc_kind
sc_process_handle::proc_kind() const
{
    return _gem5_process ? _gem5_process->procKind() : SC_NO_PROC_;
}

const std::vector<sc_object *> &
sc_process_handle::get_child_objects() const
{
    static const std::vector<sc_object *> empty;
    return _gem5_process ? _gem5_process->get_child_objects() : empty;
}

const std::vector<sc_event *> &
sc_process_handle::get_child_events() const
{
    static const std::vector<sc_event *> empty;
    return _gem5_process ? _gem5_process->get_child_events() : empty;
}

sc_object *
sc_process_handle::get_parent_object() const
{
    return _gem5_process ? _gem5_process->get_parent_object() : nullptr;
}

sc_object *
sc_process_handle::get_process_object() const
{
    return _gem5_process;
}

bool
sc_process_handle::dynamic() const
{
    return _gem5_process ? _gem5_process->dynamic() : false;
}

bool
sc_process_handle::terminated() const
{
    return _gem5_process ? _gem5_process->terminated() : false;
}

const sc_event &
sc_process_handle::terminated_event() const
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "terminated_event()");
        static sc_gem5::InternalScEvent non_event;
        return non_event;
    }
    return _gem5_process->terminatedEvent();
}

void
sc_process_handle::suspend(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "suspend()");
        return;
    }
    _gem5_process->suspend(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::resume(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "resume()");
        return;
    }
    _gem5_process->resume(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::disable(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "disable()");
        return;
    }
    _gem5_process->disable(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::enable(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "enable()");
        return;
    }
    _gem5_process->enable(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::kill(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "kill()");
        return;
    }
    _gem5_process->kill(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::reset(sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "reset()");
        return;
    }
    _gem5_process->reset(include_descendants == SC_INCLUDE_DESCENDANTS);
}

bool
sc_process_handle::is_unwinding()
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "is_unwinding()");
        return false;
    }
    return _gem5_process->isUnwinding();
}

const sc_event &
sc_process_handle::reset_event() const
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "reset()");
        static sc_gem5::InternalScEvent non_event;
        return non_event;
    }
    return _gem5_process->resetEvent();
}

void
sc_process_handle::sync_reset_on(
    sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "sync_reset_on()");
        return;
    }
    _gem5_process->syncResetOn(include_descendants == SC_INCLUDE_DESCENDANTS);
}

void
sc_process_handle::sync_reset_off(
    sc_descendent_inclusion_info include_descendants)
{
    if (!_gem5_process) {
        SC_REPORT_WARNING(SC_ID_EMPTY_PROCESS_HANDLE_, "sync_reset_off()");
        return;
    }
    _gem5_process->syncResetOff(include_descendants == SC_INCLUDE_DESCENDANTS);
}

sc_process_handle
sc_get_current_process_handle()
{
    if (sc_is_running())
        return sc_process_handle(::sc_gem5::scheduler.current());
    else
        return sc_process_handle(::sc_gem5::Process::newest());
}

bool
sc_is_unwinding()
{
    return sc_get_current_process_handle().is_unwinding();
}

bool sc_allow_process_control_corners;

} // namespace sc_core
