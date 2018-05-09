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

#include "base/logging.hh"
#include "systemc/ext/core/sc_process_handle.hh"

namespace sc_core
{

const char *
sc_unwind_exception::what() const throw()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_unwind_exception::is_reset() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

sc_unwind_exception::sc_unwind_exception()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_unwind_exception::sc_unwind_exception(const sc_unwind_exception &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_unwind_exception::~sc_unwind_exception() throw()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


sc_process_handle::sc_process_handle()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_process_handle::sc_process_handle(const sc_process_handle &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_process_handle::sc_process_handle(sc_object *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_process_handle::~sc_process_handle()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


bool
sc_process_handle::valid() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}


sc_process_handle &
sc_process_handle::operator = (const sc_process_handle &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

bool
sc_process_handle::operator == (const sc_process_handle &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return true;
}

bool
sc_process_handle::operator != (const sc_process_handle &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_process_handle::operator < (const sc_process_handle &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_process_handle::swap(sc_process_handle &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}


const char *
sc_process_handle::name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

sc_curr_proc_kind
sc_process_handle::proc_kind() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return SC_NO_PROC_;
}

const std::vector<sc_object *> &
sc_process_handle::get_child_objects() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_object *> *)nullptr;
}

const std::vector<sc_event *> &
sc_process_handle::get_child_events() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_event *> *)nullptr;
}

sc_object *
sc_process_handle::get_parent_object() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (sc_object *)nullptr;
}

sc_object *
sc_process_handle::get_process_object() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (sc_object *)nullptr;
}

bool
sc_process_handle::dynamic() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_process_handle::terminated() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

const sc_event &
sc_process_handle::terminated_event() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_event *)nullptr;
}


void
sc_process_handle::suspend(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::resume(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::disable(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::enable(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::kill(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::reset(sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

bool
sc_process_handle::is_unwinding()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

const sc_event &
sc_process_handle::reset_event() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_event *)nullptr;
}


void
sc_process_handle::sync_reset_on(
        sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::sync_reset_off(
        sc_descendent_inclusion_info include_descendants)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_process_handle::warn_unimpl(const char *func)
{
    warn("%s not implemented.\n", func);
}


sc_process_handle
sc_get_current_process_handle()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_process_handle();
}

bool
sc_is_unwinding()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

} // namespace sc_core
