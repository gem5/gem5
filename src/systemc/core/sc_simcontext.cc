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

#include "systemc/core/object.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_simcontext.hh"

namespace sc_core
{

namespace
{

size_t objIndex = 0;
sc_simcontext currContext;

sc_curr_proc_info currProcInfo;

} // anonymous namespace

sc_dt::uint64 sc_simcontext::delta_count() const { return sc_delta_count(); }
void sc_simcontext::reset() { objIndex = 0; }

sc_curr_proc_handle
sc_simcontext::get_curr_proc_info()
{
    ::sc_gem5::Process *p = ::sc_gem5::scheduler.current();
    currProcInfo.process_handle = p;
    currProcInfo.kind = p ? p->procKind() : SC_NO_PROC_;
    return &currProcInfo;
}

sc_object *
sc_simcontext::first_object()
{
    objIndex = 0;
    if (!::sc_gem5::allObjects.empty())
        return ::sc_gem5::allObjects[0];
    else
        return nullptr;
}

sc_object *
sc_simcontext::next_object()
{
    objIndex++;
    if (::sc_gem5::allObjects.size() > objIndex)
        return ::sc_gem5::allObjects[objIndex];
    else
        return nullptr;
}

bool
sc_simcontext::elaboration_done()
{
    return ::sc_gem5::scheduler.elaborationDone();
}

sc_simcontext *
sc_get_curr_simcontext()
{
    return &currContext;
}

} // namespace sc_core
