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

#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/sc_mutex.hh"
#include "systemc/ext/core/sc_module.hh" // for sc_gen_unique_name

namespace sc_core
{

sc_mutex::sc_mutex()
    : sc_interface(), sc_mutex_if(), sc_object(sc_gen_unique_name("mutex"))
{}

sc_mutex::sc_mutex(const char *name)
    : sc_interface(), sc_mutex_if(), sc_object(name)
{}

int
sc_mutex::lock()
{
    while (trylock() == -1)
        wait(unlockEvent);
    return 0;
}

int
sc_mutex::trylock()
{
    if (holder.valid())
        return -1;
    holder = ::sc_gem5::scheduler.current();
    return 0;
}

int
sc_mutex::unlock()
{
    if (holder != ::sc_gem5::scheduler.current())
        return -1;

    holder = nullptr;
    unlockEvent.notify();
    return 0;
}

} // namespace sc_core
