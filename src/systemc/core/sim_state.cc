/*
 * Copyright 2026 Google LLC
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

#include "systemc/core/sim_state_ops.hh"

#include "systemc/core/event.hh"
#include "systemc/core/kernel.hh"
#include "systemc/core/module.hh"
#include "systemc/core/module_ops.hh"
#include "systemc/core/object_ops.hh"
#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"

namespace sc_gem5
{

bool
elaborationDone()
{
    return scheduler().elaborationDone();
}

bool
startOfSimulationComplete()
{
    return Kernel::startOfSimulationComplete();
}

bool
endOfSimulationComplete()
{
    return Kernel::endOfSimulationComplete();
}

const char *
genUniqueName(const char *seed)
{
    auto mod = pickParentModule();
    if (mod) {
        return mod->uniqueName(seed);
    }

    Process *p = scheduler().current();
    if (p) {
        return p->uniqueName(seed);
    }

    return globalNameGen().gen(seed);
}

bool
hierarchicalNameExists(const char *name)
{
    return findEvent(name) || findObject(name);
}

} // namespace sc_gem5
