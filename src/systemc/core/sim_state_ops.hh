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

#ifndef __SYSTEMC_CORE_SIM_STATE_OPS_HH__
#define __SYSTEMC_CORE_SIM_STATE_OPS_HH__

/*
 * Free functions over the kernel's global simulation state, used by the
 * sc_core API sources so that simulation-state queries do not pull in the
 * scheduler, kernel, event or module headers. Defined in core/sim_state.cc.
 */

namespace sc_gem5
{

// Scheduler::elaborationDone().
bool elaborationDone();
// Kernel::startOfSimulationComplete() / endOfSimulationComplete().
bool startOfSimulationComplete();
bool endOfSimulationComplete();
// A unique name for seed: from the parent module if there is one, else
// from the current process, else from the global name generator.
const char *genUniqueName(const char *seed);
// Whether an event or object with this hierarchical name exists.
bool hierarchicalNameExists(const char *name);

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SIM_STATE_OPS_HH__
