/*
 * Copyright 2022 Google Inc.
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

#include "arch/x86/bare_metal/workload.hh"

#include "arch/x86/faults.hh"
#include "arch/x86/pcstate.hh"
#include "cpu/thread_context.hh"
#include "params/X86BareMetalWorkload.hh"
#include "sim/system.hh"

namespace gem5
{

namespace X86ISA
{

BareMetalWorkload::BareMetalWorkload(const Params &p) : Workload(p)
{}

void
BareMetalWorkload::initState()
{
    Workload::initState();

    for (auto *tc: system->threads) {
        X86ISA::InitInterrupt(0).invoke(tc);

        if (tc->contextId() == 0) {
            PCState pc = tc->pcState().as<PCState>();
            // Don't start in the microcode ROM which would halt this CPU.
            pc.upc(0);
            pc.nupc(1);
            tc->pcState(pc);
            tc->activate();
        } else {
            // This is an application processor (AP). It should be initialized
            // to look like only the BIOS POST has run on it and put then put
            // it into a halted state.
            tc->suspend();
        }
    }
}

} // namespace X86ISA

} // namespace gem5
