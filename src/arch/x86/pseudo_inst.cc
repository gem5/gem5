/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#include "arch/x86/pseudo_inst.hh"

#include "arch/x86/fs_workload.hh"
#include "arch/x86/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "debug/PseudoInst.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/process.hh"

using namespace X86ISA;

namespace X86ISA {

/*
 * This function is executed when the simulation is executing the pagefault
 * handler in System Emulation mode.
 */
void
m5PageFault(ThreadContext *tc)
{
    DPRINTF(PseudoInst, "PseudoInst::m5PageFault()\n");

    Process *p = tc->getProcessPtr();
    if (!p->fixupFault(tc->readMiscReg(MISCREG_CR2))) {
        PortProxy &proxy = tc->getVirtProxy();
        // at this point we should have 6 values on the interrupt stack
        int size = 6;
        uint64_t is[size];
        // reading the interrupt handler stack
        proxy.readBlob(ISTVirtAddr + PageBytes - size * sizeof(uint64_t),
                       &is, sizeof(is));
        panic("Page fault at addr %#x\n\tInterrupt handler stack:\n"
                "\tss: %#x\n"
                "\trsp: %#x\n"
                "\trflags: %#x\n"
                "\tcs: %#x\n"
                "\trip: %#x\n"
                "\terr_code: %#x\n",
                tc->readMiscReg(MISCREG_CR2),
                is[5], is[4], is[3], is[2], is[1], is[0]);
   }
}

} // namespace X86ISA
