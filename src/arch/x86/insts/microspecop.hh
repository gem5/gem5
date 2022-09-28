/*
 * Copyright 2021 Google Inc.
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

#ifndef __ARCH_X86_INSTS_MICROSPECOP_HH__
#define __ARCH_X86_INSTS_MICROSPECOP_HH__

#include "arch/x86/insts/microop.hh"
#include "cpu/exec_context.hh"

namespace gem5
{

namespace X86ISA
{

class MicroHalt : public InstOperands<X86MicroopBase>
{
  public:
    MicroHalt(ExtMachInst mach_inst, const char *inst_mnem,
            uint64_t set_flags) :
        InstOperands<X86MicroopBase>(mach_inst, "halt", inst_mnem,
                set_flags | (1ULL << StaticInst::IsNonSpeculative) |
                            (1ULL << StaticInst::IsQuiesce),
                No_OpClass, {})
    {}

    Fault
    execute(ExecContext *xc, trace::InstRecord *) const override
    {
        xc->tcBase()->suspend();
        return NoFault;
    }
};

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_INSTS_MICROSPECOP_HH__
