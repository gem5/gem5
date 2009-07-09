/*
 * Copyright (c) 2007-2008 The Florida State University
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
 *
 * Authors: Stephen Hines
 */

#ifndef __ARCH_ARM_REGFILE_INT_REGFILE_HH__
#define __ARCH_ARM_REGFILE_INT_REGFILE_HH__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/types.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "sim/faults.hh"
#include "sim/serialize.hh"

class Checkpoint;
class ThreadContext;

namespace ArmISA
{
    enum MiscIntRegNums {
        zero_reg = NumIntArchRegs,
        addr_reg,

        rhi,
        rlo,

        r8_fiq,    /* FIQ mode register bank */
        r9_fiq,
        r10_fiq,
        r11_fiq,
        r12_fiq,

        r13_fiq,   /* FIQ mode SP and LR */
        r14_fiq,

        r13_irq,   /* IRQ mode SP and LR */
        r14_irq,

        r13_svc,   /* SVC mode SP and LR */
        r14_svc,

        r13_undef, /* UNDEF mode SP and LR */
        r14_undef,

        r13_abt,   /* ABT mode SP and LR */
        r14_abt
    };

} // namespace ArmISA

#endif
