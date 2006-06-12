/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Ali Saidi
 */

#define m5_op 0x01

#define arm_func 0x00
#define quiesce_func 0x01
#define quiescens_func 0x02
#define quiescecycle_func 0x03
#define quiescetime_func 0x04
#define ivlb_func 0x10
#define ivle_func 0x11
#define exit_old_func 0x20 // deprectated!
#define exit_func 0x21
#define initparam_func 0x30
#define resetstats_func 0x40
#define dumpstats_func 0x41
#define dumprststats_func 0x42
#define ckpt_func 0x43
#define readfile_func 0x50
#define debugbreak_func 0x51
#define switchcpu_func 0x52
#define addsymbol_func 0x53
#define panic_func     0x54

#define INST(op, ra, rb, func) \
        .long (((op) << 26) | ((ra) << 21) | ((rb) << 16) | (func))

#define LEAF(func)    \
        .align 3;     \
        .globl  func; \
        .ent    func; \
func:

#define RET           \
        ret     ($26)

#define END(func)     \
        .end func

#define	ARM(reg) INST(m5_op, reg, 0, arm_func)
#define QUIESCE INST(m5_op, 0, 0, quiesce_func)
#define QUIESCENS(r1) INST(m5_op, r1, 0, quiescens_func)
#define QUIESCECYC(r1) INST(m5_op, r1, 0, quiescecycle_func)
#define QUIESCETIME INST(m5_op, 0, 0, quiescetime_func)
#define IVLB(reg) INST(m5_op, reg, 0, ivlb_func)
#define IVLE(reg) INST(m5_op, reg, 0, ivle_func)
#define M5EXIT(reg) INST(m5_op, reg, 0, exit_func)
#define INITPARAM(reg) INST(m5_op, reg, 0, initparam_func)
#define RESET_STATS(r1, r2) INST(m5_op, r1, r2, resetstats_func)
#define DUMP_STATS(r1, r2) INST(m5_op, r1, r2, dumpstats_func)
#define DUMPRST_STATS(r1, r2) INST(m5_op, r1, r2, dumprststats_func)
#define CHECKPOINT(r1, r2) INST(m5_op, r1, r2, ckpt_func)
#define READFILE INST(m5_op, 0, 0, readfile_func)
#define DEBUGBREAK INST(m5_op, 0, 0, debugbreak_func)
#define SWITCHCPU INST(m5_op, 0, 0, switchcpu_func)
#define ADDSYMBOL(r1,r2) INST(m5_op, r1, r2, addsymbol_func)
#define PANIC INST(m5_op, 0, 0, panic_func)

        .set noreorder

        .align 4
LEAF(arm)
        ARM(16)
        RET
END(arm)

        .align 4
LEAF(quiesce)
        QUIESCE
        RET
END(quiesce)

        .align 4
LEAF(quiesceNs)
        QUIESCENS(16)
        RET
END(quiesceNs)

        .align 4
LEAF(quiesceCycle)
        QUIESCECYC(16)
        RET
END(quiesceCycle)

        .align 4
LEAF(quiesceTime)
        QUIESCETIME
        RET
END(quiesceTime)


        .align 4
LEAF(m5_ivlb)
        IVLB(16)
        RET
END(m5_ivlb)

        .align 4
LEAF(m5_ivle)
        IVLE(16)
        RET
END(m5_ivle)

        .align 4
LEAF(m5_exit)
        M5EXIT(16)
        RET
END(m5_exit)

        .align 4
LEAF(m5_initparam)
        INITPARAM(0)
        RET
END(m5_initparam)

        .align 4
LEAF(m5_reset_stats)
        RESET_STATS(16, 17)
        RET
END(m5_reset_stats)

        .align 4
LEAF(m5_dump_stats)
        DUMP_STATS(16, 17)
        RET
END(m5_dump_stats)

        .align 4
LEAF(m5_dumpreset_stats)
        DUMPRST_STATS(16, 17)
        RET
END(m5_dumpreset_stats)

        .align 4
LEAF(m5_checkpoint)
        CHECKPOINT(16, 17)
        RET
END(m5_checkpoint)

        .align 4
LEAF(m5_readfile)
        READFILE
        RET
END(m5_readfile)

        .align 4
LEAF(m5_debugbreak)
        DEBUGBREAK
        RET
END(m5_debugbreak)

        .align 4
LEAF(m5_switchcpu)
        SWITCHCPU
        RET
END(m5_switchcpu)

        .align 4
LEAF(m5_addsymbol)
        ADDSYMBOL(16, 17)
        RET
END(m5_addsymbol)

        .align 4
LEAF(m5_panic)
        PANIC
        RET
END(m5_panic)


