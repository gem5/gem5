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

#include "m5ops.h"

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

#define SIMPLE_OP(_f, _o)       \
        LEAF(_f)                \
                _o;             \
                RET;            \
        END(_f)

#define ARM(reg) INST(m5_op, reg, 0, arm_func)
#define QUIESCE INST(m5_op, 0, 0, quiesce_func)
#define QUIESCENS(r1) INST(m5_op, r1, 0, quiescens_func)
#define QUIESCECYC(r1) INST(m5_op, r1, 0, quiescecycle_func)
#define QUIESCETIME INST(m5_op, 0, 0, quiescetime_func)
#define RPNS INST(m5_op, 0, 0, rpns_func)
#define WAKE_CPU(r1) INST(m5_op, r1, 0, wakecpu_func)
#define M5EXIT(reg) INST(m5_op, reg, 0, exit_func)
#define INITPARAM(reg) INST(m5_op, reg, 0, initparam_func)
#define LOADSYMBOL(reg) INST(m5_op, reg, 0, loadsymbol_func)
#define RESET_STATS(r1, r2) INST(m5_op, r1, r2, resetstats_func)
#define DUMP_STATS(r1, r2) INST(m5_op, r1, r2, dumpstats_func)
#define DUMPRST_STATS(r1, r2) INST(m5_op, r1, r2, dumprststats_func)
#define CHECKPOINT(r1, r2) INST(m5_op, r1, r2, ckpt_func)
#define READFILE INST(m5_op, 0, 0, readfile_func)
#define DEBUGBREAK INST(m5_op, 0, 0, debugbreak_func)
#define SWITCHCPU INST(m5_op, 0, 0, switchcpu_func)
#define ADDSYMBOL(r1,r2) INST(m5_op, r1, r2, addsymbol_func)
#define PANIC INST(m5_op, 0, 0, panic_func)

#define AN_BSM INST(m5_op, an_bsm, 0, annotate_func)
#define AN_ESM INST(m5_op, an_esm, 0, annotate_func)
#define AN_BEGIN INST(m5_op, an_begin, 0, annotate_func)
#define AN_END INST(m5_op, an_end, 0, annotate_func)
#define AN_Q INST(m5_op, an_q, 0, annotate_func)
#define AN_RQ INST(m5_op, an_rq, 0, annotate_func)
#define AN_DQ INST(m5_op, an_dq, 0, annotate_func)
#define AN_WF INST(m5_op, an_wf, 0, annotate_func)
#define AN_WE INST(m5_op, an_we, 0, annotate_func)
#define AN_WS INST(m5_op, an_ws, 0, annotate_func)
#define AN_SQ INST(m5_op, an_sq, 0, annotate_func)
#define AN_AQ INST(m5_op, an_aq, 0, annotate_func)
#define AN_PQ INST(m5_op, an_pq, 0, annotate_func)
#define AN_L INST(m5_op, an_l, 0, annotate_func)
#define AN_IDENTIFY INST(m5_op, an_identify, 0, annotate_func)
#define AN_GETID INST(m5_op, an_getid, 0, annotate_func)


        .set noreorder

SIMPLE_OP(arm, ARM(16))
SIMPLE_OP(quiesce, QUIESCE)
SIMPLE_OP(quiesceNs, QUIESCENS(16))
SIMPLE_OP(quiesceCycle, QUIESCECYC(16))
SIMPLE_OP(quiesceTime, QUIESCETIME)
SIMPLE_OP(rpns, RPNS)
SIMPLE_OP(wakeCPU, WAKE_CPU(16))
SIMPLE_OP(m5_exit, M5EXIT(16))
SIMPLE_OP(m5_initparam, INITPARAM(0))
SIMPLE_OP(m5_loadsymbol, LOADSYMBOL(0))
SIMPLE_OP(m5_reset_stats, RESET_STATS(16, 17))
SIMPLE_OP(m5_dump_stats, DUMP_STATS(16, 17))
SIMPLE_OP(m5_dumpreset_stats, DUMPRST_STATS(16, 17))
SIMPLE_OP(m5_checkpoint, CHECKPOINT(16, 17))
SIMPLE_OP(m5_readfile, READFILE)
SIMPLE_OP(m5_debugbreak, DEBUGBREAK)
SIMPLE_OP(m5_switchcpu, SWITCHCPU)
SIMPLE_OP(m5_addsymbol, ADDSYMBOL(16, 17))
SIMPLE_OP(m5_panic, PANIC)

SIMPLE_OP(m5a_bsm, AN_BSM)
SIMPLE_OP(m5a_esm, AN_ESM)
SIMPLE_OP(m5a_begin, AN_BEGIN)
SIMPLE_OP(m5a_end, AN_END)
SIMPLE_OP(m5a_q, AN_Q)
SIMPLE_OP(m5a_rq, AN_RQ)
SIMPLE_OP(m5a_dq, AN_DQ)
SIMPLE_OP(m5a_wf, AN_WF)
SIMPLE_OP(m5a_we, AN_WE)
SIMPLE_OP(m5a_ws, AN_WS)
SIMPLE_OP(m5a_sq, AN_SQ)
SIMPLE_OP(m5a_aq, AN_AQ)
SIMPLE_OP(m5a_pq, AN_PQ)
SIMPLE_OP(m5a_l, AN_L)
SIMPLE_OP(m5a_identify, AN_IDENTIFY)
SIMPLE_OP(m5a_getid, AN_GETID)

