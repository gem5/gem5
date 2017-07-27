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

#include <gem5/asm/generic/m5ops.h>

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

#define ARM(reg) INST(m5_op, reg, 0, M5OP_ARM)
#define QUIESCE INST(m5_op, 0, 0, M5OP_QUIESCE)
#define QUIESCENS(r1) INST(m5_op, r1, 0, M5OP_QUIESCE_NS)
#define QUIESCECYC(r1) INST(m5_op, r1, 0, M5OP_QUIESCE_CYCLE)
#define QUIESCETIME INST(m5_op, 0, 0, M5OP_QUIESCE_TIME)
#define RPNS INST(m5_op, 0, 0, M5OP_RPNS)
#define WAKE_CPU(r1) INST(m5_op, r1, 0, M5OP_WAKE_CPU)
#define M5EXIT(reg) INST(m5_op, reg, 0, M5OP_EXIT)
#define INITPARAM(reg) INST(m5_op, reg, 0, M5OP_INIT_PARAM)
#define LOADSYMBOL(reg) INST(m5_op, reg, 0, M5OP_LOAD_SYMBOL)
#define RESET_STATS(r1, r2) INST(m5_op, r1, r2, M5OP_RESET_STATS)
#define DUMP_STATS(r1, r2) INST(m5_op, r1, r2, M5OP_DUMP_STATS)
#define DUMPRST_STATS(r1, r2) INST(m5_op, r1, r2, M5OP_DUMP_RESET_STATS)
#define CHECKPOINT(r1, r2) INST(m5_op, r1, r2, M5OP_CHECKPOINT)
#define READFILE INST(m5_op, 0, 0, M5OP_READ_FILE)
#define DEBUGBREAK INST(m5_op, 0, 0, M5OP_DEBUG_BREAK)
#define SWITCHCPU INST(m5_op, 0, 0, M5OP_SWITCH_CPU)
#define ADDSYMBOL(r1,r2) INST(m5_op, r1, r2, M5OP_ADD_SYMBOL)
#define PANIC INST(m5_op, 0, 0, M5OP_PANIC)

#define AN_BSM INST(m5_op, M5OP_AN_BSM, 0, M5OP_ANNOTATE)
#define AN_ESM INST(m5_op, M5OP_AN_ESM, 0, M5OP_ANNOTATE)
#define AN_BEGIN INST(m5_op, M5OP_AN_BEGIN, 0, M5OP_ANNOTATE)
#define AN_END INST(m5_op, M5OP_AN_END, 0, M5OP_ANNOTATE)
#define AN_Q INST(m5_op, M5OP_AN_Q, 0, M5OP_ANNOTATE)
#define AN_RQ INST(m5_op, M5OP_AN_RQ, 0, M5OP_ANNOTATE)
#define AN_DQ INST(m5_op, M5OP_AN_DQ, 0, M5OP_ANNOTATE)
#define AN_WF INST(m5_op, M5OP_AN_WF, 0, M5OP_ANNOTATE)
#define AN_WE INST(m5_op, M5OP_AN_WE, 0, M5OP_ANNOTATE)
#define AN_WS INST(m5_op, M5OP_AN_WS, 0, M5OP_ANNOTATE)
#define AN_SQ INST(m5_op, M5OP_AN_SQ, 0, M5OP_ANNOTATE)
#define AN_AQ INST(m5_op, M5OP_AN_AQ, 0, M5OP_ANNOTATE)
#define AN_PQ INST(m5_op, M5OP_AN_PQ, 0, M5OP_ANNOTATE)
#define AN_L INST(m5_op, M5OP_AN_L, 0, M5OP_ANNOTATE)
#define AN_IDENTIFY INST(m5_op, M5OP_AN_IDENTIFY, 0, M5OP_ANNOTATE)
#define AN_GETID INST(m5_op, M5OP_AN_GETID, 0, M5OP_ANNOTATE)


        .set noreorder

SIMPLE_OP(m5_arm, ARM(16))
SIMPLE_OP(m5_quiesce, QUIESCE)
SIMPLE_OP(m5_quiesce_ns, QUIESCENS(16))
SIMPLE_OP(m5_quiesce_cycle, QUIESCECYC(16))
SIMPLE_OP(m5_quiesce_time, QUIESCETIME)
SIMPLE_OP(m5_rpns, RPNS)
SIMPLE_OP(m5_wake_cpu, WAKE_CPU(16))
SIMPLE_OP(m5_exit, M5EXIT(16))
SIMPLE_OP(m5_init_param, INITPARAM(0))
SIMPLE_OP(m5_load_symbol, LOADSYMBOL(0))
SIMPLE_OP(m5_reset_stats, RESET_STATS(16, 17))
SIMPLE_OP(m5_dump_stats, DUMP_STATS(16, 17))
SIMPLE_OP(m5_dump_reset_stats, DUMPRST_STATS(16, 17))
SIMPLE_OP(m5_checkpoint, CHECKPOINT(16, 17))
SIMPLE_OP(m5_read_file, READFILE)
SIMPLE_OP(m5_debug_break, DEBUGBREAK)
SIMPLE_OP(m5_switch_cpu, SWITCHCPU)
SIMPLE_OP(m5_add_symbol, ADDSYMBOL(16, 17))
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

