/*
 * Copyright (c) 2010-2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 *          Chander Sudanthi
 */

#define m5_op 0xFF

#include "m5ops.h"

#define INST(op, ra, rb, func) \
        .long (((op) << 24) | ((func) << 16) | ((ra) << 12) | (0x1 << 8) | (0x1 << 4) | (rb))
/*               m5ops           m5func             ra         coproc 1       op=1       rb   */

#define LEAF(func)    \
        .globl  func; \
func:

#define RET           \
        RET

#define END(func)     \

#define SIMPLE_OP(_f, _o)       \
        LEAF(_f)                \
                _o;             \
                RET;            \
        END(_f)

#define ARM INST(m5_op, 0, 0, arm_func)
#define QUIESCE INST(m5_op, 0, 0, quiesce_func)
#define QUIESCENS INST(m5_op, 0, 0, quiescens_func)
#define QUIESCECYC INST(m5_op, 0, 0, quiescecycle_func)
#define QUIESCETIME INST(m5_op, 0, 0, quiescetime_func)
#define RPNS INST(m5_op, 0, 0, rpns_func)
#define WAKE_CPU INST(m5_op, 0, 0, wakecpu_func)
#define M5EXIT INST(m5_op, 0, 0, exit_func)
#define M5FAIL INST(m5_op, 0, 0, fail_func)
#define INITPARAM INST(m5_op, 0, 0, initparam_func)
#define LOADSYMBOL INST(m5_op, 0, 0, loadsymbol_func)
#define RESET_STATS INST(m5_op, 0, 0, resetstats_func)
#define DUMP_STATS INST(m5_op, 0, 0, dumpstats_func)
#define DUMPRST_STATS INST(m5_op, 0, 0, dumprststats_func)
#define CHECKPOINT INST(m5_op, 0, 0, ckpt_func)
#define READFILE INST(m5_op, 0, 0, readfile_func)
#define WRITEFILE INST(m5_op, 0, 0, writefile_func)
#define DEBUGBREAK INST(m5_op, 0, 0, debugbreak_func)
#define SWITCHCPU INST(m5_op, 0, 0, switchcpu_func)
#define ADDSYMBOL INST(m5_op, 0, 0, addsymbol_func)
#define PANIC INST(m5_op, 0, 0, panic_func)
#define WORK_BEGIN INST(m5_op, 0, 0, work_begin_func)
#define WORK_END INST(m5_op, 0, 0, work_end_func)

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

.text

SIMPLE_OP(arm, ARM)
SIMPLE_OP(quiesce, QUIESCE)
SIMPLE_OP(quiesceNs, QUIESCENS)
SIMPLE_OP(quiesceCycle, QUIESCECYC)
SIMPLE_OP(quiesceTime, QUIESCETIME)
SIMPLE_OP(rpns, RPNS)
SIMPLE_OP(wakeCPU, WAKE_CPU)
SIMPLE_OP(m5_exit, M5EXIT)
SIMPLE_OP(m5_fail, M5FAIL)
SIMPLE_OP(m5_initparam, INITPARAM)
SIMPLE_OP(m5_loadsymbol, LOADSYMBOL)
SIMPLE_OP(m5_reset_stats, RESET_STATS)
SIMPLE_OP(m5_dump_stats, DUMP_STATS)
SIMPLE_OP(m5_dumpreset_stats, DUMPRST_STATS)
SIMPLE_OP(m5_checkpoint, CHECKPOINT)
SIMPLE_OP(m5_readfile, READFILE)
SIMPLE_OP(m5_writefile, WRITEFILE)
SIMPLE_OP(m5_debugbreak, DEBUGBREAK)
SIMPLE_OP(m5_switchcpu, SWITCHCPU)
SIMPLE_OP(m5_addsymbol, ADDSYMBOL)
SIMPLE_OP(m5_panic, PANIC)
SIMPLE_OP(m5_work_begin, WORK_BEGIN)
SIMPLE_OP(m5_work_end, WORK_END)

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

