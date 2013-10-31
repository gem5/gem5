/*
 * Copyright (c) 2010 ARM Limited
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
 *          Andreas Sandberg
 */
.syntax unified
#ifdef __thumb__
.thumb
#endif

#include "m5ops.h"

.text

.macro simple_op name, func, subfunc
        .align 2
        .globl \name
\name:
#ifdef __thumb__
        .short 0xEE00 | \func
        .short 0x0110 | (\subfunc << 12)
#else
        .long 0xEE000110 | (\func << 16) | (\subfunc << 12)
#endif
        mov pc,lr
.endm

#define SIMPLE_OP(name, func, subfunc) simple_op name, func, subfunc

SIMPLE_OP(arm, arm_func, 0)
SIMPLE_OP(quiesce, quiesce_func, 0)
SIMPLE_OP(quiesceNs, quiescens_func, 0)
SIMPLE_OP(quiesceCycle, quiescecycle_func, 0)
SIMPLE_OP(quiesceTime, quiescetime_func, 0)
SIMPLE_OP(rpns, rpns_func, 0)
SIMPLE_OP(wakeCPU, wakecpu_func, 0)
SIMPLE_OP(m5_exit, exit_func, 0)
SIMPLE_OP(m5_fail, fail_func, 0)
SIMPLE_OP(m5_initparam, initparam_func, 0)
SIMPLE_OP(m5_loadsymbol, loadsymbol_func, 0)
SIMPLE_OP(m5_reset_stats, resetstats_func, 0)
SIMPLE_OP(m5_dump_stats, dumpstats_func, 0)
SIMPLE_OP(m5_dumpreset_stats, dumprststats_func, 0)
SIMPLE_OP(m5_checkpoint, ckpt_func, 0)
SIMPLE_OP(m5_readfile, readfile_func, 0)
SIMPLE_OP(m5_writefile, writefile_func, 0)
SIMPLE_OP(m5_debugbreak, debugbreak_func, 0)
SIMPLE_OP(m5_switchcpu, switchcpu_func, 0)
SIMPLE_OP(m5_addsymbol, addsymbol_func, 0)
SIMPLE_OP(m5_panic, panic_func, 0)
SIMPLE_OP(m5_work_begin, work_begin_func, 0)
SIMPLE_OP(m5_work_end, work_end_func, 0)

SIMPLE_OP(m5a_bsm, annotate_func, an_bsm)
SIMPLE_OP(m5a_esm, annotate_func, an_esm)
SIMPLE_OP(m5a_begin, annotate_func, an_begin)
SIMPLE_OP(m5a_end, annotate_func, an_end)
SIMPLE_OP(m5a_q, annotate_func, an_q)
SIMPLE_OP(m5a_rq, annotate_func, an_rq)
SIMPLE_OP(m5a_dq, annotate_func, an_dq)
SIMPLE_OP(m5a_wf, annotate_func, an_wf)
SIMPLE_OP(m5a_we, annotate_func, an_we)
SIMPLE_OP(m5a_ws, annotate_func, an_ws)
SIMPLE_OP(m5a_sq, annotate_func, an_sq)
SIMPLE_OP(m5a_aq, annotate_func, an_aq)
SIMPLE_OP(m5a_pq, annotate_func, an_pq)
SIMPLE_OP(m5a_l, annotate_func, an_l)
SIMPLE_OP(m5a_identify, annotate_func, an_identify)
SIMPLE_OP(m5a_getid, annotate_func, an_getid)
