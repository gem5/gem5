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

#define arm_func                0x00
#define quiesce_func            0x01
#define quiescens_func          0x02
#define quiescecycle_func       0x03
#define quiescetime_func        0x04
#define rpns_func               0x07
#define wakecpu_func            0x09
#define deprecated1_func        0x10 // obsolete ivlb
#define deprecated2_func        0x11 // obsolete ivle
#define deprecated3_func        0x20 // deprecated exit function
#define exit_func               0x21
#define initparam_func          0x30
#define loadsymbol_func         0x31
#define resetstats_func         0x40
#define dumpstats_func          0x41
#define dumprststats_func       0x42
#define ckpt_func               0x43
#define writefile_func          0x4F
#define readfile_func           0x50
#define debugbreak_func         0x51
#define switchcpu_func          0x52
#define addsymbol_func          0x53
#define panic_func              0x54

#define reserved2_func          0x56 // Reserved for user
#define reserved3_func          0x57 // Reserved for user
#define reserved4_func          0x58 // Reserved for user
#define reserved5_func          0x59 // Reserved for user

#define work_begin_func          0x5a
#define work_end_func            0x5b

// These operations are for critical path annotation
#define annotate_func     0x55
#define an_bsm            0x1
#define an_esm            0x2
#define an_begin          0x3
#define an_end            0x4
#define an_q              0x6
#define an_dq             0x7
#define an_wf             0x8
#define an_we             0x9
#define an_rq             0xA
#define an_ws             0xB
#define an_sq             0xC
#define an_aq             0xD
#define an_pq             0xE
#define an_l              0xF
#define an_identify       0x10
#define an_getid          0x11

