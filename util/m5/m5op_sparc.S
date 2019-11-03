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

#define m5_op 0x2
#define m5_op3 0x37

#include <gem5/asm/generic/m5ops.h>

#define INST(func, rs1, rs2, rd) \
        .long (m5_op) << 30 | (rd) << 25 | (m5_op3) << 19 | (func) << 7 |  \
              (rs1) << 14 | (rs2) << 0;


#define LEAF(func)                \
        .section ".text";         \
        .align   4;               \
        .global  func;            \
        .type    func, #function; \
func:

#define END(func)         \
        .size    func, (.-func)

#define DEBUGBREAK INST(M5OP_DEBUG_BREAK, 0, 0, 0)
#define M5EXIT INST(M5OP_EXIT, 0, 0, 0)
#define PANIC INST(M5OP_PANIC, 0, 0, 0)
#define READFILE INST(M5OP_READ_FILE, 0, 0, 0)

LEAF(m5_exit)
    retl
    M5EXIT
END(m5_exit)

LEAF(m5_panic)
    retl
    PANIC
END(m5_panic)

LEAF(m5_read_file)
    retl
    READFILE
END(m5_read_file)

LEAF(m5_debug_break)
    retl
    DEBUGBREAK
END(m5_debug_break)

/* !!!!!! All code below here just panics !!!!!! */
LEAF(m5_arm)
    retl
    PANIC
END(m5_arm)

LEAF(m5_quiesce)
    retl
    PANIC
END(m5_quiesce)

LEAF(m5_quiesce_ns)
    retl
    PANIC
END(m5_quiesce_ns)

LEAF(m5_quiesce_cycle)
    retl
    PANIC
END(m5_quiesce_cycle)

LEAF(m5_quiesce_time)
    retl
    PANIC
END(m5_quiesce_time)

LEAF(m5_init_param)
    retl
    PANIC
END(m5_init_param)

LEAF(m5_load_symbol)
    retl
    PANIC
END(m5_load_symbol)

LEAF(m5_reset_stats)
    retl
    PANIC
END(m5_reset_stats)

LEAF(m5_dump_stats)
    retl
    PANIC
END(m5_dump_stats)

LEAF(m5_dump_reset_stats)
    retl
    PANIC
END(m5_dump_reset_stats)

LEAF(m5_checkpoint)
    retl
    PANIC
END(m5_checkpoint)

LEAF(m5_switch_cpu)
    retl
    PANIC
END(m5_switch_cpu)

LEAF(m5_add_symbol)
    retl
    PANIC
END(m5_add_symbol)

LEAF(m5_anbegin)
    retl
    PANIC
END(m5_anbegin)

LEAF(m5_anwait)
    retl
    PANIC
END(m5_anwait)


