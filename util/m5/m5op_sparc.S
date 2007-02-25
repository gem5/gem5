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

#include "m5ops.h"

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

#define DEBUGBREAK INST(debugbreak_func, 0, 0, 0)
#define M5EXIT INST(exit_func, 0, 0, 0)
#define PANIC INST(panic_func, 0, 0, 0)
#define READFILE INST(readfile_func, 0, 0, 0)

LEAF(m5_exit)
    retl
    M5EXIT
END(m5_exit)

LEAF(m5_panic)
    retl
    PANIC
END(m5_panic)

LEAF(m5_readfile)
    retl
    READFILE
END(m5_readfile)

LEAF(m5_debugbreak)
    retl
    DEBUGBREAK
END(m5_debugbreak)

/* !!!!!! All code below here just panics !!!!!! */
LEAF(arm)
    retl
    PANIC
END(arm)

LEAF(quiesce)
    retl
    PANIC
END(quiesce)

LEAF(quiesceNs)
    retl
    PANIC
END(quiesceNs)

LEAF(quiesceCycle)
    retl
    PANIC
END(quiesceCycle)

LEAF(quiesceTime)
    retl
    PANIC
END(quiesceTime)

LEAF(m5_initparam)
    retl
    PANIC
END(m5_initparam)

LEAF(m5_loadsymbol)
    retl
    PANIC
END(m5_loadsymbol)

LEAF(m5_reset_stats)
    retl
    PANIC
END(m5_reset_stats)

LEAF(m5_dump_stats)
    retl
    PANIC
END(m5_dump_stats)

LEAF(m5_dumpreset_stats)
    retl
    PANIC
END(m5_dumpreset_stats)

LEAF(m5_checkpoint)
    retl
    PANIC
END(m5_checkpoint)

LEAF(m5_switchcpu)
    retl
    PANIC
END(m5_switchcpu)

LEAF(m5_addsymbol)
    retl
    PANIC
END(m5_addsymbol)

LEAF(m5_anbegin)
    retl
    PANIC
END(m5_anbegin)

LEAF(m5_anwait)
    retl
    PANIC
END(m5_anwait)


