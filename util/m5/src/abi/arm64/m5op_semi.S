/*
 * Copyright (c) 2010-2013, 2016-2017 ARM Limited
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
 */

#include <gem5/asm/generic/m5ops.h>

.macro	m5op_func, name, func
        .globl \name
        \name:
        // Put the m5 op number in x16.
        mov x16, #(\func << 8)
        // Branch into the common handler for the rest.
        b 1f
.endm

.text
#define M5OP(name, func) m5op_func M5OP_MERGE_TOKENS(name, _semi), func;
        M5OP_FOREACH
#undef M5OP

        1:
        // Get the address of the argument block.
        ldr x17, =m5_semi_argument_block
        // Store the m5 op number in the first slot.
        str x16, [ x17 ], #8
        // Store all 8 possible arguments in the subsequent slots. We don't
        // know how many we need, so just store them all.
        str x0, [ x17 ], #8
        str x1, [ x17 ], #8
        str x2, [ x17 ], #8
        str x3, [ x17 ], #8
        str x4, [ x17 ], #8
        str x5, [ x17 ], #8
        str x6, [ x17 ], #8
        str x7, [ x17 ], #8
        // Set x0 to the m5 op semi-hosting call number.
        mov x0, #0x100
        // Set x1 to the address of the argument blob.
        ldr x1, =m5_semi_argument_block
        // Trigger the semihosting call with the gem5 specific immediate.
        hlt #0x5d57
        ret

.data
        .globl m5_semi_argument_block
        m5_semi_argument_block:
        .quad 0 // function
        .quad 0 // argument 0
        .quad 0 // argument 1
        .quad 0 // argument 2
        .quad 0 // argument 3
        .quad 0 // argument 4
        .quad 0 // argument 5
        .quad 0 // argument 6
        .quad 0 // argument 7
