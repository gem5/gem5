/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <machine/asm.h>
#include <regdef.h>
		
#define m5_op 0x01
#define arm_func 0x00
#define quiesce_func 0x01
#define ivlb_func 0x10
#define ivle_func 0x11
#define m5exit_func 0x20
#define initparam_func 0x30
#define resetstats_func 0x40
	
#define INST(op, ra, rb, func) \
	.long (((op) << 26) | ((ra) << 21) | ((rb) << 16) | (func))
	
#define	ARM(reg) INST(m5_op, reg, 0, arm_func)
#define QUIESCE() INST(m5_op, 0, 0, quiesce_func)
#define IVLB(reg) INST(m5_op, reg, 0, ivlb_func)
#define IVLE(reg) INST(m5_op, reg, 0, ivle_func)
#define M5_EXIT() INST(m5_op, 0, 0, m5exit_func)
#define INITPARAM(reg) INST(m5_op, reg, 0, initparam_func)
#define RESETSTATS() INST(m5_op, 0, 0, resetstats_func)

	.set noreorder

	.align 4
LEAF(arm)
	ARM(16)
	RET
END(arm)

	.align 4
LEAF(quiesce)
	QUIESCE()
	RET
END(quiesce)

	.align 4
LEAF(ivlb)
	IVLB(16)
	RET
END(ivlb)

	.align 4
LEAF(ivle)
	IVLE(16)
	RET
END(ivle)

	.align 4
LEAF(m5exit)
	M5_EXIT()
	RET
END(m5exit)

    .align 4
LEAF(initparam)
    INITPARAM(0)
    RET
END(initparam)

    .align 4
LEAF(resetstats)
    RESETSTATS()
    RET
END(resetstats)
