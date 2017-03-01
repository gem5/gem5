/*
 * QuickThreads -- Threads-building toolkit.
 * Copyright (c) 1993 by David Keppel
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice and this notice
 * appear in all copies.  This software is provided as a
 * proof-of-concept and for demonstration purposes; there is no
 * representation about the suitability of this software for any
 * purpose.
 */

	.text
	.globl _b_call_reg
	.globl _b_call_imm
	.globl _b_add
	.globl _b_load

_b_null:
	.word 0x0
	ret

_b_call_reg:
	.word 0x0
	movl 4(ap),r0
	moval _b_null,r1
L0:
	calls $0,(r1)
	calls $0,(r1)
	calls $0,(r1)
	calls $0,(r1)
	calls $0,(r1)

	subl2 $5,r0
	bgtr L0
	ret


_b_call_imm:
	.word 0x0
	movl 4(ap),r0
L1:
	calls $0,_b_null
	calls $0,_b_null
	calls $0,_b_null
	calls $0,_b_null
	calls $0,_b_null

	subl2 $5,r0
	bgtr L1
	ret


_b_add:
	.word 0x0
	movl 4(ap),r0
L2:
	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0

	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0
	subl2 $1,r0

	bgtr L2
	ret


_b_load:
	.word 0x0
	movl 4(ap),r0
L3:
	movl 0(sp),r1
	movl 4(sp),r1
	movl 8(sp),r1
	movl 12(sp),r1
	movl 16(sp),r1
	movl 20(sp),r1
	movl 24(sp),r1
	movl 28(sp),r1
	movl 32(sp),r1
	movl 36(sp),r1

	subl2 $1,r0
	bgtr L3
	ret
