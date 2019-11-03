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
	jmp   r1

_b_call_reg:
	subu r31, r31,8			/* Alloc ret pc save space. */
	st r1, r31,32			/* Save ret pc. */
	or.u r3, r0,hi16(_b_null)	/* Put call addr in a reg. */
	or r3, r3,lo16(_b_null)
	jsr r3
L0:
	jsr r3
	jsr r3
	jsr r3
	jsr.n r3
	subu r2, r2,5			/* Decrement #of iter to go. */
	bcnd.n gt0,r2,L0
	jsr r3

	ld r1, r31,32
	jmp r1


_b_call_imm:
	subu r31, r31,8			/* Alloc ret pc save space. */
	st r1, r31,32			/* Save ret pc. */
	bsr _b_null
L1:
	bsr _b_null
	bsr _b_null
	bsr _b_null
	bsr.n _b_null
	subu r2, r2,5			/* Decrement #of iter to go. */
	bcnd.n gt0,r2,L1
	bsr _b_null

	ld r1, r31,32
	jmp r1

_b_add:
	add r0, r3,r4
L2:
	add r3, r4,r5
	add r4, r5,r6
	add r5, r6,r7
	add r8, r9,r0
	add r0, r3,r4
	add r3, r4,r5
	add r4, r5,r6
	add r5, r6,r7
	add r8, r9,r0

	add r0, r3,r4
	add r3, r4,r5
	add r4, r5,r6
	add r5, r6,r7
	add r8, r9,r0
	add r0, r3,r4
	add r3, r4,r5
	add r4, r5,r6
	add r5, r6,r7
	add r8, r9,r0

	subu r2, r2,20			/* Decrement #of iter to go. */
	bcnd.n gt0,r2,L2
	add r0, r3,r4

	jmp r1


_b_load:
	ld r0, r31,0
L3:
	ld r3, r31,4
	ld r4, r31,8
	ld r5, r31,12
	ld r6, r31,16
	ld r0, r31,0
	ld r3, r31,4
	ld r4, r31,8
	ld r5, r31,12
	ld r6, r31,16

	ld r0, r31,0
	ld r3, r31,4
	ld r4, r31,8
	ld r5, r31,12
	ld r6, r31,16
	ld r0, r31,0
	ld r3, r31,4
	ld r4, r31,8
	ld r5, r31,12
	ld r6, r31,16

	subu r2, r2,20			/* Decrement #of iter to go. */
	bcnd.n gt0,r2,L3
	ld r0, r31,0

	jmp   r1
