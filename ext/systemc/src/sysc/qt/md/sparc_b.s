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

	.globl _b_call_reg
	.globl _b_call_imm
	.globl _b_add
	.globl _b_load

_b_null:
	retl
	nop

_b_call_reg:
	sethi %hi(_b_null),%o4
	or %o4,%lo(_b_null),%o4
	add %o7,%g0, %o3
L0:
	call %o4
	nop
	call %o4
	nop
	call %o4
	nop
	call %o4
	nop
	call %o4
	nop

	subcc %o0,1,%o0
	bg L0
	nop
	add %o3,%g0, %o7
	retl
	nop

_b_call_imm:
	sethi %hi(_b_null),%o4
	or %o4,%lo(_b_null),%o4
	add %o7,%g0, %o3
L1:
	call _b_null
	call _b_null
	call _b_null
	call _b_null
	call _b_null

	subcc %o0,1,%o0
	bg L0
	nop
	add %o3,%g0, %o7
	retl
	nop


_b_add:
	add %o0,%g0,%o1
	add %o0,%g0,%o2
	add %o0,%g0,%o3
	add %o0,%g0,%o4
L2:
	sub %o0,5,%o0
	sub %o1,5,%o1
	sub %o2,5,%o2
	sub %o3,5,%o3
	sub %o4,5,%o4

	subcc %o0,5,%o0
	sub %o1,5,%o1
	sub %o2,5,%o2
	sub %o3,5,%o3
	sub %o4,5,%o4

	bg L2
	nop
	retl
	nop


_b_load:
	ld [%sp+ 0], %g0
L3:
	ld [%sp+ 4],%g0
	ld [%sp+ 8],%g0
	ld [%sp+12],%g0
	ld [%sp+16],%g0
	ld [%sp+20],%g0
	ld [%sp+24],%g0
	ld [%sp+28],%g0
	ld [%sp+32],%g0
	ld [%sp+36],%g0

	subcc %o0,10,%o0
	bg L3
	ld [%sp+ 0],%g0
	retl
	nop
