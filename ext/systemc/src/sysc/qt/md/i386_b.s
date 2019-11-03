/*
// QuickThreads -- Threads-building toolkit.
// Copyright (c) 1993 by David Keppel
//
// Permission to use, copy, modify and distribute this software and
// its documentation for any purpose and without fee is hereby
// granted, provided that the above copyright notice and this notice
// appear in all copies.  This software is provided as a
// proof-of-concept and for demonstration purposes; there is no
// representation about the suitability of this software for any
// purpose.  */

	.globl _b_call_reg
	.globl b_call_reg
	.globl _b_call_imm
	.globl b_call_imm
	.globl _b_add
	.globl b_add
	.globl _b_load
	.globl b_load

_b_call_reg:
b_call_reg:
_b_call_imm:
b_call_imm:
_b_add:
b_add:
_b_load:
b_load:
	hlt
