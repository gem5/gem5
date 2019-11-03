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
	.globl b_call_reg
	.globl b_call_imm
	.globl b_add
	.globl b_load

	.ent b_null
b_null:
	ret $31,($18),1
	.end b_null

	.ent b_call_reg
b_call_reg:
	lda $27,b_null
$L0:
	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)

	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)
	jsr $18,($27)

	subq $16,1,$16
	bgt $16,$L0

	ret $31,($26),1
	.end


	.ent b_call_imm
b_call_imm:
$L1:
	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null

	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null
	jsr $18,b_null

	subq $16,1,$16
	bgt $16,$L1

	ret $31,($26),1
	.end


	.ent b_add
b_add:
$L2:
	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31

	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31
	addq $31,$31,$31

	subq $16,1,$16
	bgt $16,$L2

	ret $31,($26),1
	.end


	.ent b_load
b_load:
$L3:
	ldq $31,0($30)
	ldq $31,8($30)
	ldq $31,16($30)
	ldq $31,24($30)
	ldq $31,32($30)

	ldq $31,0($30)
	ldq $31,8($30)
	ldq $31,16($30)
	ldq $31,24($30)
	ldq $31,32($30)

	subq $16,1,$16
	bgt $16,$L3

	ret $31,($26),1
	.end
