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

	.globl b_call_reg
	.globl b_call_imm
	.globl b_add
	.globl b_load

	.ent b_null
b_null:
	j $31
	.end b_null

	.ent b_call_reg
b_call_reg:
	la $5,b_null
	add $6, $31,0
$L0:
	jal $5
	jal $5
	jal $5
	jal $5
	jal $5

	sub $4, $4,5
	bgtz $4,$L0
	j $6
	.end


	.ent b_call_imm
b_call_imm:
	add $6, $31,0
$L1:
	jal b_null
	jal b_null
	jal b_null
	jal b_null
	jal b_null

	sub $4, $4,5
	bgtz $4,$L1
	j $6
	.end


	.ent b_add
b_add:
	add $5, $0,$4
	add $6, $0,$4
	add $7, $0,$4
	add $8, $0,$4
$L2:
	sub $4, $4,5
	sub $5, $5,5
	sub $6, $6,5
	sub $7, $7,5
	sub $8, $8,5

	sub $4, $4,5
	sub $5, $5,5
	sub $6, $6,5
	sub $7, $7,5
	sub $8, $8,5

	bgtz $4,$L2
	j $31
	.end


	.ent b_load
b_load:
$L3:
	ld $0, 0($sp)
	ld $0, 4($sp)
	ld $0, 8($sp)
	ld $0, 12($sp)
	ld $0, 16($sp)

	ld $0, 20($sp)
	ld $0, 24($sp)
	ld $0, 28($sp)
	ld $0, 32($sp)
	ld $0, 36($sp)

	sub $4, $4,10
	bgtz $4,$L3
	j $31
	.end
