/* mips.s -- assembly support. */

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

/* Callee-save $16-$23, $30-$31.
 *
 * On startup, restore regs so retpc === call to a function to start.
 * We're going to call a function ($4) from within this routine.
 * We're passing 3 args, therefore need to allocate 12 extra bytes on
 * the stack for a save area.  The start routine needs a like 16-byte
 * save area.  Must be doubleword aligned (_mips r3000 risc
 * architecture_, gerry kane, pg d-23).
 */

	.globl qt_block
	.globl qt_blocki
	.globl qt_abort
	.globl qt_start
	.globl qt_vstart

	/*
	** $4: ptr to function to call once curr is suspended
	**	and control is on $7's stack.
	** $5: 1'th arg to $4.
	** $6: 2'th arg to $4
	** $7: sp of thread to suspend.
	**
	** Totally gross hack: The MIPS calling convention reserves
	** 4 words on the stack for a0..a3.  This routine "ought" to
	** allocate space for callee-save registers plus 4 words for
	** the helper function, but instead we use the 4 words
	** provided by the function that called us (we don't need to
	** save our argument registers).  So what *appears* to be
	** allocating only 40 bytes is actually allocating 56, by
	** using the caller's 16 bytes.
	**
	** The helper routine returns a value that is passed on as the
	** return value from the blocking routine.  Since we don't
	** touch $2 between the helper's return and the end of
	** function, we get this behavior for free.
	*/
qt_blocki:
	sub $sp,$sp,40		/* Allocate reg save space. */
	sw $16, 0+16($sp)
	sw $17, 4+16($sp)
	sw $18, 8+16($sp)
	sw $19,12+16($sp)
	sw $20,16+16($sp)
	sw $21,20+16($sp)
	sw $22,24+16($sp)
	sw $23,28+16($sp)
	sw $30,32+16($sp)
	sw $31,36+16($sp)
	add $2, $sp,$0		/* $2 <= old sp to pass to func@$4. */
qt_abort:
	add $sp, $7,$0		/* $sp <= new sp. */
	.set noreorder
	jal $31,$4		/* Call helper func@$4 . */
	add $4, $2,$0		/* $a0 <= pass old sp as a parameter. */
	.set reorder
	lw $31,36+16($sp)	/* Restore callee-save regs... */
	lw $30,32+16($sp)
	lw $23,28+16($sp)
	lw $22,24+16($sp)
	lw $21,20+16($sp)
	lw $20,16+16($sp)
	lw $19,12+16($sp)
	lw $18, 8+16($sp)
	lw $17, 4+16($sp)
	lw $16, 0+16($sp)	/* Restore callee-save */

	add $sp,$sp,40		/* Deallocate reg save space. */
	j $31			/* Return to caller. */

	/*
	** Non-varargs thread startup.
	** Note: originally, 56 bytes were allocated on the stack.
	** The thread restore routine (_blocki/_abort) removed 40
	** of them, which means there is still 16 bytes for the
	** argument area required by the MIPS calling convention.
	*/
qt_start:
	add $4, $16,$0		/* Load up user function pu. */
	add $5, $17,$0		/* ... user function pt. */
	add $6, $18,$0		/* ... user function userf. */
	jal $31,$19		/* Call `only'. */
	j qt_error


	/*
	** Save calle-save floating-point regs $f20-$f30
	** See comment in `qt_block' about calling conventinos and
	** reserved space.  Use the same trick here, but here we
	** actually have to allocate all the bytes since we have to
	** leave 4 words leftover for `qt_blocki'.
	**
	** Return value from `qt_block' is the same as the return from
	** `qt_blocki'.  We get that for free since we don't touch $2
	** between the return from `qt_blocki' and the return from
	** `qt_block'.
	*/
qt_block:
	sub $sp, $sp,56		/* 6 8-byte regs, saved ret pc, aligned. */
	swc1 $f20,  0+16($sp)
	swc1 $f22,  8+16($sp)
	swc1 $f24, 16+16($sp)
	swc1 $f26, 24+16($sp)
	swc1 $f28, 32+16($sp)
	swc1 $f30, 40+16($sp)
	sw $31, 48+16($sp)
	jal qt_blocki
	lwc1 $f20,  0+16($sp)
	lwc1 $f22,  8+16($sp)
	lwc1 $f24, 16+16($sp)
	lwc1 $f26, 24+16($sp)
	lwc1 $f28, 32+16($sp)
	lwc1 $f30, 40+16($sp)
	lw $31, 48+16($sp)
	add $sp, $sp,56
	j $31


	/*
	** First, call `startup' with the `pt' argument.
	**
	** Next, call the user's function with all arguments.
	** Note that we don't know whether args were passed in
	** integer regs, fp regs, or on the stack (See Gerry Kane
	** "MIPS R2000 RISC Architecture" pg D-22), so we reload
	** all the registers, possibly with garbage arguments.
	**
	** Finally, call `cleanup' with the `pt' argument and with
	** the return value from the user's function.  It is an error
	** for `cleanup' to return.
	*/
qt_vstart:
	add $4, $17,$0		/* `pt' is arg0 to `startup'. */
	jal $31, $18		/* Call `startup'. */

	add $sp, $sp,16		/* Free extra save space. */
	lw $4,  0($sp)		/* Load up args. */
	lw $5,  4($sp)
	lw $6,  8($sp)
	lw $7, 12($sp)
	lwc1 $f12, 0($sp)	/* Load up fp args. */
	lwc1 $f14, 8($sp)
	jal $31,$19		/* Call `userf'. */

	add $4, $17,$0		/* `pt' is arg0 to `cleanup'. */
	add $5, $2,$0		/* Ret. val is arg1 to `cleanup'. */
	jal $31, $16		/* Call `cleanup'. */

	j qt_error
