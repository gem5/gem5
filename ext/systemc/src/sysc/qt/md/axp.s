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

/* axp.s -- assembly support. */

	.text
	.align 4
	.file 2 "axp.s"

	.globl qt_block
	.globl qt_blocki
	.globl qt_abort
	.globl qt_start
	.globl qt_vstart

	/*
	** $16: ptr to function to call once curr is suspended
	**	and control is on r19's stack.
	** $17: 1'th arg to (*$16)(...).
	** $18: 2'th arg to (*$16)(...).
	** $19: sp of thread to resume.
	**
	** The helper routine returns a value that is passed on as the
	** return value from the blocking routine.  Since we don't
	** touch r0 between the helper's return and the end of
	** function, we get this behavior for free.
	*/

	.ent qt_blocki
qt_blocki:
	subq $30,80, $30	/* Allocate save area. */
	stq $26, 0($30)		/* Save registers. */
	stq  $9, 8($30)
	stq $10,16($30)
	stq $11,24($30)
	stq $12,32($30)
	stq $13,40($30)
	stq $14,48($30)
	stq $15,56($30)
	stq $29,64($30)
	.end qt_blocki
	.ent qt_abort
qt_abort:
	addq $16,$31, $27	/* Put argument function in PV. */
	addq $30,$31, $16	/* Save stack ptr in outgoing arg. */
	addq $19,$31, $30	/* Set new stack pointer. */
	jsr $26,($27),0		/* Call helper function. */

	ldq $26, 0($30)		/* Restore registers. */
	ldq  $9, 8($30)
	ldq $10,16($30)
	ldq $11,24($30)
	ldq $12,32($30)
	ldq $13,40($30)
	ldq $14,48($30)
	ldq $15,56($30)
	ldq $29,64($30)

	addq $30,80, $30	/* Deallocate save area. */
	ret $31,($26),1		/* Return, predict===RET. */
	.end qt_abort


	/*
	** Non-varargs thread startup.
	*/
	.ent qt_start
qt_start:
	addq $9,$31,  $16	/* Load up `qu'. */
	addq $10,$31, $17	/* ... user function's `pt'. */
	addq $11,$31, $18	/* ... user function's `userf'. */
	addq $12,$31, $27	/* ... set procedure value to `only'. */
	jsr $26,($27),0		/* Call `only'. */

	jsr $26,qt_error	/* `only' erroniously returned. */
	.end qt_start


	.ent qt_vstart:
qt_vstart:
	/* Call startup function. */
	addq $9,$31, $16	/* Arg0 to `startup'. */
	addq $12,$31, $27	/* Set procedure value. */
	jsr $26,($27),0		/* Call `startup'. */

	/* Call user function. */
	ldt $f16, 0($30)	/* Load fp arg regs. */
	ldt $f17, 8($30)
	ldt $f18,16($30)
	ldt $f19,24($30)
	ldt $f20,32($30)
	ldt $f21,40($30)
	ldq $16,48($30)		/* And integer arg regs. */
	ldq $17,56($30)
	ldq $18,64($30)
	ldq $19,72($30)
	ldq $20,80($30)
	ldq $21,88($30)
	addq $30,96 $30		/* Pop 6*2*8 saved arg regs. */
	addq $11,$31, $27	/* Set procedure value. */
	jsr $26,($27),0		/* Call `vuserf'. */

	/* Call cleanup. */
	addq $9,$31, $16	/* Arg0 to `cleanup'. */
	addq $0,$31, $17	/* Users's return value is arg1. */
	addq $10,$31, $27	/* Set procedure value. */
	jsr $26,($27),0		/* Call `cleanup'. */

	jsr $26,qt_error	/* Cleanup erroniously returned. */
	.end qt_start


	/*
	** Save calle-save floating-point regs $f2..$f9.
	** Also save return pc from whomever called us.
	**
	** Return value from `qt_block' is the same as the return from
	** `qt_blocki'.  We get that for free since we don't touch $0
	** between the return from `qt_blocki' and the return from
	** `qt_block'.
	*/
	.ent qt_block
qt_block:
	subq $30,80, $30	/* Allocate a save space. */
	stq $26, 0($30)		/* Save registers. */
	stt $f2, 8($30)
	stt $f3,16($30)
	stt $f4,24($30)
	stt $f5,32($30)
	stt $f6,40($30)
	stt $f7,48($30)
	stt $f8,56($30)
	stt $f9,64($30)

	jsr $26,qt_blocki	/* Call helper. */
				/* .. who will also restore $gp. */

	ldq $26, 0($30)		/* restore registers. */
	ldt $f2, 8($30)
	ldt $f3,16($30)
	ldt $f4,24($30)
	ldt $f5,32($30)
	ldt $f6,40($30)
	ldt $f7,48($30)
	ldt $f8,56($30)
	ldt $f9,64($30)

	addq $30,80, $30	/* Deallcate save space. */
	ret $31,($26),1		/* Return, predict===RET. */
	.end qt_block
