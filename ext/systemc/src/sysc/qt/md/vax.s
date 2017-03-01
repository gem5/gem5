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

	.globl _qt_abort
	.globl _qt_block
	.globl _qt_blocki
	.globl _qt_start
	.globl _qt_vstart


/*
// Calls to these routines have the signature
//
//	void *block (func, arg1, arg2, newsp)
//
// Since the prologue saves 5 registers, nargs, pc, fp, ap, mask, and
// a condition handler (at sp+0), the first argument is 40=4*10 bytes
// offset from the stack pointer.
*/
_qt_block:
_qt_blocki:
_qt_abort:
	.word 0x7c0		/* Callee-save mask: 5 registers. */
	movl 56(sp),r1		/* Get stack pointer of new thread. */
	movl 52(sp),-(r1)	/* Push arg2 */
	movl 48(sp),-(r1)	/* Push arg1 */
	movl sp,-(r1)		/* Push arg0 */

	movl 44(sp),r0		/* Get helper to call. */
	movl r1,sp		/* Move to new thread's stack. */
	addl3 sp,$12,fp		/* .. including the frame pointer. */
	calls $3,(r0)		/* Call helper. */

	ret

_qt_start:
	movl (sp)+,r0		/* Get `only'. */
	calls $3,(r0)		/* Call `only'. */
	calls $0,_qt_error	/* `only' erroniously returned. */


_qt_vstart:
	movl (sp)+,r10		/* Get `pt'. */
	movl (sp)+,r9		/* Get `startup'. */
	movl (sp)+,r8		/* Get `vuserf'. */
	movl (sp)+,r7		/* Get `cleanup'. */

	pushl r10		/* Push `qt'. */
	calls $1,(r9)		/* Call `startup', pop `qt' on return. */

	calls (sp)+,(r8)	/* Call user's function. */

	pushl r0		/* Push `vuserf_retval'. */
	pushl r10		/* Push `qt'. */
	calls $2,(r7)		/* Call `cleanup', never return. */

	calls $0,_qt_error	/* `cleanup' erroniously returned. */
