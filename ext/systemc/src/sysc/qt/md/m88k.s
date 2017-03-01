/* m88k.s -- assembly support. */

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

/* Callee-save r14..r25, r31(sp), r30(fp).  r1 === return pc.
 * Argument registers r2..r9, return value r2..r3.
 *
 * On startup, restore regs so retpc === call to a function to start.
 *
 * We're going to call a function (r2) from within the context switch
 * routine.  Call it on the new thread's stack on behalf of the old
 * thread.
 */

	.globl _qt_block
	.globl _qt_blocki
	.globl _qt_abort
	.globl _qt_start
	.globl _qt_vstart

	/*
	** r2: ptr to function to call once curr is suspended
	**	and control is on r5's stack.
	** r3: 1'th arg to *r2.
	** r4: 2'th arg to *r2.
	** r5: sp of thread to suspend.
	**
	** The helper routine returns a value that is passed on as the
	** return value from the blocking routine.  Since we don't
	** touch r2 between the helper's return and the end of
	** function, we get this behavior for free.
	**
	** Same entry for integer-only and floating-point, since there
	** are no separate integer and floating-point registers.
	**
	** Each procedure call sets aside a ``home region'' of 8 regs
	** for r2-r9 for varargs.  For context switches we don't use
	** the ``home region'' for varargs so use it to save regs.
	** Allocate 64 bytes of save space -- use 32 bytes of register
	** save area passed in to us plus 32 bytes we allcated, use
	** the other 32 bytes for save area for a save area to call
	** the helper function.
	*/
_qt_block:
_qt_blocki:
	sub r31, r31,64		/* Allocate reg save space. */
	st r1, r31,8+32		/* Save callee-save registers. */
	st r14, r31,12+32
	st.d r15, r31,16+32
	st.d r17, r31,24+32
	st.d r19, r31,32+32
	st.d r21, r31,40+32
	st.d r23, r31,48+32
	st r25, r31,56+32
	st r30, r31,60+32

_qt_abort:
	addu r14, r31,0		/* Remember old sp. */
	addu r31, r5,0		/* Set new sp. */
	jsr.n r2		/* Call helper. */
	addu r2, r14,0		/* Pass old sp as an arg0 to helper. */

	ld r1, r31,8+32		/* Restore callee-save registers. */
	ld r14, r31,12+32
	ld.d r15, r31,16+32
	ld.d r17, r31,24+32
	ld.d r19, r31,32+32
	ld.d r21, r31,40+32
	ld.d r23, r31,48+32
	ld r25, r31,56+32
	ld r30, r31,60+32

	jmp.n r1		/* Return to new thread's caller. */
	addu r31, r31,64	/* Free register save space. */


	/*
	** Non-varargs thread startup.
	** See `m88k.h' for register use conventions.
	*/
_qt_start:
	addu r2, r14,0		/* Set user arg `pu'. */
	addu r3, r15,0		/* ... user function pt. */
	jsr.n r17		/* Call `only'. */
	addu r4, r16,0		/* ... user function userf. */

	bsr _qt_error		/* `only' erroniously returned. */


	/*
	** Varargs thread startup.
	** See `m88k.h' for register use conventions.
	**
	** Call the `startup' function with just argument `pt'.
	** Then call `vuserf' with 8 register args plus any
	** stack args.
	** Then call `cleanup' with `pt' and the return value
	** from `vuserf'.
	*/
_qt_vstart:
	addu r18, r30,0		/* Remember arg7 to `vuserf'. */
	addu r30, r0,0		/* Null-terminate call chain. */

	jsr.n r17		/* Call `startup'. */
	addu r2, r15,0		/* `pt' is arg0 to `startup'. */

	addu r2, r19,0		/* Set arg0. */
	addu r3, r20,0		/* Set arg1. */
	addu r4, r21,0		/* Set arg2. */
	addu r5, r22,0		/* Set arg3. */
	addu r6, r23,0		/* Set arg4. */
	addu r7, r24,0		/* Set arg5. */
	addu r8, r25,0		/* Set arg6. */
	jsr.n r16		/* Call `vuserf'. */
	addu r9, r18,0		/* Set arg7. */

	addu r3, r2,0		/* Ret. value is arg1 to `cleanup'. */
	jsr.n r14		/* Call `cleanup'. */
	addu r2, r15,0		/* `pt' is arg0 to `cleanup'. */

	bsr _qt_error		/* `cleanup' erroniously returned. */
