/* i386.s -- assembly support. */

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
// purpose. */

/* NOTE: double-labeled `_name' and `name' for System V compatability.  */
/* NOTE: Mixed C/C++-style comments used. Sorry! */

/* Callee-save: %esi, %edi, %ebx, %ebp
// Caller-save: %eax, %ecx
// Can't tell: %edx (seems to work w/o saving it.)
//
// Assignment:
//
// See ``i386.h'' for the somewhat unconventional stack layout.  */


	.text
	.align 2

	.globl _qt_abort
	.globl qt_abort
	.globl _qt_block
	.globl qt_block
	.globl _qt_blocki
	.globl qt_blocki
	.globl _qt_align
	.globl qt_align

/* These all have the type signature
//
//	void *blocking (helper, arg0, arg1, new)
//
// On procedure entry, the helper is at 4(sp), args at 8(sp) and
// 12(sp) and the new thread's sp at 16(sp).  It *appears* that the
// calling convention for the 8X86 requires the caller to save all
// floating-point registers, this makes our life easy.  */

/* Halt the currently-running thread.  Save it's callee-save regs on
// to the stack, 32 bytes.  Switch to the new stack (next == 16+32(sp))
// and call the user function (f == 4+32(sp) with arguments: old sp
// arg1 (8+32(sp)) and arg2 (12+32(sp)).  When the user function is
// done, restore the new thread's state and return.
//
// `qt_abort' is (currently) an alias for `qt_block' because most of
// the work is shared.  We could save the insns up to `qt_common' by
// replicating, but w/o replicating we need an inital subtract (to
// offset the stack as if it had been a qt_block) and then a jump
// to qt_common.  For the cost of a jump, we might as well just do
// all the work.
//
// The helper function (4(sp)) can return a void* that is returned
// by the call to `qt_blockk{,i}'.  Since we don't touch %eax in
// between, we get that ``for free''.  */

_qt_abort:
qt_abort:
_qt_block:
qt_block:
_qt_blocki:
qt_blocki:
	pushl %ebp		/* Save callee-save, sp-=4. */
	pushl %esi		/* Save callee-save, sp-=4. */
	pushl %edi		/* Save callee-save, sp-=4. */
	pushl %ebx		/* Save callee-save, sp-=4. */
	movl %esp, %eax		/* Remember old stack pointer. */

qt_common:
	movl 32(%esp), %esp	/* Move to new thread. */
	pushl 28(%eax)		/* Push arg 2. */
	pushl 24(%eax)		/* Push arg 1. */
	pushl %eax		/* Push arg 0. */
	movl 20(%eax), %ebx	/* Get function to call. */
	call *%ebx		/* Call f. */
	addl $12, %esp		/* Pop args. */

	popl %ebx		/* Restore callee-save, sp+=4. */
	popl %edi		/* Restore callee-save, sp+=4. */
	popl %esi		/* Restore callee-save, sp+=4. */
	popl %ebp		/* Restore callee-save, sp+=4. */
_qt_align:
qt_align:
	ret			/* Resume the stopped function. */

	.globl _qt_tramp
	.globl qt_tramp
_qt_tramp:
qt_tramp:
	movl 12(%esp), %eax	/* Load 'qt_error' address */
	sub $4, %esp		/* Align stack pointer to 16-byte boundary */
	jmp *%eax		/* call 'qt_error' */
	hlt			/* 'qt_error' never returns */

/* Start a varargs thread. */

	.globl _qt_vstart
	.globl qt_vstart
_qt_vstart:
qt_vstart:
	pushl %edi		/* Push `pt' arg to `startup'. */
	call *%ebp		/* Call `startup'. */
	popl %eax		/* Clean up the stack. */

	call *%ebx		/* Call the user's function. */

	pushl %eax		/* Push return from user's. */
	pushl %edi		/* Push `pt' arg to `cleanup'. */
	call *%esi		/* Call `cleanup'. */

	hlt			/* `cleanup' never returns. */
