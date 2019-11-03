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

#ifndef QUICKTHREADS_KSR1_H
#define QUICKTHREADS_KSR1_H

/* 
   Stack layout:

   Registers are saved in strictly low to high order, FPU regs first 
   (only if qt_block is called), CEU regs second, IPU regs next, with no
   padding between the groups.

   Callee-save:  f16..f63; c15..c30; i12..i30.
   Args passed in i2..i5.

   Note: c31 is a private data pointer.  It is not changed on thread
   swaps with the assumption that it represents per-processor rather
   than per-thread state.

   Note: i31 is an instruction count register that is updated by the
   context switch routines.  Like c31, it is not changed on context
   switches.

   This is what we want on startup:


   +------		<-- BOS: Bottom of stack (grows down)
   | 80 (128 - 48) bytes of padding to a 128-byte boundary
   +---
   | only
   | userf
   | t
   | u
   | qt_start$TXT
   | (empty)     	<-- qt.sp
   +------		<-- (BOS - 128)

   This is why we want this on startup:
   
   A thread begins running when the restore procedure switches thread stacks
   and pops a return address off of the top of the new stack (see below
   for the reason why we explicitly store qt_start$TXT).  The
   block procedure pushes two jump addresses on a thread's stack before
   it switches stacks.  The first is the return address for the block
   procedure, and the second is a restore address.  The return address
   is used to jump back to the thread that has been switched to;  the
   restore address is a jump within the block code to restore the registers.
   Normally, this is just a jump to the next address.  However, on thread
   startup, this is a jump to qt_start$TXT.  (The block procedure stores
   the restore address at an offset of 8 bytes from the top of the stack,
   which is also the offset at which qt_start$TXT is stored on the stacks
   of new threads.  Hence, when the block procedure switches to a new
   thread stack, it will initially jump to qt_start$TXT; thereafter,
   it jumps to the restore code.)

   qt_start$TXT, after it has read the initial data on the new thread's
   stack and placed it in registers, pops the initial stack frame
   and gives the thread the entire stack to use for execution.

   The KSR runtime system has an unusual treatment of pointers to
   functions.  From C, taking the `name' of a function yields a
   pointer to a _constant block_ and *not* the address of the
   function.  The zero'th entry in the constant block is a pointer to
   the function.

   We have to be careful: the restore procedure expects a return
   address on the top of the stack (pointed to by qt.sp).  This is not
   a problem when restoring a thread that has run before, since the
   block routine would have stored the return address on top of the
   stack.  However, when ``faking up'' a thread start (bootstrapping a
   thread stack frame), the top of the stack needs to contain a
   pointer to the code that will start the thread running.

   The pointer to the startup code is *not* `qt_start'.  It is the
   word *pointed to* by `qt_start'.  Thus, we dereference `qt_start',
   see QUICKTHREADS_ARGS_MD below.

   On varargs startup (still unimplemented):

   | padding to 128 byte boundary
   | varargs		<-- padded to a 128-byte-boundary
   +---
   | caller's frame, 16 bytes
   | 80 bytes of padding (frame padded to a 128-byte boundary)
   +---
   | cleanup
   | vuserf
   | startup
   | t
   +---
   | qt_start		<-- qt.sp
   +---

   Of a suspended thread:

   +---
   | caller's frame, 16 bytes
   | fpu registers 47 regs * 8 bytes/reg 376 bytes
   | ceu registers 16 regs * 8 bytes/reg 128 bytes
   | ipu registers 19 regs * 8 bytes/reg 152 bytes
   |  :
   | 80 bytes of padding
   |  :
   | qt_restore		<-- qt.sp
   +---

   */


#define QUICKTHREADS_STKALIGN	128
#define QUICKTHREADS_GROW_DOWN
typedef unsigned long qt_word_t;

#define QUICKTHREADS_STKBASE	QUICKTHREADS_STKALIGN
#define QUICKTHREADS_VSTKBASE	QUICKTHREADS_STKBASE

extern void qt_start(void);
/*
 * See the discussion above for what indexing into a procedure ptr 
 * does for us (it's lovely, though, isn't it?).
 *
 * This assumes that the address of a procedure's code is the
 * first word in a procedure's constant block.  That's how the manual
 * says it will be arranged.
 */
#define QUICKTHREADS_ARGS_MD(sp)	(QUICKTHREADS_SPUT (sp, 1, ((qt_word_t *)qt_start)[0]))

/* 
 * The *index* (positive offset) of where to put each value.
 * See the picture of the stack above that explains the offsets.
 */
#define QUICKTHREADS_ONLY_INDEX	(5)
#define QUICKTHREADS_USER_INDEX	(4)
#define QUICKTHREADS_ARGT_INDEX	(3)
#define QUICKTHREADS_ARGU_INDEX	(2)

#define QUICKTHREADS_VARGS_DEFAULT
#define QUICKTHREADS_VARGS(sp, nb, vargs, pt, startup, vuserf, cleanup) \
      (qt_vargs (sp, nbytes, &vargs, pt, startup, vuserf, cleanup))


#define QUICKTHREADS_VARGS_MD0(sp, vabytes) \
  ((qt_t *)(((char *)(sp)) - 4*8 - QUICKTHREADS_STKROUNDUP(vabytes)))

extern void qt_vstart(void);
#define QUICKTHREADS_VARGS_MD1(sp)	(QUICKTHREADS_SPUT (sp, 0, ((qt_word_t *)qt_vstart)[0]))

#define QUICKTHREADS_VCLEANUP_INDEX	(4)
#define QUICKTHREADS_VUSERF_INDEX		(3)
#define QUICKTHREADS_VSTARTUP_INDEX	(2)
#define QUICKTHREADS_VARGT_INDEX		(1)

#endif /* def QUICKTHREADS_KSR1_H */
