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

#ifndef QUICKTHREADS_AXP_H
#define QUICKTHREADS_AXP_H

#define QUICKTHREADS_GROW_DOWN

typedef unsigned long qt_word_t;


/* Stack layout on the Alpha:

   Integer:

     Caller-save: r0..r8, r22..r25, r27..r29
     argument/caller-save: r16..r21
     callee-save: r9..r15
     return pc *callee-save*: r26
     stack pointer: r30
     zero: r31

   Floating-point:

     Caller-save: f0..f1, f10..f15
     argument/caller-save: f16..f21, f22..f30
     callee-save: f2..f9
     zero: f31

   Non-varargs:

   +---
   | padding
   | f9
   | f8
   | f7
   | f6
   | f5
   | f4
   | f3
   | f2
   | r26
   +---
   | padding
   | r29
   | r15
   | r14
   | r13
   | r12	on startup === `only'
   | r11	on startup === `userf'
   | r10	on startup === `qt'
   | r9		on startup === `qu'
   | r26	on startup === qt_start		<--- qt.sp
   +---

   Conventions for varargs startup:

   |  :
   | arg6
   | iarg5
   |  :
   | iarg0
   | farg5
   |  :
   | farg0
   +---
   | padding
   | r29
   | r15
   | r14
   | r13
   | r12	on startup === `startup'
   | r11	on startup === `vuserf'
   | r10	on startup === `cleanup'
   | r9		on startup === `qt'
   | r26	on startup === qt_vstart	<--- qt.sp
   +---

   Note: this is a pretty cheap/sleazy way to get things going,
   but ``there must be a better way.''  For instance, some varargs
   parameters could be loaded in to integer registers, or the return
   address could be stored on top of the stack. */


/* Stack must be 16-byte aligned. */
#define QUICKTHREADS_STKALIGN	(16)

/* How much space is allocated to hold all the crud for
   initialization: 7 registers times 8 bytes/register. */

#define QUICKTHREADS_STKBASE	(10 * 8)
#define QUICKTHREADS_VSTKBASE	QUICKTHREADS_STKBASE


/* Offsets of various registers. */
#define QUICKTHREADS_R26	0
#define QUICKTHREADS_R9	1
#define QUICKTHREADS_R10	2
#define QUICKTHREADS_R11	3
#define QUICKTHREADS_R12	4


/* When a never-before-run thread is restored, the return pc points
   to a fragment of code that starts the thread running.  For
   non-vargs functions, it just calls the client's `only' function.
   For varargs functions, it calls the startup, user, and cleanup
   functions.

   The varargs startup routine always reads 12 8-byte arguments from
   the stack.  If fewer argumets were pushed, the startup routine
   would read off the top of the stack.  To prevent errors we always
   allocate enough space.  When there are fewer args, the preallocated
   words are simply wasted. */

extern void qt_start(void);
#define QUICKTHREADS_ARGS_MD(sp)	(QUICKTHREADS_SPUT (sp, QUICKTHREADS_R26, qt_start))


/* The AXP uses a struct for `va_list', so pass a pointer to the
   struct.  This may break some uses of `QUICKTHREADS_VARGS', but then we never
   claimed it was totally portable. */

typedef void (qt_function_t)(void);

struct qt_t;
struct va_list;
extern struct qt_t *qt_vargs (struct qt_t *sp, int nbytes,
			      struct va_list *vargs, void *pt,
			      qt_function_t *startup,
			      qt_function_t *vuserf,
			      qt_function_t *cleanup);

#define QUICKTHREADS_VARGS(sp, nbytes, vargs, pt, startup, vuserf, cleanup) \
  (qt_vargs (sp, nbytes, (struct va_list *)(&(vargs)), pt, \
	     (qt_function_t *) startup, (qt_function_t *)vuserf, \
	     (qt_function_t *)cleanup));


/* The *index* (positive offset) of where to put each value. */
#define QUICKTHREADS_ONLY_INDEX	(QUICKTHREADS_R12)
#define QUICKTHREADS_USER_INDEX	(QUICKTHREADS_R11)
#define QUICKTHREADS_ARGT_INDEX	(QUICKTHREADS_R10)
#define QUICKTHREADS_ARGU_INDEX	(QUICKTHREADS_R9)

#define QUICKTHREADS_VCLEANUP_INDEX	(QUICKTHREADS_R10)
#define QUICKTHREADS_VUSERF_INDEX		(QUICKTHREADS_R11)
#define QUICKTHREADS_VSTARTUP_INDEX	(QUICKTHREADS_R12)
#define QUICKTHREADS_VARGT_INDEX		(QUICKTHREADS_R9)

#endif /* ndef QUICKTHREADS_AXP_H */
