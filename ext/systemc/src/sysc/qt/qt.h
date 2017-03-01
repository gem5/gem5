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

#ifndef QUICKTHREADS_QT_H
#define QUICKTHREADS_QT_H

#if !defined(SC_USE_PTHREADS)

#ifdef __cplusplus
extern "C" {
#endif

#include <sysc/qt/qtmd.h>


/* A QuickThreads thread is represented by it's current stack pointer.
   To restart a thread, you merely need pass the current sp (qt_t*) to
   a QuickThreads primitive.  `qt_t*' is a location on the stack.  To
   improve type checking, represent it by a particular struct. */

typedef struct qt_t {
  char dummy;
} qt_t;


/* Alignment is guaranteed to be a power of two. */
#ifndef QUICKTHREADS_STKALIGN
  #error "Need to know the machine-dependent stack alignment."
#endif

#define QUICKTHREADS_STKROUNDUP(bytes) \
  (((bytes)+QUICKTHREADS_STKALIGN) & ~(QUICKTHREADS_STKALIGN-1))


/* Find ``top'' of the stack, space on the stack. */
#ifndef QUICKTHREADS_SP
#ifdef QUICKTHREADS_GROW_DOWN
#define QUICKTHREADS_SP(sto, size)	((qt_t *)(&((char *)(sto))[(size)]))
#endif
#ifdef QUICKTHREADS_GROW_UP
#define QUICKTHREADS_SP(sto, size)	((qt_t *)(sto))
#endif
#if !defined(QUICKTHREADS_SP)
  #error "QUICKTHREADS_QT_H: Stack must grow up or down!"
#endif
#endif


/* The type of the user function:
   For non-varargs, takes one void* function.
   For varargs, takes some number of arguments. */
typedef void *(qt_userf_t)(void *pu);
typedef void *(qt_vuserf_t)(int arg0, ...);

/* For non-varargs, just call a client-supplied function,
   it does all startup and cleanup, and also calls the user's
   function. */
typedef void (qt_only_t)(void *pu, void *pt, qt_userf_t *userf);

/* For varargs, call `startup', then call the user's function,
   then call `cleanup'. */
typedef void (qt_startup_t)(void *pt);
typedef void (qt_cleanup_t)(void *pt, void *vuserf_return);


/* Internal helper for putting stuff on stack. */
#ifndef QUICKTHREADS_SPUT
#define QUICKTHREADS_SPUT(top, at, val)	\
    (((qt_word_t *)(top))[(at)] = (qt_word_t)(val))
#endif


/* Push arguments for the non-varargs case. */
#ifndef QUICKTHREADS_ARGS

#ifndef QUICKTHREADS_ARGS_MD
#define QUICKTHREADS_ARGS_MD (0)
#endif

#ifndef QUICKTHREADS_STKBASE
  #error "Need to know the machine-dependent stack allocation."
#endif

/* All things are put on the stack relative to the final value of
   the stack pointer. */
#ifdef QUICKTHREADS_GROW_DOWN
#define QUICKTHREADS_ADJ(sp)	(((char *)sp) - QUICKTHREADS_STKBASE)
#else
#define QUICKTHREADS_ADJ(sp)	(((char *)sp) + QUICKTHREADS_STKBASE)
#endif

#define QUICKTHREADS_ARGS(sp, pu, pt, userf, only) \
    (QUICKTHREADS_ARGS_MD (QUICKTHREADS_ADJ(sp)), \
     QUICKTHREADS_SPUT (QUICKTHREADS_ADJ(sp), QUICKTHREADS_ONLY_INDEX, only), \
     QUICKTHREADS_SPUT (QUICKTHREADS_ADJ(sp), QUICKTHREADS_USER_INDEX, userf), \
     QUICKTHREADS_SPUT (QUICKTHREADS_ADJ(sp), QUICKTHREADS_ARGT_INDEX, pt), \
     QUICKTHREADS_SPUT (QUICKTHREADS_ADJ(sp), QUICKTHREADS_ARGU_INDEX, pu), \
     ((qt_t *)QUICKTHREADS_ADJ(sp)))

#endif


/* Push arguments for the varargs case.
   Has to be a function call because initialization is an expression
   and we need to loop to copy nbytes of stuff on to the stack.
   But that's probably OK, it's not terribly cheap, anyway. */

#ifdef QUICKTHREADS_VARGS_DEFAULT
#ifndef QUICKTHREADS_VARGS_MD0
#define QUICKTHREADS_VARGS_MD0(sp, vasize)	(sp)
#endif
#ifndef QUICKTHREADS_VARGS_MD1
#define QUICKTHREADS_VARGS_MD1(sp)	do { ; } while (0)
#endif

#ifndef QUICKTHREADS_VSTKBASE
  #error "Need base stack size for varargs functions."
#endif

/* Sometimes the stack pointer needs to munged a bit when storing
   the list of arguments. */
#ifndef QUICKTHREADS_VARGS_ADJUST
#define QUICKTHREADS_VARGS_ADJUST(sp)	(sp)
#endif

/* All things are put on the stack relative to the final value of
   the stack pointer. */
#ifdef QUICKTHREADS_GROW_DOWN
#define QUICKTHREADS_VADJ(sp)	(((char *)sp) - QUICKTHREADS_VSTKBASE)
#else
#define QUICKTHREADS_VADJ(sp)	(((char *)sp) + QUICKTHREADS_VSTKBASE)
#endif

extern qt_t *qt_vargs (qt_t *sp, int nbytes, void *vargs,
		       void *pt, qt_startup_t *startup,
		       qt_vuserf_t *vuserf, qt_cleanup_t *cleanup);

#ifndef QUICKTHREADS_VARGS
#define QUICKTHREADS_VARGS(sp, nbytes, vargs, pt, startup, vuserf, cleanup) \
      (qt_vargs (sp, nbytes, vargs, pt, startup, vuserf, cleanup))
#endif

#endif


/* Save the state of the thread and call the helper function
   using the stack of the new thread. */
typedef void *(qt_helper_t)(qt_t *old, void *a0, void *a1);
typedef void *(qt_block_t)(qt_helper_t *helper, void *a0, void *a1,
			  qt_t *newthread);

/* Rearrange the parameters so that things passed to the helper
   function are already in the right argument registers. */
#ifndef QUICKTHREADS_ABORT
extern void *qt_abort (qt_helper_t *h, void *a0, void *a1, qt_t *newthread);
/* The following does, technically, `return' a value, but the
   user had better not rely on it, since the function never
   returns. */ 
#define QUICKTHREADS_ABORT(h, a0, a1, newthread) \
    do { qt_abort (h, a0, a1, newthread); } while (0)
#endif

#ifndef QUICKTHREADS_BLOCK
extern void *qt_block (qt_helper_t *h, void *a0, void *a1,
		       qt_t *newthread);
#define QUICKTHREADS_BLOCK(h, a0, a1, newthread) \
    (qt_block (h, a0, a1, newthread))
#endif

#ifndef QUICKTHREADS_BLOCKI
extern void *qt_blocki (qt_helper_t *h, void *a0, void *a1,
			qt_t *newthread);
#define QUICKTHREADS_BLOCKI(h, a0, a1, newthread) \
    (qt_blocki (h, a0, a1, newthread))
#endif

#ifdef __cplusplus
}		/* Match `extern "C" {' at top. */
#endif

#endif // !defined(SC_USE_PTHREADS)
#endif /* ndef QUICKTHREADS_H */
