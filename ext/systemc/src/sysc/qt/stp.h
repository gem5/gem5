#ifndef STP_H
#define STP_H

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

typedef struct stp_t stp_t;

/* Each thread starts by calling a user-supplied function of this
   type. */

typedef void (stp_userf_t)(void *p0);

/* Call this before any other primitives. */
extern void stp_init();

/* When one or more threads are created by the main thread,
   the system goes multithread when this is called.  It is done
   (no more runable threads) when this returns. */

extern void stp_start (void);

/* Create a thread and make it runable.  When the thread starts
   running it will call `f' with arguments `p0' and `p1'. */

extern void stp_create (stp_userf_t *f, void *p0);

/* The current thread stops running but stays runable.
   It is an error to call `stp_yield' before `stp_start'
   is called or after `stp_start' returns. */

extern void stp_yield (void);

/* Like `stp_yield' but the thread is discarded.  Any intermediate
   state is lost.  The thread can also terminate by simply
   returning. */

extern void stp_abort (void);


#endif /* ndef STP_H */
