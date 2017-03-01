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

/*
 * This file (pa-risc.h) is part of the port of QuickThreads for the
 * PA-RISC 1.1 architecture.  This file is a machine dependent header
 * file.  It was written in 1994 by Uwe Reder
 * (`uereder@cip.informatik.uni-erlangen.de') for the Operating Systems
 * Department (IMMD4) at the University of Erlangen/Nuernberg Germany.
 */


#ifndef QUICKTHREADS_PA_RISC_H
#define QUICKTHREADS_PA_RISC_H

#if 0
#include <qt.h>
#endif

/* size of an integer-register (32 bit) */
typedef unsigned long qt_word_t;

/* PA-RISC's stack grows up */
#define QUICKTHREADS_GROW_UP

/* Stack layout on PA-RISC according to PA-RISC Procedure Calling Conventions:

    Callee-save registers are: gr3-gr18, fr12-fr21.
    Also save gr2, return pointer.

    +---
    | fr12          Each floating register is a double word (8 bytes).
    | fr13          Floating registers are only saved if `qt_block' is
    | fr14          called, in which case it saves the floating-point
    | fr15          registers then calls `qt_blocki' to save the integer
    | fr16	    registers.
    | fr17
    | fr18
    | fr19
    | fr20
    | fr21
    | <arg word 3>  fixed arguments (must be allocated; may remain unused)
    | <arg word 2>
    | <arg word 1>
    | <arg word 0>
    | <LPT>         frame marker
    | <LPT'>
    | <RP'>
    | <Current RP>
    | <Static Link>
    | <Clean Up>
    | <RP''>
    | <Previous SP>
    +---
    | gr3           word each (4 bytes)
    | gr4
    | gr5
    | gr6
    | gr7
    | gr8
    | gr9
    | gr10
    | gr11
    | gr12
    | gr13
    | gr14
    | gr15
    | gr16
    | gr17
    | gr18
    | <16 bytes filled in (sp has to be 64-bytes aligned)>
    | <arg word 3>  fixed arguments (must be allocated; may remain unused)
    | <arg word 2>
    | <arg word 1>
    | <arg word 0>
    | <LPT>         frame marker
    | <LPT'>
    | <RP'>
    | <Current RP>
    | <Static Link>
    | <Clean Up>
    | <RP''>
    | <Previous SP>
    +---            <--- sp
*/

/* When a never-before-run thread is restored, the return pc points
   to a fragment of code that starts the thread running.  For
   non-vargs functions, it just calls the client's `only' function.
   For varargs functions, it calls the startup, user, and cleanup
   functions. */

/* Note: Procedue Labels on PA-RISC

   <--2--><-------28---------><1-><1->
   -----------------------------------
   | SID |    Adress Part    | L | X |
   -----------------------------------

   On HP-UX the L field is used to flag wheather the procedure
   label (plabel) is a pointer to an LT entry or to the entry point
   of the procedure (PA-RISC Procedure Calling Conventions Reference
   Manual, 5.3.2 Procedure Labels and Dynamic Calls). */

#define QUICKTHREADS_PA_RISC_READ_PLABEL(plabel) \
    ( (((int)plabel) & 2) ? \
        ( (*((int *)(((int)plabel) & 0xfffffffc)))) : ((int)plabel) )

/* Stack must be 64 bytes aligned. */
#define QUICKTHREADS_STKALIGN (64)

/* Internal helper for putting stuff on stack (negative index!). */
#define QUICKTHREADS_SPUT(top, at, val)   \
    (((qt_word_t *)(top))[-(at)] = (qt_word_t)(val))

/* Offsets of various registers which are modified on the stack.
   rp (return-pointer) has to be stored in the frame-marker-area
   of the "older" stack-segment. */

#define QUICKTHREADS_crp  (12+4+16+5)
#define QUICKTHREADS_15   (12+4+4)
#define QUICKTHREADS_16   (12+4+3)
#define QUICKTHREADS_17   (12+4+2)
#define QUICKTHREADS_18   (12+4+1)


/** This stuff is for NON-VARARGS. **/

/* Stack looks like this (2 stack frames):

    <--- 64-bytes aligned --><------- 64-bytes aligned ------------>
   |                        ||                                      |
    <--16--><------48-------><----16*4-----><--16-><------48------->
   ||      |                ||             |      |                ||
   ||filler|arg|frame-marker||register-save|filler|arg|frame-marker||
   ------------------------------------------------------------------
 */

#define QUICKTHREADS_STKBASE  (16+48+(16*sizeof(qt_word_t))+16+48)

/* The index, relative to sp, of where to put each value. */
#define QUICKTHREADS_ONLY_INDEX   (QUICKTHREADS_15)
#define QUICKTHREADS_USER_INDEX   (QUICKTHREADS_16)
#define QUICKTHREADS_ARGT_INDEX   (QUICKTHREADS_17)
#define QUICKTHREADS_ARGU_INDEX   (QUICKTHREADS_18)

extern void qt_start(void);
#define QUICKTHREADS_ARGS_MD(sp)  \
    (QUICKTHREADS_SPUT (sp, QUICKTHREADS_crp, QUICKTHREADS_PA_RISC_READ_PLABEL(qt_start)))


/** This is for VARARGS. **/

#define QUICKTHREADS_VARGS_DEFAULT

/* Stack looks like this (2 stack frames):

    <------ 64-bytes aligned -------><--------- 64-bytes aligned ---------->
   |                                ||                                      |
    <---?--><--?---><16><----32-----><----16*4-----><-16--><16><----32----->
   ||      |       |   |            ||             |      |   |            ||
   ||filler|varargs|arg|frame-marker||register-save|filler|arg|frame-marker||
   --------------------------------------------------------------------------
 */

/* Sp is moved to the end of the first stack frame. */
#define QUICKTHREADS_VARGS_MD0(sp, vasize) \
    ((qt_t *)(((char *)sp) + QUICKTHREADS_STKROUNDUP(vasize + 4*4 + 32)))

/* To reach the arguments from the end of the first stack frame use 32
   as a negative adjustment. */
#define QUICKTHREADS_VARGS_ADJUST(sp)	((qt_t *)(((char *)sp) - 32))

/* Offset to reach the end of the second stack frame. */
#define QUICKTHREADS_VSTKBASE	((16*sizeof(qt_word_t)) + 16 + 4*4 + 32)

extern void qt_vstart(void);
#define QUICKTHREADS_VARGS_MD1(sp) \
    (QUICKTHREADS_SPUT (sp, QUICKTHREADS_crp, QUICKTHREADS_PA_RISC_READ_PLABEL(qt_vstart)))

#define QUICKTHREADS_VARGT_INDEX      (QUICKTHREADS_15)
#define QUICKTHREADS_VSTARTUP_INDEX   (QUICKTHREADS_16)
#define QUICKTHREADS_VUSERF_INDEX     (QUICKTHREADS_17)
#define QUICKTHREADS_VCLEANUP_INDEX   (QUICKTHREADS_18)

#endif /* ndef QUICKTHREADS_PA_RISC_H */
