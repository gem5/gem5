/*
   + * QuickThreads -- Threads-building toolkit.
   + * Copyright (c) 1993 by David Keppel
   + *
   + * Permission to use, copy, modify and distribute this software and
   + * its documentation for any purpose and without fee is hereby
   + * granted, provided that the above copyright notice and this notice
   + * appear in all copies.  This software is provided as a
   + * proof-of-concept and for demonstration purposes; there is no
   + * representation about the suitability of this software for any
   + * purpose.
   + *

   + * PowerPC-Mach thread switching module.
   + * 
   + * This software is largely based on the original PowerPC-Linux porting
   + * developed by Ken Aaker <kenaaker@silverbacksystems.com>
   + * 
   + * Marco Bucci <marco.bucci@inwind.it>
   + * December 2002
   + *
   + */


#ifndef QUICKTHREADS_POWERPC_H
#define QUICKTHREADS_POWERPC_H


/*****************************************************************************
 *
 * DESCRIPTION
 *
 * This is the QuickThreads switching module implementation for PowerPC 
 * running under Mach kernel. It was developed and tested under MacOS X, that
 * is under Darwin (the UNIX-BSD fundation of MacOS X).
 *
 * Notice that the Mach PowerPC ABI (Application Binary Interface) [1] is
 * not the same than System V ABI [2] used by most of the LINUX PowerPC
 * implementations.
 *
 * IMPLEMENTATION NOTES
 *
 * 1) Porting on System V ABI
 * Excluding the variant argument calling convention, Mach and System V ABI
 * are enough similar and it could be possible to use some simple macro, to
 * adapt the code for both the ABIs. Actually, the only relevant difference 
 * is in the linkage area structure and in the position where the Link and
 * the Condition registers are saved. As to the calling conventions, there
 * are differences with floating points argument passing and with variant
 * argument lists. Notice that, on Mach, the caller's stack frame allocates
 * space to hold all arguments ([1] p.51), while on System V, the caller's
 * stack frame allocates space to hold just the arguments that don't fit into
 * registers ([2] p.3.18).
 *
 * 2) Variant argument list implementation
 * Variant argument calling on a RISC machine is not easy to implement since
 * parameters are passed via registers instead of via stack. In a general
 * variant argument implementation, the caller's stack must map the whole
 * parameter list following the rules related to the use of the GPR and FPR
 * parameter registers and the stack alignment ([1] p.54). 
 * This implementation is quite simple and not general. It works under the
 * hypothesis that arguments are 4-bytes aligned integers.
 *
 * 3) This heather file organisation
 * I preferred to not make confusion between macros that are needed (i.e.
 * directly used) by QuickThreads and internal "implementation" macros. You
 * will find QuickThreds macros in the end of this header. Sometime they just
 * refer to an analogous "internal" macro. On the top, there are the macros
 * that I used to make more clean (I hope) the implementation. I could include
 * some system heather (as to stack layout definitions, prologs and epilogs,
 * etc.), but I preferred to have a self-contained heather in order to make
 * all more clear for mantaining and for possible porting on another ABI.
 *
 *
 * REFERENCES
 *
 * [1] - Mach-O Runtime Architecture
 *       Runtime Concepts and Conventions for Mac OS X Programs
 *       Preliminary July 2002
 *
 * [2] - SYSTEM V APPLICATION BINARY INTERFACE
 *       PowerPC Processor Supplement
 *       September 1995
 *
 * On MacOS X, more documentation is available by installing the "Developer
 * Tools". Useful macros and documentation can be found in the system headers
 * files such as asm.h, asm_help.h etc. (see /usr/architecture/ppc/ or
 * /System/Library/Frameworks/Kernel.framework/Headers/architecture/ppc/).	

 *****************************************************************************/

/*****************************************************************************
 *
 *  PowerPC Mach-O Stack frame (see [1])
 *  
  
                      ................
                +                          +
                |                          | reserved
                +  CALLER'S LINKAGE AREA   +
                |                          | Caller's LR
                +                          + 
                |                          | Caller's CR
                +                          +
 backchain ->   |                          | Caller's backchain
                +==========================+
                |                          | FPR31
                +      FPR SAVE AREA       +
                       ..............
                +                          +
                |                          | FPRn
                +--------------------------+
                |                          | GPR31
                +      GPR SAVE AREA       +
                       ..............
                +                          +
                |                          | GPRn
                +--------------------------+
                |                          | 
                +      ALIGNMEBNT PAD      +
                       ..............
                +       (if needed)        +
                |                          |
                +--------------------------+
                |                          | 
                +   LOCAL VARIABLES AREA   +
                       ..............
                +                          +
                |                          |
                +--------------------------+
                |                          | PAR(n)
                +                          +
                |                          | 
                +      PARAMETER AREA      +
                       ..............
                +      for FUTURE call     +
                |                          | PAR(1)
                +                          +
 SP + 24 ->     |                          | PAR(0)
                +--------------------------+
 SP + 20 ->     |                          | Caller's TOC
                +                          + 
 SP + 16 ->     |                          | reserved
                +                          +
 SP + 12 ->     |                          | reserved
                +       LINKAGE AREA       +
 SP + 8 ->      |                          | LR callee-save for FUTURE call
                +                          + 
 SP + 4 ->      |                          | CR callee-save for FUTURE call
                +                          +
 SP ->          |                          | backchain
                +==========================+
                STACK TOP (lower address)
  
                     Stack grows down
                             |
                             V
 * NOTE:
 *
 * 1) Parameter are allocated in the CALLER's parameter area. This area must
 * be large enough to hold all parameters regardless if they are or not passed
 * in registers.
 *
 * The caller parameter area is used:
 * - by the caller, to store parameters to the callee that cannot fit in
 *  registers (no more parameter registers are available);
 * - by the callee, to save parameter registers (for istance because they are
 * needed for a further call).
 *
 * Obviously, the callee saves parameter registers, in the location in which
 * they are mapped on the caller's stack frame. So, be aware that, if
 * something else is stored in that location, it could be deleted after a call. 
 *
 * 2) The callee saves LR and CR in the caller's linkage area. All other
 * callee's state are saved in its own stack frame.
 *
 
 *****************************************************************************/ 


/*****************************************************************************
 * 
 * Stack initialization for a single argument thread
 *


 top + QUICKTHREADS_STKBASE ->           STACK BOTTOM (higher address)
                               +==========================+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
                               |                          |
                               +--------------------------+
 top + QUICKTHREADS_ONLY_INDEX * 4 ->    | only param               | PAR(3)
                               +                          +
 top + QUICKTHREADS_USER_INDEX * 4 ->    | userf param              | PAR(2)
                               +                          +
 top + QUICKTHREADS_ARGT_INDEX * 4 ->    | t param                  | PAR(1)
                               +                          +
 top + QUICKTHREADS_ARGU_INDEX * 4 ->    | u param                  | PAR(0)
                               +--------------------------+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
 top + QUICKTHREADS_RETURN_INDEX * 4 ->  | qt_start                 | LR save
                               +                          +
                                     ..............
                               +                          +
 top + QUICKTHREADS_BLOCKI_FRAME_SIZE -> | top + QUICKTHREADS_STKBASE         | backchain                     
                               +==========================+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
                               |                          |
                               +--------------------------+
                               |                          | 
                               +                          +
                                     ..............
                               +                          +
 top ->                        |top + QUICKTHREADS_BLOCKI_FRAME_SIZE| backchain
                               +==========================+
                               STACK TOP (lower address)
  
                                    Stack grows down
                                           |
                                           V

 *****************************************************************************
 *
 * Stack initialization for a variant argument thread
 *

 bottom ->                     STACK BOTTOM (higher address)
                               +==========================+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
 top + QUICKTHREADS_VSTKBASE ->          | arg(0)                   | PAR(4)
                               +--------------------------+
 top + QUICKTHREADS_CLEANUP_INDEX * 4 -> | cleanup param            | PAR(3)
                               +                          +
 top + QUICKTHREADS_USER_INDEX * 4 ->    | userf param              | PAR(2)  
                               +                          +
 top + QUICKTHREADS_VSTARTUP_INDEX * 4 ->| startup param            | PAR(1)
                               +                          +
 top + QUICKTHREADS_ARGT_INDEX * 4 ->    | t param                  | PAR(0)
                               +--------------------------+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
 top + QUICKTHREADS_RETURN_INDEX * 4 ->  | qt_start                 | LR save
                               +                          +
                                     ..............
 top + QUICKTHREADS_BLOCKI_FRAME_SIZE -> | top + QUICKTHREADS_STKBASE         | backchain                     
                               +==========================+
                               |                          |
                               +                          +
                                     ..............
                               +                          +
                               |                          |
                               +--------------------------+
                               |                          | 
                               +                          +
                                     ..............
                               +                          +
 top ->                        |top + QUICKTHREADS_BLOCKI_FRAME_SIZE| backchain
                               +==========================+
                               STACK TOP (lower address)
  
                                    Stack grows down
                                          |
                                          V

* NOTE:
*
* Parameters are passed to "qt_start" or to "qt_vstart" putting them into
* the stack frames of "qt_start" or "qt_vstart" themselves. This not a
* conventional parameter passing because parameters should be put into the
* caller's stack, not into the callee's one. Actually  we must consider
* that as a preload of the parameter area that "qt_start" or "qt_vstart"
* will use for their own calls.
*  Be aware of the fact that, during a call, the caller's parameter area is,
* in a certain sense, volatile. In facts, the callee can save parameter
* registers on the caller's parameter area.
*
 *****************************************************************************/ 


/*****************************************************************************
 
   Define PowerPC Mach-O related macros
 
 *****************************************************************************/ 



typedef unsigned long PPC_W;

/* Stack pointer must always be a multiple of 16 */
#define	PPC_STACK_INCR	16		
#define	PPC_ROUND_STACK(length)	\
	(((length)+PPC_STACK_INCR-1) & ~(PPC_STACK_INCR-1))


#define PPC_LINKAGE_AREA 24					
#define PPC_CR_SAVE 4
#define PPC_LR_SAVE 8

#define PPC_PARAM_AREA(n) (4*(n))

#define PPC_GPR_SAVE_AREA (4*19)		/* GPR13-GPR31 must be saved */
#define PPC_FPR_SAVE_AREA (8*18)		/* FPR14-FPR31 must be saved */

/* Define parameter offset on the stack.
 * NOTICE: Parameters are numbered 0, 1, ..., n. 
*/
#define PPC_PAR(i) (PPC_LINKAGE_AREA+(i)*4)

/*****************************************************************************
 
   Define stack frames
 
 *****************************************************************************/ 


/* Define the "qt_blocki" and "qt_abort" stack frame. We use the same stack
 * frame for both. 
 *

 top + S ->  
                        +==========================+
 top + S - 4 ->         |                          | GPR31 
                        +      GPR SAVE AREA       +
                               ..............
                        +                          +
 top + S - 19 * 4 ->    |                          | GPR13
                        +--------------------------+
                        |                          | 
                        +      ALIGNMEBNT PAD      +
                               ..............
                        +       (if needed)        +
                        |                          |
                        +--------------------------+
                        |                          |
                        +                          +
                        |                          |
                        +      PARAMETER AREA      +
                        |                          |
                        +                          +
 top + 24 ->            |                          |
                        +--------------------------+
                        |                          |
                        +       LINKAGE AREA       +
 top ->                 |                          |
                        +==========================+
 */

#define QUICKTHREADS_BLOCKI_FRAME_SIZE \
	PPC_ROUND_STACK(PPC_LINKAGE_AREA+PPC_PARAM_AREA(4)+PPC_GPR_SAVE_AREA)

/* Offset to the base of the GPR save area. Save from GPR13 to GPR31
 * increasing address. 
 */
#define QUICKTHREADS_BLOCKI_GPR_SAVE(i) (QUICKTHREADS_BLOCKI_FRAME_SIZE-4+(i-31)*4)



/* Define the "qt_block" stack frame. Notice that since "qt_black" calls
 * "qt_blocki", GPR registers are saved into "qt_blocki" stack frame.
 *

 top + S ->  
                        +==========================+
 top + S - 8 ->         |                          | FPR31 
                        +      FPR SAVE AREA       +
                               ..............
                        +                          +
 top + S - 18 * 8 ->    |                          | FPR14
                        +--------------------------+
                        |                          | 
                        +      ALIGNMEBNT PAD      +
                               ..............
                        +       (if needed)        +
                        |                          |
                        +--------------------------+
                        |                          |
                        +                          +
                        |                          |
                        +      PARAMETER AREA      +
                        |                          |
                        +                          +
 top + 24 ->            |                          |
                        +--------------------------+
                        |                          |
                        +       LINKAGE AREA       +
 top ->                 |                          |
                        +==========================+
 */

#define QUICKTHREADS_BLOCK_FRAME_SIZE \
	PPC_ROUND_STACK(PPC_LINKAGE_AREA+PPC_PARAM_AREA(4)+PPC_FPR_SAVE_AREA)

/* Offset to the location where registers are saved.
 */
#define QUICKTHREADS_BLOCK_FPR_SAVE(i) (QUICKTHREADS_BLOCK_FRAME_SIZE-8+(i-31)*8)


/* Define the "qt_start" frame size. It consists just of the linkage area and 
 * the parameter area. 
 *

                        +==========================+
                        |                          | 
                        +      ALIGNMEBNT PAD      +
                               ..............
                        +       (if needed)        +
                        |                          |
                        +--------------------------+
                        |                          | only par
                        +                          +
                        |                          | userf par
                        +      PARAMETER AREA      +
                        |                          | t par
                        +                          +
 top + 24 ->            |                          | u par
                        +--------------------------+
                        |                          |
                        +       LINKAGE AREA       +
 top ->                 |                          |
                        +==========================+

 */
#define QUICKTHREADS_START_FRAME_SIZE PPC_ROUND_STACK(PPC_LINKAGE_AREA+PPC_PARAM_AREA(4))



/* Define the "qt_vstart" frame. It consists of the linkage area, the fix parameter 
 * area, the variant argument list and a local variable area used in "qt_vstart"
 * implementation.
 *

 backchain ->  
                        +==========================+
 backchain - 4 ->       |                          | 
                        +   LOCAL VARIABLES AREA   +
                               ..............
                        +                          +
                        |                          |
                        +--------------------------+
                        |                          | 
                        +      ALIGNMEBNT PAD      +
                               ..............
                        +       (if needed)        +
                        |                          |
                        +--------------------------+
                        |                          | arg(n)
                        +                          +
                        |                          | 
                        +  VARIABLE ARGUMENT LIST  +
                               ..............
                        +      for userf call      +
                        |                          | arg(1)
                        +                          +
 top + 24 + 16 ->       |                          | arg(0)
                        +--------------------------+
                        |                          | cleanup par
                        +                          +
                        |                          | userf par
                        +      PARAMETER AREA      +
                        |                          | startup par
                        +                          +
 top + 24 ->            |                          | t par
                        +--------------------------+
                        |                          |
                        +       LINKAGE AREA       +
 top ->                 |                          |
                        +==========================+

 */
#define QUICKTHREADS_VARGS_LOCAL_AREA (4*4)		/* local variable area */

/* The offset the stack will be moved back before to call "userf(...)".
 * The linckage area must be moved to be adiacent to the part of the variant
 * argument list that is in the stack.
 */
#define QUICKTHREADS_VARGS_BKOFF PPC_PARAM_AREA(4)

#define QUICKTHREADS_VSTART_FRAME_SIZE(varbytes) \
	PPC_ROUND_STACK(PPC_LINKAGE_AREA+PPC_PARAM_AREA(4)+(varbytes)+ \
		QUICKTHREADS_VARGS_LOCAL_AREA)

/* Offset to the base of the varian argument list */
#define QUICKTHREADS_VSTART_LIST_BASE (PPC_LINKAGE_AREA+PPC_PARAM_AREA(4))


/* Notice that qt_start and qt_vstart have no parameters, actually their
 * parameters are written in their stack frame during thread initialization
 */
extern void qt_start(void);
extern void qt_vstart(void);



/* Offset (in words) of the location where the block routine saves its return
 * address (i.e. LR). SP points the top of the block routine stack and,
 * following ppc calling conventions, the return address is saved in the
 * previous (caller's) stack frame.
 */
#define QUICKTHREADS_RETURN_INDEX ((QUICKTHREADS_BLOCKI_FRAME_SIZE+PPC_LR_SAVE)/sizeof(PPC_W))

/* static variable used to get the stack bottom in "VARGS" initialization */
/* static void *qt_sp_bottom_save; */

#define QUICKTHREADS_ARG_INDEX(i) ((QUICKTHREADS_BLOCKI_FRAME_SIZE+PPC_PAR(i))/sizeof(PPC_W))

/*****************************************************************************

	QuickThreads needed definitions

 *****************************************************************************/


#define QUICKTHREADS_GROW_DOWN
#define QUICKTHREADS_STKALIGN	PPC_STACK_INCR
typedef PPC_W qt_word_t;


/* This macro is used by "QUICKTHREADS_ARGS" to initialize a single argument thread.
 * - set "qt_start" as the "qt_block" or "qt_blocki" return address;
 * - set the top of the stack backchain;
 * - set the next backchain (not needed, but just to be "clean").   
 */
#define QUICKTHREADS_ARGS_MD(sp) \
	(QUICKTHREADS_SPUT (sp, QUICKTHREADS_RETURN_INDEX, qt_start), \
	QUICKTHREADS_SPUT (sp, 0, sp+QUICKTHREADS_BLOCKI_FRAME_SIZE), \
	QUICKTHREADS_SPUT (sp, QUICKTHREADS_BLOCKI_FRAME_SIZE/sizeof(PPC_W), \
		sp+QUICKTHREADS_BLOCKI_FRAME_SIZE+QUICKTHREADS_START_FRAME_SIZE))


/* This macro is used by "QUICKTHREADS_VARGS" to initialize a variant argument thread. 
 * It returns the pointer to the top of the argument list.
 * We also use it to get the stack bottom via a static variable. This is a bit 
 * "dirty", it could be better to do it in "qt_vargs", but we don't want change 
 * anything out of this file.
 * We need the stack bottom to allocate a local variable area used by
 * "qt_vstart". 
 */
#define QUICKTHREADS_VARGS_MD0(sp, varbytes) \
  ((qt_sp_bottom_save = sp), \
  ((qt_t *)(((char *)(sp)) - \
		(QUICKTHREADS_VSTART_FRAME_SIZE(varbytes)-QUICKTHREADS_VSTART_LIST_BASE))))


/* This macro is used by "QUICKTHREADS_VARGS" to initialize a variant argument thread.
 * - set "qt_start" as the "qt_block" or "qt_blocki" return address;
 * - set the top of the stackback chain;
 * - set the next backchain (it points the stack botton).   
 */
#define QUICKTHREADS_VARGS_MD1(sp) \
	(QUICKTHREADS_SPUT (sp, QUICKTHREADS_RETURN_INDEX, qt_vstart), \
	QUICKTHREADS_SPUT (sp, 0, sp+QUICKTHREADS_BLOCKI_FRAME_SIZE), \
	QUICKTHREADS_SPUT (sp, (QUICKTHREADS_BLOCKI_FRAME_SIZE)/sizeof(PPC_W), \
		qt_sp_bottom_save))


/* Activate "qt_vargs" as the initialization routine for the variant 
 * argument threads
 */ 
#define QUICKTHREADS_VARGS_DEFAULT 

/* Override "qt_vargs" with "qt_vargs_stdarg".
 * On LinuxPPC "qt_vargs" doesn't work, "qt_vargs_stdarg" uses a more
 * standard way to retrieve arguments from the variant list.
 */
#define QUICKTHREADS_VARGS(sp, nbytes, vargs, pt, startup, vuserf, cleanup) \
      ((qt_t *)qt_vargs_stdarg (sp, nbytes, vargs, pt, startup, vuserf, cleanup))


/* This macro is used by "QUICKTHREADS_ADJ(sp)" to get the stack top form the stack
 * bottom during a single argument thread initialization.
 * It is the space we need to allocate for a single argument thread: the stack 
 * frame for the block routine ("qt_block" or "qt_blocki") and for "qt_start".
 */
#define QUICKTHREADS_STKBASE \
	(QUICKTHREADS_BLOCKI_FRAME_SIZE+QUICKTHREADS_START_FRAME_SIZE)

/* This macro is used by "QUICKTHREADS_VADJ(sp)" to get the stack top from the base
 * of the variant argument list during a variant argument thread initialization.
 */
#define QUICKTHREADS_VSTKBASE	(QUICKTHREADS_BLOCKI_FRAME_SIZE+QUICKTHREADS_VSTART_LIST_BASE)

/* The *index* (positive offset) of where to put each value. */

#define QUICKTHREADS_ARGU_INDEX	QUICKTHREADS_ARG_INDEX(0)
#define QUICKTHREADS_ARGT_INDEX	QUICKTHREADS_ARG_INDEX(1)
#define QUICKTHREADS_USER_INDEX	QUICKTHREADS_ARG_INDEX(2)
#define QUICKTHREADS_ONLY_INDEX	QUICKTHREADS_ARG_INDEX(3)


#define QUICKTHREADS_VARGT_INDEX		QUICKTHREADS_ARG_INDEX(0)
#define QUICKTHREADS_VSTARTUP_INDEX	QUICKTHREADS_ARG_INDEX(1)
#define QUICKTHREADS_VUSERF_INDEX		QUICKTHREADS_ARG_INDEX(2)
#define QUICKTHREADS_VCLEANUP_INDEX	QUICKTHREADS_ARG_INDEX(3)

#endif /* ndef QUICKTHREADS_POWERPC_H */

