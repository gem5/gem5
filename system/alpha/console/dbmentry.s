
/* taken from ebfw/rom/dbmentry.s */

#define EB164 
/*#ifndef LINT
.data
.asciiz "$Id: dbmentry.s,v 1.1.1.1 1997/10/30 23:27:12 verghese Exp $"
.text
#endif
*/
/*
 * Debug Monitor Entry code
 */

#ifndef MAKEDEPEND
#include "ev5_impure.h"
#include "cserve.h"
#include "fromHudsonOsf.h"
#endif

//#include "paldefs.h"
#include "regdefs.h"
#include "eb164.h"
//#include "ledcodes.h"

	.text

/* return address and padding to octaword align */
#define STARTFRM 16

        .globl  __start
        .ent    __start, 0
__start:
_entry: 
	br      t0, 2f			# get the current PC
2:	ldgp    gp, 0(t0)               # init gp


#ifdef original_xxm	
	lda	a2, CSERVE_K_RD_IMPURE
	call_pal PAL_CSERVE_ENTRY
	lda	v0,  CNS_Q_BASE(v0)

	# Add KSEG offset to the impure area
	subq	zero, 1, t0
	sll	t0, 42, t0
	addq	t0, v0, v0

	lda	t0, CNS_Q_SIGNATURE(v0)
	bic 	t0, 0x07, t0		# Clear bottom 3 bits to avoid
					# allignment errors if the
					# impure area is total rubbish
	ldq	t0, 0x00(t0)
	srl	t0, 16, t0		# Shift signature into bottom 16 bits.
	lda	t6, 0xDECB(zero)	# Load the expected valid signature.
	zap	t6, 0xFC, t6		# Clear the upper bits.
	cmpeq	t0, t6, t0		# Is this a valid signature?
	beq	t0, 1f			# Not valid, don't trust input params.

/*
 *	Init the stack at the first 8K boundary
 *	below the top of memory.
 */
	lda	t0, CNS_Q_MEM_SIZE(v0)
	ldq	t0, 0x00(t0)		# Load memory size.
	subq	t0, 1, t0		# Last address in memory
	srl	t0, 13, t0		# Align to first 8KB boundary
	sll	t0, 13, sp		# below the top of memory.
	br	zero, 2f

/*
 *	If memory size was not passed in via the
 *	PALcode impure data use the system specific
 *	MINIMUM_SYSTEM_MEMORY definition.
 */
1:
	lda	sp, (MINIMUM_SYSTEM_MEMORY&0xffff)(zero)
	ldah	sp, ((MINIMUM_SYSTEM_MEMORY+0x8000)>>16)(sp)
	lda	t0, (8*1024)(zero)	# Allow for 8KB guard page.
	subq	sp, t0, sp

2:

#endif /* original_xxm */

	
	/*
	 * SimOS. Stack pointer is start of a valid phys or KSEG page
	 */
	
	bis	sp,sp,s0 /* save sp */
		
slave:	lda	v0,(8*1024)(sp) /* end of page  */

	subq	zero, 1, t0
	sll	t0, 42, t0
	bis	t0, v0, sp
	
#ifdef original_xxm		
	# Add KSEG offset to the stack pointer
	subq	zero, 1, t0
	sll	t0, 42, t0
	addq	t0, sp, sp
#endif
	
	lda     sp, -STARTFRM(sp)	# Create a stack frame
	stq     ra, 0(sp)		# Place return address on the stack

	.mask   0x84000000, -8
	.frame  sp, STARTFRM, ra

/*
 *	Enable the Floating Point Unit
 */
	lda	a0, 1(zero)
	call_pal PAL_WRFEN_ENTRY

/*
 *	Every good C program has a main()
 */

	beq	s0,master

	call_pal PAL_WHAMI_ENTRY
	bis	v0,v0,a0
	jsr	ra, SlaveLoop
master:	
	jsr	ra, main



/*
 *	The Debug Monitor should never return.
 *	However, just incase...
 */
	ldgp	gp, 0(ra)
	bsr	zero, _exit

.end	__start



        .globl  _exit
        .ent    _exit, 0
_exit:

	ldq     ra, 0(sp)		# restore return address
	lda	sp, STARTFRM(sp)	# prune back the stack
	ret	zero, (ra)		# Back from whence we came
.end	_exit

        	.globl	cServe
	.ent	cServe 2
cServe:
	.option	O1
	.frame	sp, 0, ra
	call_pal PAL_CSERVE_ENTRY
	ret	zero, (ra)
	.end	cServe

	.globl	wrfen
	.ent	wrfen 2
wrfen:
	.option	O1
	.frame	sp, 0, ra
	call_pal PAL_WRFEN_ENTRY
	ret	zero, (ra)
	.end	wrfen
       	.globl	consoleCallback
	.ent	consoleCallback 2
consoleCallback:               
        br      t0, 2f			# get the current PC
2:	ldgp    gp, 0(t0)               # init gp
        lda     sp,-64(sp)
        stq     ra,0(sp)
        jsr     CallBackDispatcher
        ldq     ra,0(sp)
        lda     sp,64(sp)
        ret     zero,(ra)
        .end    consoleCallback
        
        
       	.globl	consoleFixup
	.ent	consoleFixup 2
consoleFixup:   
        br      t0, 2f			# get the current PC
2:	ldgp    gp, 0(t0)               # init gp
        lda     sp,-64(sp)
        stq     ra,0(sp)
        jsr     CallBackFixup
        ldq     ra,0(sp)
        lda     sp,64(sp)
        ret     zero,(ra)
        .end    consoleFixup



	.globl	SpinLock
	.ent	SpinLock 2
SpinLock:	
1:
	ldq_l	a1,0(a0)		# interlock complete lock state
	subl	ra,3,v0			# get calling addr[31:0] + 1
	blbs	a1,2f			# branch if lock is busy
	stq_c	v0,0(a0)		# attempt to acquire lock
	beq	v0,2f			# branch if lost atomicity
	mb				# ensure memory coherence
	ret	zero,(ra)		# return to caller (v0 is 1)
2:
	br	zero,1b
	.end	SpinLock

        .globl	loadContext
	.ent	loadContext 2
loadContext:
	.option	O1
	.frame	sp, 0, ra
	call_pal PAL_SWPCTX_ENTRY
	ret	zero, (ra)
	.end	loadContext

	
	.globl	SlaveSpin          # Very carefully spin wait 
	.ent	SlaveSpin 2        # and swap context without
SlaveSpin:                         # using any stack space
	.option	O1
	.frame	sp, 0, ra
        mov a0, t0                 # cpu number 
        mov a1, t1                 # cpu rpb pointer (virtual)
        mov a2, t2                 # what to spin on
       
test:   ldl  t3, 0(t2)
        beq  t3, test
        zapnot t1,0x1f,a0          # make rpb physical 
	call_pal PAL_SWPCTX_ENTRY  # switch to pcb
        mov t0, a0                 # setup args for SlaveCmd
        mov t1, a1
        jsr SlaveCmd               # call SlaveCmd
	ret	zero, (ra)         # Should never be reached
	.end	SlaveSpin


