/*
 * Copyright (c) 2003, 2004
 * The Regents of The University of Michigan
 * All Rights Reserved
 *
 * This code is part of the M5 simulator.
 *
 * Permission is granted to use, copy, create derivative works and
 * redistribute this software and such derivative works for any purpose,
 * so long as the copyright notice above, this grant of permission, and
 * the disclaimer below appear in all copies made; and so long as the
 * name of The University of Michigan is not used in any advertising or
 * publicity pertaining to the use or distribution of this software
 * without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
 * UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
 * WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
 * IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
 * THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
 * INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *
 * Modified for M5 by: Ali G. Saidi
 */

/*
 * Copyright 1993 Hewlett-Packard Development Company, L.P.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Debug Monitor Entry code
 */
#include "fromHudsonOsf.h"

        .extern myAlphaAccess
        .text

/* return address and padding to octaword align */
#define STARTFRM 16

        .globl  _start
        .ent    _start, 0
_start:
_entry:
        br      t0, 2f			# get the current PC
2:	ldgp    gp, 0(t0)               # init gp

/* Processor 0 start stack frame is begining of physical memory (0)
   Other processors spin here waiting to get their stacks from
   Processor 0, then they can progress as normal.
*/
        call_pal PAL_WHAMI_ENTRY
        beq v0, cpuz
        ldq  t3, m5AlphaAccess
        addq t3,0x70,t3 # *** If offset in console alpha access struct changes
                        # This must be changed as well!
        bis  zero,8,t4
        mulq t4,v0,t4
        addq t3,t4,t3
        ldah a0, 3(zero)  # load arg0 with 65536*3
cpuwait: .long 0x6000002  # jsr quiesceNs
        ldq  t4, 0(t3)
        beq  t4, cpuwait
        bis  t4,t4,sp


cpuz:	bis	sp,sp,s0 /* save sp */

slave:	lda	v0,(8*1024)(sp) /* end of page  */

        subq	zero, 1, t0
        sll	t0, 42, t0
        bis	t0, v0, sp

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

/* If stack pointer was 0, then this is CPU0*/
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

.end	_start



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
        ldah a0, 3(zero)  # load arg0 with 65536
test:   .long 0x6000002  # jsr quiesceNs     # wait 65us*3
        ldl  t3, 0(t2)
        beq  t3, test
        zapnot t1,0x1f,a0          # make rpb physical
        call_pal PAL_SWPCTX_ENTRY  # switch to pcb
        mov t0, a0                 # setup args for SlaveCmd
        mov t1, a1
        jsr SlaveCmd               # call SlaveCmd
        ret	zero, (ra)         # Should never be reached
        .end	SlaveSpin


