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

#include "dc21164FromGasSources.h"	// DECchip 21164 specific definitions
#include "ev5_defs.h"
#include "fromHudsonOsf.h"		// OSF/1 specific definitions
#include "fromHudsonMacros.h"		// Global macro definitions

/*
 * args:
 *   a0: here
 *   a1: boot location
 *   a2: CSERVE_J_KTOPAL
 *   a3: restrart_pv
 *   a4: vptb
 *   a5: my_rpb
 *
 * SRM Console Architecture III 3-26
 */

        .global	palJToSlave
        .text	3
palJToSlave:

        ALIGN_BRANCH

        bis	a3, zero, pv
        bis	zero, zero, t11
        bis	zero, zero, ra

        /* Point the Vptbr to a2 */

        mtpr	a4, mVptBr	// Load Mbox copy
        mtpr	a4, iVptBr	// Load Ibox copy
        STALL			// don't dual issue the load with mtpr -pb

        /* Turn on superpage mapping in the mbox and icsr */
        lda	t0, (2<<MCSR_V_SP)(zero) // Get a '10' (binary) in MCSR<SP>
        STALL			// don't dual issue the load with mtpr -pb
        mtpr	t0, mcsr	// Set the super page mode enable bit
        STALL			// don't dual issue the load with mtpr -pb

        lda	t0, 0(zero)
        mtpr	t0, dtbAsn
        mtpr	t0, itbAsn

        LDLI	(t1,0x20000000)
        STALL			// don't dual issue the load with mtpr -pb
        mfpr	t0, icsr	// Enable superpage mapping
        STALL			// don't dual issue the load with mtpr -pb
        bis	t0, t1, t0
        mtpr	t0, icsr

        STALL			// Required stall to update chip ...
        STALL
        STALL
        STALL
        STALL

        ldq_p	s0, PCB_Q_PTBR(a5)
        sll	s0, VA_S_OFF, s0 // Shift PTBR into position
        STALL			// don't dual issue the load with mtpr -pb
        mtpr	s0, ptPtbr	// PHYSICAL MBOX INST -> MT PT20 IN 0,1
        STALL			// don't dual issue the load with mtpr -pb
        ldq_p	sp, PCB_Q_KSP(a5)

        mtpr	zero, dtbIa	// Flush all D-stream TB entries
        mtpr	zero, itbIa	// Flush all I-stream TB entries

        mtpr	a1, excAddr	// Load the dispatch address.

        STALL			// don't dual issue the load with mtpr -pb
        STALL			// don't dual issue the load with mtpr -pb
        mtpr	zero, dtbIa	// Flush all D-stream TB entries
        mtpr	zero, itbIa	// Flush all I-stream TB entries
        br	zero, 2f

        ALIGN_BLOCK

2:	NOP
        mtpr	zero, icFlush	// Flush the icache.
        NOP
        NOP

        NOP			// Required NOPs ... 1-10
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP			// Required NOPs ... 11-20
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP			// Required NOPs ... 21-30
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP			// Required NOPs ... 31-40
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP			// Required NOPs ... 41-44
        NOP
        NOP
        NOP

        hw_rei_stall		// Dispatch to kernel

