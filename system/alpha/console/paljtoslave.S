#include	"dc21164FromGasSources.h"	// DECchip 21164 specific definitions
#include    "ev5_defs.h"
#include	"fromHudsonOsf.h"		// OSF/1 specific definitions
#include	"fromHudsonMacros.h"	// Global macro definitions
#include	"ev5_impure.h"	// Scratch & logout area data structures
#include	"platform.h"	// Platform specific definitions


        .global	palJToSlave
        .text	3

        /*
         * args:
            a0:	 here
            a1:	 boot location
            a2:	 CSERVE_J_KTOPAL
            a3:	 restrart_pv
            a4:	 vptb
            a5:  my_rpb

         */
palJToSlave:

        /*
         * SRM Console Architecture III 3-26
         */

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
        mtpr	t0, mcsr		// Set the super page mode enable bit
        STALL			// don't dual issue the load with mtpr -pb

        lda	t0, 0(zero)
        mtpr	t0, dtbAsn
        mtpr	t0, itbAsn

        LDLI	(t1,0x20000000)
        STALL			// don't dual issue the load with mtpr -pb
        mfpr	t0, icsr		// Enable superpage mapping
        STALL			// don't dual issue the load with mtpr -pb
        bis	t0, t1, t0
        mtpr	t0, icsr

        STALL                           // Required stall to update chip ...
        STALL
        STALL
        STALL
        STALL

        ldq_p	s0, PCB_Q_PTBR(a5)
        sll	s0, VA_S_OFF, s0	// Shift PTBR into position
        STALL			// don't dual issue the load with mtpr -pb
        mtpr	s0, ptPtbr		// PHYSICAL MBOX INST -> MT PT20 IN 0,1
        STALL			// don't dual issue the load with mtpr -pb
        ldq_p	sp, PCB_Q_KSP(a5)

        //mtpr	a0, excAddr		// Load the dispatch address.
        //STALL			// don't dual issue the load with mtpr -pb
        //bis	a3, zero, a0		// first free PFN
        // ldq_p	a1, PCB_Q_PTBR(a5)	// ptbr

        //ldq_p	a2, 24(zero)		// argc
        //ldq_p	a3, 32(zero)		// argv
        //ldq_p	a4, 40(zero)		// environ
        //lda	a5, 0(zero)		// osf_param
        //STALL			// don't dual issue the load with mtpr -pb
        mtpr	zero, dtbIa		// Flush all D-stream TB entries
        mtpr	zero, itbIa		// Flush all I-stream TB entries


        mtpr	a1, excAddr		// Load the dispatch address.

        STALL			// don't dual issue the load with mtpr -pb
        STALL			// don't dual issue the load with mtpr -pb
        mtpr	zero, dtbIa		// Flush all D-stream TB entries
        mtpr	zero, itbIa		// Flush all I-stream TB entries
        br	zero, 2f

        ALIGN_BLOCK

2:	NOP
        mtpr	zero, icFlush		// Flush the icache.
        NOP
        NOP

        NOP				// Required NOPs ... 1-10
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP                           // Required NOPs ... 11-20
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP                           // Required NOPs ... 21-30
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        NOP                           // Required NOPs ... 31-40
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP



        NOP				// Required NOPs ... 41-44
        NOP
        NOP
        NOP

        hw_rei_stall				// Dispatch to kernel

