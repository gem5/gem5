/*
 * Copyright (c) 2012, 2020, 2022 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

        .text

        .globl	_start
_start:
        /* Save some values initialized by gem5. */
        /* DTB address. */
        mov x21, x0
        /* Kernel entry point. */
        mov x20, x3
        /* cpu-release-addr. */
        mov x22, x5

        /*
         * EL3 initialisation
         */
        mrs	x0, CurrentEL
        cmp	x0, #0xc			// EL3?
        b.ne	start_ns			// skip EL3 initialisation

        mov	x0, #0x30			// RES1
        orr	x0, x0, #(1 << 0)		// Non-secure EL1
        orr	x0, x0, #(1 << 8)		// HVC enable
        orr	x0, x0, #(1 << 10)		// 64-bit EL2
        orr	x0, x0, #(1 << 16)		// Disable FEAT_PAuth traps (APK)
        orr	x0, x0, #(1 << 17)		// Disable FEAT_PAuth traps (API)
        msr	scr_el3, x0

        mov	x0, #(1 << 8)			// Disable SVE trap to EL3
        orr	x0, x0, #(1 << 12)		// Disable SME trap to EL3
        msr	cptr_el3, x0			// Disable copro. traps to EL3

        /*
         * Check for the primary CPU to avoid a race on the distributor
         * registers.
         */
        mrs	x0, mpidr_el1
        // ARM MPIDR_EL1 bytes: Aff3 (AArch64), Stuff, Aff2, Aff1, Aff0
        // Test the the MPIDR_EL1 register against 0xff00ffffff to
        // extract the primary CPU.
        ldr x1, =0xff00ffffff
#ifdef GICV3
        tst	x0, x1 // check for cpuid==zero
        b.ne	3f // secondary CPU

        ldr	x1, =GIC_DIST_BASE // GICD_CTLR
        mov	w0, #7 // EnableGrp0 | EnableGrp1NS | EnableGrp1S
        str	w0, [x1]
        mov	x2, #0xffe8
        ldr	w2, [x1, x2] // GICD_PIDR2
        and	w3, w2, #0xf0 // GICD_PIDR2.Read ArchRev
        cmp	w3, #0x30 // Check if GICv3
        mov	w4, #0x20000 // 128KiB redistributor stride
        mov	w5, #0x40000 // 256KiB redistributor stride
        csel	w2, w4, w5, eq // Select the correct redistributor stride

        ldr	x1, =GIC_REDIST_BASE
        mov	w0, #~0 // Grp1 interrupts
1:  ldr	w3, [x1, #0x14] // GICR_WAKER
        and	w3, w3, #~0x2 // Setting GICR_WAKER.ProcessorSleep to 0
        str	w3, [x1, #0x14]
        dsb	sy
2:  ldr	w3, [x1, #0x14] // Re-reading GICR_WAKER
        ands	w3, w3, #0x4 // Checking GICR_WAKER.ChildrenAsleep
        b.ne	2b // Keep reading GICR_WAKER.ChildrenAsleep until awake

        add	x5, x1, #0x10000 // SGI base
        str	w0, [x5, #0x80] // GICR_IGROUPR0
        ldr	w4, [x1, #0x8] // GICR_TYPER
        add	x1, x1, x2 // Point to next redistributor
        // Check GICR_TYPER.Last, which is set to 1
        // if this is the last redistributor
        ands	w4, w4, #0x10
        b.eq	1b // Branch back if not last redistributor

        ldr	x1, =GIC_DIST_BASE + 0x84 // GICD_IGROUPR
        str	w0, [x1], #4
        str	w0, [x1], #4
        str	w0, [x1], #4

        /* SRE & Disable IRQ/FIQ Bypass & Allow EL2 access to ICC_SRE_EL2 */
3:  mrs	x10, S3_6_C12_C12_5 // read ICC_SRE_EL3
        orr	x10, x10, #0xf      // enable 0xf
        msr	S3_6_C12_C12_5, x10 // write ICC_SRE_EL3
        isb

        mov	x0, #1
        msr	S3_0_c12_c12_6, x0 // ICC_IGRPEN0_EL1 Enable
        msr	S3_0_C12_C12_7, x0 // ICC_IGRPEN1_EL1 Enable
#else
        tst x0, x1                    // check for cpuid==zero
        b.ne	1f				      // secondary CPU

        ldr	x1, =GIC_DIST_BASE		// GICD_CTLR
        mov	w0, #3				// EnableGrp0 | EnableGrp1
        str	w0, [x1]

1:	ldr	x1, =GIC_DIST_BASE + 0x80	// GICD_IGROUPR
        mov	w0, #~0				// Grp1 interrupts
        str	w0, [x1], #4
        b.ne	2f				// Only local interrupts for secondary CPUs
        str	w0, [x1], #4
        str	w0, [x1], #4

2:	ldr	x1, =GIC_CPU_BASE		// GICC_CTLR
        ldr	w0, [x1]
        mov	w0, #3				// EnableGrp0 | EnableGrp1
        str	w0, [x1]

        mov	w0, #1 << 7			// allow NS access to GICC_PMR
        str	w0, [x1, #4]			// GICC_PMR
#endif

        msr	sctlr_el2, xzr

        /*
         * Prepare the switch to the EL2_SP1 mode from EL3
         */
        ldr	x0, =start_ns			// Return after mode switch
        mov	x1, #0x3c9			// EL2_SP1 | D | A | I | F
        msr	elr_el3, x0
        msr	spsr_el3, x1
        eret

start_ns:
        /*
         * Kernel parameters
         */
        mov	x0, xzr
        mov	x1, xzr
        mov	x2, xzr
        mov	x3, xzr

        mrs	x4, mpidr_el1
        // ARM MPIDR_EL1 bytes: Aff3 (AArch64), Stuff, Aff2, Aff1, Aff0
        // Test the the MPIDR_EL1 register against 0xff00ffffff to
        // extract the primary CPU.
        ldr x1, =0xff00ffffff
        tst x4, x1                    // check for cpuid==zero
        mov x1, xzr                   // load previous 'xzr' value back to x1
        b.eq	2f				      // secondary CPU

        /*
         * Secondary CPUs
         */
1:	wfe
        /* The Linux kernel v5.8 and older writes the entry point address
         * of the secondary CPUs to this address, and does a SEV, waking up
         * the secondary CPUs.
         *
         * gem5 informs the kernel the desired address via cpu-release-addr
         * of the DTB.
         *
         * When this is first reached immediately after the bootloader starts,
         * the value at that address must be 0, which is the default memory
         * value set by gem5 for otherwise uninitialized memory, leading to
         * WFE.
         */
        ldr x4, [x22]
        cbz	x4, 1b
        br	x4				// branch to the given address

2:
        /*
         * UART initialisation (38400 8N1)
         */
        ldr	x4, =UART_BASE			// UART base
        mov	w5, #0x10			// ibrd
        str	w5, [x4, #0x24]
        mov	w5, #0xc300
        orr	w5, w5, #0x0001			// cr
        str	w5, [x4, #0x30]

        /*
         * CLCD output site MB
         */
        ldr	x4, =SYSREGS_BASE
        ldr	w5, =(1 << 31) | (1 << 30) | (7 << 20) | (0 << 16)	// START|WRITE|MUXFPGA|SITE_MB
        str	wzr, [x4, #0xa0]		// V2M_SYS_CFGDATA
        str	w5, [x4, #0xa4]			// V2M_SYS_CFGCTRL

        /*
         * Primary CPU
         */
        // The kernel boot protocol specifies that the DTB address is placed
        // in x0.
        // https://github.com/torvalds/linux/blob/v5.7/Documentation/arm64/
        // booting.rst#4-call-the-kernel-image
        mov x0, x21
        // Jump into the kernel entry point.
        br x20

        .ltorg

        .org	0x200
