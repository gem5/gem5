/*
 * Copyright (c) 2010 ARM Limited
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
 */

/*************************************************************************
 * Super simple bootloader
 * Preserve loaded values that we need to pass to the kernel (r0, r1, r2)
 * Additionally M5 puts the kernel start address in r3
 *
 * Upon executing this code:
 * r0 = 0, r1 = machine number, r2 = atags ptr
 * r3 = kernel start address, r4 = GIC address, r5 = flag register address
 *
 * CPU 0 should branch to the kernel start address and it's done with
 * the boot loader. Other CPUs need to start in a wfi loop. When CPU0 sends
 * an IPI the slave CPUs reads a register which CPU0 has programmed with the
 * boot address for the secondary cpu
 **************************************************************************/
.text
.globl  _start
.extern	main
_start:
_entry:
    b bootldr // All the interrupt vectors jump to the boot loader
    b bootldr
    b bootldr
    b bootldr
    b bootldr
    b bootldr
    b bootldr
    b bootldr
    b bootldr

bootldr:
    mrc p15, 0, r8, c0, c0, 5 // get the MPIDR register
    bics r8, r8, #0xff000000  // isolate the lower 24 bits (affinity levels)
    bxeq r3                   // if it's 0 (CPU 0), branch to kernel
    mov  r8, #1
    str  r8, [r4, #0]         //  Enable CPU interface on GIC
    wfi                       //  wait for an interrupt
pen:
    ldr r8, [r5]              // load the value
    movs r8, r8               // set the flags on this value
    beq pen                   // if it's zero try again
    bx r8                     // Jump to where we've been told
    bkpt                      // We should never get here
