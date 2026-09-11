/*
 * Copyright (c) 2015, University of Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Jung
 *          Frederik Lauer
 */

.section INTERRUPT_VECTOR, "x"
.global _Reset
_Reset:
    B Reset_Handler    /* Reset */
    B .                /* Undefined */
    B .                /* SWI */
    B .                /* Prefetch Abort */
    B .                /* Data Abort */
    B .                /* reserved */
    B irq_handler      /* IRQ */
    B .                /* FIQ */


.equ Len_Stack,        0x1000;  // 4kB of stack memory
.equ Len_IRQ_Stack,    0x1000;  // 4kB of stack memory for IRQ Mode
//.equ stack_base,      0x18000   // stack_base defined in Linker Script

//GIC_Distributor
//.equ GIC_Dist_Base,     0x1f001000
.equ GIC_Dist_Base,		0x2c001000

//Register offsets
.equ set_enable1,       0x104
.equ set_enable2,       0x108

//Example definitions
//.equ timer_irq_id,      36   // 36 <64 => set_enable1 Reg
.equ timer_irq_id,    131   // 36 <64 => set_enable1 Reg
.equ kmio_irq_id,     44
.equ uart0_irq_id,    37
.equ rtc_irq_id,      36
.equ top_dev_id,      68

//GIC_CPU_INTERFACE
//.equ GIC_CPU_BASE,                  0x1f000100
.equ GIC_CPU_BASE,                  0x2c002000
.equ GIC_CPU_mask_reg_offset,       0x04
.equ GIC_CPU_Int_Ack_reg_offset,    0x0C
.equ GIC_CPU_End_of_int_offset,     0x10


.global Reset_Handler
Reset_Handler:
    // Set up stack pointers for IRQ processor mode
    mov R1, #0b11010010 // interrupts masked, MODE=IRQ IRQ|FIQ|0|Mode[4:0]
    msr CPSR, R1    // change to IRQ mode
    ldr SP, =stack_base + Len_Stack + Len_IRQ_Stack // set IRQ stack

    // Change back to SVC (supervisor) mode with interrupts disabled
    mov R1, #0b11010011 // interrupts masked, MODE=SVC IRQ|FIQ|0|Mode[4:0]
    msr CPSR, R1    // change to SVC mode
    ldr SP, =stack_base + Len_Stack // set stack

    // Enable individual interrupts, set target
    bl config_gic_dist

    // Enable individual interrupts, set target
    bl config_gic_cpu_interface

    // Enable interrupts in GIC Distributor
    ldr r0, =GIC_Dist_Base
    mov r1, #1
    str r1, [r0]

    // Enable IRQ interrupts in the processor:
    mov R1, #0b01010011 // IRQ not masked (=0), MODE=SVC IRQ|FIQ|0|Mode[4:0]
    msr CPSR, R1

    bl main
    B .


.global config_gic_dist
config_gic_dist:
    push {lr}
    /* Enable the Interrupt in the Set-Enable Register of the GIC Distributor
     *  Set-enable1 Reg Offset Address = 0x104
     *   Bits 0 to 31 correspond to interrupt input lines 32-63 respectively
     *   A bit set to 1 indicates an enabled interrupt.
     *  Set-enable2 Reg Offset Address = 0x108
     *   Bits 0 to 31 correspond to interrupt input lines 64-95 respectively
     *   A bit set to 1 indicates an enabled interrupt.
     *  This Example: Interrupt of timer0 => IRQ ID = 36
     */

    ldr r1, =GIC_Dist_Base + set_enable2    // r1 = Set-enable1 Reg Address
    mov r2, #1
    //IRQ ID - 32 => 5th bit = 1
    lsl r2, r2, #4

    ldr r3, [r1]    // read current register value
    orr r3, r3, r2  // set the enable bit
    str r3, [r1]    // store the new register value

    /* Configure Interrupt Processor Taget
     * Reg offset  0x820     for ID32 − ID35
     *             0x824     for ID36 − ID39
     *             ...
     * default values are 0x01010101 => CPU0 is target for all.
     */
    pop {pc}


.global config_gic_cpu_interface
config_gic_cpu_interface:
    push {lr}

    // set Interrupt Priority mask (enable all priority levels)
    ldr r1, =GIC_CPU_BASE + GIC_CPU_mask_reg_offset
    ldr r2, =0xFFFF
    str r2, [r1]

    // set the enable bit in the GIC_CPU_INTERFACE
    mov r2, #1
    ldr r1, =GIC_CPU_BASE
    str r2, [r1]
    pop {pc}


// IRQ Handler that calls the ISR function in C
.global irq_handler
irq_handler:
    push {r0-r7,lr}

    // Read the interrupt acknowledge register of the GIC_CPU_INTERFACE
    ldr r1, =GIC_CPU_BASE + GIC_CPU_Int_Ack_reg_offset
    ldr r2, [r1]

irq_top:
    cmp r2, #top_dev_id
    bne irq_end  // if irq is not from top_dev

    // Jump to C - must clear the timer interrupt!
    BL isr
    ldr r2, = top_dev_id

irq_end:
    // write the IRQ ID to the END_OF_INTERRUPT Register of GIC_CPU_INTERFACE
    ldr r1, =GIC_CPU_BASE + GIC_CPU_End_of_int_offset
    str r2, [r1]

    pop {r0-r7,lr}
    subs pc, lr, #4
