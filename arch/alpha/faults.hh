/*
 * Copyright (c) 2003 The Regents of The University of Michigan
 * All rights reserved.
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

#ifndef __FAULTS_HH__
#define __FAULTS_HH__

enum Fault {
    No_Fault,
    Reset_Fault,		// processor reset
    Machine_Check_Fault,	// machine check (also internal S/W fault)
    Arithmetic_Fault,		// FP exception
    Interrupt_Fault,		// external interrupt
    Ndtb_Miss_Fault,		// DTB miss
    Pdtb_Miss_Fault,		// nested DTB miss
    Alignment_Fault,		// unaligned access
    DTB_Fault_Fault,		// DTB page fault
    DTB_Acv_Fault,		// DTB access violation
    ITB_Miss_Fault,		// ITB miss
    ITB_Fault_Fault,		// ITB page fault
    ITB_Acv_Fault,		// ITB access violation
    Unimplemented_Opcode_Fault,	// invalid/unimplemented instruction
    Fen_Fault,			// FP not-enabled fault
    Pal_Fault,			// call_pal S/W interrupt
    Integer_Overflow_Fault,
    Num_Faults			// number of faults
};

const char *
FaultName(int index);

#endif // __FAULTS_HH__
