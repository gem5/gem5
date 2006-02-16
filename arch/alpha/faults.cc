/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/alpha/faults.hh"

ResetFaultType * ResetFault = new ResetFaultType("reset", 1, 0x0001);
ArithmeticFaultType * ArithmeticFault = new ArithmeticFaultType("arith", 3, 0x0501);
InterruptFaultType * InterruptFault = new InterruptFaultType("interrupt", 4, 0x0101);
NDtbMissFaultType * NDtbMissFault = new NDtbMissFaultType("dtb_miss_single", 5, 0x0201);
PDtbMissFaultType * PDtbMissFault = new PDtbMissFaultType("dtb_miss_double", 6, 0x0281);
DtbPageFaultType * DtbPageFault = new DtbPageFaultType("dfault", 8, 0x0381);
DtbAcvFaultType * DtbAcvFault = new DtbAcvFaultType("dfault", 9, 0x0381);
ItbMissFaultType * ItbMissFault = new ItbMissFaultType("itbmiss", 10, 0x0181);
ItbPageFaultType * ItbPageFault = new ItbPageFaultType("itbmiss", 11, 0x0181);
ItbAcvFaultType * ItbAcvFault = new ItbAcvFaultType("iaccvio", 12, 0x0081);
UnimplementedOpcodeFaultType * UnimplementedOpcodeFault = new UnimplementedOpcodeFaultType("opdec", 13, 0x0481);
FloatEnableFaultType * FloatEnableFault = new FloatEnableFaultType("fen", 14, 0x0581);
PalFaultType * PalFault = new PalFaultType("pal", 15, 0x2001);
IntegerOverflowFaultType * IntegerOverflowFault = new IntegerOverflowFaultType("intover", 16, 0x0501);

Fault ** ListOfFaults[] = {
        (Fault **)&NoFault,
        (Fault **)&ResetFault,
        (Fault **)&MachineCheckFault,
        (Fault **)&ArithmeticFault,
        (Fault **)&InterruptFault,
        (Fault **)&NDtbMissFault,
        (Fault **)&PDtbMissFault,
        (Fault **)&AlignmentFault,
        (Fault **)&DtbPageFault,
        (Fault **)&DtbAcvFault,
        (Fault **)&ItbMissFault,
        (Fault **)&ItbPageFault,
        (Fault **)&ItbAcvFault,
        (Fault **)&UnimplementedOpcodeFault,
        (Fault **)&FloatEnableFault,
        (Fault **)&PalFault,
        (Fault **)&IntegerOverflowFault,
        (Fault **)&FakeMemFault
        };

int NumFaults = sizeof(ListOfFaults) / sizeof(Fault **);
