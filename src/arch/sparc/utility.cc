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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/sparc/utility.hh"
#if FULL_SYSTEM
#include "arch/sparc/vtophys.hh"
#include "mem/vport.hh"
#endif

namespace SparcISA {


//The caller uses %o0-%05 for the first 6 arguments even if their floating
//point. Double precision floating point values take two registers/args.
//Quads, structs, and unions are passed as pointers. All arguments beyond
//the sixth are passed on the stack past the 16 word window save area,
//space for the struct/union return pointer, and space reserved for the
//first 6 arguments which the caller may use but doesn't have to.
uint64_t getArgument(ThreadContext *tc, int number, bool fp) {
#if FULL_SYSTEM
    if (number < NumArgumentRegs) {
        return tc->readIntReg(ArgumentReg[number]);
    } else {
        Addr sp = tc->readIntReg(StackPointerReg);
        VirtualPort *vp = tc->getVirtPort(tc);
        uint64_t arg = vp->read<uint64_t>(sp + 92 +
                            (number-NumArgumentRegs) * sizeof(uint64_t));
        tc->delVirtPort(vp);
        return arg;
    }
#else
    panic("getArgument() only implemented for FULL_SYSTEM\n");
    M5_DUMMY_RETURN
#endif
}
} //namespace SPARC_ISA
