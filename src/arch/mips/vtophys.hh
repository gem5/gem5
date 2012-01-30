/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Jaidev Patwardhan
 */

#ifndef __ARCH_MIPS_VTOPHYS_H__
#define __ARCH_MIPS_VTOPHYS_H__

#include "arch/mips/isa_traits.hh"
#include "arch/mips/utility.hh"

class ThreadContext;

namespace MipsISA {
    inline Addr PteAddr(Addr a) { return (a & PteMask) << PteShift; }

    // User Virtual
    inline bool IsUSeg(Addr a) { return USegBase <= a && a <= USegEnd; }

    inline bool IsKSeg0(Addr a) { return KSeg0Base <= a && a <= KSeg0End; }

    inline Addr KSeg02Phys(Addr addr) { return addr & KSeg0Mask; }

    inline Addr KSeg12Phys(Addr addr) { return addr & KSeg1Mask; }

    inline bool IsKSeg1(Addr a) { return KSeg1Base <= a && a <= KSeg1End; }

    inline bool IsKSSeg(Addr a) { return KSSegBase <= a && a <= KSSegEnd; }

    inline bool IsKSeg3(Addr a) { return KSeg3Base <= a && a <= KSeg3End; }


    Addr vtophys(Addr vaddr);
    Addr vtophys(ThreadContext *tc, Addr vaddr);

};
#endif // __ARCH_MIPS_VTOPHYS_H__

