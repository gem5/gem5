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
 */

#ifndef __ARCH_SPARC_SYSCALLRETURN_HH__
#define __ARCH_SPARC_SYSCALLRETURN_HH__

#include <inttypes.h>

#include "sim/syscallreturn.hh"
#include "arch/sparc/regfile.hh"
#include "cpu/thread_context.hh"

namespace SparcISA
{
    static inline void setSyscallReturn(SyscallReturn return_value,
            ThreadContext * tc)
    {
        // check for error condition.  SPARC syscall convention is to
        // indicate success/failure in reg the carry bit of the ccr
        // and put the return value itself in the standard return value reg ().
        if (return_value.successful()) {
            // no error, clear XCC.C
            tc->setIntReg(NumIntArchRegs + 2,
                    tc->readIntReg(NumIntArchRegs + 2) & 0xEE);
            //tc->setMiscReg(MISCREG_CCR, tc->readMiscReg(MISCREG_CCR) & 0xEE);
            tc->setIntReg(ReturnValueReg, return_value.value());
        } else {
            // got an error, set XCC.C
            tc->setIntReg(NumIntArchRegs + 2,
                    tc->readIntReg(NumIntArchRegs + 2) | 0x11);
            //tc->setMiscReg(MISCREG_CCR, tc->readMiscReg(MISCREG_CCR) | 0x11);
            tc->setIntReg(ReturnValueReg, -return_value.value());
        }
    }
};

#endif
