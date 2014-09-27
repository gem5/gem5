/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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
 * Authors: Korey Sewell
 *          Stephen Hines
 *          Timothy M. Jones
 */

#ifndef __ARCH_POWER_UTILITY_HH__
#define __ARCH_POWER_UTILITY_HH__

#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

namespace PowerISA {

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.advance();
    return retPC;
}

/**
 * Function to ensure ISA semantics about 0 registers.
 * @param tc The thread context.
 */
template <class TC>
void zeroRegisters(TC *tc);

inline void
startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate();
}

void
copyRegs(ThreadContext *src, ThreadContext *dest);

static inline void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
}

void skipFunction(ThreadContext *tc);

inline void
advancePC(PCState &pc, const StaticInstPtr &inst)
{
    pc.advance();
}

uint64_t getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

static inline bool
inUserMode(ThreadContext *tc)
{
    return 0;
}

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return 0;
}

void initCPU(ThreadContext *, int cpuId);

} // namespace PowerISA


#endif // __ARCH_POWER_UTILITY_HH__
