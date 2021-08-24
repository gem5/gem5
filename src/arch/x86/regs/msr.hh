/*
 * Copyright (c) 2011 Google
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

#ifndef __ARCH_X86_REG_MSR_HH__
#define __ARCH_X86_REG_MSR_HH__

#include <unordered_map>

#include "arch/x86/regs/misc.hh"
#include "base/types.hh"

namespace gem5
{

namespace X86ISA
{

typedef std::unordered_map<Addr, RegIndex> MsrMap;

/**
 * Map between MSR addresses and their corresponding misc registers.
 *
 * @note This map is usually only used when enumeration of supported
 * MSRs is needed (e.g., in virtualized CPUs). Code that needs to
 * look-up specific MSRs should use msrAddrToIndex().
 */
extern const MsrMap msrMap;

/**
 * Find and return the misc reg corresponding to an MSR address.
 *
 * Look for an MSR (addr) in #msrMap and return the
 * corresponding misc reg in regNum. The value of regNum is undefined
 * if the MSR was not found.
 *
 * @param regNum misc reg index (out).
 * @param addr MSR address
 * @return True if the MSR was found, false otherwise.
 */
bool msrAddrToIndex(RegIndex &regNum, Addr addr);

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_REG_MSR_HH__
