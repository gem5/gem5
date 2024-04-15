/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

#ifndef __ARCH_MIPS_UTILITY_HH__
#define __ARCH_MIPS_UTILITY_HH__

#include "arch/mips/page_size.hh"
#include "arch/mips/regs/misc.hh"
#include "arch/mips/types.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

class ThreadContext;

namespace MipsISA
{

////////////////////////////////////////////////////////////////////////
//
// Floating Point Utility Functions
//
uint64_t fpConvert(ConvertType cvt_type, double fp_val);
double roundFP(double val, int digits);
double truncFP(double val);

bool getCondCode(uint32_t fcsr, int cc);
uint32_t genCCVector(uint32_t fcsr, int num, uint32_t cc_val);
uint32_t genInvalidVector(uint32_t fcsr);

bool isNan(void *val_ptr, int size);
bool isQnan(void *val_ptr, int size);
bool isSnan(void *val_ptr, int size);

////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//
inline Addr
TruncPage(Addr addr)
{
    return addr & ~(PageBytes - 1);
}

inline Addr
RoundPage(Addr addr)
{
    return (addr + PageBytes - 1) & ~(PageBytes - 1);
}

} // namespace MipsISA
} // namespace gem5

#endif
