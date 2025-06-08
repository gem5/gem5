// Copyright (c) 2025 The Regents of the University of Michigan Ann Arbor
// All rights reserved.
//
// This file is part of gem5.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
#define __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__

#include "arch/power/regs/float.hh"
#include "arch/power/regs/int.hh"
#include "arch/power/regs/vec.hh"
#include "cpu/thread_context.hh"

namespace gem5
{
namespace PowerISA
{

void mtvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx);
void mfvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx);

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_INSTS_VECTOR_SCALAR_HH__
