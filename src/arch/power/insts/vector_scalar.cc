/*
 * Copyright (c) 2024 The University of Michigan Ann Arbor
 * Copyright (c) 2024 IBM Corporation
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

#include "arch/power/insts/misc.hh"
#include "arch/power/isa.hh"
#include "arch/power/regs/float.hh"  // Defines floatRegClass
#include "arch/power/regs/int.hh"  // Defines intRegClass
#include "arch/power/regs/vec.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "vector_scalar.hh"

using namespace gem5;

// Define 128-bit unsigned int if not already available
typedef __uint128_t uint128_t;

// Register classes are in PowerISA namespace (from src/arch/power/regs/)
using namespace PowerISA;

void mtvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx) {
    // Compute effective VSX register index (0-63)
    const RegIndex vsr_idx = (tx_sx << 5) | t_s;

    // Get integer register value
    uint64_t int_val = tc->getReg(intRegClass[ra_vsx]);

    // For VSR0-VSR31 (vector registers)
    if (vsr_idx < 32) {
        // Read existing VSR value (preserve upper 64 bits)
        uint128_t vsr_val = tc->getReg(vecRegClass[vsr_idx]);
        vsr_val = (vsr_val & ((uint128_t)0xFFFFFFFFFFFFFFFFULL << 64)) |
        int_val;
        tc->setReg(vecRegClass[vsr_idx], vsr_val);
    }
    // For VSR32-VSR63 (map to FPRs)
    else {
        tc->setReg(floatRegClass[vsr_idx - 32], int_val);
    }
}

void mfvsrd_exec(ThreadContext *tc, int t_s, int ra_vsx, int tx_sx) {
    // Compute effective VSX register index (0-63)
    const RegIndex vsr_idx = (tx_sx << 5) | t_s;
    uint64_t val;

    // For VSR0-VSR31 (vector registers)
    if (vsr_idx < 32) {
        val = static_cast<uint64_t>(tc->getReg(vecRegClass[vsr_idx]));
    }
    // For VSR32-VSR63 (map to FPRs)
    else {
        val = tc->getReg(floatRegClass[vsr_idx - 32]);
    }

    tc->setReg(intRegClass[ra_vsx], val);
}
