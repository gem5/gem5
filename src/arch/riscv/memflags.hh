/*
 * Copyright (c) 2024 National and Kapodistrian University of Athens
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

#ifndef __ARCH_RISCV_MEMFLAGS_HH__
#define __ARCH_RISCV_MEMFLAGS_HH__


namespace gem5
{


namespace RiscvISA {

    // We can only utilize the lower 8 bits of a
    // 64-bit value to encode these.
    // see src/mem/request.hh ARCH_BITS
    // Lower 3 bits are already used in mmu.hh
    // for alignment flags
    enum XlateFlags
    {
        // Some mmu accesses must be handled
        // in a special manner.
        // We use these flags to signal this fact.

        // Signal a hypervisor load that checks the
        // executable permission instead of readable
        // (i.e. can load from executable memory that might not
        // be readable)
        HLVX = 1ULL << 3,

        // Force virtualization on
        // This is needed to forcefully enable two-stage translation
        // for hypervisor special instructions (e.g. HLV)
        // These are executed in non-virtualized mode (HS)
        // but the mmu must treat the translation as if
        // virtualization is enabled.
        FORCE_VIRT = 1ULL << 4,

        // Signal a Load Reserved access
        LR = 1ULL << 5,
    };

} // namespace RiscvISA
} // namespace gem5

#endif //__ARCH_RISCV_MEMFLAGS_HH__
