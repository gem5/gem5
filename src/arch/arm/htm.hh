/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __ARCH_ARM_HTM_HH__
#define __ARCH_ARM_HTM_HH__

/**
 * @file
 *
 * ISA-specific types for hardware transactional memory.
 */

#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/generic/htm.hh"
#include "base/types.hh"

namespace gem5
{

namespace ArmISA
{

class HTMCheckpoint : public BaseHTMCheckpoint
{
  public:
    HTMCheckpoint()
      : BaseHTMCheckpoint()
    {}

    const static int MAX_HTM_DEPTH = 255;

    void reset() override;
    void save(ThreadContext *tc) override;
    void restore(ThreadContext *tc, HtmFailureFaultCause cause) override;

    void destinationRegister(RegIndex dest) { rt = dest; }
    void cancelReason(uint16_t reason) { tcreason = reason; }

  private:
    uint8_t rt; // TSTART destination register
    Addr nPc; // Fallback instruction address
    std::array<RegVal, int_reg::NumArchRegs> x; // General purpose registers
    std::array<VecRegContainer, NumVecRegs> z; // Vector registers
    std::array<VecPredRegContainer, NumVecRegs> p; // Predicate registers
    Addr sp; // Stack Pointer at current EL
    uint16_t tcreason; // TCANCEL reason
    uint32_t fpcr; // Floating-point Control Register
    uint32_t fpsr; // Floating-point Status Register
    uint32_t iccPmrEl1; // Interrupt Controller Interrupt Priority Mask
    uint8_t nzcv; // Condition flags
    uint8_t daif;
    PCState pcstateckpt;
};

} // namespace ArmISA
} // namespace gem5

#endif
