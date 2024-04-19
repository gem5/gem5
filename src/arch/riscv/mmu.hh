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

#ifndef __ARCH_RISCV_MMU_HH__
#define __ARCH_RISCV_MMU_HH__

#include "arch/generic/mmu.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/pma_checker.hh"
#include "arch/riscv/tlb.hh"

#include "params/RiscvMMU.hh"

namespace gem5
{

namespace RiscvISA {

class MMU : public BaseMMU
{
  public:
    BasePMAChecker *pma;

    MMU(const RiscvMMUParams &p)
      : BaseMMU(p), pma(p.pma_checker)
    {}

    TranslationGenPtr
    translateFunctional(Addr start, Addr size, ThreadContext *tc,
            Mode mode, Request::Flags flags) override
    {
        return TranslationGenPtr(new MMUTranslationGen(
                PageBytes, start, size, tc, this, mode, flags));
    }

    PrivilegeMode
    getMemPriv(ThreadContext *tc, BaseMMU::Mode mode)
    {
        return static_cast<TLB*>(dtb)->getMemPriv(tc, mode);
    }

    Walker *
    getDataWalker()
    {
        return static_cast<TLB*>(dtb)->getWalker();
    }

    void
    takeOverFrom(BaseMMU *old_mmu) override
    {
      MMU *ommu = dynamic_cast<MMU*>(old_mmu);
      BaseMMU::takeOverFrom(ommu);
      pma->takeOverFrom(ommu->pma);

    }

    PMP *
    getPMP()
    {
        return static_cast<TLB*>(dtb)->pmp;
    }

    /*
     * The usage of Memory Request Arch Flags for RISC-V
     *  | 7 ------------- 3 | 2 ------ 0 |
     *  |     Reserved      |  LDST Size |
     *  | ------------------| -----------|
     */
    enum RiscvFlags
    {
        ByteAlign = 0,
        HalfWordAlign = 1,
        WordAlign = 2,
        DoubleWordAlign = 3,
        QuadWordAlign = 4,

        AlignmentMask = 0x7,
    };
};

} // namespace RiscvISA
} // namespace gem5

#endif  // __ARCH_RISCV_MMU_HH__
