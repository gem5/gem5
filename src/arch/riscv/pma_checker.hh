/*
 * Copyright (c) 2021 Huawei International
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

#ifndef __ARCH_RISCV_PMA_CHECKER_HH__
#define __ARCH_RISCV_PMA_CHECKER_HH__

#include "arch/generic/mmu.hh"
#include "base/addr_range.hh"
#include "base/addr_range_map.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/BasePMAChecker.hh"
#include "params/PMAChecker.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * Based on the RISC-V ISA privileged specifications
 * V1.11, there is no implementation guidelines on the
 * Physical Memory Attributes.
 */

class BasePMAChecker : public SimObject
{
  public:
    BasePMAChecker(const BasePMACheckerParams &params) : SimObject(params) {};
    virtual Fault check(
        const RequestPtr &req, BaseMMU::Mode mode, Addr vaddr = 0) = 0;
    virtual Fault checkVAddrAlignment(
        const RequestPtr &req, BaseMMU::Mode mode) = 0;
    virtual void takeOverFrom(BasePMAChecker *old) = 0;
};

/**
 * This class provides an abstract PMAChecker for RISC-V
 * to provide PMA checking functionality. However,
 * hardware latencies are not modelled.
 */

class PMAChecker : public BasePMAChecker
{
  public:

    typedef PMACheckerParams Params;

    const Params &
    params() const
    {
        return dynamic_cast<const Params &>(_params);
    }
    PMAChecker(const Params &params);

    /*
     * Check if any exception for given address
     */
    Fault check(
        const RequestPtr &req, BaseMMU::Mode mode, Addr vaddr = 0) override;

    /*
     * Check alignment for virtual address
     */
    Fault checkVAddrAlignment(
        const RequestPtr &req, BaseMMU::Mode mode) override;

    bool isUncacheable(const AddrRange &range);
    bool isUncacheable(const Addr &addr, const unsigned size);
    bool isUncacheable(PacketPtr pkt);

    void takeOverFrom(BasePMAChecker *old) override;

  protected:
    /*
     * Check alignment for physical address
     */
    Fault checkPAddrAlignment(
        const RequestPtr &req, BaseMMU::Mode mode, Addr vaddr);

    /*
     * Create address-misaligned exception based on the MMU mode and
     * virtual address
     */
    Fault createMisalignFault(
        Addr vaddr, BaseMMU::Mode mode, bool virtualized = false);

    inline bool addressAlign(const Addr addr, const Addr size);

    /*
     * Check if the address range support misaligned load/store
     */
    inline bool misalignedSupport(const AddrRange &range);

    /*
     * Check if there is any region support misaligned load/store
     */
    inline bool hasMisaligned();

    AddrRangeList uncacheable;
    AddrRangeMap<bool, 3> misaligned;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_PMA_CHECKER_HH__
