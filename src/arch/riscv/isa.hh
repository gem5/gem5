/*
 * Copyright (c) 2009 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2014 Sven Karlsson
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
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

#ifndef __ARCH_RISCV_ISA_HH__
#define __ARCH_RISCV_ISA_HH__

#include <unordered_map>
#include <vector>

#include "arch/generic/isa.hh"
#include "arch/riscv/pcstate.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/types.hh"
#include "base/types.hh"

namespace gem5
{

struct RiscvISAParams;
class Checkpoint;

namespace RiscvISA
{

enum PrivilegeMode
{
    PRV_U = 0,
    PRV_S = 1,
    PRV_M = 3
};

enum FPUStatus
{
    OFF = 0,
    INITIAL = 1,
    CLEAN = 2,
    DIRTY = 3,
};

class ISA : public BaseISA
{
  protected:
    RiscvType rv_type;
    std::vector<RegVal> miscRegFile;
    bool checkAlignment;

    bool hpmCounterEnabled(int counter) const;

    // Load reserve - store conditional monitor
    const int WARN_FAILURE = 10000;
    const Addr INVALID_RESERVATION_ADDR = (Addr)-1;
    std::unordered_map<int, Addr> load_reservation_addrs;

  public:
    using Params = RiscvISAParams;

    void clear() override;

    PCStateBase*
    newPCState(Addr new_inst_addr=0) const override
    {
        return new PCState(new_inst_addr, rv_type);
    }

    void
    clearLoadReservation(ContextID cid) override
    {
        Addr& load_reservation_addr = load_reservation_addrs[cid];
        load_reservation_addr = INVALID_RESERVATION_ADDR;
    }

  public:
    RegVal readMiscRegNoEffect(RegIndex idx) const override;
    RegVal readMiscReg(RegIndex idx) override;
    void setMiscRegNoEffect(RegIndex idx, RegVal val) override;
    void setMiscReg(RegIndex idx, RegVal val) override;

    // Derived class could provide knowledge of non-standard CSRs to other
    // components by overriding the two getCSRxxxMap here and properly
    // implementing the corresponding read/set function. However, customized
    // maps should always be compatible with the standard maps.
    virtual const std::unordered_map<int, CSRMetadata>&
    getCSRDataMap() const
    {
        return CSRData;
    }
    virtual const std::unordered_map<int, RegVal>&
    getCSRMaskMap() const
    {
        return CSRMasks[rv_type];
    }

    bool alignmentCheckEnabled() const { return checkAlignment; }

    bool inUserMode() const override;
    void copyRegsFrom(ThreadContext *src) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    ISA(const Params &p);

    void handleLockedRead(const RequestPtr &req) override;

    bool handleLockedWrite(const RequestPtr &req,
            Addr cacheBlockMask) override;

    void handleLockedSnoop(PacketPtr pkt, Addr cacheBlockMask) override;

    void globalClearExclusive() override;

    void resetThread() override;

    RiscvType rvType() const { return rv_type; }
};

} // namespace RiscvISA
} // namespace gem5

std::ostream &operator<<(std::ostream &os, gem5::RiscvISA::PrivilegeMode pm);

#endif // __ARCH_RISCV_ISA_HH__
