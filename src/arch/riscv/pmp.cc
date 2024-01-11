/*
 * Copyright (c) 2021 The Regents of the University of California
 * Copyright (c) 2023 Google LLC
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

#include "arch/riscv/pmp.hh"
#include "arch/generic/tlb.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "base/addr_range.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "debug/PMP.hh"
#include "math.h"
#include "mem/request.hh"
#include "params/PMP.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace RiscvISA
{

PMP::PMP(const Params &params) :
    SimObject(params),
    pmpEntries(params.pmp_entries),
    numRules(0),
    hasLockEntry(false)
{
    pmpTable.resize(pmpEntries);
}

Fault
PMP::pmpCheck(const RequestPtr &req, BaseMMU::Mode mode,
              PrivilegeMode pmode, ThreadContext *tc, Addr vaddr)
{
    // First determine if pmp table should be consulted
    if (!shouldCheckPMP(pmode, tc))
        return NoFault;

    if (req->hasVaddr()) {
        DPRINTF(PMP, "Checking pmp permissions for va: %#x , pa: %#x\n",
                req->getVaddr(), req->getPaddr());
    }
    else { // this access is corresponding to a page table walk
        DPRINTF(PMP, "Checking pmp permissions for pa: %#x\n",
                req->getPaddr());
    }

    // match_index will be used to identify the pmp entry
    // which matched for the given address
    int match_index = -1;

    // all pmp entries need to be looked from the lowest to
    // the highest number
    for (int i = 0; i < pmpTable.size(); i++) {
        AddrRange pmp_range = pmpTable[i].pmpAddr;
        if (pmp_range.contains(req->getPaddr()) &&
                pmp_range.contains(req->getPaddr() + req->getSize() - 1)) {
            // according to specs address is only matched,
            // when (addr) and (addr + request_size - 1) are both
            // within the pmp range
            match_index = i;
        }

        if ((match_index > -1)
            && (PMP_OFF != pmpGetAField(pmpTable[match_index].pmpCfg))) {
            uint8_t this_cfg = pmpTable[match_index].pmpCfg;

            if ((pmode == PrivilegeMode::PRV_M) &&
                                    (PMP_LOCK & this_cfg) == 0) {
                return NoFault;
            } else if ((mode == BaseMMU::Mode::Read) &&
                                        (PMP_READ & this_cfg)) {
                return NoFault;
            } else if ((mode == BaseMMU::Mode::Write) &&
                                        (PMP_WRITE & this_cfg)) {
                return NoFault;
            } else if ((mode == BaseMMU::Mode::Execute) &&
                                        (PMP_EXEC & this_cfg)) {
                return NoFault;
            } else {
                if (req->hasVaddr()) {
                    return createAddrfault(req->getVaddr(), mode);
                } else {
                    return createAddrfault(vaddr, mode);
                }
            }
        }
    }
    // if no entry matched and we are not in M mode return fault
    if (pmode == PrivilegeMode::PRV_M) {
        return NoFault;
    } else if (req->hasVaddr()) {
        return createAddrfault(req->getVaddr(), mode);
    } else {
        return createAddrfault(vaddr, mode);
    }
}

Fault
PMP::createAddrfault(Addr vaddr, BaseMMU::Mode mode)
{
    ExceptionCode code;
    if (mode == BaseMMU::Read) {
        code = ExceptionCode::LOAD_ACCESS;
    } else if (mode == BaseMMU::Write) {
        code = ExceptionCode::STORE_ACCESS;
    } else {
        code = ExceptionCode::INST_ACCESS;
    }
    warn("pmp access fault.\n");
    return std::make_shared<AddressFault>(vaddr, code);
}

inline uint8_t
PMP::pmpGetAField(uint8_t cfg)
{
    // to get a field from pmpcfg register
    uint8_t a = cfg >> 3;
    return a & 0x03;
}


bool
PMP::pmpUpdateCfg(uint32_t pmp_index, uint8_t this_cfg)
{
    if (pmp_index >= pmpEntries) {
        DPRINTF(PMP, "Can't update pmp entry config %u"
                " because the index exceed the size of pmp entries %u",
                pmp_index, pmpEntries);
        return false;
    }

    DPRINTF(PMP, "Update pmp config with %u for pmp entry: %u \n",
                                    (unsigned)this_cfg, pmp_index);
    if (pmpTable[pmp_index].pmpCfg & PMP_LOCK) {
        DPRINTF(PMP, "Update pmp entry config %u failed because it locked\n",
                pmp_index);
        return false;
    }
    pmpTable[pmp_index].pmpCfg = this_cfg;
    pmpUpdateRule(pmp_index);
    return true;
}

void
PMP::pmpUpdateRule(uint32_t pmp_index)
{
    // In qemu, the rule is updated whenever
    // pmpaddr/pmpcfg is written

    numRules = 0;
    hasLockEntry = false;
    Addr prevAddr = 0;

    if (pmp_index >= 1) {
        prevAddr = pmpTable[pmp_index - 1].rawAddr;
    }

    Addr this_addr = pmpTable[pmp_index].rawAddr;
    uint8_t this_cfg = pmpTable[pmp_index].pmpCfg;
    AddrRange this_range;

    switch (pmpGetAField(this_cfg)) {
      // checking the address matching mode of pmp entry
      case PMP_OFF:
        // null region (pmp disabled)
        this_range = AddrRange(0, 0);
        break;
      case PMP_TOR:
        // top of range mode
        this_range = AddrRange(prevAddr << 2, (this_addr << 2));
        break;
      case PMP_NA4:
        // naturally aligned four byte region
        this_range = AddrRange(this_addr << 2, ((this_addr << 2) + 4));
        break;
      case PMP_NAPOT:
        // naturally aligned power of two region, >= 8 bytes
        this_range = AddrRange(pmpDecodeNapot(this_addr));
        break;
      default:
        this_range = AddrRange(0,0);
    }

    pmpTable[pmp_index].pmpAddr = this_range;

    for (int i = 0; i < pmpEntries; i++) {
        const uint8_t a_field = pmpGetAField(pmpTable[i].pmpCfg);
      if (PMP_OFF != a_field) {
          numRules++;
      }
      hasLockEntry |= ((pmpTable[i].pmpCfg & PMP_LOCK) != 0);
    }

    if (hasLockEntry) {
        DPRINTF(PMP, "Find lock entry\n");
    }
}

void
PMP::pmpReset()
{
    for (uint32_t i = 0; i < pmpTable.size(); i++) {
        pmpTable[i].pmpCfg &= ~(PMP_A_MASK | PMP_LOCK);
        pmpUpdateRule(i);
    }
}

bool
PMP::pmpUpdateAddr(uint32_t pmp_index, Addr this_addr)
{
    if (pmp_index >= pmpEntries) {
        DPRINTF(PMP, "Can't update pmp entry address %u"
                " because the index exceed the size of pmp entries %u",
                pmp_index, pmpEntries);
        return false;
    }

    DPRINTF(PMP, "Update pmp addr %#x for pmp entry %u \n",
                                      (this_addr << 2), pmp_index);

    if (pmpTable[pmp_index].pmpCfg & PMP_LOCK) {
        DPRINTF(PMP, "Update pmp entry %u failed because the lock bit set\n",
                pmp_index);
        return false;
    } else if (pmp_index < pmpTable.size() - 1 &&
               ((pmpTable[pmp_index+1].pmpCfg & PMP_LOCK) != 0) &&
               pmpGetAField(pmpTable[pmp_index+1].pmpCfg) == PMP_TOR) {
        DPRINTF(PMP, "Update pmp entry %u failed because the entry %u lock bit"
                " set and A field is TOR\n",
                pmp_index, pmp_index+1);
        return false;
    }

    // just writing the raw addr in the pmp table
    // will convert it into a range, once cfg
    // reg is written
    pmpTable[pmp_index].rawAddr = this_addr;
    for (int index = 0; index < pmpEntries; index++) {
        pmpUpdateRule(index);
    }

    return true;
}

bool
PMP::shouldCheckPMP(PrivilegeMode pmode, ThreadContext *tc)
{
    // The privilege mode of memory read and write
    // is modified by TLB. It can just simply check if
    // the numRule is not zero, then return true if
    // privilege mode is not M or has any lock entry
    return numRules != 0 && (pmode != PrivilegeMode::PRV_M || hasLockEntry);
}

AddrRange
PMP::pmpDecodeNapot(Addr pmpaddr)
{
    if (pmpaddr == -1) {
        AddrRange this_range(0, -1);
        return this_range;
    } else {
        uint64_t t1 = ctz64(~pmpaddr);
        uint64_t range = (1ULL << (t1+3));

        // pmpaddr reg encodes bits 55-2 of a
        // 56 bit physical address for RV64
        uint64_t base = mbits(pmpaddr, 63, t1) << 2;
        AddrRange this_range(base, base+range);
        return this_range;
    }
}

} // namespace RiscvISA
} // namespace gem5
