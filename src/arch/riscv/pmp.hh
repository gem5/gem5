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

#ifndef __ARCH_RISCV_PMP_HH__
#define __ARCH_RISCV_PMP_HH__

#include "arch/generic/tlb.hh"
#include "arch/riscv/isa.hh"
#include "base/addr_range.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/PMP.hh"
#include "sim/sim_object.hh"

/**
 * @file
 * PMP header file.
 */

namespace gem5
{

namespace RiscvISA
{

/**
 * This class helps to implement RISCV's physical memory
 * protection (pmp) primitive.
 * @todo Add statistics and debug prints.
 */
class PMP : public SimObject
{
  public:
    PARAMS(PMP);
    PMP(const Params &params);

  private:
    /** maximum number of entries in the pmp table */
    int pmpEntries;

    /** This enum is used for encoding of address matching mode of
     * pmp address register, which is present in bits 3-4 (A) of
     * pmpcfg register for a pmp entry.
     * PMP_OFF = null region (pmp disabled)
     * MP_TOR = top of range mode
     * PMP_NA4 = naturally aligned four byte region
     * PMP_NAPOT = naturally aligned power of two region, >= 8 bytes
     */
    enum pmpAmatch
    {
        PMP_OFF,
        PMP_TOR,
        PMP_NA4,
        PMP_NAPOT
    };

    /** pmpcfg address range read permission mask */
    const uint8_t PMP_READ = 1 << 0;

    /** pmpcfg address range write permission mask */
    const uint8_t PMP_WRITE = 1 << 1;

    /** pmpcfg address range execute permission mask */
    const uint8_t PMP_EXEC = 1 << 2;

    /** pmpcfg A field mask */
    const uint8_t PMP_A_MASK = 3 << 3;

    /** pmpcfg address range locked mask */
    const uint8_t PMP_LOCK = 1 << 7;

    /** variable to keep track of active number of rules any time */
    int numRules;

    /** variable to keep track of any lock of entry */
    bool hasLockEntry;

    /** single pmp entry struct*/
    struct PmpEntry
    {
        /** addr range corresponding to a single pmp entry */
        AddrRange pmpAddr = AddrRange(0, 0);
        /** raw addr in pmpaddr register for a pmp entry */
        Addr rawAddr;
        /** pmpcfg reg value for a pmp entry */
        uint8_t pmpCfg = 0;
    };

    /** a table of pmp entries */
    std::vector<PmpEntry> pmpTable;

  public:
    /**
     * pmpCheck checks if a particular memory access
     * is allowed based on the pmp rules.
     * @param req memory request.
     * @param mode mode of request (read, write, execute).
     * @param pmode current privilege mode of memory (U, S, M).
     * @param tc thread context.
     * @param vaddr optional parameter to pass vaddr of original
     * request for which a page table walk is consulted by pmp unit
     * @return Fault.
     */
    Fault pmpCheck(const RequestPtr &req, BaseMMU::Mode mode,
                  PrivilegeMode pmode, ThreadContext *tc,
                  Addr vaddr = 0);

    /**
     * pmpUpdateCfg updates the pmpcfg for a pmp
     * entry and calls pmpUpdateRule to update the
     * rule of corresponding pmp entry.
     * @param pmp_index pmp entry index.
     * @param this_cfg value to be written to pmpcfg.
     * @returns true if update pmpicfg success
     */
    bool pmpUpdateCfg(uint32_t pmp_index, uint8_t this_cfg);

    /**
     * pmpUpdateAddr updates the pmpaddr for a pmp
     * entry and calls pmpUpdateRule to update the
     * rule of corresponding pmp entry.
     * @param pmp_index pmp entry index.
     * @param this_addr value to be written to pmpaddr.
     * @returns true if update pmpaddri success
     */
    bool pmpUpdateAddr(uint32_t pmp_index, Addr this_addr);

    /**
     * pmpReset reset when reset signal in trigger from
     * CPU.
     */
    void pmpReset();

  private:
    /**
     * This function is called during a memory
     * access to determine if the pmp table
     * should be consulted for this access.
     * @param pmode current privilege mode of memory (U, S, M).
     * @param tc thread context.
     * @return true or false.
     */
    bool shouldCheckPMP(PrivilegeMode pmode, ThreadContext *tc);

    /**
     * createAddrfault creates an address fault
     * if the pmp checks fail to pass for a given
     * access. This function is used by pmpCheck().
     * given pmp entry depending on the value
     * of pmpaddr and pmpcfg for that entry.
     * @param vaddr virtual address of the access.
     * @param mode mode of access(read, write, execute).
     * @return Fault.
     */
    Fault createAddrfault(Addr vaddr, BaseMMU::Mode mode);

    /**
     * pmpUpdateRule updates the pmp rule for a
     * given pmp entry depending on the value
     * of pmpaddr and pmpcfg for that entry.
     * @param pmp_index pmp entry index.
     */
    void pmpUpdateRule(uint32_t pmp_index);

    /**
     * pmpGetAField extracts the A field (address matching mode)
     * from an input pmpcfg register
     * @param cfg pmpcfg register value.
     * @return The A field.
     */
    inline uint8_t pmpGetAField(uint8_t cfg);

    /**
     * This function decodes a pmpaddr register value
     * into an address range when A field of pmpcfg
     * register is set to NAPOT mode (naturally aligned
     * power of two region).
     * @param pmpaddr input address from a pmp entry.
     * @return an address range.
     */
    inline AddrRange pmpDecodeNapot(Addr pmpaddr);

};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_PMP_HH__
