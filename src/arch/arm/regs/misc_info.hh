/*
 * Copyright (c) 2010-2025 Arm Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#ifndef __ARCH_ARM_REGS_MISC_INFO_HH__
#define __ARCH_ARM_REGS_MISC_INFO_HH__

#include <array>
#include <bitset>
#include <functional>
#include <vector>

#include "arch/arm/regs/misc.hh"
#include "arch/arm/types.hh"
#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"
#include "sim/faults.hh"

namespace gem5
{

class ThreadContext;
class ArmSystem;
class MiscRegOp64;

namespace ArmISA
{

// clang-format off
enum MiscRegInfo
{
    MISCREG_IMPLEMENTED,
    MISCREG_UNVERIFIABLE,  // Does the value change on every read (e.g. a
                           // arch generic counter)
    MISCREG_UNSERIALIZE,   // Should the checkpointed value be restored?
    MISCREG_WARN_NOT_FAIL, // If MISCREG_IMPLEMENTED is deasserted, it
                           // tells whether the instruction should raise a
                           // warning or fail
    MISCREG_MUTEX,         // True if the register corresponds to a pair of
                           // mutually exclusive registers
    MISCREG_BANKED, // True if the register is banked between the two
                    // security states, and this is the parent node of the
                    // two banked registers
    MISCREG_BANKED64, // True if the register is banked between the two
                      // security states, and this is the parent node of
                      // the two banked registers. Used in AA64 only.
    MISCREG_BANKED_CHILD, // The entry is one of the child registers that
                          // forms a banked set of regs (along with the
                          // other child regs)
    MISCREG_SERIALIZING,  // True if the register requires CPU to serialize
                          // execution while writing to it. If false,
                          // register does allow other instructions to be
                          // executed but the register write is still
                          // non-speculative (only happening at commit)

    // Access permissions
    // User mode
    MISCREG_USR_NS_RD,
    MISCREG_USR_NS_WR,
    MISCREG_USR_S_RD,
    MISCREG_USR_S_WR,
    // Privileged modes other than hypervisor or monitor
    MISCREG_PRI_NS_RD,
    MISCREG_PRI_NS_WR,
    MISCREG_PRI_S_RD,
    MISCREG_PRI_S_WR,
    // Hypervisor mode
    MISCREG_HYP_NS_RD,
    MISCREG_HYP_NS_WR,
    MISCREG_HYP_S_RD,
    MISCREG_HYP_S_WR,
    // Monitor mode, SCR.NS == 0
    MISCREG_MON_NS0_RD,
    MISCREG_MON_NS0_WR,
    // Monitor mode, SCR.NS == 1
    MISCREG_MON_NS1_RD,
    MISCREG_MON_NS1_WR,

    NUM_MISCREG_INFOS
};

/** MiscReg metadata **/
struct MiscRegLUTEntry
{
    uint32_t lower;  // Lower half mapped to this register
    uint32_t upper;  // Upper half mapped to this register
    uint64_t _reset; // value taken on reset (i.e. initialization)
    uint64_t _res0;  // reserved
    uint64_t _res1;  // reserved
    uint64_t _raz;   // read as zero (fixed at 0)
    uint64_t _rao;   // read as one (fixed at 1)
    std::bitset<NUM_MISCREG_INFOS> info;

    using FaultCB = std::function<
        Fault(const MiscRegLUTEntry &entry, ThreadContext *tc,
              const MiscRegOp64 &inst)
    >;

    std::array<FaultCB, EL3 + 1> faultRead;
    std::array<FaultCB, EL3 + 1> faultWrite;

    Fault checkFault(ThreadContext *tc, const MiscRegOp64 &inst,
        ExceptionLevel el);

  protected:
    template <MiscRegInfo Sec, MiscRegInfo NonSec>
    static Fault defaultFault(const MiscRegLUTEntry &entry,
        ThreadContext *tc, const MiscRegOp64 &inst);

  public:
    MiscRegLUTEntry()
        : lower(0),
          upper(0),
          _reset(0),
          _res0(0),
          _res1(0),
          _raz(0),
          _rao(0),
          info(0),
          faultRead(
              {defaultFault<MISCREG_USR_S_RD, MISCREG_USR_NS_RD>,
               defaultFault<MISCREG_PRI_S_RD, MISCREG_PRI_NS_RD>,
               defaultFault<MISCREG_HYP_S_RD, MISCREG_HYP_NS_RD>,
               defaultFault<MISCREG_MON_NS0_RD, MISCREG_MON_NS1_RD>}),
          faultWrite(
              {defaultFault<MISCREG_USR_S_WR, MISCREG_USR_NS_WR>,
               defaultFault<MISCREG_PRI_S_WR, MISCREG_PRI_NS_WR>,
               defaultFault<MISCREG_HYP_S_WR, MISCREG_HYP_NS_WR>,
               defaultFault<MISCREG_MON_NS0_WR, MISCREG_MON_NS1_WR>})
    {}
    uint64_t reset() const { return _reset; }
    uint64_t res0()  const { return _res0; }
    uint64_t res1()  const { return _res1; }
    uint64_t raz()   const { return _raz; }
    uint64_t rao()   const { return _rao; }
    // raz/rao implies writes ignored
    uint64_t wi()    const { return _raz | _rao; }
};

/** Metadata table accessible via the value of the register */
class MiscRegLUTEntryInitializer
{
    struct MiscRegLUTEntry &entry;
    typedef const MiscRegLUTEntryInitializer& chain;
  public:
    chain
    mapsTo(uint32_t l, uint32_t u = 0) const
    {
        entry.lower = l;
        entry.upper = u;
        return *this;
    }
    chain
    reset(uint64_t res_val) const
    {
        entry._reset = res_val;
        return *this;
    }
    chain
    res0(uint64_t mask) const
    {
        entry._res0 = mask;
        return *this;
    }
    chain
    res1(uint64_t mask) const
    {
        entry._res1 = mask;
        return *this;
    }
    chain
    raz(uint64_t mask = (uint64_t)-1) const
    {
        entry._raz  = mask;
        return *this;
    }
    chain
    rao(uint64_t mask = (uint64_t)-1) const
    {
        entry._rao  = mask;
        return *this;
    }
    chain
    implemented(bool v = true) const
    {
        entry.info[MISCREG_IMPLEMENTED] = v;
        return *this;
    }
    chain
    unimplemented() const
    {
        return implemented(false);
    }
    chain
    unverifiable(bool v = true) const
    {
        entry.info[MISCREG_UNVERIFIABLE] = v;
        return *this;
    }
    chain
    unserialize(bool v = true) const
    {
        entry.info[MISCREG_UNSERIALIZE] = v;
        return *this;
    }
    chain
    warnNotFail(bool v = true) const
    {
        entry.info[MISCREG_WARN_NOT_FAIL] = v;
        return *this;
    }
    chain
    mutex(bool v = true) const
    {
        entry.info[MISCREG_MUTEX] = v;
        return *this;
    }
    chain
    banked(bool v = true) const
    {
        entry.info[MISCREG_BANKED] = v;
        return *this;
    }
    chain
    banked64(bool v = true) const
    {
        entry.info[MISCREG_BANKED64] = v;
        return *this;
    }
    chain
    bankedChild(bool v = true) const
    {
        entry.info[MISCREG_BANKED_CHILD] = v;
        return *this;
    }
    chain
    serializing(bool v = true) const
    {
        entry.info[MISCREG_SERIALIZING] = v;
        return *this;
    }
    chain
    userNonSecureRead(bool v = true) const
    {
        entry.info[MISCREG_USR_NS_RD] = v;
        return *this;
    }
    chain
    userNonSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_USR_NS_WR] = v;
        return *this;
    }
    chain
    userSecureRead(bool v = true) const
    {
        entry.info[MISCREG_USR_S_RD] = v;
        return *this;
    }
    chain
    userSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_USR_S_WR] = v;
        return *this;
    }
    chain
    user(bool v = true) const
    {
        userNonSecureRead(v);
        userNonSecureWrite(v);
        userSecureRead(v);
        userSecureWrite(v);
        return *this;
    }
    chain
    privNonSecureRead(bool v = true) const
    {
        entry.info[MISCREG_PRI_NS_RD] = v;
        return *this;
    }
    chain
    privNonSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_PRI_NS_WR] = v;
        return *this;
    }
    chain
    privNonSecure(bool v = true) const
    {
        privNonSecureRead(v);
        privNonSecureWrite(v);
        return *this;
    }
    chain
    privSecureRead(bool v = true) const
    {
        entry.info[MISCREG_PRI_S_RD] = v;
        return *this;
    }
    chain
    privSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_PRI_S_WR] = v;
        return *this;
    }
    chain
    privSecure(bool v = true) const
    {
        privSecureRead(v);
        privSecureWrite(v);
        return *this;
    }
    chain
    priv(bool v = true) const
    {
        privSecure(v);
        privNonSecure(v);
        return *this;
    }
    chain
    privRead(bool v = true) const
    {
        privSecureRead(v);
        privNonSecureRead(v);
        return *this;
    }
    chain
    hypSecureRead(bool v = true) const
    {
        entry.info[MISCREG_HYP_S_RD] = v;
        return *this;
    }
    chain
    hypNonSecureRead(bool v = true) const
    {
        entry.info[MISCREG_HYP_NS_RD] = v;
        return *this;
    }
    chain
    hypRead(bool v = true) const
    {
        hypSecureRead(v);
        hypNonSecureRead(v);
        return *this;
    }
    chain
    hypSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_HYP_S_WR] = v;
        return *this;
    }
    chain
    hypNonSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_HYP_NS_WR] = v;
        return *this;
    }
    chain
    hypWrite(bool v = true) const
    {
        hypSecureWrite(v);
        hypNonSecureWrite(v);
        return *this;
    }
    chain
    hypSecure(bool v = true) const
    {
        hypSecureRead(v);
        hypSecureWrite(v);
        return *this;
    }
    chain
    hyp(bool v = true) const
    {
        hypRead(v);
        hypWrite(v);
        return *this;
    }
    chain
    monSecureRead(bool v = true) const
    {
        entry.info[MISCREG_MON_NS0_RD] = v;
        return *this;
    }
    chain
    monSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_MON_NS0_WR] = v;
        return *this;
    }
    chain
    monNonSecureRead(bool v = true) const
    {
        entry.info[MISCREG_MON_NS1_RD] = v;
        return *this;
    }
    chain
    monNonSecureWrite(bool v = true) const
    {
        entry.info[MISCREG_MON_NS1_WR] = v;
        return *this;
    }
    chain
    mon(bool v = true) const
    {
        monSecureRead(v);
        monSecureWrite(v);
        monNonSecureRead(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    monWrite(bool v = true) const
    {
        monSecureWrite(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    monSecure(bool v = true) const
    {
        monSecureRead(v);
        monSecureWrite(v);
        return *this;
    }
    chain
    monNonSecure(bool v = true) const
    {
        monNonSecureRead(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    allPrivileges(bool v = true) const
    {
        userNonSecureRead(v);
        userNonSecureWrite(v);
        userSecureRead(v);
        userSecureWrite(v);
        privNonSecureRead(v);
        privNonSecureWrite(v);
        privSecureRead(v);
        privSecureWrite(v);
        hypRead(v);
        hypWrite(v);
        monSecureRead(v);
        monSecureWrite(v);
        monNonSecureRead(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    nonSecure(bool v = true) const
    {
        userNonSecureRead(v);
        userNonSecureWrite(v);
        privNonSecureRead(v);
        privNonSecureWrite(v);
        hypRead(v);
        hypWrite(v);
        monNonSecureRead(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    secure(bool v = true) const
    {
        userSecureRead(v);
        userSecureWrite(v);
        privSecureRead(v);
        privSecureWrite(v);
        monSecureRead(v);
        monSecureWrite(v);
        return *this;
    }
    chain
    reads(bool v) const
    {
        userNonSecureRead(v);
        userSecureRead(v);
        privNonSecureRead(v);
        privSecureRead(v);
        hypRead(v);
        monSecureRead(v);
        monNonSecureRead(v);
        return *this;
    }
    chain
    writes(bool v) const
    {
        userNonSecureWrite(v);
        userSecureWrite(v);
        privNonSecureWrite(v);
        privSecureWrite(v);
        hypWrite(v);
        monSecureWrite(v);
        monNonSecureWrite(v);
        return *this;
    }
    chain
    exceptUserMode() const
    {
        user(0);
        return *this;
    }
    chain highest(ArmSystem *const sys) const;

    chain
    faultRead(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
    {
        entry.faultRead[el] = cb;
        return *this;
    }

    chain
    faultWrite(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
    {
        entry.faultWrite[el] = cb;
        return *this;
    }

    chain
    fault(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
    {
        return faultRead(el, cb).faultWrite(el, cb);
    }

    chain
    fault(MiscRegLUTEntry::FaultCB cb) const
    {
        return fault(EL0, cb).fault(EL1, cb).fault(EL2, cb).fault(EL3, cb);
    }

    MiscRegLUTEntryInitializer(struct MiscRegLUTEntry &e)
      : entry(e)
    {
        // force unimplemented registers to be thusly declared
        implemented(true).serializing(true).unserialize(true);
    }
};

extern std::vector<struct MiscRegLUTEntry> lookUpMiscReg;

static inline const MiscRegLUTEntryInitializer
InitReg(uint32_t reg)
{
    return MiscRegLUTEntryInitializer(lookUpMiscReg[reg]);
}

class MiscRegClassOps : public RegClassOps
{
  public:
    std::string
    regName(const RegId &id) const override
    {
        return miscRegName[id.index()];
    }

    bool
    serializing(const RegId &id) const override
    {
        return lookUpMiscReg[id.index()].info[MISCREG_SERIALIZING];
    }
};

inline MiscRegClassOps miscRegClassOps;

inline constexpr RegClass miscRegClass =
    RegClass(MiscRegClass, MiscRegClassName, NUM_MISCREGS,
            debug::MiscRegs).
        ops(miscRegClassOps);

} // namespace ArmISA

} // namespace gem5

#endif // __ARCH_ARM_REGS_MISC_INFO_HH__
