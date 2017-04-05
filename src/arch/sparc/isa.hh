/*
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_SPARC_ISA_HH__
#define __ARCH_SPARC_ISA_HH__

#include <ostream>
#include <string>

#include "arch/sparc/registers.hh"
#include "arch/sparc/types.hh"
#include "cpu/cpuevent.hh"
#include "cpu/reg_class.hh"
#include "sim/sim_object.hh"

class Checkpoint;
class EventManager;
struct SparcISAParams;
class ThreadContext;

namespace SparcISA
{
class ISA : public SimObject
{
  private:

    /* ASR Registers */
    // uint64_t y;          // Y (used in obsolete multiplication)
    // uint8_t ccr;         // Condition Code Register
    uint8_t asi;            // Address Space Identifier
    uint64_t tick;          // Hardware clock-tick counter
    uint8_t fprs;           // Floating-Point Register State
    uint64_t gsr;           // General Status Register
    uint64_t softint;
    uint64_t tick_cmpr;     // Hardware tick compare registers
    uint64_t stick;         // Hardware clock-tick counter
    uint64_t stick_cmpr;    // Hardware tick compare registers


    /* Privileged Registers */
    uint64_t tpc[MaxTL];    // Trap Program Counter (value from
                            // previous trap level)
    uint64_t tnpc[MaxTL];   // Trap Next Program Counter (value from
                            // previous trap level)
    uint64_t tstate[MaxTL]; // Trap State
    uint16_t tt[MaxTL];     // Trap Type (Type of trap which occured
                            // on the previous level)
    uint64_t tba;           // Trap Base Address

    PSTATE pstate;        // Process State Register
    uint8_t tl;             // Trap Level
    uint8_t pil;            // Process Interrupt Register
    uint8_t cwp;            // Current Window Pointer
    // uint8_t cansave;     // Savable windows
    // uint8_t canrestore;  // Restorable windows
    // uint8_t cleanwin;    // Clean windows
    // uint8_t otherwin;    // Other windows
    // uint8_t wstate;      // Window State
    uint8_t gl;             // Global level register

    /** Hyperprivileged Registers */
    HPSTATE hpstate;       // Hyperprivileged State Register
    uint64_t htstate[MaxTL];// Hyperprivileged Trap State Register
    uint64_t hintp;
    uint64_t htba;          // Hyperprivileged Trap Base Address register
    uint64_t hstick_cmpr;   // Hardware tick compare registers

    uint64_t strandStatusReg;// Per strand status register

    /** Floating point misc registers. */
    uint64_t fsr;           // Floating-Point State Register

    /** MMU Internal Registers */
    uint16_t priContext;
    uint16_t secContext;
    uint16_t partId;
    uint64_t lsuCtrlReg;

    uint64_t scratchPad[8];

    uint64_t cpu_mondo_head;
    uint64_t cpu_mondo_tail;
    uint64_t dev_mondo_head;
    uint64_t dev_mondo_tail;
    uint64_t res_error_head;
    uint64_t res_error_tail;
    uint64_t nres_error_head;
    uint64_t nres_error_tail;

    // These need to check the int_dis field and if 0 then
    // set appropriate bit in softint and checkinterrutps on the cpu
    void  setFSReg(int miscReg, const MiscReg &val, ThreadContext *tc);
    MiscReg readFSReg(int miscReg, ThreadContext * tc);

    // Update interrupt state on softint or pil change
    void checkSoftInt(ThreadContext *tc);

    /** Process a tick compare event and generate an interrupt on the cpu if
     * appropriate. */
    void processTickCompare(ThreadContext *tc);
    void processSTickCompare(ThreadContext *tc);
    void processHSTickCompare(ThreadContext *tc);

    typedef CpuEventWrapper<ISA,
            &ISA::processTickCompare> TickCompareEvent;
    TickCompareEvent *tickCompare;

    typedef CpuEventWrapper<ISA,
            &ISA::processSTickCompare> STickCompareEvent;
    STickCompareEvent *sTickCompare;

    typedef CpuEventWrapper<ISA,
            &ISA::processHSTickCompare> HSTickCompareEvent;
    HSTickCompareEvent *hSTickCompare;

    static const int NumGlobalRegs = 8;
    static const int NumWindowedRegs = 24;
    static const int WindowOverlap = 8;

    static const int TotalGlobals = (MaxGL + 1) * NumGlobalRegs;
    static const int RegsPerWindow = NumWindowedRegs - WindowOverlap;
    static const int TotalWindowed = NWindows * RegsPerWindow;

    enum InstIntRegOffsets {
        CurrentGlobalsOffset = 0,
        CurrentWindowOffset = CurrentGlobalsOffset + NumGlobalRegs,
        MicroIntOffset = CurrentWindowOffset + NumWindowedRegs,
        NextGlobalsOffset = MicroIntOffset + NumMicroIntRegs,
        NextWindowOffset = NextGlobalsOffset + NumGlobalRegs,
        PreviousGlobalsOffset = NextWindowOffset + NumWindowedRegs,
        PreviousWindowOffset = PreviousGlobalsOffset + NumGlobalRegs,
        TotalInstIntRegs = PreviousWindowOffset + NumWindowedRegs
    };

    RegIndex intRegMap[TotalInstIntRegs];
    void installWindow(int cwp, int offset);
    void installGlobals(int gl, int offset);
    void reloadRegMap();

  public:

    void clear();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void startup(ThreadContext *tc) {}

    /// Explicitly import the otherwise hidden startup
    using SimObject::startup;

  protected:
    bool isHyperPriv() { return hpstate.hpriv; }
    bool isPriv() { return hpstate.hpriv || pstate.priv; }
    bool isNonPriv() { return !isPriv(); }

  public:

    MiscReg readMiscRegNoEffect(int miscReg) const;
    MiscReg readMiscReg(int miscReg, ThreadContext *tc);

    void setMiscRegNoEffect(int miscReg, const MiscReg val);
    void setMiscReg(int miscReg, const MiscReg val,
            ThreadContext *tc);

    RegId
    flattenRegId(const RegId& regId) const
    {
        switch (regId.classValue()) {
          case IntRegClass:
            return RegId(IntRegClass, flattenIntIndex(regId.index()));
          case FloatRegClass:
            return RegId(FloatRegClass, flattenFloatIndex(regId.index()));
          case CCRegClass:
            return RegId(CCRegClass, flattenCCIndex(regId.index()));
          case MiscRegClass:
            return RegId(MiscRegClass, flattenMiscIndex(regId.index()));
          default:
            break;
        }
        return regId;
    }

    int
    flattenIntIndex(int reg) const
    {
        assert(reg < TotalInstIntRegs);
        RegIndex flatIndex = intRegMap[reg];
        assert(flatIndex < NumIntRegs);
        return flatIndex;
    }

    int
    flattenFloatIndex(int reg) const
    {
        return reg;
    }

    int
    flattenVecIndex(int reg) const
    {
        return reg;
    }

    int
    flattenVecElemIndex(int reg) const
    {
        return reg;
    }

    // dummy
    int
    flattenCCIndex(int reg) const
    {
        return reg;
    }

    int
    flattenMiscIndex(int reg) const
    {
        return reg;
    }


    typedef SparcISAParams Params;
    const Params *params() const;

    ISA(Params *p);
};
}

#endif
