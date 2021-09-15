/*
 * Copyright (c) 2010, 2012-2013, 2016-2019 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARM_FAULTS_HH__
#define __ARM_FAULTS_HH__

#include "arch/arm/pagetable.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/types.hh"
#include "base/logging.hh"
#include "cpu/null_static_inst.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"

namespace gem5
{

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace ArmISA
{
typedef Addr FaultOffset;

class ArmStaticInst;

class ArmFault : public FaultBase
{
  protected:
    ExtMachInst machInst;
    uint32_t issRaw;

    // Helper variables for ARMv8 exception handling
    bool bStep; // True if the Arm Faul exception is a software Step exception
    bool from64;  // True if the exception is generated from the AArch64 state
    bool to64;  // True if the exception is taken in AArch64 state
    ExceptionLevel fromEL;  // Source exception level
    ExceptionLevel toEL;  // Target exception level
    OperatingMode fromMode;  // Source operating mode (aarch32)
    OperatingMode toMode;  // Next operating mode (aarch32)

    // This variable is true if the above fault specific informations
    // have been updated. This is to prevent that a client is using their
    // un-updated default constructed value.
    bool faultUpdated;

    bool hypRouted; // True if the fault has been routed to Hypervisor
    bool span; // True if the fault is setting the PSTATE.PAN bit

    virtual Addr getVector(ThreadContext *tc);
    Addr getVector64(ThreadContext *tc);

  public:
    /// Generic fault source enums used to index into
    /// {short/long/aarch64}DescFaultSources[] to get the actual encodings based
    /// on the current register width state and the translation table format in
    /// use
    enum FaultSource
    {
        AlignmentFault = 0,
        InstructionCacheMaintenance,  // Short-desc. format only
        SynchExtAbtOnTranslTableWalkLL,
        SynchPtyErrOnTranslTableWalkLL = SynchExtAbtOnTranslTableWalkLL + 4,
        TranslationLL = SynchPtyErrOnTranslTableWalkLL + 4,
        AccessFlagLL = TranslationLL + 4,
        DomainLL = AccessFlagLL + 4,
        PermissionLL = DomainLL + 4,
        DebugEvent = PermissionLL + 4,
        SynchronousExternalAbort,
        TLBConflictAbort,  // Requires LPAE
        SynchPtyErrOnMemoryAccess,
        AsynchronousExternalAbort,
        AsynchPtyErrOnMemoryAccess,
        AddressSizeLL,  // AArch64 only

        // Not real faults. These are faults to allow the translation function
        // to inform the memory access function not to proceed for a prefetch
        // that misses in the TLB or that targets an uncacheable address
        PrefetchTLBMiss = AddressSizeLL + 4,
        PrefetchUncacheable,

        NumFaultSources,
        FaultSourceInvalid = 0xff
    };

    /// Encodings of the fault sources when the short-desc. translation table
    /// format is in use (ARM ARM Issue C B3.13.3)
    static uint8_t shortDescFaultSources[NumFaultSources];
    /// Encodings of the fault sources when the long-desc. translation table
    /// format is in use (ARM ARM Issue C B3.13.3)
    static uint8_t longDescFaultSources[NumFaultSources];
    /// Encodings of the fault sources in AArch64 state
    static uint8_t aarch64FaultSources[NumFaultSources];

    enum AnnotationIDs
    {
        S1PTW, // DataAbort, PrefetchAbort: Stage 1 Page Table Walk,
        OVA,   // DataAbort, PrefetchAbort: stage 1 Virtual Address for stage 2 faults
        SAS,   // DataAbort: Syndrome Access Size
        SSE,   // DataAbort: Syndrome Sign Extend
        SRT,   // DataAbort: Syndrome Register Transfer
        CM,    // DataAbort: Cache Maintenance/Address Translation Op
        OFA,   // DataAbort: Override fault Address. This is needed when
               // the abort is triggered by a CMO. The faulting address is
               // then the address specified in the register argument of the
               // instruction and not the cacheline address (See FAR doc)

        // AArch64 only
        SF,    // DataAbort: width of the accessed register is SixtyFour
        AR     // DataAbort: Acquire/Release semantics
    };

    enum TranMethod
    {
        LpaeTran,
        VmsaTran,
        UnknownTran
    };

    enum DebugType
    {
        NODEBUG = 0,
        BRKPOINT,
        VECTORCATCH,
        WPOINT_CM,
        WPOINT_NOCM
    };

    struct FaultVals
    {
        const FaultName name;

        const FaultOffset offset;

        // Offsets used for exceptions taken in AArch64 state
        const uint16_t currELTOffset;
        const uint16_t currELHOffset;
        const uint16_t lowerEL64Offset;
        const uint16_t lowerEL32Offset;

        const OperatingMode nextMode;

        const uint8_t armPcOffset;
        const uint8_t thumbPcOffset;
        // The following two values are used in place of armPcOffset and
        // thumbPcOffset when the exception return address is saved into ELR
        // registers (exceptions taken in HYP mode or in AArch64 state)
        const uint8_t armPcElrOffset;
        const uint8_t thumbPcElrOffset;

        const bool hypTrappable;
        const bool abortDisable;
        const bool fiqDisable;

        // Exception class used to appropriately set the syndrome register
        // (exceptions taken in HYP mode or in AArch64 state)
        const ExceptionClass ec;

        FaultStat count;
        FaultVals(const FaultName& name_, const FaultOffset& offset_,
                const uint16_t& currELTOffset_, const uint16_t& currELHOffset_,
                const uint16_t& lowerEL64Offset_,
                const uint16_t& lowerEL32Offset_,
                const OperatingMode& nextMode_, const uint8_t& armPcOffset_,
                const uint8_t& thumbPcOffset_, const uint8_t& armPcElrOffset_,
                const uint8_t& thumbPcElrOffset_, const bool& hypTrappable_,
                const bool& abortDisable_, const bool& fiqDisable_,
                const ExceptionClass& ec_)
        : name(name_), offset(offset_), currELTOffset(currELTOffset_),
          currELHOffset(currELHOffset_), lowerEL64Offset(lowerEL64Offset_),
          lowerEL32Offset(lowerEL32Offset_), nextMode(nextMode_),
          armPcOffset(armPcOffset_), thumbPcOffset(thumbPcOffset_),
          armPcElrOffset(armPcElrOffset_), thumbPcElrOffset(thumbPcElrOffset_),
          hypTrappable(hypTrappable_), abortDisable(abortDisable_),
          fiqDisable(fiqDisable_), ec(ec_) {}
    };

    ArmFault(ExtMachInst _machInst = 0, uint32_t _iss = 0) :
        machInst(_machInst), issRaw(_iss), bStep(false), from64(false),
        to64(false), fromEL(EL0), toEL(EL0), fromMode(MODE_UNDEFINED),
        faultUpdated(false), hypRouted(false), span(false) {}

    // Returns the actual syndrome register to use based on the target
    // exception level
    MiscRegIndex getSyndromeReg64() const;
    // Returns the actual fault address register to use based on the target
    // exception level
    MiscRegIndex getFaultAddrReg64() const;

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    void invoke32(ThreadContext *tc, const StaticInstPtr &inst =
                  nullStaticInstPtr);
    void invoke64(ThreadContext *tc, const StaticInstPtr &inst =
                  nullStaticInstPtr);
    void update(ThreadContext *tc);
    bool isResetSPSR(){ return bStep; }

    bool vectorCatch(ThreadContext *tc, const StaticInstPtr &inst);

    ArmStaticInst *instrAnnotate(const StaticInstPtr &inst);
    virtual void annotate(AnnotationIDs id, uint64_t val) {}
    virtual FaultStat& countStat() = 0;
    virtual FaultOffset offset(ThreadContext *tc) = 0;
    virtual FaultOffset offset64(ThreadContext *tc) = 0;
    virtual OperatingMode nextMode() = 0;
    virtual bool routeToMonitor(ThreadContext *tc) const = 0;
    virtual bool routeToHyp(ThreadContext *tc) const { return false; }
    virtual uint8_t armPcOffset(bool isHyp) = 0;
    virtual uint8_t thumbPcOffset(bool isHyp) = 0;
    virtual uint8_t armPcElrOffset() = 0;
    virtual uint8_t thumbPcElrOffset() = 0;
    virtual bool abortDisable(ThreadContext *tc) = 0;
    virtual bool fiqDisable(ThreadContext *tc) = 0;
    virtual ExceptionClass ec(ThreadContext *tc) const = 0;
    virtual uint32_t vectorCatchFlag() const { return 0x0; }
    virtual uint32_t iss() const = 0;
    virtual bool isStage2() const { return false; }
    virtual FSR getFsr(ThreadContext *tc) const { return 0; }
    virtual void setSyndrome(ThreadContext *tc, MiscRegIndex syndrome_reg);
    virtual bool getFaultVAddr(Addr &va) const { return false; }
    OperatingMode getToMode() const { return toMode; }
};

template<typename T>
class ArmFaultVals : public ArmFault
{
  protected:
    static FaultVals vals;

  public:
    ArmFaultVals<T>(ExtMachInst _machInst = 0, uint32_t _iss = 0) :
        ArmFault(_machInst, _iss) {}
    FaultName name() const override { return vals.name; }
    FaultStat & countStat() override { return vals.count; }
    FaultOffset offset(ThreadContext *tc) override;

    FaultOffset offset64(ThreadContext *tc) override;

    OperatingMode nextMode() override { return vals.nextMode; }
    virtual bool routeToMonitor(ThreadContext *tc) const override {
        return false;
    }
    uint8_t armPcOffset(bool isHyp) override {
        return isHyp ? vals.armPcElrOffset
                     : vals.armPcOffset;
    }
    uint8_t thumbPcOffset(bool isHyp) override {
        return isHyp ? vals.thumbPcElrOffset
                     : vals.thumbPcOffset;
    }
    uint8_t armPcElrOffset() override { return vals.armPcElrOffset; }
    uint8_t thumbPcElrOffset() override { return vals.thumbPcElrOffset; }
    bool abortDisable(ThreadContext* tc) override { return vals.abortDisable; }
    bool fiqDisable(ThreadContext* tc) override { return vals.fiqDisable; }
    ExceptionClass ec(ThreadContext *tc) const override { return vals.ec; }
    uint32_t iss() const override { return issRaw; }
};

class Reset : public ArmFaultVals<Reset>
{
  protected:
    Addr getVector(ThreadContext *tc) override;

  public:
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
};

class UndefinedInstruction : public ArmFaultVals<UndefinedInstruction>
{
  protected:
    bool unknown;
    bool disabled;
    ExceptionClass overrideEc;
    const char *mnemonic;

  public:
    UndefinedInstruction(ExtMachInst _machInst,
                         bool _unknown,
                         const char *_mnemonic = NULL,
                         bool _disabled = false) :
        ArmFaultVals<UndefinedInstruction>(_machInst),
        unknown(_unknown), disabled(_disabled),
        overrideEc(EC_INVALID), mnemonic(_mnemonic)
    {}
    UndefinedInstruction(ExtMachInst _machInst, uint32_t _iss,
            ExceptionClass _overrideEc, const char *_mnemonic = NULL) :
        ArmFaultVals<UndefinedInstruction>(_machInst, _iss),
        unknown(false), disabled(true), overrideEc(_overrideEc),
        mnemonic(_mnemonic)
    {}

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    bool routeToHyp(ThreadContext *tc) const override;
    ExceptionClass ec(ThreadContext *tc) const override;
    uint32_t iss() const override;
    uint32_t vectorCatchFlag() const override { return 0x02000002; }
};

class SupervisorCall : public ArmFaultVals<SupervisorCall>
{
  protected:
    ExceptionClass overrideEc;
  public:
    SupervisorCall(ExtMachInst _machInst, uint32_t _iss,
                   ExceptionClass _overrideEc = EC_INVALID) :
        ArmFaultVals<SupervisorCall>(_machInst, _iss),
        overrideEc(_overrideEc)
    {
        bStep = true;
    }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    bool routeToHyp(ThreadContext *tc) const override;
    ExceptionClass ec(ThreadContext *tc) const override;
    uint32_t iss() const override;
    uint32_t vectorCatchFlag() const override { return 0x04000404; }
};

class SecureMonitorCall : public ArmFaultVals<SecureMonitorCall>
{
  public:
    SecureMonitorCall(ExtMachInst _machInst) :
        ArmFaultVals<SecureMonitorCall>(_machInst)
    {
        bStep = true;
    }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    ExceptionClass ec(ThreadContext *tc) const override;
    uint32_t iss() const override;
    uint32_t vectorCatchFlag() const override { return 0x00000400; }
};

class SupervisorTrap : public ArmFaultVals<SupervisorTrap>
{
  protected:
    ExtMachInst machInst;
    ExceptionClass overrideEc;

  public:
    SupervisorTrap(ExtMachInst _machInst, uint32_t _iss,
                   ExceptionClass _overrideEc = EC_INVALID) :
        ArmFaultVals<SupervisorTrap>(_machInst, _iss),
        overrideEc(_overrideEc)
    {}

    bool routeToHyp(ThreadContext *tc) const override;
    uint32_t iss() const override;
    ExceptionClass ec(ThreadContext *tc) const override;
};

class SecureMonitorTrap : public ArmFaultVals<SecureMonitorTrap>
{
 protected:
    ExtMachInst machInst;
    ExceptionClass overrideEc;

  public:
    SecureMonitorTrap(ExtMachInst _machInst, uint32_t _iss,
                      ExceptionClass _overrideEc = EC_INVALID) :
        ArmFaultVals<SecureMonitorTrap>(_machInst, _iss),
        overrideEc(_overrideEc)
    {}

    ExceptionClass ec(ThreadContext *tc) const override;
};

class HypervisorCall : public ArmFaultVals<HypervisorCall>
{
  public:
    HypervisorCall(ExtMachInst _machInst, uint32_t _imm);

    bool routeToHyp(ThreadContext *tc) const override;
    bool routeToMonitor(ThreadContext *tc) const override;
    ExceptionClass ec(ThreadContext *tc) const override;
    uint32_t vectorCatchFlag() const override { return 0xFFFFFFFF; }
};

class HypervisorTrap : public ArmFaultVals<HypervisorTrap>
{
  protected:
    ExtMachInst machInst;
    ExceptionClass overrideEc;

  public:
    HypervisorTrap(ExtMachInst _machInst, uint32_t _iss,
                   ExceptionClass _overrideEc = EC_INVALID) :
      ArmFaultVals<HypervisorTrap>(_machInst, _iss),
      overrideEc(_overrideEc)
    {}

    ExceptionClass ec(ThreadContext *tc) const override;
};

template <class T>
class AbortFault : public ArmFaultVals<T>
{
  protected:
    /**
     * The virtual address the fault occured at. If 2 stages of
     * translation are being used then this is the intermediate
     * physical address that is the starting point for the second
     * stage of translation.
     */
    Addr faultAddr;
    /**
     * Original virtual address. If the fault was generated on the
     * second stage of translation then this variable stores the
     * virtual address used in the original stage 1 translation.
     */
    Addr OVAddr;
    bool write;
    TlbEntry::DomainType domain;
    uint8_t source;
    uint8_t srcEncoded;
    bool stage2;
    bool s1ptw;
    ArmFault::TranMethod tranMethod;
    ArmFault::DebugType debugType;

  public:
    AbortFault(Addr _faultAddr, bool _write, TlbEntry::DomainType _domain,
               uint8_t _source, bool _stage2,
               ArmFault::TranMethod _tranMethod = ArmFault::UnknownTran,
               ArmFault::DebugType _debug = ArmFault::NODEBUG) :
        faultAddr(_faultAddr), OVAddr(0), write(_write),
        domain(_domain), source(_source), srcEncoded(0),
        stage2(_stage2), s1ptw(false), tranMethod(_tranMethod),
        debugType(_debug)
    {}

    bool getFaultVAddr(Addr &va) const override;

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;

    FSR getFsr(ThreadContext *tc) const override;
    uint8_t getFaultStatusCode(ThreadContext *tc) const;
    bool abortDisable(ThreadContext *tc) override;
    uint32_t iss() const override;
    bool isStage2() const override { return stage2; }
    void annotate(ArmFault::AnnotationIDs id, uint64_t val) override;
    void setSyndrome(ThreadContext *tc, MiscRegIndex syndrome_reg) override;
    bool isMMUFault() const;
};

class PrefetchAbort : public AbortFault<PrefetchAbort>
{
  public:
    static const MiscRegIndex FsrIndex  = MISCREG_IFSR;
    static const MiscRegIndex FarIndex  = MISCREG_IFAR;
    static const MiscRegIndex HFarIndex = MISCREG_HIFAR;

    PrefetchAbort(Addr _addr, uint8_t _source, bool _stage2 = false,
                  ArmFault::TranMethod _tranMethod = ArmFault::UnknownTran,
                  ArmFault::DebugType _debug = ArmFault::NODEBUG) :
        AbortFault<PrefetchAbort>(_addr, false, TlbEntry::DomainType::NoAccess,
                _source, _stage2, _tranMethod, _debug)
    {}

    ExceptionClass ec(ThreadContext *tc) const override;
    // @todo: external aborts should be routed if SCR.EA == 1
    bool routeToMonitor(ThreadContext *tc) const override;
    bool routeToHyp(ThreadContext *tc) const override;
    uint32_t vectorCatchFlag() const override { return 0x08000808; }
};

class DataAbort : public AbortFault<DataAbort>
{
  public:
    static const MiscRegIndex FsrIndex  = MISCREG_DFSR;
    static const MiscRegIndex FarIndex  = MISCREG_DFAR;
    static const MiscRegIndex HFarIndex = MISCREG_HDFAR;
    bool    isv;
    uint8_t sas;
    uint8_t sse;
    uint8_t srt;
    uint8_t cm;

    // AArch64 only
    bool sf;
    bool ar;

    DataAbort(Addr _addr, TlbEntry::DomainType _domain, bool _write, uint8_t _source,
              bool _stage2=false,
              ArmFault::TranMethod _tranMethod=ArmFault::UnknownTran,
              ArmFault::DebugType _debug_type=ArmFault::NODEBUG) :
        AbortFault<DataAbort>(_addr, _write, _domain, _source, _stage2,
                              _tranMethod, _debug_type),
        isv(false), sas (0), sse(0), srt(0), cm(0), sf(false), ar(false)
    {}

    ExceptionClass ec(ThreadContext *tc) const override;
    // @todo: external aborts should be routed if SCR.EA == 1
    bool routeToMonitor(ThreadContext *tc) const override;
    bool routeToHyp(ThreadContext *tc) const override;
    uint32_t iss() const override;
    void annotate(AnnotationIDs id, uint64_t val) override;
    uint32_t vectorCatchFlag() const override { return 0x10001010; }
};

class VirtualDataAbort : public AbortFault<VirtualDataAbort>
{
  public:
    static const MiscRegIndex FsrIndex  = MISCREG_DFSR;
    static const MiscRegIndex FarIndex  = MISCREG_DFAR;
    static const MiscRegIndex HFarIndex = MISCREG_HDFAR;

    VirtualDataAbort(Addr _addr, TlbEntry::DomainType _domain, bool _write,
                     uint8_t _source) :
        AbortFault<VirtualDataAbort>(_addr, _write, _domain, _source, false)
    {}

    void invoke(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class Interrupt : public ArmFaultVals<Interrupt>
{
  public:
    bool routeToMonitor(ThreadContext *tc) const override;
    bool routeToHyp(ThreadContext *tc) const override;
    bool abortDisable(ThreadContext *tc) override;
    uint32_t vectorCatchFlag() const override { return 0x40004040; }
};

class VirtualInterrupt : public ArmFaultVals<VirtualInterrupt>
{
  public:
    VirtualInterrupt();
};

class FastInterrupt : public ArmFaultVals<FastInterrupt>
{
  public:
    bool routeToMonitor(ThreadContext *tc) const override;
    bool routeToHyp(ThreadContext *tc) const override;
    bool abortDisable(ThreadContext *tc) override;
    bool fiqDisable(ThreadContext *tc) override;
    uint32_t vectorCatchFlag() const override { return 0x80008080; }
};

class VirtualFastInterrupt : public ArmFaultVals<VirtualFastInterrupt>
{
  public:
    VirtualFastInterrupt();
};

/// PC alignment fault (AArch64 only)
class PCAlignmentFault : public ArmFaultVals<PCAlignmentFault>
{
  protected:
    /// The unaligned value of the PC
    Addr faultPC;
  public:
    PCAlignmentFault(Addr _faultPC) : faultPC(_faultPC)
    {}
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    bool routeToHyp(ThreadContext *tc) const override;
};

/// Stack pointer alignment fault (AArch64 only)
class SPAlignmentFault : public ArmFaultVals<SPAlignmentFault>
{
  public:
    SPAlignmentFault();
    bool routeToHyp(ThreadContext *tc) const override;
};

/// System error (AArch64 only)
class SystemError : public ArmFaultVals<SystemError>
{
  public:
    SystemError();
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    bool routeToMonitor(ThreadContext *tc) const override;
    bool routeToHyp(ThreadContext *tc) const override;
};

/// System error (AArch64 only)
class SoftwareBreakpoint : public ArmFaultVals<SoftwareBreakpoint>
{
  public:
    SoftwareBreakpoint(ExtMachInst _mach_inst, uint32_t _iss);

    bool routeToHyp(ThreadContext *tc) const override;
    ExceptionClass ec(ThreadContext *tc) const override;
};

class HardwareBreakpoint : public ArmFaultVals<HardwareBreakpoint>
{
  private:
    Addr vAddr;
  public:
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    HardwareBreakpoint(Addr _vaddr, uint32_t _iss);
    bool routeToHyp(ThreadContext *tc) const override;
    ExceptionClass ec(ThreadContext *tc) const override;
};

class Watchpoint : public ArmFaultVals<Watchpoint>
{
  private:
    Addr vAddr;
    bool write;
    bool cm;

  public:
    Watchpoint(ExtMachInst _mach_inst, Addr _vaddr, bool _write, bool _cm);
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
    bool routeToHyp(ThreadContext *tc) const override;
    uint32_t iss() const override;
    ExceptionClass ec(ThreadContext *tc) const override;
    void annotate(AnnotationIDs id, uint64_t val) override;
};

class SoftwareStepFault : public ArmFaultVals<SoftwareStepFault>
{
  private:
    bool isldx;
    bool stepped;

  public:
    SoftwareStepFault(ExtMachInst _mach_inst, bool is_ldx, bool stepped);
    bool routeToHyp(ThreadContext *tc) const override;
    uint32_t iss() const override;
    ExceptionClass ec(ThreadContext *tc) const override;
};

// A fault that flushes the pipe, excluding the faulting instructions
class ArmSev : public ArmFaultVals<ArmSev>
{
  public:
    ArmSev () {}
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
};

/// Illegal Instruction Set State fault (AArch64 only)
class IllegalInstSetStateFault : public ArmFaultVals<IllegalInstSetStateFault>
{
  public:
    IllegalInstSetStateFault();

    bool routeToHyp(ThreadContext *tc) const override;
};

/*
 * Explicitly declare template static member variables to avoid warnings
 * in some clang versions
 */
template<> ArmFault::FaultVals ArmFaultVals<Reset>::vals;
template<> ArmFault::FaultVals ArmFaultVals<UndefinedInstruction>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SupervisorCall>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SecureMonitorCall>::vals;
template<> ArmFault::FaultVals ArmFaultVals<HypervisorCall>::vals;
template<> ArmFault::FaultVals ArmFaultVals<PrefetchAbort>::vals;
template<> ArmFault::FaultVals ArmFaultVals<DataAbort>::vals;
template<> ArmFault::FaultVals ArmFaultVals<VirtualDataAbort>::vals;
template<> ArmFault::FaultVals ArmFaultVals<HypervisorTrap>::vals;
template<> ArmFault::FaultVals ArmFaultVals<Interrupt>::vals;
template<> ArmFault::FaultVals ArmFaultVals<VirtualInterrupt>::vals;
template<> ArmFault::FaultVals ArmFaultVals<FastInterrupt>::vals;
template<> ArmFault::FaultVals ArmFaultVals<VirtualFastInterrupt>::vals;
template<> ArmFault::FaultVals ArmFaultVals<IllegalInstSetStateFault>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SupervisorTrap>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SecureMonitorTrap>::vals;
template<> ArmFault::FaultVals ArmFaultVals<PCAlignmentFault>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SPAlignmentFault>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SystemError>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SoftwareBreakpoint>::vals;
template<> ArmFault::FaultVals ArmFaultVals<HardwareBreakpoint>::vals;
template<> ArmFault::FaultVals ArmFaultVals<Watchpoint>::vals;
template<> ArmFault::FaultVals ArmFaultVals<SoftwareStepFault>::vals;
template<> ArmFault::FaultVals ArmFaultVals<ArmSev>::vals;

/**
 * Returns true if the fault passed as a first argument was triggered
 * by a memory access, false otherwise.
 * If true it is storing the faulting address in the va argument
 *
 * @param fault generated fault
 * @param va function will modify this passed-by-reference parameter
 *           with the correct faulting virtual address
 * @return true if va contains a valid value, false otherwise
 */
bool getFaultVAddr(Fault fault, Addr &va);

} // namespace ArmISA
} // namespace gem5

#endif // __ARM_FAULTS_HH__
