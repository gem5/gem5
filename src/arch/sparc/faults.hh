/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __SPARC_FAULTS_HH__
#define __SPARC_FAULTS_HH__

#include "cpu/null_static_inst.hh"
#include "cpu/static_inst.hh"
#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace gem5
{

namespace SparcISA
{

typedef uint32_t TrapType;
typedef uint32_t FaultPriority;

class ITB;

class SparcFaultBase : public FaultBase
{
  public:
    enum PrivilegeLevel
    {
        U,
        User = U,
        P,
        Privileged = P,
        H,
        Hyperprivileged = H,
        NumLevels,
        SH = -1,
        ShouldntHappen = SH
    };

    using PrivilegeLevelSpec = std::array<PrivilegeLevel, NumLevels>;

    struct FaultVals
    {
        const FaultName name;
        const TrapType trapType;
        const FaultPriority priority;
        const PrivilegeLevelSpec nextPrivilegeLevel;

        FaultVals(const FaultName &name_, const TrapType &trapType_,
                  const FaultPriority &priority_, const PrivilegeLevelSpec &il)
            : name(name_),
              trapType(trapType_),
              priority(priority_),
              nextPrivilegeLevel(il)
        {}
    };

    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
    virtual TrapType trapType() = 0;
    virtual FaultPriority priority() = 0;
    virtual PrivilegeLevel getNextLevel(PrivilegeLevel current) = 0;
};

template <typename T>
class SparcFault : public SparcFaultBase
{
  protected:
    static FaultVals vals;

  public:
    FaultName
    name() const
    {
        return vals.name;
    }

    TrapType
    trapType()
    {
        return vals.trapType;
    }

    FaultPriority
    priority()
    {
        return vals.priority;
    }

    PrivilegeLevel
    getNextLevel(PrivilegeLevel current)
    {
        return vals.nextPrivilegeLevel[current];
    }
};

class PowerOnReset : public SparcFault<PowerOnReset>
{
  public:
    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

class WatchDogReset : public SparcFault<WatchDogReset>
{
};

class ExternallyInitiatedReset : public SparcFault<ExternallyInitiatedReset>
{
};

class SoftwareInitiatedReset : public SparcFault<SoftwareInitiatedReset>
{
};

class REDStateException : public SparcFault<REDStateException>
{
};

class StoreError : public SparcFault<StoreError>
{
};

class InstructionAccessException :
    public SparcFault<InstructionAccessException>
{
};

// class InstructionAccessMMUMiss : public SparcFault<InstructionAccessMMUMiss>
// {};

class InstructionAccessError : public SparcFault<InstructionAccessError>
{
};

class IllegalInstruction : public SparcFault<IllegalInstruction>
{
};

class PrivilegedOpcode : public SparcFault<PrivilegedOpcode>
{
};

// class UnimplementedLDD : public SparcFault<UnimplementedLDD> {};

// class UnimplementedSTD : public SparcFault<UnimplementedSTD> {};

class FpDisabled : public SparcFault<FpDisabled>
{
};

class VecDisabled : public SparcFault<VecDisabled>
{
};

class FpExceptionIEEE754 : public SparcFault<FpExceptionIEEE754>
{
};

class FpExceptionOther : public SparcFault<FpExceptionOther>
{
};

class TagOverflow : public SparcFault<TagOverflow>
{
};

class CleanWindow : public SparcFault<CleanWindow>
{
};

class DivisionByZero : public SparcFault<DivisionByZero>
{
};

class InternalProcessorError : public SparcFault<InternalProcessorError>
{
};

class InstructionInvalidTSBEntry :
    public SparcFault<InstructionInvalidTSBEntry>
{
};

class DataInvalidTSBEntry : public SparcFault<DataInvalidTSBEntry>
{
};

class DataAccessException : public SparcFault<DataAccessException>
{
};

// class DataAccessMMUMiss : public SparcFault<DataAccessMMUMiss> {};

class DataAccessError : public SparcFault<DataAccessError>
{
};

class DataAccessProtection : public SparcFault<DataAccessProtection>
{
};

class MemAddressNotAligned : public SparcFault<MemAddressNotAligned>
{
};

class LDDFMemAddressNotAligned : public SparcFault<LDDFMemAddressNotAligned>
{
};

class STDFMemAddressNotAligned : public SparcFault<STDFMemAddressNotAligned>
{
};

class PrivilegedAction : public SparcFault<PrivilegedAction>
{
};

class LDQFMemAddressNotAligned : public SparcFault<LDQFMemAddressNotAligned>
{
};

class STQFMemAddressNotAligned : public SparcFault<STQFMemAddressNotAligned>
{
};

class InstructionRealTranslationMiss :
    public SparcFault<InstructionRealTranslationMiss>
{
};

class DataRealTranslationMiss : public SparcFault<DataRealTranslationMiss>
{
};

// class AsyncDataError : public SparcFault<AsyncDataError> {};

template <class T>
class EnumeratedFault : public SparcFault<T>
{
  protected:
    uint32_t _n;

  public:
    EnumeratedFault(uint32_t n) : SparcFault<T>(), _n(n) {}

    TrapType
    trapType()
    {
        return SparcFault<T>::trapType() + _n;
    }
};

class InterruptLevelN : public EnumeratedFault<InterruptLevelN>
{
  public:
    InterruptLevelN(uint32_t n) : EnumeratedFault<InterruptLevelN>(n) { ; }

    FaultPriority
    priority()
    {
        return 3200 - _n * 100;
    }
};

class HstickMatch : public SparcFault<HstickMatch>
{
};

class TrapLevelZero : public SparcFault<TrapLevelZero>
{
};

class InterruptVector : public SparcFault<InterruptVector>
{
};

class PAWatchpoint : public SparcFault<PAWatchpoint>
{
};

class VAWatchpoint : public SparcFault<VAWatchpoint>
{
};

class FastInstructionAccessMMUMiss :
    public SparcFault<FastInstructionAccessMMUMiss>
{
  protected:
    Addr vaddr;

  public:
    FastInstructionAccessMMUMiss(Addr addr) : vaddr(addr) {}

    FastInstructionAccessMMUMiss() : vaddr(0) {}

    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

class FastDataAccessMMUMiss : public SparcFault<FastDataAccessMMUMiss>
{
  protected:
    Addr vaddr;

  public:
    FastDataAccessMMUMiss(Addr addr) : vaddr(addr) {}

    FastDataAccessMMUMiss() : vaddr(0) {}

    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

class FastDataAccessProtection : public SparcFault<FastDataAccessProtection>
{
};

class InstructionBreakpoint : public SparcFault<InstructionBreakpoint>
{
};

class CpuMondo : public SparcFault<CpuMondo>
{
};

class DevMondo : public SparcFault<DevMondo>
{
};

class ResumableError : public SparcFault<ResumableError>
{
};

class SpillNNormal : public EnumeratedFault<SpillNNormal>
{
  public:
    SpillNNormal(uint32_t n) : EnumeratedFault<SpillNNormal>(n) { ; }

    // These need to be handled specially to enable spill traps in SE
    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

class SpillNOther : public EnumeratedFault<SpillNOther>
{
  public:
    SpillNOther(uint32_t n) : EnumeratedFault<SpillNOther>(n) {}
};

class FillNNormal : public EnumeratedFault<FillNNormal>
{
  public:
    FillNNormal(uint32_t n) : EnumeratedFault<FillNNormal>(n) {}

    // These need to be handled specially to enable fill traps in SE
    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

class FillNOther : public EnumeratedFault<FillNOther>
{
  public:
    FillNOther(uint32_t n) : EnumeratedFault<FillNOther>(n) {}
};

class TrapInstruction : public EnumeratedFault<TrapInstruction>
{
  public:
    TrapInstruction(uint32_t n) : EnumeratedFault<TrapInstruction>(n) {}

    // In SE, trap instructions are requesting services from the OS.
    void invoke(ThreadContext *tc,
                const StaticInstPtr &inst = nullStaticInstPtr);
};

/*
 * Explicitly declare template static member variables to avoid warnings
 * in some clang versions
 */
template <>
SparcFaultBase::FaultVals SparcFault<PowerOnReset>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<WatchDogReset>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<ExternallyInitiatedReset>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<SoftwareInitiatedReset>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<REDStateException>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<StoreError>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InstructionAccessException>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InstructionAccessError>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<IllegalInstruction>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<PrivilegedOpcode>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FpDisabled>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<VecDisabled>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FpExceptionIEEE754>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FpExceptionOther>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<TagOverflow>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<CleanWindow>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DivisionByZero>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InternalProcessorError>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InstructionInvalidTSBEntry>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DataInvalidTSBEntry>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DataAccessException>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DataAccessError>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DataAccessProtection>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<MemAddressNotAligned>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<LDDFMemAddressNotAligned>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<STDFMemAddressNotAligned>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<PrivilegedAction>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<LDQFMemAddressNotAligned>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<STQFMemAddressNotAligned>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InstructionRealTranslationMiss>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DataRealTranslationMiss>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InterruptLevelN>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<HstickMatch>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<TrapLevelZero>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InterruptVector>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<PAWatchpoint>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<VAWatchpoint>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FastInstructionAccessMMUMiss>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FastDataAccessMMUMiss>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FastDataAccessProtection>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<InstructionBreakpoint>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<CpuMondo>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<DevMondo>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<ResumableError>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<SpillNNormal>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<SpillNOther>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FillNNormal>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<FillNOther>::vals;
template <>
SparcFaultBase::FaultVals SparcFault<TrapInstruction>::vals;

void enterREDState(ThreadContext *tc);

void doREDFault(ThreadContext *tc, TrapType tt);

void doNormalFault(ThreadContext *tc, TrapType tt, bool gotoHpriv);

void getREDVector(RegVal TT, Addr &PC, Addr &NPC);

void getHyperVector(ThreadContext *tc, Addr &PC, Addr &NPC, RegVal TT);

void getPrivVector(ThreadContext *tc, Addr &PC, Addr &NPC, RegVal TT,
                   RegVal TL);

} // namespace SparcISA
} // namespace gem5

#endif // __SPARC_FAULTS_HH__
