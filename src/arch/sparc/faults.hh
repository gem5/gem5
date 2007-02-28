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
 *
 * Authors: Gabe Black
 *          Kevin Lim
 */

#ifndef __SPARC_FAULTS_HH__
#define __SPARC_FAULTS_HH__

#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace SparcISA
{

typedef uint32_t TrapType;
typedef uint32_t FaultPriority;

class SparcFaultBase : public FaultBase
{
  public:
    enum PrivilegeLevel
    {
        U, User = U,
        P, Privileged = P,
        H, Hyperprivileged = H,
        NumLevels,
        SH = -1,
        ShouldntHappen = SH
    };
    struct FaultVals
    {
        const FaultName name;
        const TrapType trapType;
        const FaultPriority priority;
        const PrivilegeLevel nextPrivilegeLevel[NumLevels];
        FaultStat count;
    };
#if FULL_SYSTEM
    void invoke(ThreadContext * tc);
#endif
    virtual FaultName name() = 0;
    virtual TrapType trapType() = 0;
    virtual FaultPriority priority() = 0;
    virtual FaultStat & countStat() = 0;
    virtual PrivilegeLevel getNextLevel(PrivilegeLevel current) = 0;
};

template<typename T>
class SparcFault : public SparcFaultBase
{
  protected:
    static FaultVals vals;
  public:
    FaultName name() {return vals.name;}
    TrapType trapType() {return vals.trapType;}
    FaultPriority priority() {return vals.priority;}
    FaultStat & countStat() {return vals.count;}
    PrivilegeLevel getNextLevel(PrivilegeLevel current)
    {
        return vals.nextPrivilegeLevel[current];
    }
};

class PowerOnReset : public SparcFault<PowerOnReset>
{
    void invoke(ThreadContext * tc);
};

class WatchDogReset : public SparcFault<WatchDogReset> {};

class ExternallyInitiatedReset : public SparcFault<ExternallyInitiatedReset> {};

class SoftwareInitiatedReset : public SparcFault<SoftwareInitiatedReset> {};

class REDStateException : public SparcFault<REDStateException> {};

class StoreError : public SparcFault<StoreError> {};

class InstructionAccessException : public SparcFault<InstructionAccessException> {};

//class InstructionAccessMMUMiss : public SparcFault<InstructionAccessMMUMiss> {};

class InstructionAccessError : public SparcFault<InstructionAccessError> {};

class IllegalInstruction : public SparcFault<IllegalInstruction> {};

class PrivilegedOpcode : public SparcFault<PrivilegedOpcode> {};

//class UnimplementedLDD : public SparcFault<UnimplementedLDD> {};

//class UnimplementedSTD : public SparcFault<UnimplementedSTD> {};

class FpDisabled : public SparcFault<FpDisabled> {};

class FpExceptionIEEE754 : public SparcFault<FpExceptionIEEE754> {};

class FpExceptionOther : public SparcFault<FpExceptionOther> {};

class TagOverflow : public SparcFault<TagOverflow> {};

class CleanWindow : public SparcFault<CleanWindow> {};

class DivisionByZero : public SparcFault<DivisionByZero> {};

class InternalProcessorError :
    public SparcFault<InternalProcessorError>
{
  public:
    bool isMachineCheckFault() {return true;}
};

class InstructionInvalidTSBEntry : public SparcFault<InstructionInvalidTSBEntry> {};

class DataInvalidTSBEntry : public SparcFault<DataInvalidTSBEntry> {};

class DataAccessException : public SparcFault<DataAccessException> {};

//class DataAccessMMUMiss : public SparcFault<DataAccessMMUMiss> {};

class DataAccessError : public SparcFault<DataAccessError> {};

class DataAccessProtection : public SparcFault<DataAccessProtection> {};

class MemAddressNotAligned :
    public SparcFault<MemAddressNotAligned>
{
  public:
    bool isAlignmentFault() {return true;}
};

class LDDFMemAddressNotAligned : public SparcFault<LDDFMemAddressNotAligned> {};

class STDFMemAddressNotAligned : public SparcFault<STDFMemAddressNotAligned> {};

class PrivilegedAction : public SparcFault<PrivilegedAction> {};

class LDQFMemAddressNotAligned : public SparcFault<LDQFMemAddressNotAligned> {};

class STQFMemAddressNotAligned : public SparcFault<STQFMemAddressNotAligned> {};

class InstructionRealTranslationMiss :
    public SparcFault<InstructionRealTranslationMiss> {};

class DataRealTranslationMiss : public SparcFault<DataRealTranslationMiss> {};

//class AsyncDataError : public SparcFault<AsyncDataError> {};

template <class T>
class EnumeratedFault : public SparcFault<T>
{
  protected:
    uint32_t _n;
  public:
    EnumeratedFault(uint32_t n) : SparcFault<T>(), _n(n) {}
    TrapType trapType() {return SparcFault<T>::trapType() + _n;}
};

class InterruptLevelN : public EnumeratedFault<InterruptLevelN>
{
  public:
    InterruptLevelN(uint32_t n) : EnumeratedFault<InterruptLevelN>(n) {;}
    FaultPriority priority() {return 3200 - _n*100;}
};

class HstickMatch : public SparcFault<HstickMatch> {};

class TrapLevelZero : public SparcFault<TrapLevelZero> {};

class PAWatchpoint : public SparcFault<PAWatchpoint> {};

class VAWatchpoint : public SparcFault<VAWatchpoint> {};

class FastInstructionAccessMMUMiss :
    public SparcFault<FastInstructionAccessMMUMiss> {};

class FastDataAccessMMUMiss : public SparcFault<FastDataAccessMMUMiss> {};

class FastDataAccessProtection : public SparcFault<FastDataAccessProtection> {};

class InstructionBreakpoint : public SparcFault<InstructionBreakpoint> {};

class CpuMondo : public SparcFault<CpuMondo> {};

class DevMondo : public SparcFault<DevMondo> {};

class ResumableError : public SparcFault<ResumableError> {};

class SpillNNormal : public EnumeratedFault<SpillNNormal>
{
  public:
    SpillNNormal(uint32_t n) : EnumeratedFault<SpillNNormal>(n) {;}
    //These need to be handled specially to enable spill traps in SE
#if !FULL_SYSTEM
    void invoke(ThreadContext * tc);
#endif
};

class SpillNOther : public EnumeratedFault<SpillNOther>
{
  public:
    SpillNOther(uint32_t n) : EnumeratedFault<SpillNOther>(n) {;}
};

class FillNNormal : public EnumeratedFault<FillNNormal>
{
  public:
    FillNNormal(uint32_t n) : EnumeratedFault<FillNNormal>(n) {;}
    //These need to be handled specially to enable fill traps in SE
#if !FULL_SYSTEM
    void invoke(ThreadContext * tc);
#endif
};

class FillNOther : public EnumeratedFault<FillNOther>
{
  public:
    FillNOther(uint32_t n) : EnumeratedFault<FillNOther>(n) {;}
};

class TrapInstruction : public EnumeratedFault<TrapInstruction>
{
  public:
    TrapInstruction(uint32_t n) : EnumeratedFault<TrapInstruction>(n) {;}
    //In SE, trap instructions are requesting services from the OS.
#if !FULL_SYSTEM
    void invoke(ThreadContext * tc);
#endif
};

#if !FULL_SYSTEM
class PageTableFault : public SparcFault<PageTableFault>
{
  private:
    Addr vaddr;
  public:
    PageTableFault(Addr va) : vaddr(va) {}
    void invoke(ThreadContext * tc);
};

static inline Fault genPageTableFault(Addr va)
{
    return new PageTableFault(va);
}
#endif

static inline Fault genMachineCheckFault()
{
    return new InternalProcessorError;
}

static inline Fault genAlignmentFault()
{
    return new MemAddressNotAligned;
}


} // SparcISA namespace

#endif // __SPARC_FAULTS_HH__
