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

#ifndef __ALPHA_FAULTS_HH__
#define __ALPHA_FAULTS_HH__

#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace SparcISA
{

typedef const uint32_t TrapType;
typedef const uint32_t FaultPriority;

class SparcFault : public FaultBase
{
  public:
#if FULL_SYSTEM
    void invoke(ExecContext * xc);
#endif
    virtual TrapType trapType() = 0;
    virtual FaultPriority priority() = 0;
    virtual FaultStat & countStat() = 0;
};

class InternalProcessorError : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
    bool isMachineCheckFault() {return true;}
};

class MemAddressNotAligned : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
    bool isAlignmentFault() {return true;}
};

static inline Fault genMachineCheckFault()
{
    return new InternalProcessorError;
}

static inline Fault genAlignmentFault()
{
    return new MemAddressNotAligned;
}

class PowerOnReset : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class WatchDogReset : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class ExternallyInitiatedReset : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class SoftwareInitiatedReset : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class REDStateException : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class InstructionAccessException : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class InstructionAccessMMUMiss : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class InstructionAccessError : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class IllegalInstruction : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class PrivelegedOpcode : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class UnimplementedLDD : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class UnimplementedSTD : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class FpDisabled : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class FpExceptionIEEE754 : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class FpExceptionOther : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class TagOverflow : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class DivisionByZero : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class DataAccessException : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class DataAccessMMUMiss : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class DataAccessError : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class DataAccessProtection : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class LDDFMemAddressNotAligned : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class STDFMemAddressNotAligned : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class PrivelegedAction : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class LDQFMemAddressNotAligned : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class STQFMemAddressNotAligned : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class AsyncDataError : public SparcFault
{
  private:
    static FaultName _name;
    static TrapType _trapType;
    static FaultPriority _priority;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    TrapType trapType() {return _trapType;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class EnumeratedFault : public SparcFault
{
  protected:
    uint32_t _n;
    virtual TrapType baseTrapType() = 0;
  public:
    EnumeratedFault(uint32_t n) : SparcFault() {_n = n;}
    TrapType trapType() {return baseTrapType() + _n;}
};

class CleanWindow : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    CleanWindow(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class InterruptLevelN : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    InterruptLevelN(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return 32 - _n;}
    FaultStat & countStat() {return _count;}
};

class SpillNNormal : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    SpillNNormal(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class SpillNOther : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    SpillNOther(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class FillNNormal : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    FillNNormal(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class FillNOther : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    FillNOther(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

class TrapInstruction : public EnumeratedFault
{
  private:
    static FaultName _name;
    static TrapType _baseTrapType;
    static FaultPriority _priority;
    static FaultStat _count;
    TrapType baseTrapType() {return _baseTrapType;}
  public:
    TrapInstruction(uint32_t n) : EnumeratedFault(n) {;}
    FaultName name() {return _name;}
    FaultPriority priority() {return _priority;}
    FaultStat & countStat() {return _count;}
};

} // SparcISA namespace

#endif // __FAULTS_HH__
