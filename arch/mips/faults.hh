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

#ifndef __MIPS_FAULTS_HH__
#define __MIPS_FAULTS_HH__

#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace MipsISA
{

typedef const Addr FaultVect;

class MipsFault : public FaultBase
{
  protected:
    virtual bool skipFaultingInstruction() {return false;}
    virtual bool setRestartAddress() {return true;}
  public:
#if FULL_SYSTEM
    void invoke(ExecContext * xc);
#endif
    virtual FaultVect vect() = 0;
    virtual FaultStat & countStat() = 0;
};

class MachineCheckFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    bool isMachineCheckFault() {return true;}
};

class AlignmentFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    bool isAlignmentFault() {return true;}
};

static inline Fault genMachineCheckFault()
{
    return new MachineCheckFault;
}

static inline Fault genAlignmentFault()
{
    return new AlignmentFault;
}

class ResetFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ArithmeticFault : public MipsFault
{
  protected:
    bool skipFaultingInstruction() {return true;}
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
#if FULL_SYSTEM
    void invoke(ExecContext * xc);
#endif
};

class InterruptFault : public MipsFault
{
  protected:
    bool setRestartAddress() {return false;}
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class NDtbMissFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class PDtbMissFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbPageFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbAcvFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ItbMissFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ItbPageFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ItbAcvFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class UnimplementedOpcodeFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class FloatEnableFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class PalFault : public MipsFault
{
  protected:
    bool skipFaultingInstruction() {return true;}
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class IntegerOverflowFault : public MipsFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

} // MipsISA namespace

#endif // __FAULTS_HH__
