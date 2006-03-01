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

namespace AlphaISA
{

typedef const Addr FaultVect;

class AlphaFault : public virtual FaultBase
{
  public:
#if FULL_SYSTEM
    void invoke(ExecContext * xc);
#endif
    virtual FaultVect vect() = 0;
};

class AlphaMachineCheckFault :
    public MachineCheckFault,
    public AlphaFault
{
  private:
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class AlphaAlignmentFault :
    public AlignmentFault,
    public AlphaFault
{
  private:
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

static inline Fault genMachineCheckFault()
{
    return new AlphaMachineCheckFault;
}

static inline Fault genAlignmentFault()
{
    return new AlphaAlignmentFault;
}

class ResetFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class ArithmeticFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class InterruptFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class NDtbMissFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class PDtbMissFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class DtbPageFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class DtbAcvFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class ItbMissFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class ItbPageFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class ItbAcvFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class UnimplementedOpcodeFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class FloatEnableFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class PalFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

class IntegerOverflowFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _stat;
  public:
    FaultName name() {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & stat() {return _stat;}
};

} // AlphaISA namespace

#endif // __FAULTS_HH__
