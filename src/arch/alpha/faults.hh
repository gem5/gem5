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

#ifndef __ARCH_ALPHA_FAULTS_HH__
#define __ARCH_ALPHA_FAULTS_HH__

#include "arch/alpha/pagetable.hh"
#include "mem/request.hh"
#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace AlphaISA {

typedef const Addr FaultVect;

class AlphaFault : public FaultBase
{
  protected:
    virtual bool skipFaultingInstruction() {return false;}
    virtual bool setRestartAddress() {return true;}
  public:
    virtual ~AlphaFault() {}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
    virtual FaultVect vect() = 0;
    virtual FaultStat & countStat() = 0;
};

class MachineCheckFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class AlignmentFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    bool isAlignmentFault() const {return true;}
};

class ResetFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ArithmeticFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  protected:
    bool skipFaultingInstruction() {return true;}

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class InterruptFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  protected:
    bool setRestartAddress() {return false;}

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbFault : public AlphaFault
{
  protected:
    VAddr vaddr;
    Request::Flags reqFlags;
    uint64_t flags;

  public:
    DtbFault(VAddr _vaddr, Request::Flags _reqFlags, uint64_t _flags)
        : vaddr(_vaddr), reqFlags(_reqFlags), flags(_flags)
    { }
    FaultName name() const = 0;
    FaultVect vect() = 0;
    FaultStat & countStat() = 0;
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class NDtbMissFault : public DtbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    NDtbMissFault(VAddr vaddr, Request::Flags reqFlags, uint64_t flags)
        : DtbFault(vaddr, reqFlags, flags)
    { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class PDtbMissFault : public DtbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    PDtbMissFault(VAddr vaddr, Request::Flags reqFlags, uint64_t flags)
        : DtbFault(vaddr, reqFlags, flags)
    { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbPageFault : public DtbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    DtbPageFault(VAddr vaddr, Request::Flags reqFlags, uint64_t flags)
        : DtbFault(vaddr, reqFlags, flags)
    { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbAcvFault : public DtbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    DtbAcvFault(VAddr vaddr, Request::Flags reqFlags, uint64_t flags)
        : DtbFault(vaddr, reqFlags, flags)
    { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class DtbAlignmentFault : public DtbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    DtbAlignmentFault(VAddr vaddr, Request::Flags reqFlags, uint64_t flags)
        : DtbFault(vaddr, reqFlags, flags)
    { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class ItbFault : public AlphaFault
{
  protected:
    Addr pc;

  public:
    ItbFault(Addr _pc) : pc(_pc) { }
    FaultName name() const = 0;
    FaultVect vect() = 0;
    FaultStat & countStat() = 0;
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class ItbPageFault : public ItbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    ItbPageFault(Addr pc) : ItbFault(pc) { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class ItbAcvFault : public ItbFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    ItbAcvFault(Addr pc) : ItbFault(pc) { }
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class UnimplementedOpcodeFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class FloatEnableFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class PalFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  protected:
    bool skipFaultingInstruction() {return true;}

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

class IntegerOverflowFault : public AlphaFault
{
  private:
    static FaultName _name;
    static FaultVect _vect;
    static FaultStat _count;

  public:
    FaultName name() const {return _name;}
    FaultVect vect() {return _vect;}
    FaultStat & countStat() {return _count;}
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_FAULTS_HH__
