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
 * Authors: Nathan Binkert
 *          Gabe Black
 */

#ifndef __FAULTS_HH__
#define __FAULTS_HH__

#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "sim/stats.hh"

class ThreadContext;

typedef const char * FaultName;
typedef Stats::Scalar FaultStat;

// Each class has it's name statically define in _name,
// and has a virtual function to access it's name.
// The function is necessary because otherwise, all objects
// which are being accessed cast as a FaultBase * (namely
// all faults returned using the Fault type) will use the
// generic FaultBase name.

class FaultBase
{
  public:
    virtual FaultName name() const = 0;
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                        StaticInst::nullStaticInstPtr);
};

class UnimpFault : public FaultBase
{
  private:
    std::string panicStr;
  public:
    UnimpFault(std::string _str)
        : panicStr(_str)
    { }

    FaultName name() const {return "Unimplemented simulator feature";}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class ReExec : public FaultBase
{
  public:
    virtual FaultName name() const { return "Re-execution fault";}
    ReExec() {}
    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class GenericPageTableFault : public FaultBase
{
  private:
    Addr vaddr;
  public:
    FaultName name() const {return "Generic page table fault";}
    GenericPageTableFault(Addr va) : vaddr(va) {}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

class GenericAlignmentFault : public FaultBase
{
  private:
    Addr vaddr;
  public:
    FaultName name() const {return "Generic alignment fault";}
    GenericAlignmentFault(Addr va) : vaddr(va) {}
    void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                StaticInst::nullStaticInstPtr);
};

#endif // __FAULTS_HH__
