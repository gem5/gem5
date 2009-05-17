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
 * Authors: Steve Reinhardt
 *          Gabe Black
 */

#ifndef __ARCH_ALPHA_MISCREGFILE_HH__
#define __ARCH_ALPHA_MISCREGFILE_HH__

#include <iosfwd>

#include "arch/alpha/ipr.hh"
#include "arch/alpha/types.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

class Checkpoint;
class ThreadContext;
class BaseCPU;

namespace AlphaISA {

enum MiscRegIndex
{
    MISCREG_FPCR = NumInternalProcRegs,
    MISCREG_UNIQ,
    MISCREG_LOCKFLAG,
    MISCREG_LOCKADDR,
    MISCREG_INTR
};

class MiscRegFile
{
  public:
    friend class RegFile;
    typedef uint64_t InternalProcReg;

  protected:
    uint64_t fpcr;       // floating point condition codes
    uint64_t uniq;       // process-unique register
    bool lock_flag;      // lock flag for LL/SC
    Addr lock_addr;      // lock address for LL/SC
    int intr_flag;

    InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

    BaseCPU *cpu;

  protected:
    InternalProcReg readIpr(int idx, ThreadContext *tc);
    void setIpr(int idx, InternalProcReg val, ThreadContext *tc);

  public:
    MiscRegFile()
    {
        initializeIprTable();
    }

    MiscRegFile(BaseCPU *cpu);

    // These functions should be removed once the simplescalar cpu
    // model has been replaced.
    int getInstAsid();
    int getDataAsid();

    MiscReg readRegNoEffect(int misc_reg, unsigned tid = 0);
    MiscReg readReg(int misc_reg, ThreadContext *tc, unsigned tid = 0);

    void setRegNoEffect(int misc_reg, const MiscReg &val, unsigned tid = 0);
    void setReg(int misc_reg, const MiscReg &val, ThreadContext *tc, unsigned tid = 0);

    void
    clear()
    {
        fpcr = 0;
        uniq = 0;
        lock_flag = 0;
        lock_addr = 0;
        intr_flag = 0;
    }

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    void reset(std::string core_name, unsigned num_threads,
               unsigned num_vpes, BaseCPU *_cpu)
    { }


    void expandForMultithreading(unsigned num_threads, unsigned num_vpes)
    { }


};

void copyIprs(ThreadContext *src, ThreadContext *dest);

} // namespace AlphaISA

#endif // __ARCH_ALPHA_MISCREGFILE_HH__
