/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_MIPS_CPU_HH__
#define __CPU_O3_MIPS_CPU_HH__

#include "arch/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "cpu/o3/cpu.hh"
#include "sim/byteswap.hh"

class EndQuiesceEvent;
namespace Kernel {
    class Statistics;
};

class TranslatingPort;

/**
 * MipsO3CPU class.  Derives from the FullO3CPU class, and
 * implements all ISA and implementation specific functions of the
 * CPU.  This is the CPU class that is used for the SimObjects, and is
 * what is given to the DynInsts.  Most of its state exists in the
 * FullO3CPU; the state is has is mainly for ISA specific
 * functionality.
 */
template <class Impl>
class MipsO3CPU : public FullO3CPU<Impl>
{
  protected:
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MiscRegFile MiscRegFile;

  public:
    typedef O3ThreadState<Impl> ImplState;
    typedef O3ThreadState<Impl> Thread;
    typedef typename Impl::Params Params;

    /** Constructs an MipsO3CPU with the given parameters. */
    MipsO3CPU(Params *params);

    /** Registers statistics. */
    void regStats();

    /** Translates instruction requestion in syscall emulation mode. */
    Fault translateInstReq(RequestPtr &req, Thread *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }

    /** Translates data read request in syscall emulation mode. */
    Fault translateDataReadReq(RequestPtr &req, Thread *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }

    /** Translates data write request in syscall emulation mode. */
    Fault translateDataWriteReq(RequestPtr &req, Thread *thread)
    {
        return thread->getProcessPtr()->pTable->translate(req);
    }

    /** Reads a miscellaneous register. */
    MiscReg readMiscReg(int misc_reg, unsigned tid);

    /** Reads a misc. register, including any side effects the read
     * might have as defined by the architecture.
     */
    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault, unsigned tid);

    /** Sets a miscellaneous register. */
    Fault setMiscReg(int misc_reg, const MiscReg &val, unsigned tid);

    /** Sets a misc. register, including any side effects the write
     * might have as defined by the architecture.
     */
    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val, unsigned tid);

    /** Initiates a squash of all in-flight instructions for a given
     * thread.  The source of the squash is an external update of
     * state through the TC.
     */
    void squashFromTC(unsigned tid);

    /** Traps to handle given fault. */
    void trap(Fault fault, unsigned tid);

    /** Executes a syscall.
     * @todo: Determine if this needs to be virtual.
     */
    void syscall(int64_t callnum, int tid);
    /** Gets a syscall argument. */
    IntReg getSyscallArg(int i, int tid);

    /** Used to shift args for indirect syscall. */
    void setSyscallArg(int i, IntReg val, int tid);

    /** Sets the return value of a syscall. */
    void setSyscallReturn(SyscallReturn return_value, int tid);

    /** CPU read function, forwards read to LSQ. */
    template <class T>
    Fault read(RequestPtr &req, T &data, int load_idx)
    {
        return this->iew.ldstQueue.read(req, data, load_idx);
    }

    /** CPU write function, forwards write to LSQ. */
    template <class T>
    Fault write(RequestPtr &req, T &data, int store_idx)
    {
        return this->iew.ldstQueue.write(req, data, store_idx);
    }

    Addr lockAddr;

    /** Temporary fix for the lock flag, works in the UP case. */
    bool lockFlag;
};

#endif // __CPU_O3_MIPS_CPU_HH__
