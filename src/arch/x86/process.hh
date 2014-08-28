/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 */

#ifndef __ARCH_X86_PROCESS_HH__
#define __ARCH_X86_PROCESS_HH__

#include <string>
#include <vector>

#include "sim/process.hh"
#include "mem/multi_level_page_table.hh"

class SyscallDesc;

namespace X86ISA
{
    enum X86AuxiliaryVectorTypes {
        M5_AT_SYSINFO = 32,
        M5_AT_SYSINFO_EHDR = 33
    };

    class X86LiveProcess : public LiveProcess
    {
      protected:
        Addr _gdtStart;
        Addr _gdtSize;

        SyscallDesc *syscallDescs;
        const int numSyscallDescs;

        X86LiveProcess(LiveProcessParams * params, ObjectFile *objFile,
                SyscallDesc *_syscallDescs, int _numSyscallDescs);

        template<class IntType>
        void argsInit(int pageSize,
                std::vector<AuxVector<IntType> > extraAuxvs);

      public:
        Addr gdtStart()
        { return _gdtStart; }
        
        Addr gdtSize()
        { return _gdtSize; }

        SyscallDesc* getDesc(int callnum);

        void setSyscallReturn(ThreadContext *tc, SyscallReturn return_value);
    };

    class X86_64LiveProcess : public X86LiveProcess
    {
      protected:
        X86_64LiveProcess(LiveProcessParams *params, ObjectFile *objFile,
                SyscallDesc *_syscallDescs, int _numSyscallDescs);

        class VSyscallPage
        {
          public:
            Addr base;
            Addr size;
            Addr vtimeOffset;
            Addr vgettimeofdayOffset;
        };
        VSyscallPage vsyscallPage;

      public:
        void argsInit(int intSize, int pageSize);
        void initState();

        X86ISA::IntReg getSyscallArg(ThreadContext *tc, int &i);
        /// Explicitly import the otherwise hidden getSyscallArg
        using LiveProcess::getSyscallArg;
        void setSyscallArg(ThreadContext *tc, int i, X86ISA::IntReg val);
    };

    class I386LiveProcess : public X86LiveProcess
    {
      protected:
        I386LiveProcess(LiveProcessParams *params, ObjectFile *objFile,
                SyscallDesc *_syscallDescs, int _numSyscallDescs);

        class VSyscallPage
        {
          public:
            Addr base;
            Addr size;
            Addr vsyscallOffset;
            Addr vsysexitOffset;
        };
        VSyscallPage vsyscallPage;

      public:
        void argsInit(int intSize, int pageSize);
        void initState();

        void syscall(int64_t callnum, ThreadContext *tc);
        X86ISA::IntReg getSyscallArg(ThreadContext *tc, int &i);
        X86ISA::IntReg getSyscallArg(ThreadContext *tc, int &i, int width);
        void setSyscallArg(ThreadContext *tc, int i, X86ISA::IntReg val);
    };

    /**
     * Declaration of architectural page table for x86.
     *
     * These page tables are stored in system memory and respect x86 specification.
     */
    typedef MultiLevelPageTable<PageTableOps> ArchPageTable;

}

#endif // __ARCH_X86_PROCESS_HH__
