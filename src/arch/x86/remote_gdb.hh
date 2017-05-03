/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
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
 *          Boris Shingarov
 */

#ifndef __ARCH_X86_REMOTEGDB_HH__
#define __ARCH_X86_REMOTEGDB_HH__

#include <algorithm>

#include "arch/x86/types.hh"
#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace X86ISA
{
class RemoteGDB : public BaseRemoteGDB
{
  protected:
    bool acc(Addr addr, size_t len);
    bool checkBpLen(size_t len) { return len == 1; }
    class X86GdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct {
          uint32_t eax;
          uint32_t ecx;
          uint32_t edx;
          uint32_t ebx;
          uint32_t esp;
          uint32_t ebp;
          uint32_t esi;
          uint32_t edi;
          uint32_t eip;
          uint32_t eflags;
          uint32_t cs;
          uint32_t ss;
          uint32_t ds;
          uint32_t es;
          uint32_t fs;
          uint32_t gs;
        } r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string
        name() const
        {
            return gdb->name() + ".X86GdbRegCache";
        }
    };

    class AMD64GdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct M5_ATTR_PACKED {
          uint64_t rax;
          uint64_t rbx;
          uint64_t rcx;
          uint64_t rdx;
          uint64_t rsi;
          uint64_t rdi;
          uint64_t rbp;
          uint64_t rsp;
          uint64_t r8;
          uint64_t r9;
          uint64_t r10;
          uint64_t r11;
          uint64_t r12;
          uint64_t r13;
          uint64_t r14;
          uint64_t r15;
          uint64_t rip;
          uint32_t eflags;
          uint32_t cs;
          uint32_t ss;
          uint32_t ds;
          uint32_t es;
          uint32_t fs;
          uint32_t gs;
          /*
           * We do not model st[], FPU status regs, xmm[] etc.
           * While it's not ok to have G-packets larger than what gdb
           * knows about, it is ok to have smaller ones.
           */
        } r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string
        name() const
        {
            return gdb->name() + ".AMD64GdbRegCache";
        }
    };

    X86GdbRegCache regCache32;
    AMD64GdbRegCache regCache64;

  public:
    RemoteGDB(System *system, ThreadContext *context);
    BaseGdbRegCache *gdbRegs();
};
} // namespace X86ISA

#endif // __ARCH_X86_REMOTEGDB_HH__
