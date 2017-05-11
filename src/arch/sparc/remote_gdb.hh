/*
 * Copyright (c) 2015 LabWare
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *          Boris Shingarov
 */

#ifndef __ARCH_SPARC_REMOTE_GDB_HH__
#define __ARCH_SPARC_REMOTE_GDB_HH__

#include <map>

#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace SparcISA
{

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    bool acc(Addr addr, size_t len);

    class SPARCGdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct {
            uint32_t gpr[32];
            uint32_t hole[32];
            uint32_t y;
            uint32_t psr;
            uint32_t wim;
            uint32_t tbr;
            uint32_t pc;
            uint32_t npc;
            uint32_t fsr;
            uint32_t csr;
        } r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string name() const { return gdb->name() + ".SPARCGdbRegCache"; }
    };

    class SPARC64GdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct {
            uint64_t gpr[32];
            uint64_t fpr[32];
            uint64_t pc;
            uint64_t npc;
            uint64_t state;
            uint64_t fsr;
            uint64_t fprs;
            uint64_t y;
        } r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string name() const { return gdb->name() + ".SPARC64GdbRegCache"; }
    };

    SPARCGdbRegCache regCache32;
    SPARC64GdbRegCache regCache64;

  public:
    RemoteGDB(System *_system, ThreadContext *tc);
    BaseGdbRegCache *gdbRegs();
};
} // namespace SparcISA

#endif /* __ARCH_SPARC_REMOTE_GDB_H__ */
