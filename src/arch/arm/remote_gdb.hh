/*
 * Copyright 2015 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2013, 2016, 2018-2019 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARCH_ARM_REMOTE_GDB_HH__
#define __ARCH_ARM_REMOTE_GDB_HH__

#include <algorithm>

#include "arch/arm/registers.hh"
#include "arch/arm/utility.hh"
#include "base/compiler.hh"
#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace ArmISA
{

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    bool acc(Addr addr, size_t len);

    class AArch32GdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct {
          uint32_t gpr[16];
          uint32_t cpsr;
          uint64_t fpr[32];
          uint32_t fpscr;
        } M5_ATTR_PACKED r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string
        name() const
        {
            return gdb->name() + ".AArch32GdbRegCache";
        }
    };

    class AArch64GdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct {
          uint64_t x[31];
          uint64_t spx;
          uint64_t pc;
          uint32_t cpsr;
          VecElem v[NumVecV8ArchRegs * NumVecElemPerNeonVecReg];
          uint32_t fpsr;
          uint32_t fpcr;
        } M5_ATTR_PACKED r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string
        name() const
        {
            return gdb->name() + ".AArch64GdbRegCache";
        }
    };

    AArch32GdbRegCache regCache32;
    AArch64GdbRegCache regCache64;

  public:
    RemoteGDB(System *_system, ThreadContext *tc, int _port);
    BaseGdbRegCache *gdbRegs();
    std::vector<std::string>
    availableFeatures() const
    {
        return {"qXfer:features:read+"};
    };
    bool getXferFeaturesRead(const std::string &annex, std::string &output);
};
} // namespace ArmISA

#endif /* __ARCH_ARM_REMOTE_GDB_H__ */
