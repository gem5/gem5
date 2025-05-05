/*
 * Copyright 2015-2020 LabWare
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2007 The Regents of The University of Michigan
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

#ifndef __ARCH_MIPS_REMOTE_GDB_HH__
#define __ARCH_MIPS_REMOTE_GDB_HH__

#include "base/bitfield.hh"
#include "base/remote_gdb.hh"

namespace gem5
{

class System;
class ThreadContext;

namespace MipsISA
{

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    bool acc(Addr addr, size_t len);

    class MipsGdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        struct
        {
            uint32_t gpr[32];
            uint32_t sr;
            uint32_t lo;
            uint32_t hi;
            uint32_t badvaddr;
            uint32_t cause;
            uint32_t pc;
            uint32_t fpr[32];
            uint32_t fsr;
            uint32_t fir;
        } r;
      public:
        char *data() { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;
        const std::string
        name() const
        {
            return gdb->name() + ".MipsGdbRegCache";
        }
    };

    MipsGdbRegCache regCache;

  public:
    RemoteGDB(System *_system, ListenSocketConfig _listen_config);
    BaseGdbRegCache *gdbRegs();
    std::vector<std::string>
    availableFeatures() const
    {
        return {"qXfer:features:read+"};
    };
    bool getXferFeaturesRead(const std::string &annex, std::string &output);
};

} // namespace MipsISA
} // namespace gem5

#endif /* __ARCH_MIPS_REMOTE_GDB_H__ */
