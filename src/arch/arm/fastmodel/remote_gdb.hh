/*
 * Copyright 2022 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_REMOTE_GDB_HH__
#define __ARCH_ARM_FASTMODEL_REMOTE_GDB_HH__

#include "arch/arm/remote_gdb.hh"

namespace gem5
{

namespace fastmodel
{

class FastmodelRemoteGDB : public ArmISA::RemoteGDB
{
  public:
    FastmodelRemoteGDB(System *_system, ListenSocketConfig _listen_config);

  protected:
    class AArch64GdbRegCache : public ArmISA::RemoteGDB::AArch64GdbRegCache
    {
        using ArmISA::RemoteGDB::AArch64GdbRegCache::AArch64GdbRegCache;

      public:
        void setRegs(ThreadContext *) const override;
    };

    bool readBlob(Addr vaddr, size_t size, char *data) override;
    bool writeBlob(Addr vaddr, size_t size, const char *data) override;
    BaseGdbRegCache *gdbRegs() override;

    AArch64GdbRegCache regCache64;
};

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_FASTMODEL_REMOTE_GDB_HH__
