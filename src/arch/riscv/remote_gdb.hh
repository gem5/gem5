/*
 * Copyright (c) 2021 Huawei International
 * Copyright (c) 2017 The University of Virginia
 * Copyright 2015 LabWare
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

#ifndef __ARCH_RISCV_REMOTE_GDB_HH__
#define __ARCH_RISCV_REMOTE_GDB_HH__

#include <string>

#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "base/remote_gdb.hh"

namespace gem5
{

class System;
class ThreadContext;

namespace RiscvISA
{

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    static const int NumGDBRegs = 4162;
    static const int NumCSRs = 4096;

    bool acc(Addr addr, size_t len) override;
    // A breakpoint will be 2 bytes if it is compressed and 4 if not
    bool checkBpKind(size_t kind) override { return kind == 2 || kind == 4; }

    class RiscvGdbRegCache : public BaseGdbRegCache
    {
      using BaseGdbRegCache::BaseGdbRegCache;
      private:
        /**
         * RISC-V Register Cache
         * Order and sizes of registers found in ext/gdb-xml/riscv.xml
         * To add support for more CSRs:
         * 1. Uncomment relevant lines in ext/gdb-xml/riscv-64bit-csr.xml
         * 2. Add register to struct below
         * 3. Modify RiscvGdbRegCache::getRegs and setRegs
         */
        struct
        {
            uint64_t gpr[NumIntArchRegs];
            uint64_t pc;
            uint64_t fpu[NumFloatRegs];
            uint32_t fflags;
            uint32_t frm;
            uint32_t fcsr;
            // Placeholder for byte alignment
            uint32_t placeholder;
            uint64_t cycle;
            uint64_t time;
            uint64_t ustatus;
            uint64_t uie;
            uint64_t utvec;
            uint64_t uscratch;
            uint64_t uepc;
            uint64_t ucause;
            uint64_t utval;
            uint64_t uip;
            uint64_t sstatus;
            uint64_t sedeleg;
            uint64_t sideleg;
            uint64_t sie;
            uint64_t stvec;
            uint64_t scounteren;
            uint64_t sscratch;
            uint64_t sepc;
            uint64_t scause;
            uint64_t stval;
            uint64_t sip;
            uint64_t satp;
            uint64_t mvendorid;
            uint64_t marchid;
            uint64_t mimpid;
            uint64_t mhartid;
            uint64_t mstatus;
            uint64_t misa;
            uint64_t medeleg;
            uint64_t mideleg;
            uint64_t mie;
            uint64_t mtvec;
            uint64_t mcounteren;
            uint64_t mscratch;
            uint64_t mepc;
            uint64_t mcause;
            uint64_t mtval;
            uint64_t mip;
            uint64_t hstatus;
            uint64_t hedeleg;
            uint64_t hideleg;
            uint64_t hie;
            uint64_t htvec;
            uint64_t hscratch;
            uint64_t hepc;
            uint64_t hcause;
            uint64_t hbadaddr;
            uint64_t hip;
        } r;
      public:
        char *data() const { return (char *)&r; }
        size_t size() const { return sizeof(r); }
        void getRegs(ThreadContext*);
        void setRegs(ThreadContext*) const;

        const std::string
        name() const
        {
            return gdb->name() + ".RiscvGdbRegCache";
        }
    };

    RiscvGdbRegCache regCache;

  public:
    RemoteGDB(System *_system, int _port);
    BaseGdbRegCache *gdbRegs() override;
    /**
     * Informs GDB remote serial protocol that XML features are supported
     * GDB then queries for xml blobs using qXfer:features:read:xxx.xml
     */
    std::vector<std::string>
    availableFeatures() const override
    {
        return {"qXfer:features:read+"};
    };
    /**
     * Reply to qXfer:features:read:xxx.xml qeuries
     */
    bool getXferFeaturesRead(const std::string &annex,
                             std::string &output) override;
};

} // namespace RiscvISA
} // namespace gem5

#endif /* __ARCH_RISCV_REMOTE_GDB_H__ */
