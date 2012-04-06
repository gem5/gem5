/*
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
 *
 * Authors: Nathan Binkert
 */

#ifndef __ARCH_MIPS_REMOTE_GDB_HH__
#define __ARCH_MIPS_REMOTE_GDB_HH__

#include "arch/mips/registers.hh"
#include "base/bitfield.hh"
#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace MipsISA
{

    // The number of special regs depends on gdb.
    // Two 32-bit regs are packed into one 64-bit reg.
    const int GdbIntArchRegs = NumIntArchRegs / 2;
    const int GdbIntSpecialRegs = 6 / 2;
    const int GdbFloatArchRegs = NumFloatArchRegs / 2;
    const int GdbFloatSpecialRegs = 2 / 2;

    const int GdbIntRegs = GdbIntArchRegs + GdbIntSpecialRegs;
    const int GdbFloatRegs = GdbFloatArchRegs + GdbFloatSpecialRegs;
    const int GdbNumRegs = GdbIntRegs + GdbFloatRegs;

    class RemoteGDB : public BaseRemoteGDB
    {
      protected:
        Addr notTakenBkpt;
        Addr takenBkpt;

      public:
        RemoteGDB(System *_system, ThreadContext *tc);

      protected:
        bool acc(Addr addr, size_t len);

        void getregs();
        void setregs();

        void clearSingleStep();
        void setSingleStep();

      private:
        uint64_t
        pack(uint32_t lo, uint32_t hi)
        {
            return static_cast<uint64_t>(hi) << 32 | lo;
        }
        uint32_t
        unpackLo(uint64_t val)
        {
            return bits(val, 31, 0);
        }
        uint32_t
        unpackHi(uint64_t val)
        {
            return bits(val, 63, 32);
        }
    };
}

#endif /* __ARCH_MIPS_REMOTE_GDB_H__ */
