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

#ifndef __ARCH_X86_INSTS_MICROFPOP_HH__
#define __ARCH_X86_INSTS_MICROFPOP_HH__

#include "arch/x86/insts/microop.hh"

namespace X86ISA
{

    /**
     * Base classes for FpOps which provides a generateDisassembly method.
     */
    class FpOp : public X86MicroopBase
    {
      protected:
        const RegIndex src1;
        const RegIndex src2;
        const RegIndex dest;
        const uint8_t dataSize;
        const int8_t spm;

        // Constructor
        FpOp(ExtMachInst _machInst,
                const char *mnem, const char *_instMnem,
                uint64_t setFlags,
                InstRegIndex _src1, InstRegIndex _src2, InstRegIndex _dest,
                uint8_t _dataSize, int8_t _spm,
                OpClass __opClass) :
            X86MicroopBase(_machInst, mnem, _instMnem, setFlags,
                    __opClass),
            src1(_src1.idx), src2(_src2.idx), dest(_dest.idx),
            dataSize(_dataSize), spm(_spm)
        {}
/*
        //Figure out what the condition code flags should be.
        uint64_t genFlags(uint64_t oldFlags, uint64_t flagMask,
                uint64_t _dest, uint64_t _src1, uint64_t _src2,
                bool subtract = false) const;
        bool checkCondition(uint64_t flags) const;*/

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;
    };
}

#endif //__ARCH_X86_INSTS_MICROFPOP_HH__
