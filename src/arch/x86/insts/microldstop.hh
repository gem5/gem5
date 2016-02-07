/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_X86_INSTS_MICROLDSTOP_HH__
#define __ARCH_X86_INSTS_MICROLDSTOP_HH__

#include "arch/x86/insts/microop.hh"
#include "arch/x86/ldstflags.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/faults.hh"

namespace X86ISA
{
    /**
     * Base class for memory ops
     */
    class MemOp : public X86MicroopBase
    {
      protected:
        const uint8_t scale;
        const RegIndex index;
        const RegIndex base;
        const uint64_t disp;
        const uint8_t segment;
        const uint8_t dataSize;
        const uint8_t addressSize;
        const Request::FlagsType memFlags;
        RegIndex foldOBit, foldABit;

        //Constructor
        MemOp(ExtMachInst _machInst,
                const char * mnem, const char * _instMnem,
                uint64_t setFlags,
                uint8_t _scale, InstRegIndex _index, InstRegIndex _base,
                uint64_t _disp, InstRegIndex _segment,
                uint8_t _dataSize, uint8_t _addressSize,
                Request::FlagsType _memFlags,
                OpClass __opClass) :
        X86MicroopBase(_machInst, mnem, _instMnem, setFlags, __opClass),
                scale(_scale), index(_index.idx), base(_base.idx),
                disp(_disp), segment(_segment.idx),
                dataSize(_dataSize), addressSize(_addressSize),
                memFlags(_memFlags | _segment.idx)
        {
            assert(_segment.idx < NUM_SEGMENTREGS);
            foldOBit =
                (dataSize == 1 && !_machInst.rex.present) ? 1 << 6 : 0;
            foldABit =
                (addressSize == 1 && !_machInst.rex.present) ? 1 << 6 : 0;
        }
    };

    /**
     * Base class for load and store ops using one register
     */
    class LdStOp : public MemOp
    {
      protected:
        const RegIndex data;

        //Constructor
        LdStOp(ExtMachInst _machInst,
                const char * mnem, const char * _instMnem,
                uint64_t setFlags,
                uint8_t _scale, InstRegIndex _index, InstRegIndex _base,
                uint64_t _disp, InstRegIndex _segment,
                InstRegIndex _data,
                uint8_t _dataSize, uint8_t _addressSize,
                Request::FlagsType _memFlags,
                OpClass __opClass) :
        MemOp(_machInst, mnem, _instMnem, setFlags,
                _scale, _index, _base, _disp, _segment,
                _dataSize, _addressSize, _memFlags,
                __opClass),
                data(_data.idx)
        {
        }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;
    };

    /**
     * Base class for load and store ops using two registers, we will
     * call them split ops for this reason. These are mainly  used to
     * implement cmpxchg8b and cmpxchg16b.
     */
    class LdStSplitOp : public MemOp
    {
      protected:
        const RegIndex dataLow;
        const RegIndex dataHi;

        //Constructor
        LdStSplitOp(ExtMachInst _machInst,
                const char * mnem, const char * _instMnem,
                uint64_t setFlags,
                uint8_t _scale, InstRegIndex _index, InstRegIndex _base,
                uint64_t _disp, InstRegIndex _segment,
                InstRegIndex _dataLow, InstRegIndex _dataHi,
                uint8_t _dataSize, uint8_t _addressSize,
                Request::FlagsType _memFlags,
                OpClass __opClass) :
        MemOp(_machInst, mnem, _instMnem, setFlags,
                _scale, _index, _base, _disp, _segment,
                _dataSize, _addressSize, _memFlags,
                __opClass),
                dataLow(_dataLow.idx),
                dataHi(_dataHi.idx)
        {
        }

        std::string generateDisassembly(Addr pc,
            const SymbolTable *symtab) const;
    };
}

#endif //__ARCH_X86_INSTS_MICROLDSTOP_HH__
