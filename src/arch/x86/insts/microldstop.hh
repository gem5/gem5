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
 */

#ifndef __ARCH_X86_INSTS_MICROLDSTOP_HH__
#define __ARCH_X86_INSTS_MICROLDSTOP_HH__

#include <tuple>

#include "arch/x86/insts/microop.hh"
#include "arch/x86/insts/microop_args.hh"
#include "arch/x86/ldstflags.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/faults.hh"

namespace gem5
{

namespace X86ISA
{

/**
 * Base class for memory ops
 */
class MemOp : public X86MicroopBase
{
  protected:
    const Request::FlagsType memFlags;

    //Constructor
    MemOp(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, OpClass op_class,
            uint8_t data_size, uint8_t address_size,
            Request::FlagsType mem_flags) :
    X86MicroopBase(mach_inst, mnem, inst_mnem, set_flags, op_class),
            memFlags(mem_flags),
            dataSize(data_size), addressSize(address_size),
            foldOBit((dataSize == 1 && !mach_inst.rex.present) ? 1 << 6 : 0),
            foldABit((addressSize == 1 && !mach_inst.rex.present) ? 1 << 6 : 0)
    {}

  public:
    const uint8_t dataSize;
    const uint8_t addressSize;
    const RegIndex foldOBit, foldABit;
};

/**
 * Base class for load ops using one integer register.
 */
class LdStOp : public InstOperands<MemOp, FoldedDataOp, AddrOp>
{
  protected:
    LdStOp(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, GpRegIndex _data,
            uint8_t _scale, GpRegIndex _index, GpRegIndex _base,
            uint64_t _disp, SegRegIndex _segment,
            uint8_t data_size, uint8_t address_size,
            Request::FlagsType mem_flags, OpClass op_class) :
    InstOperands<MemOp, FoldedDataOp, AddrOp>(
            mach_inst, mnem, inst_mnem, set_flags, op_class,
            { _data, { _scale, _index, _base, _disp, _segment } },
            data_size, address_size, mem_flags | _segment.index)
    {}
};

/**
 * Base class for load ops using one FP register.
 */
class LdStFpOp : public InstOperands<MemOp, FloatDataOp, AddrOp>
{
  protected:
    LdStFpOp(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, FpRegIndex _data,
            uint8_t _scale, GpRegIndex _index, GpRegIndex _base,
            uint64_t _disp, SegRegIndex _segment,
            uint8_t data_size, uint8_t address_size,
            Request::FlagsType mem_flags, OpClass op_class) :
    InstOperands<MemOp, FloatDataOp, AddrOp>(
            mach_inst, mnem, inst_mnem, set_flags, op_class,
            { _data, { _scale, _index, _base, _disp, _segment } },
            data_size, address_size, mem_flags | _segment.index)
    {}
};

/**
 * Base class for the tia microop which has no destination register.
 */
class MemNoDataOp : public InstOperands<MemOp, AddrOp>
{
  protected:
    MemNoDataOp(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, uint8_t _scale, GpRegIndex _index,
            GpRegIndex _base, uint64_t _disp, SegRegIndex _segment,
            uint8_t data_size, uint8_t address_size,
            Request::FlagsType mem_flags, OpClass op_class) :
    InstOperands<MemOp, AddrOp>(
            mach_inst, mnem, inst_mnem, set_flags, op_class,
            { { _scale, _index, _base, _disp, _segment } },
            data_size, address_size, mem_flags | _segment.index)
    {}
};

/**
 * Base class for load and store ops using two registers, we will
 * call them split ops for this reason. These are mainly  used to
 * implement cmpxchg8b and cmpxchg16b.
 */
class LdStSplitOp :
    public InstOperands<MemOp, FoldedDataLowOp, FoldedDataHiOp, AddrOp>
{
  protected:
    LdStSplitOp(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, GpRegIndex data_low, GpRegIndex data_hi,
            uint8_t _scale, GpRegIndex _index, GpRegIndex _base,
            uint64_t _disp, SegRegIndex _segment,
            uint8_t data_size, uint8_t address_size,
            Request::FlagsType mem_flags, OpClass op_class) :
    InstOperands<MemOp, FoldedDataLowOp, FoldedDataHiOp, AddrOp>(
            mach_inst, mnem, inst_mnem, set_flags, op_class,
            { data_low, data_hi, { _scale, _index, _base, _disp, _segment } },
            data_size, address_size, mem_flags | _segment.index)
    {}
};

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_INSTS_MICROLDSTOP_HH__
