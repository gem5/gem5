/*
 * Copyright (c) 2010 ARM Limited
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

#ifndef __ARCH_ARM_INSTS_MULT_HH__
#define __ARCH_ARM_INSTS_MULT_HH__

#include "arch/arm/insts/static_inst.hh"
#include "base/trace.hh"

namespace gem5
{

namespace ArmISA
{

/**
 * Base class for multipy instructions using three registers.
 */
class Mult3 : public PredOp
{
  protected:
    RegIndex reg0, reg1, reg2;

    Mult3(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
          RegIndex _reg0, RegIndex _reg1, RegIndex _reg2)
        : PredOp(mnem, _machInst, __opClass),
          reg0(_reg0),
          reg1(_reg1),
          reg2(_reg2)
    {}
};

/**
 * Base class for multipy instructions using four registers.
 */
class Mult4 : public Mult3
{
  protected:
    RegIndex reg3;

    Mult4(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
          RegIndex _reg0, RegIndex _reg1, RegIndex _reg2, RegIndex _reg3)
        : Mult3(mnem, _machInst, __opClass, _reg0, _reg1, _reg2), reg3(_reg3)
    {}
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_INSTS_MULT_HH__
