/*
 * Copyright (c) 2016 ARM Limited
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
 *
 * Authors: Rekai Gonzalez
 */

#ifndef __CPU__REG_CLASS_IMPL_HH__
#define __CPU__REG_CLASS_IMPL_HH__

#include <cassert>
#include <cstddef>
#include <iostream>

#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "cpu/reg_class.hh"

bool RegId::isZeroReg() const
{
    return ((regClass == IntRegClass && regIdx == TheISA::ZeroReg) ||
            (THE_ISA == ALPHA_ISA && regClass == FloatRegClass &&
             regIdx == TheISA::ZeroReg));
}

static constexpr size_t Scale = TheISA::NumVecElemPerVecReg;

RegIndex RegId::flatIndex() const {
    switch (regClass) {
    case IntRegClass:
    case FloatRegClass:
    case VecRegClass:
    case CCRegClass:
    case MiscRegClass:
        return regIdx;
    case VecElemClass:
        return Scale*regIdx + elemIdx;
    }
    panic("Trying to flatten a register without class!");
    return -1;
}

#endif // __CPU__REG_CLASS_IMPL_HH__
