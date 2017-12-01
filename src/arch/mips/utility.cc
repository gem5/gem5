/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 */

#include "arch/mips/utility.hh"

#include <cmath>

#include "arch/mips/isa_traits.hh"
#include "arch/mips/registers.hh"
#include "arch/mips/vtophys.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/serialize.hh"

using namespace MipsISA;
using namespace std;

namespace MipsISA {

uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    panic("getArgument() not implemented\n");
    M5_DUMMY_RETURN
}

uint64_t
fpConvert(ConvertType cvt_type, double fp_val)
{

    switch (cvt_type)
    {
      case SINGLE_TO_DOUBLE:
        {
            double sdouble_val = fp_val;
            void  *sdouble_ptr = &sdouble_val;
            uint64_t sdp_bits  = *(uint64_t *) sdouble_ptr;
            return sdp_bits;
        }

      case SINGLE_TO_WORD:
        {
            int32_t sword_val  = (int32_t) fp_val;
            void  *sword_ptr   = &sword_val;
            uint64_t sword_bits= *(uint32_t *) sword_ptr;
            return sword_bits;
        }

      case WORD_TO_SINGLE:
        {
            float wfloat_val   = fp_val;
            void  *wfloat_ptr  = &wfloat_val;
            uint64_t wfloat_bits = *(uint32_t *) wfloat_ptr;
            return wfloat_bits;
        }

      case WORD_TO_DOUBLE:
        {
            double wdouble_val = fp_val;
            void  *wdouble_ptr = &wdouble_val;
            uint64_t wdp_bits  = *(uint64_t *) wdouble_ptr;
            return wdp_bits;
        }

      default:
        panic("Invalid Floating Point Conversion Type (%d). See \"types.hh\" for List of Conversions\n",cvt_type);
        return 0;
    }
}

double
roundFP(double val, int digits)
{
    double digit_offset = pow(10.0,digits);
    val = val * digit_offset;
    val = val + 0.5;
    val = floor(val);
    val = val / digit_offset;
    return val;
}

double
truncFP(double val)
{
    int trunc_val = (int) val;
    return (double) trunc_val;
}

bool
getCondCode(uint32_t fcsr, int cc_idx)
{
    int shift = (cc_idx == 0) ? 23 : cc_idx + 24;
    bool cc_val = (fcsr >> shift) & 0x00000001;
    return cc_val;
}

uint32_t
genCCVector(uint32_t fcsr, int cc_num, uint32_t cc_val)
{
    int cc_idx = (cc_num == 0) ? 23 : cc_num + 24;

    fcsr = bits(fcsr, 31, cc_idx + 1) << (cc_idx + 1) |
           cc_val << cc_idx |
           bits(fcsr, cc_idx - 1, 0);

    return fcsr;
}

uint32_t
genInvalidVector(uint32_t fcsr_bits)
{
    //Set FCSR invalid in "flag" field
    int invalid_offset = Invalid + Flag_Field;
    fcsr_bits = fcsr_bits | (1 << invalid_offset);

    //Set FCSR invalid in "cause" flag
    int cause_offset = Invalid + Cause_Field;
    fcsr_bits = fcsr_bits | (1 << cause_offset);

    return fcsr_bits;
}

bool
isNan(void *val_ptr, int size)
{
    switch (size)
    {
      case 32:
        {
            uint32_t val_bits = *(uint32_t *) val_ptr;
            return (bits(val_bits, 30, 23) == 0xFF);
        }

      case 64:
        {
            uint64_t val_bits = *(uint64_t *) val_ptr;
            return (bits(val_bits, 62, 52) == 0x7FF);
        }

      default:
        panic("Type unsupported. Size mismatch\n");
    }
}


bool
isQnan(void *val_ptr, int size)
{
    switch (size)
    {
      case 32:
        {
            uint32_t val_bits = *(uint32_t *) val_ptr;
            return (bits(val_bits, 30, 22) == 0x1FE);
        }

      case 64:
        {
            uint64_t val_bits = *(uint64_t *) val_ptr;
            return (bits(val_bits, 62, 51) == 0xFFE);
        }

      default:
        panic("Type unsupported. Size mismatch\n");
    }
}

bool
isSnan(void *val_ptr, int size)
{
    switch (size)
    {
      case 32:
        {
            uint32_t val_bits = *(uint32_t *) val_ptr;
            return (bits(val_bits, 30, 22) == 0x1FF);
        }

      case 64:
        {
            uint64_t val_bits = *(uint64_t *) val_ptr;
            return (bits(val_bits, 62, 51) == 0xFFF);
        }

      default:
        panic("Type unsupported. Size mismatch\n");
    }
}

template <class CPU>
void
zeroRegisters(CPU *cpu)
{
    // Insure ISA semantics
    // (no longer very clean due to the change in setIntReg() in the
    // cpu model.  Consider changing later.)
    cpu->thread->setIntReg(ZeroReg, 0);
    cpu->thread->setFloatReg(ZeroReg, 0.0);
}

void
startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate();
}

void
initCPU(ThreadContext *tc, int cpuId)
{}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    // First loop through the integer registers.
    for (int i = 0; i < NumIntRegs; i++)
        dest->setIntRegFlat(i, src->readIntRegFlat(i));

    // Then loop through the floating point registers.
    for (int i = 0; i < NumFloatRegs; i++)
        dest->setFloatRegFlat(i, src->readFloatRegFlat(i));

    // Would need to add condition-code regs if implemented
    assert(NumCCRegs == 0);

    // Copy misc. registers
    for (int i = 0; i < NumMiscRegs; i++)
        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    // Copy over the PC State
    dest->pcState(src->pcState());
}

void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
}
void
skipFunction(ThreadContext *tc)
{
    TheISA::PCState newPC = tc->pcState();
    newPC.set(tc->readIntReg(ReturnAddressReg));
    tc->pcState(newPC);
}


} // namespace MipsISA
