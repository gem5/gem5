/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Korey L. Sewell
 */

#include "arch/mips/isa_traits.hh"
#include "arch/mips/utility.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#include "cpu/static_inst.hh"
#include "sim/serialize.hh"
#include "base/bitfield.hh"
#include "base/misc.hh"

#if FULL_SYSTEM
#include "arch/mips/vtophys.hh"
#include "mem/vport.hh"
#endif


using namespace MipsISA;
using namespace std;

namespace MipsISA {

uint64_t
getArgument(ThreadContext *tc, int number, bool fp)
{
#if FULL_SYSTEM
    if (number < NumArgumentRegs) {
        if (fp)
            return tc->readFloatRegBits(ArgumentReg[number]);
        else
            return tc->readIntReg(ArgumentReg[number]);
    } else {
        Addr sp = tc->readIntReg(StackPointerReg);
        VirtualPort *vp = tc->getVirtPort(tc);
        uint64_t arg = vp->read<uint64_t>(sp +
                           (number-NumArgumentRegs) * sizeof(uint64_t));
        tc->delVirtPort(vp);
        return arg;
    }
#else
    panic("getArgument() is Full system only\n");
    M5_DUMMY_RETURN
#endif
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

    fcsr = bits(fcsr, 31, cc_idx + 1) << cc_idx + 1 |
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

int
flattenIntIndex(ThreadContext * tc, int reg)
{
    return reg;
}


void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Regs Not Implemented Yet\n");
}

void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
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
    tc->activate(0/*tc->getThreadNum()*/);
}

} // namespace MipsISA
