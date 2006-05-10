/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/mips/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/static_inst.hh"
#include "sim/serialize.hh"
#include "base/bitfield.hh"

using namespace MipsISA;
using namespace std;


void
MipsISA::copyRegs(ExecContext *src, ExecContext *dest)
{
    /*fpcr = xc->readMiscReg(MipsISA::Fpcr_DepTag);
    uniq = xc->readMiscReg(MipsISA::Uniq_DepTag);
    lock_flag = xc->readMiscReg(MipsISA::Lock_Flag_DepTag);
    lock_addr = xc->readMiscReg(MipsISA::Lock_Addr_DepTag);

#if FULL_SYSTEM
    copyIprs(xc);
    #endif*/
}

void
MipsISA::MiscRegFile::copyMiscRegs(ExecContext *xc)
{
    /*fpcr = xc->readMiscReg(MipsISA::Fpcr_DepTag);
    uniq = xc->readMiscReg(MipsISA::Uniq_DepTag);
    lock_flag = xc->readMiscReg(MipsISA::Lock_Flag_DepTag);
    lock_addr = xc->readMiscReg(MipsISA::Lock_Addr_DepTag);

    #endif*/
}

uint64_t
MipsISA::convert_and_round(uint32_t fp_val, ConvertType cvt_type, int rnd_mode)
{

    uint64_t ret_val = 0;

    switch (cvt_type)
    {
      case SINGLE_TO_DOUBLE:
        uint64_t single_sign     = fp_val & 0x80000000;

        uint64_t single_exp      = (fp_val & 0x7F800000) >> 22;
        single_exp -= 127;

        uint64_t single_mantissa = fp_val & 0x007FFFFF;

        uint64_t double_exp = single_exp + 1023;
        double_exp = double_exp << 51;

        uint64_t double_val =  single_sign << 63 | double_exp | single_mantissa;

        return double_val;

      default:
        panic("Invalid Floating Point Conversion Type (%d) being used.\n",cvt_type);
        return ret_val;
    }
}

uint64_t
MipsISA::convert_and_round(uint64_t fp_val, ConvertType cvt_type, int rnd_mode)
{

    uint64_t ret_val = 0;

    switch (cvt_type)
    {
      case SINGLE_TO_DOUBLE:
        uint64_t single_sign     = fp_val & 0x80000000;

        uint64_t single_exp      = (fp_val & 0x7F800000) >> 22;
        single_exp -= 127;

        uint64_t single_mantissa = fp_val & 0x007FFFFF;

        uint64_t double_exp = single_exp + 1023;
        double_exp = double_exp << 51;

        uint64_t double_val =  single_sign << 63 | double_exp | single_mantissa;

        return double_val;

      default:
        panic("Invalid Floating Point Conversion Type (%d) being used.\n",cvt_type);
        return ret_val;
    }
}


uint64_t
MipsISA::convert_and_round(double fp_val, ConvertType cvt_type, int rnd_mode)
{

    switch (cvt_type)
    {
      case SINGLE_TO_DOUBLE:
        double sdouble_val = fp_val;
        void  *sdouble_ptr = &sdouble_val;
        uint64_t sdp_bits  = *(uint64_t *) sdouble_ptr ;
        return sdp_bits;

      case SINGLE_TO_WORD:
        int32_t sword_val  = (int32_t) fp_val;
        void  *sword_ptr   = &sword_val;
        uint64_t sword_bits= *(uint32_t *) sword_ptr ;
        return sword_bits;

      case WORD_TO_SINGLE:
        float wfloat_val  = fp_val;
        void  *wfloat_ptr = &wfloat_val;
        uint64_t wfloat_bits = *(uint32_t *) wfloat_ptr ;
        return wfloat_bits;

      case WORD_TO_DOUBLE:
        double wdouble_val = fp_val;
        void  *wdouble_ptr = &wdouble_val;
        uint64_t wdp_bits  = *(uint64_t *) wdouble_ptr ;
        return wdp_bits;

      default:
        panic("Invalid Floating Point Conversion Type (%d). See types.hh for Conversion List\n",cvt_type);
        return 0;
    }
}

uint64_t
MipsISA::fpConvert(double fp_val, ConvertType cvt_type)
{

    switch (cvt_type)
    {
      case SINGLE_TO_DOUBLE:
        double sdouble_val = fp_val;
        void  *sdouble_ptr = &sdouble_val;
        uint64_t sdp_bits  = *(uint64_t *) sdouble_ptr;
        return sdp_bits;

      case SINGLE_TO_WORD:
        int32_t sword_val  = (int32_t) fp_val;
        void  *sword_ptr   = &sword_val;
        uint64_t sword_bits= *(uint32_t *) sword_ptr;
        return sword_bits;

      case WORD_TO_SINGLE:
        float wfloat_val   = fp_val;
        void  *wfloat_ptr  = &wfloat_val;
        uint64_t wfloat_bits = *(uint32_t *) wfloat_ptr;
        return wfloat_bits;

      case WORD_TO_DOUBLE:
        double wdouble_val = fp_val;
        void  *wdouble_ptr = &wdouble_val;
        uint64_t wdp_bits  = *(uint64_t *) wdouble_ptr;
        return wdp_bits;

      default:
        panic("Invalid Floating Point Conversion Type (%d). See \"types.hh\" for List of Conversions\n",cvt_type);
        return 0;
    }
}

double
MipsISA::roundFP(double val)
{
    double trunc_val = trunc(val);
    double fraction = val - trunc_val;

    if (fraction < 0.5)
        return val;
    else
        return val + 1;
}

inline double
MipsISA::truncFP(double val)
{
    int trunc_val = (int) val;
    return (double) trunc_val;
}

bool
MipsISA::unorderedFP(uint32_t val)
{
}

bool
MipsISA::unorderedFP(uint64_t val)
{
}

bool
MipsISA::getConditionCode(int cc)
{
}

void
MipsISA::setConditionCode(int num, bool val)
{
}

#if FULL_SYSTEM

static inline Addr
TruncPage(Addr addr)
{ return addr & ~(MipsISA::PageBytes - 1); }

static inline Addr
RoundPage(Addr addr)
{ return (addr + MipsISA::PageBytes - 1) & ~(MipsISA::PageBytes - 1); }
#endif

void
IntRegFile::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(regs, NumIntRegs);
}

void
IntRegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(regs, NumIntRegs);
}

void
RegFile::serialize(std::ostream &os)
{
    intRegFile.serialize(os);
    //SERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    //SERIALIZE_SCALAR(miscRegs.fpcr);
    //SERIALIZE_SCALAR(miscRegs.uniq);
    //SERIALIZE_SCALAR(miscRegs.lock_flag);
    //SERIALIZE_SCALAR(miscRegs.lock_addr);
    SERIALIZE_SCALAR(pc);
    SERIALIZE_SCALAR(npc);
    SERIALIZE_SCALAR(nnpc);
#if FULL_SYSTEM
    SERIALIZE_ARRAY(palregs, NumIntRegs);
    SERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    SERIALIZE_SCALAR(intrflag);
    SERIALIZE_SCALAR(pal_shadow);
#endif
}


void
RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    intRegFile.unserialize(cp, section);
    //UNSERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    //UNSERIALIZE_SCALAR(miscRegs.fpcr);
    //UNSERIALIZE_SCALAR(miscRegs.uniq);
    //UNSERIALIZE_SCALAR(miscRegs.lock_flag);
    //UNSERIALIZE_SCALAR(miscRegs.lock_addr);
    UNSERIALIZE_SCALAR(pc);
    UNSERIALIZE_SCALAR(npc);
    UNSERIALIZE_SCALAR(nnpc);
#if FULL_SYSTEM
    UNSERIALIZE_ARRAY(palregs, NumIntRegs);
    UNSERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    UNSERIALIZE_SCALAR(intrflag);
    UNSERIALIZE_SCALAR(pal_shadow);
#endif
}


#if FULL_SYSTEM
void
PTE::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(tag);
    SERIALIZE_SCALAR(ppn);
    SERIALIZE_SCALAR(xre);
    SERIALIZE_SCALAR(xwe);
    SERIALIZE_SCALAR(asn);
    SERIALIZE_SCALAR(asma);
    SERIALIZE_SCALAR(fonr);
    SERIALIZE_SCALAR(fonw);
    SERIALIZE_SCALAR(valid);
}


void
PTE::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(tag);
    UNSERIALIZE_SCALAR(ppn);
    UNSERIALIZE_SCALAR(xre);
    UNSERIALIZE_SCALAR(xwe);
    UNSERIALIZE_SCALAR(asn);
    UNSERIALIZE_SCALAR(asma);
    UNSERIALIZE_SCALAR(fonr);
    UNSERIALIZE_SCALAR(fonw);
    UNSERIALIZE_SCALAR(valid);
}

#endif //FULL_SYSTEM
