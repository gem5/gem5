/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
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
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
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

#ifndef __ARCH_MIPS_REGFILE_REGFILE_HH__
#define __ARCH_MIPS_REGFILE_REGFILE_HH__

#include "arch/mips/types.hh"
#include "arch/mips/isa_traits.hh"
#include "arch/mips/mt.hh"
#include "arch/mips/regfile/int_regfile.hh"
#include "arch/mips/regfile/float_regfile.hh"
#include "arch/mips/regfile/misc_regfile.hh"
#include "sim/faults.hh"

class Checkpoint;
class ThreadContext;

using namespace MipsISA;

void RegFile::clear()
{
    intRegFile.clear();
    floatRegFile.clear();
    miscRegFile.clear();
}

void RegFile::reset(std::string core_name, unsigned num_threads, unsigned num_vpes)
{
    bzero(&intRegFile, sizeof(intRegFile));
    bzero(&floatRegFile, sizeof(floatRegFile));
    miscRegFile.reset(core_name, num_threads, num_vpes);
}

IntReg RegFile::readIntReg(int intReg)
{
    return intRegFile.readReg(intReg);
}

Fault RegFile::setIntReg(int intReg, const IntReg &val)
{
    return intRegFile.setReg(intReg, val);
}

MiscReg RegFile::readMiscRegNoEffect(int miscReg, unsigned tid = 0)
{
    return miscRegFile.readRegNoEffect(miscReg, tid);
}

MiscReg RegFile::readMiscReg(int miscReg, ThreadContext *tc,
                             unsigned tid = 0)
{
    return miscRegFile.readReg(miscReg, tc, tid);
}

void RegFile::setMiscRegNoEffect(int miscReg, const MiscReg &val, unsigned tid = 0)
{
    miscRegFile.setRegNoEffect(miscReg, val, tid);
}

void RegFile::setMiscReg(int miscReg, const MiscReg &val,
                         ThreadContext * tc, unsigned tid = 0)
{
    miscRegFile.setReg(miscReg, val, tc, tid);
}

FloatRegVal RegFile::readFloatReg(int floatReg)
{
    return floatRegFile.readReg(floatReg,SingleWidth);
}

FloatRegVal RegFile::readFloatReg(int floatReg, int width)
{
    return floatRegFile.readReg(floatReg,width);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg)
{
    return floatRegFile.readRegBits(floatReg,SingleWidth);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg, int width)
{
    return floatRegFile.readRegBits(floatReg,width);
}

Fault RegFile::setFloatReg(int floatReg, const FloatRegVal &val)
{
    return floatRegFile.setReg(floatReg, val, SingleWidth);
}

Fault RegFile::setFloatReg(int floatReg, const FloatRegVal &val, int width)
{
    return floatRegFile.setReg(floatReg, val, width);
}

Fault RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val)
{
    return floatRegFile.setRegBits(floatReg, val, SingleWidth);
}

Fault RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
{
    return floatRegFile.setRegBits(floatReg, val, width);
}

Addr RegFile::readPC()
{
    return pc;
}

void RegFile::setPC(Addr val)
{
    pc = val;
}

Addr RegFile::readNextPC()
{
    return npc;
}

void RegFile::setNextPC(Addr val)
{
    npc = val;
}

Addr RegFile::readNextNPC()
{
    return nnpc;
}

void RegFile::setNextNPC(Addr val)
{
    nnpc = val;
}

void
RegFile::serialize(std::ostream &os)
{
    intRegFile.serialize(os);
    floatRegFile.serialize(os);
    miscRegFile.serialize(os);

    SERIALIZE_SCALAR(pc);
    SERIALIZE_SCALAR(npc);
    SERIALIZE_SCALAR(nnpc);
}


void
RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    intRegFile.unserialize(cp, section);
    floatRegFile.unserialize(cp, section);
    miscRegFile.unserialize(cp, section);
    UNSERIALIZE_SCALAR(pc);
    UNSERIALIZE_SCALAR(npc);
    UNSERIALIZE_SCALAR(nnpc);

}


void RegFile::changeContext(RegContextParam param, RegContextVal val)
{
    panic("Change Context Not Implemented for MipsISA");
}

static inline int flattenIntIndex(ThreadContext * tc, int reg)
{
    return reg;
}

void
MipsISA::copyRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Regs Not Implemented Yet\n");
}

void
MipsISA::copyRegs(ThreadContext *src, ThreadContext *dest);
{
    panic("Copy Regs Not Implemented Yet\n");
}

void
MipsISA::copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
}
