/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/regfile.hh"
#include "sim/serialize.hh"
#include "cpu/thread_context.hh"

class Checkpoint;

using namespace X86ISA;
using namespace std;

//RegFile class methods
Addr RegFile::readPC()
{
    return rip;
}

void RegFile::setPC(Addr val)
{
    rip = val;
}

Addr RegFile::readNextPC()
{
    return nextRip;
}

void RegFile::setNextPC(Addr val)
{
    nextRip = val;
}

Addr RegFile::readNextNPC()
{
    return nextRip + sizeof(MachInst);
}

void RegFile::setNextNPC(Addr val)
{ }

void RegFile::clear()
{
    floatRegFile.clear();
    intRegFile.clear();
    miscRegFile.clear();
}

MiscReg RegFile::readMiscReg(int miscReg)
{
    return miscRegFile.readReg(miscReg);
}

MiscReg RegFile::readMiscRegWithEffect(int miscReg, ThreadContext *tc)
{
    return miscRegFile.readRegWithEffect(miscReg, tc);
}

void RegFile::setMiscReg(int miscReg, const MiscReg &val)
{
    miscRegFile.setReg(miscReg, val);
}

void RegFile::setMiscRegWithEffect(int miscReg, const MiscReg &val,
        ThreadContext * tc)
{
    miscRegFile.setRegWithEffect(miscReg, val, tc);
}

FloatReg RegFile::readFloatReg(int floatReg, int width)
{
    return floatRegFile.readReg(floatReg, width);
}

FloatReg RegFile::readFloatReg(int floatReg)
{
    //Use the "natural" width of a single float
    return floatRegFile.readReg(floatReg, FloatRegFile::SingleWidth);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg, int width)
{
    return floatRegFile.readRegBits(floatReg, width);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg)
{
    //Use the "natural width of a single float
    return floatRegFile.readRegBits(floatReg,
            FloatRegFile::SingleWidth);
}

void RegFile::setFloatReg(int floatReg, const FloatReg &val, int width)
{
    floatRegFile.setReg(floatReg, val, width);
}

void RegFile::setFloatReg(int floatReg, const FloatReg &val)
{
    //Use the "natural" width of a single float
    setFloatReg(floatReg, val, FloatRegFile::SingleWidth);
}

void RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
{
    floatRegFile.setRegBits(floatReg, val, width);
}

void RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val)
{
    //Use the "natural" width of a single float
    floatRegFile.setRegBits(floatReg, val, FloatRegFile::SingleWidth);
}

IntReg RegFile::readIntReg(int intReg)
{
    return intRegFile.readReg(intReg);
}

void RegFile::setIntReg(int intReg, const IntReg &val)
{
    intRegFile.setReg(intReg, val);
}

int X86ISA::flattenIntIndex(ThreadContext * tc, int reg)
{
    //For right now, don't do any flattening
    return reg;
}

void RegFile::serialize(std::ostream &os)
{
    intRegFile.serialize(os);
    floatRegFile.serialize(os);
    miscRegFile.serialize(os);
    SERIALIZE_SCALAR(rip);
    SERIALIZE_SCALAR(nextRip);
}

void RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    intRegFile.unserialize(cp, section);
    floatRegFile.unserialize(cp, section);
    miscRegFile.unserialize(cp, section);
    UNSERIALIZE_SCALAR(rip);
    UNSERIALIZE_SCALAR(nextRip);
}

void RegFile::changeContext(RegContextParam param, RegContextVal val)
{
    panic("changeContext not implemented for x86!\n");
}

void X86ISA::copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("copyMiscRegs not implemented for x86!\n");
}

void X86ISA::copyRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("copyRegs not implemented for x86!\n");
    //copy int regs
    //copy float regs
    copyMiscRegs(src, dest);

    dest->setPC(src->readPC());
    dest->setNextPC(src->readNextPC());
}
