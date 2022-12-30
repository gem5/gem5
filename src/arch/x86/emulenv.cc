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
 */

#include "arch/x86/emulenv.hh"

#include <cassert>

#include "base/logging.hh"

namespace gem5
{

using namespace X86ISA;

void EmulEnv::doModRM(const ExtMachInst & machInst)
{
    assert(machInst.modRM.mod != 3);
    //Use the SIB byte for addressing if the modrm byte calls for it.
    if (machInst.modRM.rm == 4 && machInst.addrSize != 2) {
        scale = 1 << machInst.sib.scale;
        index = intRegClass[machInst.sib.index | (machInst.rex.x << 3)];
        base = intRegClass[machInst.sib.base | (machInst.rex.b << 3)];
        //In this special case, we don't use a base. The displacement also
        //changes, but that's managed by the decoder.
        if (machInst.sib.base == (RegIndex)int_reg::Rbp &&
                machInst.modRM.mod == 0)
            base = int_reg::T0;
        //In -this- special case, we don't use an index.
        if (index == int_reg::Rsp)
            index = int_reg::T0;
    } else {
        if (machInst.addrSize == 2) {
            unsigned rm = machInst.modRM.rm;
            if (rm <= 3) {
                scale = 1;
                if (rm < 2) {
                    base = int_reg::Rbx;
                } else {
                    base = int_reg::Rbp;
                }
                index = intRegClass[(rm % 2) ? int_reg::Rdi : int_reg::Rsi];
            } else {
                scale = 0;
                switch (rm) {
                  case 4:
                    base = int_reg::Rsi;
                    break;
                  case 5:
                    base = int_reg::Rdi;
                    break;
                  case 6:
                    // There is a special case when mod is 0 and rm is 6.
                    base = machInst.modRM.mod == 0 ? int_reg::T0 :
                        int_reg::Rbp;
                    break;
                  case 7:
                    base = int_reg::Rbx;
                    break;
                }
            }
        } else {
            scale = 0;
            base = intRegClass[machInst.modRM.rm | (machInst.rex.b << 3)];
            if (machInst.modRM.mod == 0 && machInst.modRM.rm == 5) {
                //Since we need to use a different encoding of this
                //instruction anyway, just ignore the base in those cases
                base = int_reg::T0;
            }
        }
    }
    //Figure out what segment to use.
    if (base != int_reg::Rbp && base != int_reg::Rsp) {
        seg = segment_idx::Ds;
    } else {
        seg = segment_idx::Ss;
    }
    //Handle any segment override that might have been in the instruction
    int segFromInst = machInst.legacy.seg;
    if (segFromInst)
        seg = segFromInst - 1;
}

void EmulEnv::setSeg(const ExtMachInst & machInst)
{
    seg = segment_idx::Ds;
    //Handle any segment override that might have been in the instruction
    int segFromInst = machInst.legacy.seg;
    if (segFromInst)
        seg = segFromInst - 1;
}

} // namespace gem5
