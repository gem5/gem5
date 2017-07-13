/*
 * Copyright (c) 2012 Google
 * Copyright (c) The University of Virginia
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
 *          Alec Roelke
 */

#include "arch/riscv/decoder.hh"
#include "arch/riscv/types.hh"
#include "debug/Decode.hh"

namespace RiscvISA
{

static const MachInst LowerBitMask = (1 << sizeof(MachInst) * 4) - 1;
static const MachInst UpperBitMask = LowerBitMask << sizeof(MachInst) * 4;

void Decoder::reset()
{
    aligned = true;
    mid = false;
    more = true;
    emi = NoopMachInst;
    instDone = false;
}

void
Decoder::moreBytes(const PCState &pc, Addr fetchPC, MachInst inst)
{
    DPRINTF(Decode, "Requesting bytes 0x%08x from address %#x\n", inst,
            fetchPC);

    bool aligned = pc.pc() % sizeof(MachInst) == 0;
    if (aligned) {
        emi = inst;
        if (compressed(emi))
            emi &= LowerBitMask;
        more = !compressed(emi);
        instDone = true;
    } else {
        if (mid) {
            assert((emi & UpperBitMask) == 0);
            emi |= (inst & LowerBitMask) << sizeof(MachInst)*4;
            mid = false;
            more = false;
            instDone = true;
        } else {
            emi = (inst & UpperBitMask) >> sizeof(MachInst)*4;
            mid = !compressed(emi);
            more = true;
            instDone = compressed(emi);
        }
    }
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    DPRINTF(Decode, "Decoding instruction 0x%08x at address %#x\n",
            mach_inst, addr);
    if (instMap.find(mach_inst) != instMap.end())
        return instMap[mach_inst];
    else {
        StaticInstPtr si = decodeInst(mach_inst);
        instMap[mach_inst] = si;
        return si;
    }
}

StaticInstPtr
Decoder::decode(RiscvISA::PCState &nextPC)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    if (compressed(emi)) {
        nextPC.npc(nextPC.instAddr() + sizeof(MachInst) / 2);
    } else {
        nextPC.npc(nextPC.instAddr() + sizeof(MachInst));
    }

    return decode(emi, nextPC.instAddr());
}

}
