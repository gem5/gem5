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

void
Decoder::moreBytes(const PCState &pc, Addr fetchPC, MachInst inst)
{
    DPRINTF(Decode, "Getting bytes 0x%08x from address %#x\n",
            inst, pc.pc());

    bool aligned = pc.pc() % sizeof(MachInst) == 0;
    if (mid) {
        assert(!aligned);
        emi |= (inst & 0xFFFF) << 16;
        instDone = true;
    } else {
        MachInst instChunk = aligned ? inst & 0xFFFF :
                                      (inst & 0xFFFF0000) >> 16;
        if (aligned) {
            emi = (inst & 0x3) < 0x3 ? instChunk : inst;
            instDone = true;
        } else {
            emi = instChunk;
            instDone = (instChunk & 0x3) < 0x3;
        }
    }
    mid = !instDone;
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

    if ((emi & 0x3) < 0x3) {
        nextPC.compressed(true);
        nextPC.npc(nextPC.pc() + sizeof(MachInst)/2);
    } else {
        nextPC.compressed(false);
        nextPC.npc(nextPC.pc() + sizeof(MachInst));
    }

    return decode(emi, nextPC.instAddr());
}

}
