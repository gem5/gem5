/*
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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

#include "arch/x86/types.hh"

#include "sim/serialize.hh"

using namespace X86ISA;
using namespace std;

template <>
void
paramOut(CheckpointOut &cp, const string &name, ExtMachInst const &machInst)
{
    // Prefixes
    paramOut(cp, name + ".legacy", (uint8_t)machInst.legacy);
    paramOut(cp, name + ".rex", (uint8_t)machInst.rex);
    paramOut(cp, name + ".vex", (uint32_t)machInst.vex);

    // Opcode
    paramOut(cp, name + ".opcode.type", (uint8_t)machInst.opcode.type);
    paramOut(cp, name + ".opcode.op", (uint8_t)machInst.opcode.op);

    // Modifier bytes
    paramOut(cp, name + ".modRM", (uint8_t)machInst.modRM);
    paramOut(cp, name + ".sib", (uint8_t)machInst.sib);

    // Immediate fields
    paramOut(cp, name + ".immediate", machInst.immediate);
    paramOut(cp, name + ".displacement", machInst.displacement);

    // Sizes
    paramOut(cp, name + ".opSize", machInst.opSize);
    paramOut(cp, name + ".addrSize", machInst.addrSize);
    paramOut(cp, name + ".stackSize", machInst.stackSize);
    paramOut(cp, name + ".dispSize", machInst.dispSize);

    // Mode
    paramOut(cp, name + ".mode", (uint8_t)machInst.mode);
}

template <>
void
paramIn(CheckpointIn &cp, const string &name, ExtMachInst &machInst)
{
    uint8_t temp8;
    // Prefixes
    paramIn(cp, name + ".legacy", temp8);
    machInst.legacy = temp8;
    paramIn(cp, name + ".rex", temp8);
    machInst.rex = temp8;

    uint32_t temp32;
    paramIn(cp, name + ".vex", temp32);
    machInst.vex = temp32;

    // Opcode
    paramIn(cp, name + ".opcode.type", temp8);
    machInst.opcode.type = (OpcodeType)temp8;
    paramIn(cp, name + ".opcode.op", temp8);
    machInst.opcode.op = temp8;

    // Modifier bytes
    paramIn(cp, name + ".modRM", temp8);
    machInst.modRM = temp8;
    paramIn(cp, name + ".sib", temp8);
    machInst.sib = temp8;;

    // Immediate fields
    paramIn(cp, name + ".immediate", machInst.immediate);
    paramIn(cp, name + ".displacement", machInst.displacement);

    // Sizes
    paramIn(cp, name + ".opSize", machInst.opSize);
    paramIn(cp, name + ".addrSize", machInst.addrSize);
    paramIn(cp, name + ".stackSize", machInst.stackSize);
    paramIn(cp, name + ".dispSize", machInst.dispSize);

    // Mode
    paramIn(cp, name + ".mode", temp8);
    machInst.mode = temp8;
}
