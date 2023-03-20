/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014 Sven Karlsson
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
 * Copyright (c) 2017 The University of Virginia
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

#ifndef __ARCH_RISCV_TYPES_HH__
#define __ARCH_RISCV_TYPES_HH__

#include "arch/riscv/pcstate.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace RiscvISA
{

typedef uint32_t MachInst;

// This should be further extend someday when we start to support 64b+ inst.
// For now, we should be safe using the msbs to store extra information.
BitUnion64(ExtMachInst)
    // Decoder state
    Bitfield<63, 62>    rv_type;
    Bitfield<61>        compressed;
    // More bits for vector extension
    Bitfield<57, 41>    vl;     // [0, 2**16]
    Bitfield<40>        vill;
    SubBitUnion(vtype8, 39, 32) // exclude vill
        Bitfield<39> vma;
        Bitfield<38> vta;
        Bitfield<37, 35> vsew;
        Bitfield<34, 32> vlmul;
    EndSubBitUnion(vtype8)
    // Common
    uint32_t            instBits;
    Bitfield< 1,  0>    quadRant;
    Bitfield< 6,  2>    opcode5;
    Bitfield< 6,  0>    opcode;
    // R-Type
    Bitfield<31,  0>    all;
    Bitfield<11,  7>    rd;
    Bitfield<14, 12>    funct3;
    Bitfield<19, 15>    rs1;
    Bitfield<24, 20>    rs2;
    Bitfield<31, 25>    funct7;
    // Bit shifts
    Bitfield<30>        srType;
    Bitfield<24, 20>    shamt5;
    Bitfield<25, 20>    shamt6;
    // I-Type
    Bitfield<31, 20>    imm12;
    // Sync
    Bitfield<23, 20>    succ;
    Bitfield<27, 24>    pred;
    // S-Type
    Bitfield<11,  7>    imm5;
    Bitfield<31, 25>    imm7;
    // U-Type
    Bitfield<31, 12>    imm20;
    // SB-Type
    Bitfield<7>         bimm12bit11;
    Bitfield<11,  8>    bimm12bits4to1;
    Bitfield<30, 25>    bimm12bits10to5;
    Bitfield<31>        immsign;
    // UJ-Type
    Bitfield<30, 21>    ujimmbits10to1;
    Bitfield<20>        ujimmbit11;
    Bitfield<19, 12>    ujimmbits19to12;
    // System
    Bitfield<31, 20>    funct12;
    Bitfield<19, 15>    csrimm;
    // Floating point
    Bitfield<11,  7>    fd;
    Bitfield<19, 15>    fs1;
    Bitfield<24, 20>    fs2;
    Bitfield<31, 27>    fs3;
    Bitfield<14, 12>    round_mode;
    Bitfield<24, 20>    conv_sgn;
    Bitfield<26, 25>    funct2;
    // AMO
    Bitfield<31, 27>    amofunct;
    Bitfield<26>        aq;
    Bitfield<25>        rl;
    // Compressed
    Bitfield<15, 13>    copcode;
    Bitfield<12>        cfunct1;
    Bitfield<11, 10>    cfunct2high;
    Bitfield< 6,  5>    cfunct2low;
    Bitfield<11,  7>    rc1;
    Bitfield< 6,  2>    rc2;
    Bitfield< 9,  7>    rp1;
    Bitfield< 4,  2>    rp2;
    Bitfield<11,  7>    fc1;
    Bitfield< 6,  2>    fc2;
    Bitfield< 4,  2>    fp2;
    Bitfield<12,  2>    cjumpimm;
    Bitfield< 5,  3>    cjumpimm3to1;
    Bitfield<11, 11>    cjumpimm4to4;
    Bitfield< 2,  2>    cjumpimm5to5;
    Bitfield< 7,  7>    cjumpimm6to6;
    Bitfield< 6,  6>    cjumpimm7to7;
    Bitfield<10,  9>    cjumpimm9to8;
    Bitfield< 8,  8>    cjumpimm10to10;
    Bitfield<12>        cjumpimmsign;
    Bitfield<12,  5>    cimm8;
    Bitfield<12,  7>    cimm6;
    Bitfield< 6,  2>    cimm5;
    Bitfield<12, 10>    cimm3;
    Bitfield< 6,  5>    cimm2;
    Bitfield<12>        cimm1;
    // Pseudo instructions
    Bitfield<31, 25>    m5func;
    // vector
    Bitfield<31, 26>    vfunct6;
    Bitfield<31, 27>    vfunct5;
    Bitfield<27, 25>    vfunct3;
    Bitfield<26, 25>    vfunct2;
    Bitfield<31, 29>    nf;
    Bitfield<28>        mew;
    Bitfield<27, 26>    mop;
    Bitfield<25>        vm;
    Bitfield<24, 20>    lumop;
    Bitfield<24, 20>    sumop;
    Bitfield<14, 12>    width;
    Bitfield<24, 20>    vs2;
    Bitfield<19, 15>    vs1;
    Bitfield<11,  7>    vd;
    Bitfield<11,  7>    vs3;
    Bitfield<19, 15>    vecimm;
    Bitfield<17, 15>    simm3;
    // vsetvli
    Bitfield<31>        bit31;
    Bitfield<30>        bit30;
    Bitfield<30, 20>    zimm_vsetvli;
    // vsetivli
    Bitfield<31, 30>    bit31_30;
    Bitfield<29, 20>    zimm_vsetivli;
    Bitfield<19, 15>    uimm_vsetivli;
    // vsetvl
    Bitfield<31, 25>    bit31_25;

EndBitUnion(ExtMachInst)

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_TYPES_HH__
