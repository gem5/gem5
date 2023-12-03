/*
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#ifndef __ARCH_POWER_TYPES_HH__
#define __ARCH_POWER_TYPES_HH__

#include <cstdint>

#include "arch/power/pcstate.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace PowerISA
{

typedef uint32_t MachInst;

BitUnion32(ExtMachInst)

    // Registers
    Bitfield<25, 21> rs;
    Bitfield<20, 16> ra;

    // Shifts and masks
    Bitfield<15, 11> sh;
    Bitfield<1> shn;
    Bitfield<10, 6> mb;
    Bitfield<5> mbn;
    Bitfield<5, 1> me;
    Bitfield<5> men;

    // Immediate fields
    Bitfield<15, 0> si;
    Bitfield<15, 0> ui;
    Bitfield<15, 0> d;
    Bitfield<15, 2> ds;
    Bitfield<15, 6> d0;
    Bitfield<20, 16> d1;
    Bitfield<1, 0> d2;

    // Compare fields
    Bitfield<21> l;

    // Special purpose register identifier
    Bitfield<20, 11> spr;
    Bitfield<25, 23> bf;
    Bitfield<20, 18> bfa;

    // Branch instruction fields
    Bitfield<1> aa;
    Bitfield<15, 2> bd;
    Bitfield<20, 16> bi;
    Bitfield<12, 11> bh;
    Bitfield<25, 21> bo;
    Bitfield<25, 2> li;
    Bitfield<0> lk;

    // Record bits
    Bitfield<0> rc;
    Bitfield<10> oe;

    // Condition register fields
    Bitfield<25, 21> bt;
    Bitfield<20, 16> ba;
    Bitfield<15, 11> bb;

    // Trap instruction fields
    Bitfield<25, 21> to;

    // FXM field for mtcrf instruction
    Bitfield<19, 12> fxm;
EndBitUnion(ExtMachInst)

// typedef uint64_t LargestRead;
// // Need to use 64 bits to make sure that read requests get handled properly

// typedef int RegContextParam;
// typedef int RegContextVal;

} // namespace PowerISA
} // namespace gem5

namespace std
{

template <>
struct hash<gem5::PowerISA::ExtMachInst> : public hash<uint32_t>
{
    size_t
    operator()(const gem5::PowerISA::ExtMachInst &emi) const
    {
        return hash<uint32_t>::operator()((uint32_t)emi);
    };
};

} // namespace std

#endif // __ARCH_POWER_TYPES_HH__
