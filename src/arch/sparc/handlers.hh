/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_HANDLERS_HH__
#define __ARCH_SPARC_HANDLERS_HH__

#include "arch/sparc/types.hh"
#include "sim/byteswap.hh"

namespace gem5
{

namespace SparcISA
{

// We only use 19 instructions for the trap handlers, but there would be
// space for 32 in a real SPARC trap table.
const int numFillInsts = 32;
const int numSpillInsts = 32;

const MachInst fillHandler64[numFillInsts] = {
    htobe<MachInst>(0x87802016), // wr %g0, ASI_AIUP, %asi
    htobe<MachInst>(0xe0dba7ff), // ldxa [%sp + BIAS + (0*8)] %asi, %l0
    htobe<MachInst>(0xe2dba807), // ldxa [%sp + BIAS + (1*8)] %asi, %l1
    htobe<MachInst>(0xe4dba80f), // ldxa [%sp + BIAS + (2*8)] %asi, %l2
    htobe<MachInst>(0xe6dba817), // ldxa [%sp + BIAS + (3*8)] %asi, %l3
    htobe<MachInst>(0xe8dba81f), // ldxa [%sp + BIAS + (4*8)] %asi, %l4
    htobe<MachInst>(0xeadba827), // ldxa [%sp + BIAS + (5*8)] %asi, %l5
    htobe<MachInst>(0xecdba82f), // ldxa [%sp + BIAS + (6*8)] %asi, %l6
    htobe<MachInst>(0xeedba837), // ldxa [%sp + BIAS + (7*8)] %asi, %l7
    htobe<MachInst>(0xf0dba83f), // ldxa [%sp + BIAS + (8*8)] %asi, %i0
    htobe<MachInst>(0xf2dba847), // ldxa [%sp + BIAS + (9*8)] %asi, %i1
    htobe<MachInst>(0xf4dba84f), // ldxa [%sp + BIAS + (10*8)] %asi, %i2
    htobe<MachInst>(0xf6dba857), // ldxa [%sp + BIAS + (11*8)] %asi, %i3
    htobe<MachInst>(0xf8dba85f), // ldxa [%sp + BIAS + (12*8)] %asi, %i4
    htobe<MachInst>(0xfadba867), // ldxa [%sp + BIAS + (13*8)] %asi, %i5
    htobe<MachInst>(0xfcdba86f), // ldxa [%sp + BIAS + (14*8)] %asi, %i6
    htobe<MachInst>(0xfedba877), // ldxa [%sp + BIAS + (15*8)] %asi, %i7
    htobe<MachInst>(0x83880000), // restored
    htobe<MachInst>(0x83F00000), // retry
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000)  // illtrap
};

const MachInst fillHandler32[numFillInsts] = {
    htobe<MachInst>(0x87802016), // wr %g0, ASI_AIUP, %asi
    htobe<MachInst>(0xe083a000), // lduwa [%sp + (0*4)] %asi, %l0
    htobe<MachInst>(0xe283a004), // lduwa [%sp + (1*4)] %asi, %l1
    htobe<MachInst>(0xe483a008), // lduwa [%sp + (2*4)] %asi, %l2
    htobe<MachInst>(0xe683a00c), // lduwa [%sp + (3*4)] %asi, %l3
    htobe<MachInst>(0xe883a010), // lduwa [%sp + (4*4)] %asi, %l4
    htobe<MachInst>(0xea83a014), // lduwa [%sp + (5*4)] %asi, %l5
    htobe<MachInst>(0xec83a018), // lduwa [%sp + (6*4)] %asi, %l6
    htobe<MachInst>(0xee83a01c), // lduwa [%sp + (7*4)] %asi, %l7
    htobe<MachInst>(0xf083a020), // lduwa [%sp + (8*4)] %asi, %i0
    htobe<MachInst>(0xf283a024), // lduwa [%sp + (9*4)] %asi, %i1
    htobe<MachInst>(0xf483a028), // lduwa [%sp + (10*4)] %asi, %i2
    htobe<MachInst>(0xf683a02c), // lduwa [%sp + (11*4)] %asi, %i3
    htobe<MachInst>(0xf883a030), // lduwa [%sp + (12*4)] %asi, %i4
    htobe<MachInst>(0xfa83a034), // lduwa [%sp + (13*4)] %asi, %i5
    htobe<MachInst>(0xfc83a038), // lduwa [%sp + (14*4)] %asi, %i6
    htobe<MachInst>(0xfe83a03c), // lduwa [%sp + (15*4)] %asi, %i7
    htobe<MachInst>(0x83880000), // restored
    htobe<MachInst>(0x83F00000), // retry
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000)  // illtrap
};

const MachInst spillHandler64[numSpillInsts] = {
    htobe<MachInst>(0x87802016), // wr %g0, ASI_AIUP, %asi
    htobe<MachInst>(0xe0f3a7ff), // stxa %l0, [%sp + BIAS + (0*8)] %asi
    htobe<MachInst>(0xe2f3a807), // stxa %l1, [%sp + BIAS + (1*8)] %asi
    htobe<MachInst>(0xe4f3a80f), // stxa %l2, [%sp + BIAS + (2*8)] %asi
    htobe<MachInst>(0xe6f3a817), // stxa %l3, [%sp + BIAS + (3*8)] %asi
    htobe<MachInst>(0xe8f3a81f), // stxa %l4, [%sp + BIAS + (4*8)] %asi
    htobe<MachInst>(0xeaf3a827), // stxa %l5, [%sp + BIAS + (5*8)] %asi
    htobe<MachInst>(0xecf3a82f), // stxa %l6, [%sp + BIAS + (6*8)] %asi
    htobe<MachInst>(0xeef3a837), // stxa %l7, [%sp + BIAS + (7*8)] %asi
    htobe<MachInst>(0xf0f3a83f), // stxa %i0, [%sp + BIAS + (8*8)] %asi
    htobe<MachInst>(0xf2f3a847), // stxa %i1, [%sp + BIAS + (9*8)] %asi
    htobe<MachInst>(0xf4f3a84f), // stxa %i2, [%sp + BIAS + (10*8)] %asi
    htobe<MachInst>(0xf6f3a857), // stxa %i3, [%sp + BIAS + (11*8)] %asi
    htobe<MachInst>(0xf8f3a85f), // stxa %i4, [%sp + BIAS + (12*8)] %asi
    htobe<MachInst>(0xfaf3a867), // stxa %i5, [%sp + BIAS + (13*8)] %asi
    htobe<MachInst>(0xfcf3a86f), // stxa %i6, [%sp + BIAS + (14*8)] %asi
    htobe<MachInst>(0xfef3a877), // stxa %i7, [%sp + BIAS + (15*8)] %asi
    htobe<MachInst>(0x81880000), // saved
    htobe<MachInst>(0x83F00000), // retry
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000)  // illtrap
};

const MachInst spillHandler32[numSpillInsts] = {
    htobe<MachInst>(0x87802016), // wr %g0, ASI_AIUP, %asi
    htobe<MachInst>(0xe0a3a000), // stwa %l0, [%sp + (0*4)] %asi
    htobe<MachInst>(0xe2a3a004), // stwa %l1, [%sp + (1*4)] %asi
    htobe<MachInst>(0xe4a3a008), // stwa %l2, [%sp + (2*4)] %asi
    htobe<MachInst>(0xe6a3a00c), // stwa %l3, [%sp + (3*4)] %asi
    htobe<MachInst>(0xe8a3a010), // stwa %l4, [%sp + (4*4)] %asi
    htobe<MachInst>(0xeaa3a014), // stwa %l5, [%sp + (5*4)] %asi
    htobe<MachInst>(0xeca3a018), // stwa %l6, [%sp + (6*4)] %asi
    htobe<MachInst>(0xeea3a01c), // stwa %l7, [%sp + (7*4)] %asi
    htobe<MachInst>(0xf0a3a020), // stwa %i0, [%sp + (8*4)] %asi
    htobe<MachInst>(0xf2a3a024), // stwa %i1, [%sp + (9*4)] %asi
    htobe<MachInst>(0xf4a3a028), // stwa %i2, [%sp + (10*4)] %asi
    htobe<MachInst>(0xf6a3a02c), // stwa %i3, [%sp + (11*4)] %asi
    htobe<MachInst>(0xf8a3a030), // stwa %i4, [%sp + (12*4)] %asi
    htobe<MachInst>(0xfaa3a034), // stwa %i5, [%sp + (13*4)] %asi
    htobe<MachInst>(0xfca3a038), // stwa %i6, [%sp + (14*4)] %asi
    htobe<MachInst>(0xfea3a03c), // stwa %i7, [%sp + (15*4)] %asi
    htobe<MachInst>(0x81880000), // saved
    htobe<MachInst>(0x83F00000), // retry
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000), // illtrap
    htobe<MachInst>(0x00000000)  // illtrap
};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_HANDLERS_HH__
