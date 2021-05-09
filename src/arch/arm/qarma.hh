// -*- mode:c++ -*-

// Copyright (c) 2020 Metempsy Technology Consulting
// All rights reserved
//
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __ARCH_ARM_QARMA_HH__
#define __ARCH_ARM_QARMA_HH__

#include "base/bitfield.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace QARMA
{

  BitUnion64(BIT64)
      Bitfield<63, 60> b15;
      Bitfield<59, 56> b14;
      Bitfield<55, 52> b13;
      Bitfield<51, 48> b12;
      Bitfield<47, 44> b11;
      Bitfield<43, 40> b10;
      Bitfield<39, 36> b9;
      Bitfield<35, 32> b8;
      Bitfield<31, 28> b7;
      Bitfield<27, 24> b6;
      Bitfield<23, 20> b5;
      Bitfield<19, 16> b4;
      Bitfield<15, 12> b3;
      Bitfield<11, 8>  b2;
      Bitfield<7, 4>   b1;
      Bitfield<3, 0>   b0;
  EndBitUnion(BIT64)


  uint8_t rotCell(uint8_t incell, int amount);

  uint8_t tweakCellInvRot(uint8_t incell);

  uint8_t tweakCellRot(uint8_t incell);

  BIT64 tweakInvShuffle(BIT64 indata);

  BIT64
  tweakShuffle(BIT64 indata);

  BIT64
  PACCellInvShuffle(BIT64 indata);

  BIT64
  PACCellShuffle(BIT64 indata);

  uint64_t PACInvSub(uint64_t tInput);

  uint64_t PACSub(uint64_t tInput);

  uint64_t PACMult(uint64_t tInput);

  BIT64
  computePAC(BIT64 data, BIT64 modifier, BIT64 key0, BIT64 key1);

} // namespace QARMA
} // namespace gem5

#endif //__ARCH_ARM_QARMA_HH__
