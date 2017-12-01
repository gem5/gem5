/*
 * Copyright (c) 2012 Google
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
 *
 * Authors: Gabe Black
 *          Alec Roelke
 */

#ifndef __ARCH_RISCV_DECODER_HH__
#define __ARCH_RISCV_DECODER_HH__

#include "arch/generic/decode_cache.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/types.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "debug/Decode.hh"

namespace RiscvISA
{

class ISA;
class Decoder
{
  private:
    DecodeCache::InstMap instMap;
    bool aligned;
    bool mid;
    bool more;

  protected:
    //The extended machine instruction being generated
    ExtMachInst emi;
    bool instDone;

  public:
    Decoder(ISA* isa=nullptr) { reset(); }

    void process() {}
    void reset();

    inline bool compressed(ExtMachInst inst) { return (inst & 0x3) < 0x3; }

    //Use this to give data to the decoder. This should be used
    //when there is control flow.
    void moreBytes(const PCState &pc, Addr fetchPC, MachInst inst);

    bool needMoreBytes() { return more; }
    bool instReady() { return instDone; }
    void takeOverFrom(Decoder *old) {}

    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr);

    StaticInstPtr decode(RiscvISA::PCState &nextPC);
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_DECODER_HH__
