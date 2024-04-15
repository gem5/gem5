/*
 * Copyright (c) 2023 Arm Limited
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

#ifndef __CPU_CAPSTONE_HH__
#define __CPU_CAPSTONE_HH__

#include <capstone/capstone.h>

#include "params/CapstoneDisassembler.hh"
#include "sim/insttracer.hh"

namespace gem5
{

class ThreadContext;

namespace trace
{

/**
 * Capstone Disassembler:
 * The disassembler relies on the capstone library to convert
 * the StaticInst encoding into the disassembled string.
 *
 * One thing to keep in mind is that the disassembled
 * instruction might not coincide with the instruction being
 * decoded + executed in gem5. This could be the case if
 * there was a bug in either gem5 or in capstone itself.
 * This scenatio is not possible with the native gem5 disassembler
 * as the instruction mnemonic is tightly coupled with the
 * decoded(=generated) instruction (you print what you decode)
 *
 * The Capstone dispatches to the native disassembler in
 * two cases:
 *
 * a) m5 pseudo ops
 * b) micro-ops
 */
class CapstoneDisassembler : public InstDisassembler
{
  public:
    PARAMS(CapstoneDisassembler);
    CapstoneDisassembler(const Params &p);

    std::string disassemble(StaticInstPtr inst, const PCStateBase &pc,
                            const loader::SymbolTable *symtab) const override;

  protected:
    /**
     * Return a pointer to the current capstone handle (csh).
     *
     * Any ISA extension of the Capstone disassembler should
     * initialize (with cs_open) one or more capstone handles
     * at construcion time.
     * (You might need more than one handle in case the ISA
     * has more than one mode of operation, e.g. arm and arm64)
     * The current handle in use should be returned every time
     * the currHandle is called.
     */
    virtual const csh *currHandle(const PCStateBase &pc) const = 0;
};

} // namespace trace
} // namespace gem5

#endif // __CPU_CAPSTONE_HH__
