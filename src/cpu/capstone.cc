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

#include "cpu/capstone.hh"

#include "base/output.hh"

namespace gem5
{

namespace trace
{

std::string
CapstoneDisassembler::disassemble(StaticInstPtr inst, const PCStateBase &pc,
                                  const loader::SymbolTable *symtab) const
{
    std::string inst_dist;
    if (inst->isPseudo() || inst->isMicroop()) {
        // Capstone doesn't have any visibility over microops nor over
        // gem5 pseudo ops. Use native disassembler instead
        inst_dist = InstDisassembler::disassemble(inst, pc, symtab);
    } else {
        // Stripping the extended fields from the ExtMachInst
        auto mach_inst = inst->getEMI() & mask(inst->size() * 8);

        cs_insn *insn;
        // capstone disassembler
        if (const csh *curr_handle = currHandle(pc); curr_handle != nullptr) {
            size_t count = cs_disasm(*curr_handle, (uint8_t *)&mach_inst,
                                     inst->size(), 0, 0, &insn);

            // As we are passing only one instruction, we are expecting one
            // instruction only being disassembled
            assert(count <= 1);

            for (int idx = 0; idx < count; idx++) {
                inst_dist += csprintf("  %s   %s", insn[idx].mnemonic,
                                      insn[idx].op_str);
            }
        } else {
            // No valid handle; return an invalid string
            inst_dist += "  capstone failure";
        }
    }

    return inst_dist;
}

CapstoneDisassembler::CapstoneDisassembler(const Params &p)
    : InstDisassembler(p)
{}

} // namespace trace
} // namespace gem5
