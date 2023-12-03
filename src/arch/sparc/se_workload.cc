/*
 * Copyright 2020 Google Inc.
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

#include "arch/sparc/se_workload.hh"

#include "arch/sparc/process.hh"
#include "arch/sparc/regs/int.hh"
#include "arch/sparc/regs/misc.hh"
#include "arch/sparc/types.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "mem/se_translating_port_proxy.hh"

namespace gem5
{

namespace SparcISA
{

const std::vector<RegId> SEWorkload::BaseSyscallABI::ArgumentRegs = {
    int_reg::O0, int_reg::O1, int_reg::O2,
    int_reg::O3, int_reg::O4, int_reg::O5
};

bool
SEWorkload::is64(ThreadContext *tc)
{
    return dynamic_cast<Sparc64Process *>(tc->getProcessPtr());
}

void
SEWorkload::handleTrap(ThreadContext *tc, int trapNum)
{
    auto &pc = tc->pcState().as<PCState>();
    switch (trapNum) {
    case 0x01: // Software breakpoint
        warn("Software breakpoint encountered at pc %#x.", pc.pc());
        break;
    case 0x02: // Division by zero
        warn("Software signaled a division by zero at pc %#x.", pc.pc());
        break;
    case 0x03: // Flush window trap
        flushWindows(tc);
        break;
    case 0x04: // Clean windows
        warn("Ignoring process request for clean register "
             "windows at pc %#x.",
             pc.pc());
        break;
    case 0x05: // Range check
        warn("Software signaled a range check at pc %#x.", pc.pc());
        break;
    case 0x06: // Fix alignment
        warn("Ignoring process request for os assisted unaligned accesses "
             "at pc %#x.",
             pc.pc());
        break;
    case 0x07: // Integer overflow
        warn("Software signaled an integer overflow at pc %#x.", pc.pc());
        break;
    case 0x32: // Get integer condition codes
        warn("Ignoring process request to get the integer condition codes "
             "at pc %#x.",
             pc.pc());
        break;
    case 0x33: // Set integer condition codes
        warn("Ignoring process request to set the integer condition codes "
             "at pc %#x.",
             pc.pc());
        break;
    default:
        panic("Unimplemented trap to operating system: trap number %#x.",
              trapNum);
    }
}

void
SEWorkload::flushWindows(ThreadContext *tc)
{
    RegVal Cansave = tc->getReg(int_reg::Cansave);
    RegVal Canrestore = tc->getReg(int_reg::Canrestore);
    RegVal Otherwin = tc->getReg(int_reg::Otherwin);
    RegVal CWP = tc->readMiscReg(MISCREG_CWP);
    RegVal origCWP = CWP;

    const bool is_64 = is64(tc);
    const size_t reg_bytes = is_64 ? 8 : 4;
    uint8_t bytes[8];

    SETranslatingPortProxy proxy(tc);

    CWP = (CWP + Cansave + 2) % NWindows;
    while (NWindows - 2 - Cansave != 0) {
        panic_if(Otherwin, "Otherwin non-zero.");

        tc->setMiscReg(MISCREG_CWP, CWP);
        // Do the stores
        RegVal sp = tc->getReg(StackPointerReg);

        Addr addr = is_64 ? sp + 2047 : sp;
        for (int index = 16; index < 32; index++) {
            RegId reg = intRegClass[index];
            if (is_64) {
                uint64_t regVal = htobe<uint64_t>(tc->getReg(reg));
                memcpy(bytes, &regVal, reg_bytes);
            } else {
                uint32_t regVal = htobe<uint32_t>(tc->getReg(reg));
                memcpy(bytes, &regVal, reg_bytes);
            }
            if (!proxy.tryWriteBlob(addr, bytes, reg_bytes)) {
                warn("Failed to save register to the stack when "
                     "flushing windows.");
            }
            addr += reg_bytes;
        }
        Canrestore--;
        Cansave++;
        CWP = (CWP + 1) % NWindows;
    }

    tc->setReg(int_reg::Cansave, Cansave);
    tc->setReg(int_reg::Canrestore, Canrestore);
    tc->setMiscReg(MISCREG_CWP, origCWP);
}

} // namespace SparcISA
} // namespace gem5
