/*
 * Copyright (c) 2026 The Regents of The University of California
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

#ifndef __ARCH_ARM_APPLE_VIRT_EXIT_DECODE_HH__
#define __ARCH_ARM_APPLE_VIRT_EXIT_DECODE_HH__

#include <cstdint>

namespace gem5
{
namespace ArmISA
{
namespace AppleVirt
{

enum class ExitKind
{
    Unsupported,
    DataAbort,
    SystemRegister,
    Wfi,
    Wfe,
};

struct SystemRegisterSyndrome
{
    uint8_t op0 = 0;
    uint8_t op1 = 0;
    uint8_t crn = 0;
    uint8_t crm = 0;
    uint8_t op2 = 0;
    uint8_t targetRegister = 0;
    bool read = false;
};

struct DataAbortSyndrome
{
    /** The instruction syndrome fields are valid (ESR_EL2.ISV). */
    bool valid = false;
    bool write = false;
    /** Access size in bytes, or zero when valid is false. */
    uint8_t size = 0;
    bool signExtend = false;
    /** Architectural integer register number, or zero when invalid. */
    uint8_t targetRegister = 0;
    bool sf = false;
    bool acquireRelease = false;
    bool cacheMaintenance = false;
    bool s1ptw = false;
};

struct DecodedExit
{
    ExitKind kind = ExitKind::Unsupported;
    uint8_t ec = 0;
    uint32_t iss = 0;
    DataAbortSyndrome dataAbort;
    SystemRegisterSyndrome systemRegister;
};

/** Decode the architecture-defined fields of an AArch64 ESR value. */
constexpr DecodedExit
decodeExit(uint64_t esr)
{
    constexpr uint8_t trappedWfxEc = 0x01;
    constexpr uint8_t trappedSystemRegisterEc = 0x18;
    constexpr uint8_t dataAbortLowerElEc = 0x24;
    constexpr uint8_t dataAbortSameElEc = 0x25;
    constexpr uint32_t issMask = (1U << 25) - 1;

    DecodedExit exit;
    exit.ec = (esr >> 26) & 0x3f;
    exit.iss = esr & issMask;

    if (exit.ec == trappedWfxEc) {
        const uint8_t trapped_instruction = exit.iss & 0x3;
        if (trapped_instruction == 0) {
            exit.kind = ExitKind::Wfi;
        } else if (trapped_instruction == 1) {
            exit.kind = ExitKind::Wfe;
        }
        return exit;
    }

    if (exit.ec == trappedSystemRegisterEc) {
        exit.kind = ExitKind::SystemRegister;
        auto &sys_reg = exit.systemRegister;
        sys_reg.op0 = (exit.iss >> 20) & 0x3;
        sys_reg.op1 = (exit.iss >> 14) & 0x7;
        sys_reg.crn = (exit.iss >> 10) & 0xf;
        sys_reg.crm = (exit.iss >> 1) & 0xf;
        sys_reg.op2 = (exit.iss >> 17) & 0x7;
        sys_reg.targetRegister = (exit.iss >> 5) & 0x1f;
        sys_reg.read = exit.iss & 0x1;
        return exit;
    }

    if (exit.ec != dataAbortLowerElEc && exit.ec != dataAbortSameElEc) {
        return exit;
    }

    exit.kind = ExitKind::DataAbort;
    auto &abort = exit.dataAbort;
    abort.valid = (exit.iss >> 24) & 0x1;
    abort.write = (exit.iss >> 6) & 0x1;
    abort.cacheMaintenance = (exit.iss >> 8) & 0x1;
    abort.s1ptw = (exit.iss >> 7) & 0x1;

    if (abort.valid) {
        const uint8_t sas = (exit.iss >> 22) & 0x3;
        abort.size = 1U << sas;
        abort.signExtend = (exit.iss >> 21) & 0x1;
        abort.targetRegister = (exit.iss >> 16) & 0x1f;
        abort.sf = (exit.iss >> 15) & 0x1;
        abort.acquireRelease = (exit.iss >> 14) & 0x1;
    }

    return exit;
}

} // namespace AppleVirt
} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_APPLE_VIRT_EXIT_DECODE_HH__
