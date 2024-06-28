/*
 * Copyright (c) 2018, 2019 ARM Limited
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

#include "arch/riscv/semihosting.hh"

#include <unistd.h>

#include <cerrno>
#include <cstdio>

#include "arch/riscv/isa.hh"
#include "arch/riscv/page_size.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/time.hh"
#include "cpu/exec_context.hh"
#include "debug/Semihosting.hh"
#include "dev/serial/serial.hh"
#include "mem/physical.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/RiscvSemihosting.hh"
#include "pcstate.hh"
#include "sim/byteswap.hh"
#include "sim/full_system.hh"
#include "sim/pseudo_inst.hh"
#include "sim/system.hh"

namespace gem5
{

const std::map<uint32_t, RiscvSemihosting::SemiCall> RiscvSemihosting::calls{
        {SYS_OPEN, {"SYS_OPEN", &RiscvSemihosting::callOpen}},
        {SYS_CLOSE, {"SYS_CLOSE", &RiscvSemihosting::callClose}},
        {SYS_WRITEC, {"SYS_WRITEC", &RiscvSemihosting::callWriteC}},
        {SYS_WRITE0, {"SYS_WRITE0", &RiscvSemihosting::callWrite0}},
        {SYS_WRITE, {"SYS_WRITE", &RiscvSemihosting::callWrite}},
        {SYS_READ, {"SYS_READ", &RiscvSemihosting::callRead}},
        {SYS_READC, {"SYS_READC", &RiscvSemihosting::callReadC}},
        {SYS_ISERROR, {"SYS_ISERROR", &RiscvSemihosting::callIsError}},
        {SYS_ISTTY, {"SYS_ISTTY", &RiscvSemihosting::callIsTTY}},
        {SYS_SEEK, {"SYS_SEEK", &RiscvSemihosting::callSeek}},
        {SYS_FLEN, {"SYS_FLEN", &RiscvSemihosting::callFLen}},
        {SYS_TMPNAM, {"SYS_TMPNAM", &RiscvSemihosting::callTmpNam}},
        {SYS_REMOVE, {"SYS_REMOVE", &RiscvSemihosting::callRemove}},
        {SYS_RENAME, {"SYS_RENAME", &RiscvSemihosting::callRename}},
        {SYS_CLOCK, {"SYS_CLOCK", &RiscvSemihosting::callClock}},
        {SYS_TIME, {"SYS_TIME", &RiscvSemihosting::callTime}},
        {SYS_SYSTEM, {"SYS_SYSTEM", &RiscvSemihosting::callSystem}},
        {SYS_ERRNO, {"SYS_ERRNO", &RiscvSemihosting::callErrno}},
        {SYS_GET_CMDLINE,
                {"SYS_GET_CMDLINE", &RiscvSemihosting::callGetCmdLine}},
        {SYS_HEAPINFO, {"SYS_HEAPINFO", &RiscvSemihosting::callHeapInfo32,
                               &RiscvSemihosting::callHeapInfo64}},

        {SYS_EXIT, {"SYS_EXIT", &RiscvSemihosting::callExit32,
                           &RiscvSemihosting::callExit64}},
        {SYS_EXIT_EXTENDED,
                {"SYS_EXIT_EXTENDED", &RiscvSemihosting::callExitExtended}},

        {SYS_ELAPSED, {"SYS_ELAPSED", &RiscvSemihosting::callElapsed32,
                              &RiscvSemihosting::callElapsed64}},
        {SYS_TICKFREQ, {"SYS_TICKFREQ", &RiscvSemihosting::callTickFreq}},
};

RiscvSemihosting::
RiscvSemihosting(const RiscvSemihostingParams &p) : BaseSemihosting(p)
{}

bool
RiscvSemihosting::call64(ThreadContext *tc)
{
    RegVal op = tc->getReg(RiscvISA::int_reg::A0) & mask(32);
    auto it = calls.find(op);
    if (it == calls.end()) {
        unrecognizedCall<Abi64>(tc, "Unknown semihosting call: op = 0x%x", op);
        return false;
    }
    const SemiCall &call = it->second;

    DPRINTF(Semihosting, "Semihosting call64: %s\n", call.dump64(tc));
    auto err = call.call64(this, tc);
    semiErrno = err.second;
    DPRINTF(Semihosting, "\t ->: 0x%x, %i\n", err.first, err.second);

    return true;
}

bool
RiscvSemihosting::call32(ThreadContext *tc)
{
    RegVal op = tc->getReg(RiscvISA::int_reg::A0);
    auto it = calls.find(op);
    if (it == calls.end()) {
        unrecognizedCall<Abi32>(
                tc, "Unknown aarch32 semihosting call: op = 0x%x", op);
        return false;
    }
    const SemiCall &call = it->second;

    DPRINTF(Semihosting, "Semihosting call32: %s\n", call.dump32(tc));
    auto err = call.call32(this, tc);
    semiErrno = err.second;
    DPRINTF(Semihosting, "\t ->: 0x%x, %i\n", err.first, err.second);

    return true;
}

bool
RiscvSemihosting::call(ThreadContext *tc)
{
    auto isa = dynamic_cast<RiscvISA::ISA *>(tc->getIsaPtr());
    panic_if(!isa, "Cannot derive rv_type from non-riscv isa");
    return isa->rvType() == enums::RV32 ? call32(tc) : call64(tc);
}

PortProxy &
RiscvSemihosting::portProxyImpl(ThreadContext *tc)
{
    static std::unique_ptr<PortProxy> port_proxy([=]() {
        return FullSystem ? new TranslatingPortProxy(tc) :
                            new SETranslatingPortProxy(tc);
    }());
    return *port_proxy;
}

bool
RiscvSemihosting::isSemihostingEBreak(ExecContext *xc)
{
    // Check if the surrounding bytes match the semihosting magic sequence.
    PortProxy& proxy = portProxyImpl(xc->tcBase());
    Addr PrevInstAddr = xc->pcState().instAddr() - 4;
    Addr NextInstAddr = xc->pcState().instAddr() + 4;
    if (roundDown(PrevInstAddr, RiscvISA::PageBytes) !=
        roundDown(NextInstAddr, RiscvISA::PageBytes)) {
      DPRINTF(Semihosting,
              "Ebreak cannot be a semihosting ebreak since previous "
              "and next instruction are on different pages\n");
      return false;
    }
    uint32_t instSequence[3];
    if (!proxy.tryReadBlob(PrevInstAddr, instSequence, sizeof(instSequence))) {
        DPRINTF(Semihosting,
                "Ebreak cannot be a semihosting ebreak since surrounding "
                "instructions at %#x cannot be accessed\n");
        return false;
    }
    uint32_t PrevInst = gtoh(instSequence[0], ByteOrder::little);
    uint32_t EBreakInst = gtoh(instSequence[1], ByteOrder::little);
    uint32_t NextInst = gtoh(instSequence[2], ByteOrder::little);
    DPRINTF(Semihosting,
            "Checking ebreak for semihosting: Prev=%#x EBreak=%#x Next=%#x\n",
            PrevInst, EBreakInst, NextInst);
    return PrevInst == (uint32_t)Opcode::Prefix &&
           EBreakInst == (uint32_t)Opcode::EBreak &&
           NextInst == (uint32_t)Opcode::Suffix;
}

} // namespace gem5
