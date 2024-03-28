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

#include "arch/arm/semihosting.hh"

#include <unistd.h>

#include <cerrno>
#include <cstdio>

#include "arch/arm/utility.hh"
#include "base/output.hh"
#include "base/time.hh"
#include "debug/Semihosting.hh"
#include "dev/serial/serial.hh"
#include "mem/physical.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/ArmSemihosting.hh"
#include "sim/byteswap.hh"
#include "sim/full_system.hh"
#include "sim/pseudo_inst.hh"
#include "sim/system.hh"

namespace gem5
{

const std::map<uint32_t, ArmSemihosting::SemiCall> ArmSemihosting::calls{
    { SYS_OPEN,     { "SYS_OPEN", &ArmSemihosting::callOpen } },
    { SYS_CLOSE,    { "SYS_CLOSE", &ArmSemihosting::callClose } },
    { SYS_WRITEC,   { "SYS_WRITEC", &ArmSemihosting::callWriteC } },
    { SYS_WRITE0,   { "SYS_WRITE0", &ArmSemihosting::callWrite0 } },
    { SYS_WRITE,    { "SYS_WRITE", &ArmSemihosting::callWrite } },
    { SYS_READ,     { "SYS_READ", &ArmSemihosting::callRead } },
    { SYS_READC,    { "SYS_READC", &ArmSemihosting::callReadC } },
    { SYS_ISERROR,  { "SYS_ISERROR", &ArmSemihosting::callIsError } },
    { SYS_ISTTY,    { "SYS_ISTTY", &ArmSemihosting::callIsTTY } },
    { SYS_SEEK,     { "SYS_SEEK", &ArmSemihosting::callSeek } },
    { SYS_FLEN,     { "SYS_FLEN", &ArmSemihosting::callFLen } },
    { SYS_TMPNAM,   { "SYS_TMPNAM", &ArmSemihosting::callTmpNam } },
    { SYS_REMOVE,   { "SYS_REMOVE", &ArmSemihosting::callRemove } },
    { SYS_RENAME,   { "SYS_RENAME", &ArmSemihosting::callRename } },
    { SYS_CLOCK,    { "SYS_CLOCK", &ArmSemihosting::callClock } },
    { SYS_TIME,     { "SYS_TIME", &ArmSemihosting::callTime } },
    { SYS_SYSTEM,   { "SYS_SYSTEM", &ArmSemihosting::callSystem } },
    { SYS_ERRNO,    { "SYS_ERRNO", &ArmSemihosting::callErrno } },
    { SYS_GET_CMDLINE,
        { "SYS_GET_CMDLINE", &ArmSemihosting::callGetCmdLine } },
    { SYS_HEAPINFO, { "SYS_HEAPINFO", &ArmSemihosting::callHeapInfo32,
                                      &ArmSemihosting::callHeapInfo64 } },

    { SYS_EXIT,     { "SYS_EXIT", &ArmSemihosting::callExit32,
                                  &ArmSemihosting::callExit64} },
    { SYS_EXIT_EXTENDED,
        { "SYS_EXIT_EXTENDED", &ArmSemihosting::callExitExtended } },

    { SYS_ELAPSED,  { "SYS_ELAPSED", &ArmSemihosting::callElapsed32,
                                     &ArmSemihosting::callElapsed64 } },
    { SYS_TICKFREQ, { "SYS_TICKFREQ", &ArmSemihosting::callTickFreq } },
    { SYS_GEM5_PSEUDO_OP,
        { "SYS_GEM5_PSEUDO_OP", &ArmSemihosting::callGem5PseudoOp32,
                                &ArmSemihosting::callGem5PseudoOp64 } },
};

ArmSemihosting::ArmSemihosting(const ArmSemihostingParams &p)
    : BaseSemihosting(p) {}

bool
ArmSemihosting::call64(ThreadContext *tc, bool gem5_ops)
{
    RegVal op = tc->getReg(ArmISA::int_reg::X0) & mask(32);
    if (op > MaxStandardOp && !gem5_ops) {
        unrecognizedCall<Abi64>(
                tc, "Gem5 semihosting op (0x%x) disabled from here.", op);
        return false;
    }

    auto it = calls.find(op);
    if (it == calls.end()) {
        unrecognizedCall<Abi64>(
                tc, "Unknown aarch64 semihosting call: op = 0x%x", op);
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
ArmSemihosting::call32(ThreadContext *tc, bool gem5_ops)
{
    RegVal op = tc->getReg(ArmISA::int_reg::R0);
    if (op > MaxStandardOp && !gem5_ops) {
        unrecognizedCall<Abi32>(
                tc, "Gem5 semihosting op (0x%x) disabled from here.", op);
        return false;
    }

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

PortProxy &
ArmSemihosting::portProxyImpl(ThreadContext *tc)
{
    static std::unique_ptr<PortProxy> port_proxy_s;
    static std::unique_ptr<PortProxy> port_proxy_ns;
    static System *secure_sys = nullptr;

    if (ArmISA::isSecure(tc)) {
        System *sys = tc->getSystemPtr();
        if (sys != secure_sys) {
            if (FullSystem) {
                port_proxy_s.reset(
                        new TranslatingPortProxy(tc, Request::SECURE));
            } else {
                port_proxy_s.reset(
                        new SETranslatingPortProxy(
                            tc, SETranslatingPortProxy::NextPage,
                            Request::SECURE));
            }
        }
        secure_sys = sys;
        return *port_proxy_s;
    } else {
        if (!port_proxy_ns) {
            if (FullSystem) {
                port_proxy_ns.reset(new TranslatingPortProxy(tc));
            } else {
                port_proxy_ns.reset(new SETranslatingPortProxy(tc));
            }
        }

        return *port_proxy_ns;
    }
}

struct SemiPseudoAbi32 : public ArmSemihosting::Abi32
{
    class State : public ArmSemihosting::Abi32::State
    {
      public:
        State(const ThreadContext *tc) : ArmSemihosting::Abi32::State(tc)
        {
            // Use getAddr() to skip the func number in the first slot.
            getAddr();
        }
    };
};

struct SemiPseudoAbi64 : public ArmSemihosting::Abi64
{
    class State : public ArmSemihosting::Abi64::State
    {
      public:
        State(const ThreadContext *tc) : ArmSemihosting::Abi64::State(tc)
        {
            // Use getAddr() to skip the func number in the first slot.
            getAddr();
        }
    };
};

namespace guest_abi
{

// Handle arguments the same as for semihosting operations. Skipping the first
// slot is handled internally by the State type.
template <typename T>
struct Argument<SemiPseudoAbi32, T> :
    public Argument<ArmSemihosting::Abi32, T>
{};
template <typename T>
struct Argument<SemiPseudoAbi64, T> :
    public Argument<ArmSemihosting::Abi64, T>
{};

} // namespace guest_abi

ArmSemihosting::RetErrno
ArmSemihosting::callGem5PseudoOp32(ThreadContext *tc, uint32_t encoded_func)
{
    uint8_t func;
    pseudo_inst::decodeAddrOffset(encoded_func, func);

    uint64_t ret;
    if (pseudo_inst::pseudoInst<SemiPseudoAbi32>(tc, func, ret))
        return retOK(ret);
    else
        return retError(EINVAL);
}

ArmSemihosting::RetErrno
ArmSemihosting::callGem5PseudoOp64(ThreadContext *tc, uint64_t encoded_func)
{
    uint8_t func;
    pseudo_inst::decodeAddrOffset(encoded_func, func);

    uint64_t ret;
    if (pseudo_inst::pseudoInst<SemiPseudoAbi64>(tc, func, ret))
        return retOK(ret);
    else
        return retError(EINVAL);
}

} // namespace gem5
