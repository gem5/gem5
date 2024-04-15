/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "sim/syscall_desc.hh"

#include "base/types.hh"
#include "sim/eventq.hh"
#include "sim/syscall_debug_macros.hh"

namespace gem5
{

class ThreadContext;

void
SyscallDesc::doSyscall(ThreadContext *tc)
{
    DPRINTF_SYSCALL(Base, "Calling %s...\n", dumper(name(), tc));

    SyscallReturn retval = executor(this, tc);

    if (retval.needsRetry()) {
        // Suspend this ThreadContext while the syscall is pending.
        tc->suspend();

        DPRINTF_SYSCALL(Base, "%s needs retry.\n", name());
        setupRetry(tc);
        return;
    }

    handleReturn(tc, retval);
}

void
SyscallDesc::retrySyscall(ThreadContext *tc)
{
    DPRINTF_SYSCALL(Base, "Retrying %s...\n", dumper(name(), tc));

    SyscallReturn retval = executor(this, tc);

    if (retval.needsRetry()) {
        DPRINTF_SYSCALL(Base, "%s still needs retry.\n", name());
        setupRetry(tc);
        return;
    }

    // We're done retrying, so reactivate this ThreadContext.
    tc->activate();

    handleReturn(tc, retval);
}

void
SyscallDesc::setupRetry(ThreadContext *tc)
{
    // Create an event which will retry the system call later.
    auto retry = [this, tc]() { retrySyscall(tc); };
    auto *event = new EventFunctionWrapper(retry, name(), true);

    // Schedule it in about 100 CPU cycles. That will give other contexts
    // a chance to execute a bit of code before trying again.
    auto *cpu = tc->getCpuPtr();
    curEventQueue()->schedule(event,
                              curTick() + cpu->cyclesToTicks(Cycles(100)));
}

void
SyscallDesc::handleReturn(ThreadContext *tc, const SyscallReturn &ret)
{
    if (ret.suppressed()) {
        DPRINTF_SYSCALL(Base, "No return value.\n", name());
    } else {
        returnInto(tc, ret);
        DPRINTF_SYSCALL(Base, "Returned %d.\n", ret.encodedValue());
    }
}

} // namespace gem5
