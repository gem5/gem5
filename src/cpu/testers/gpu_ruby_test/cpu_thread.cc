/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/testers/gpu_ruby_test/cpu_thread.hh"

#include "debug/ProtocolTest.hh"

namespace gem5
{

CpuThread::CpuThread(const Params &p) : TesterThread(p)
{
    threadName = "CpuThread(Thread ID " + std::to_string(threadId) + ")";
    threadEvent.setDesc("CpuThread tick");
    assert(numLanes == 1);
}

void
CpuThread::issueLoadOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::LOAD);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    fatal("CpuThread::issueLoadOps - not yet implemented");
}

void
CpuThread::issueStoreOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::STORE);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    fatal("CpuThread::issueStoreOps - not yet implemented");
}

void
CpuThread::issueAtomicOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ATOMIC);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    fatal("CpuThread::issueAtomicOps - not yet implemented");
}

void
CpuThread::issueAcquireOp()
{
    DPRINTF(ProtocolTest, "Issuing Acquire Op ...\n");

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ACQUIRE);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // no-op: Acquire does not apply to CPU threads
}

void
CpuThread::issueReleaseOp()
{
    DPRINTF(ProtocolTest, "Issuing Release Op ...\n");

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::RELEASE);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // no-op: Release does not apply to CPU threads
}

void
CpuThread::hitCallback(PacketPtr pkt)
{
    fatal("CpuThread::hitCallback - not yet implemented");
}

} // namespace gem5
