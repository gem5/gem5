/*
 * Copyright (c) 2012, 2018 ARM Limited
 * All rights reserved.
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

#include "cpu/simple/noncaching.hh"

#include <cassert>

#include "arch/generic/decoder.hh"

namespace gem5
{

NonCachingSimpleCPU::NonCachingSimpleCPU(
        const BaseNonCachingSimpleCPUParams &p)
    : AtomicSimpleCPU(p)
{
    assert(p.numThreads == 1);
    fatal_if(!FullSystem && p.workload.size() != 1,
             "only one workload allowed");
}

void
NonCachingSimpleCPU::verifyMemoryMode() const
{
    if (!(system->isAtomicMode() && system->bypassCaches())) {
        fatal("The direct CPU requires the memory system to be in the "
              "'atomic_noncaching' mode.\n");
    }
}

Tick
NonCachingSimpleCPU::sendPacket(RequestPort &port, const PacketPtr &pkt)
{
    MemBackdoorPtr bd = nullptr;
    Tick latency = port.sendAtomicBackdoor(pkt, bd);

    // If the target gave us a backdoor for next time and we didn't
    // already have it, record it.
    if (bd && memBackdoors.insert(bd->range(), bd) != memBackdoors.end()) {
        // Install a callback to erase this backdoor if it goes away.
        auto callback = [this](const MemBackdoor &backdoor) {
                for (auto it = memBackdoors.begin();
                        it != memBackdoors.end(); it++) {
                    if (it->second == &backdoor) {
                        memBackdoors.erase(it);
                        return;
                    }
                }
                panic("Got invalidation for unknown memory backdoor.");
            };
        bd->addInvalidationCallback(callback);
    }
    return latency;
}

Tick
NonCachingSimpleCPU::fetchInstMem()
{
    auto bd_it = memBackdoors.contains(ifetch_req->getPaddr());
    if (bd_it == memBackdoors.end())
        return AtomicSimpleCPU::fetchInstMem();

    auto &decoder = threadInfo[curThread]->thread->decoder;

    auto *bd = bd_it->second;
    Addr offset = ifetch_req->getPaddr() - bd->range().start();
    memcpy(decoder->moreBytesPtr(), bd->ptr() + offset, ifetch_req->getSize());
    return 0;
}

} // namespace gem5
