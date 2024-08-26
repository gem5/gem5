/*
 * Copyright (c) 2024 The University of Wisconsin
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

#include "mem/ruby/system/VIPERSequencer.hh"

#include "debug/RubyHitMiss.hh"
#include "debug/RubySequencer.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "params/VIPERSequencer.hh"

namespace gem5
{

namespace ruby
{

VIPERSequencer::VIPERSequencer(const Params &p)
    : Sequencer(p)
{
}


VIPERSequencer::~VIPERSequencer()
{
}

void
VIPERSequencer::hitCallback(SequencerRequest* srequest, DataBlock& data,
                            bool llscSuccess,
                            const MachineType mach, const bool externalHit,
                            const Cycles initialRequestTime,
                            const Cycles forwardRequestTime,
                            const Cycles firstResponseTime,
                            const bool was_coalesced)
{
    if (srequest->m_type != RubyRequestType_hasNoAddr) {
        return Sequencer::hitCallback(
            srequest, data, llscSuccess, mach, externalHit, initialRequestTime,
            forwardRequestTime, firstResponseTime, was_coalesced);
    }

    PacketPtr pkt = srequest->pkt;

    assert(!was_coalesced);
    DPRINTF(RubySequencer, "Setting hasNoAddr ticks\n");
    Cycles curCycle =
        pkt->findNextSenderState
        <ComputeUnit::ScalarDataPort::SenderState>()
        ->_gpuDynInst->computeUnit()->curCycle();
    pkt->setData((const uint8_t *)&curCycle);

    // If using the RubyTester, update the RubyTester sender state's
    // subBlock with the recieved data.  The tester will later access
    // this state.
    assert(!m_usingRubyTester);
    assert(!RubySystem::getWarmupEnabled());
    assert(!RubySystem::getCooldownEnabled());
    ruby_hit_callback(pkt);
    testDrainComplete();
}

bool
VIPERSequencer::processReadCallback(SequencerRequest &seq_req,
                                    DataBlock& data,
                                    const bool ruby_request,
                                    bool externalHit,
                                    const MachineType mach,
                                    Cycles initialRequestTime,
                                    Cycles forwardRequestTime,
                                    Cycles firstResponseTime)
{
    if (seq_req.m_type != RubyRequestType_hasNoAddr) {
        return Sequencer::processReadCallback(
            seq_req, data, ruby_request, externalHit, mach, initialRequestTime,
            forwardRequestTime, firstResponseTime);
    }
    return false;
}

} // namespace ruby
} // namespace gem5
