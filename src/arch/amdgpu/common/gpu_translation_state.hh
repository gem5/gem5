/*
 * Copyright (c) 2022 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_AMDGPU_COMMON_GPU_TRANSLATION_STATE_HH__
#define __ARCH_AMDGPU_COMMON_GPU_TRANSLATION_STATE_HH__

#include "arch/generic/mmu.hh"

namespace gem5
{

class ResponsePort;

/**
 * GPU TranslationState: this currently is a somewhat bastardization of
 * the usage of SenderState, whereby the receiver of a packet is not
 * usually supposed to need to look at the contents of the senderState,
 * you're really only supposed to look at what you pushed on, pop it
 * off, and send it back.
 *
 * However, since there is state that we want to pass to the TLBs using
 * the send/recv Timing/Functional/etc. APIs, which don't allow for new
 * arguments, we need a common TLB senderState to pass between TLBs,
 * both "forwards" and "backwards."
 *
 * So, basically, the rule is that any packet received by a TLB port
 * (cpuside OR memside) must be safely castable to a GpuTranslationState.
 */

struct GpuTranslationState : public Packet::SenderState
{
    // TLB mode, read or write
    BaseMMU::Mode tlbMode;
    // SE mode thread context associated with this req
    ThreadContext *tc;
    // FS mode related fields
    int deviceId;
    int pasId; // Process Address Space ID

    /*
    * TLB entry to be populated and passed back and filled in
    * previous TLBs.  Equivalent to the data cache concept of
    * "data return."
    */
    Serializable *tlbEntry;
    // Is this a TLB prefetch request?
    bool isPrefetch;
    // When was the req for this translation issued
    uint64_t issueTime;
    // Remember where this came from
    std::vector<ResponsePort*>ports;

    // keep track of #uncoalesced reqs per packet per TLB level;
    // reqCnt per level >= reqCnt higher level
    std::vector<int> reqCnt;
    // TLB level this packet hit in; 0 if it hit in the page table
    int hitLevel;
    Packet::SenderState *saved;

    GpuTranslationState(BaseMMU::Mode tlb_mode, ThreadContext *_tc,
                        bool _prefetch=false,
                        Packet::SenderState *_saved=nullptr)
        : tlbMode(tlb_mode), tc(_tc), deviceId(0), pasId(0), tlbEntry(nullptr),
          isPrefetch(_prefetch), issueTime(0), hitLevel(0), saved(_saved)
    { }

    GpuTranslationState(BaseMMU::Mode tlb_mode,
                       bool _prefetch=false,
                       Packet::SenderState *_saved=nullptr)
        : tlbMode(tlb_mode), tc(nullptr), deviceId(0), pasId(0),
          tlbEntry(nullptr), isPrefetch(_prefetch), issueTime(0), hitLevel(0),
          saved(_saved)
    { }
};

} // namespace gem5

#endif // __ARCH_AMDGPU_COMMON_GPU_TRANSLATION_STATE_HH__
