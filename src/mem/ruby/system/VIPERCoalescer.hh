/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#ifndef __MEM_RUBY_SYSTEM_VIPERCOALESCER_HH__
#define __MEM_RUBY_SYSTEM_VIPERCOALESCER_HH__

#include <iostream>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"
#include "mem/ruby/system/GPUCoalescer.hh"
#include "mem/ruby/system/RubyPort.hh"

class DataBlock;
class CacheMsg;
class MachineID;
class CacheMemory;

class VIPERCoalescerParams;

class VIPERCoalescer : public GPUCoalescer
{
  public:
    typedef VIPERCoalescerParams Params;
    VIPERCoalescer(const Params *);
    ~VIPERCoalescer();
    void writeCompleteCallback(Addr address, uint64_t instSeqNum);
    void invTCPCallback(Addr address);
    RequestStatus makeRequest(PacketPtr pkt) override;
    void issueRequest(CoalescedRequest* crequest) override;

  private:
    void invTCP();

    // make write-complete response packets from original write request packets
    void makeWriteCompletePkts(CoalescedRequest* crequest);

    // current cache invalidation packet
    // nullptr if there is no active cache invalidation request
    PacketPtr m_cache_inv_pkt;

    // number of remaining cache lines to be invalidated in TCP
    int m_num_pending_invs;

    // a map of instruction sequence number and corresponding pending
    // write-complete response packets. Each write-complete response
    // corresponds to a pending store request that is waiting for
    // writeCompleteCallback. We may have multiple pending store requests per
    // wavefront at a time. Each time writeCompleteCallback is called, an entry
    // with a corresponding seqNum is popped off from map and returned to
    // compute unit.
    std::unordered_map<uint64_t, std::vector<PacketPtr>> m_writeCompletePktMap;
};
#endif //__MEM_RUBY_SYSTEM_VIPERCOALESCER_HH__
