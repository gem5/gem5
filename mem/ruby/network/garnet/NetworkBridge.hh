/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_NETWORK_GARNET_0_NETWORK_BRIDGE_HH__
#define __MEM_RUBY_NETWORK_GARNET_0_NETWORK_BRIDGE_HH__

#include <iostream>
#include <queue>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/CommonTypes.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/GarnetLink.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "mem/ruby/network/garnet/flitBuffer.hh"
#include "params/NetworkBridge.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

class GarnetNetwork;

class NetworkBridge: public CreditLink
{
  public:
    typedef NetworkBridgeParams Params;
    NetworkBridge(const Params &p);
    ~NetworkBridge();

    void initBridge(NetworkBridge *coBrid, bool cdc_en, bool serdes_en);

    void wakeup();
    void neutralize(int vc, int eCredit);

    void scheduleFlit(flit *t_flit, Cycles latency);
    void flitisizeAndSend(flit *t_flit);
    void setVcsPerVnet(uint32_t consumerVcs);

  protected:
    // Pointer to co-existing bridge
    // CreditBridge for Network Bridge and vice versa
    NetworkBridge *coBridge;

    // Link connected toBridge
    // could be a source or destination
    // depending on mType
    NetworkLink *nLink;

    // CDC enable/disable
    bool enCdc;
    // SerDes enable/disable
    bool enSerDes;

    // Type of Bridge
    int mType;

    Cycles cdcLatency;
    Cycles serDesLatency;

    Tick lastScheduledAt;

    // Used by Credit Deserializer
    std::vector<int> lenBuffer;
    std::vector<int> sizeSent;
    std::vector<int> flitsSent;
    std::vector<std::queue<int>> extraCredit;

};

} // namespace garnet
} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_GARNET_0_NETWORK_BRIDGE_HH__
