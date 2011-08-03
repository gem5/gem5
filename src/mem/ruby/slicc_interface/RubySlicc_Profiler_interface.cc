/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/profiler/AddressProfiler.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Profiler_interface.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

void
profile_request(const string& L1CacheState, const string& L2CacheState,
                const string& directoryState, const string& requestType)
{
    string requestStr = L1CacheState + ":" + L2CacheState + ":" +
        directoryState + ":" + requestType;

    g_system_ptr->getProfiler()->profileRequest(requestStr);
}

void
profile_outstanding_request(int outstanding)
{
    g_system_ptr->getProfiler()->profileOutstandingRequest(outstanding);
}

void
profile_average_latency_estimate(int latency)
{
    g_system_ptr->getProfiler()->profileAverageLatencyEstimate(latency);
}

void
profile_sharing(const Address& addr, AccessType type, NodeID requestor,
                const Set& sharers, const Set& owner)
{
    g_system_ptr->getProfiler()->
        profileSharing(addr, type, requestor, sharers, owner);
}

void
profileMsgDelay(int virtualNetwork, int delayCycles)
{
    g_system_ptr->getProfiler()->profileMsgDelay(virtualNetwork, delayCycles);
}

void
profileGetX(const Address& datablock, const Address& PC, const Set& owner,
            const Set& sharers, NodeID requestor)
{
    g_system_ptr->getProfiler()->getAddressProfiler()->
        profileGetX(datablock, PC, owner, sharers, requestor);
}

void
profileGetS(const Address& datablock, const Address& PC, const Set& owner,
            const Set& sharers, NodeID requestor)
{
    g_system_ptr->getProfiler()->getAddressProfiler()->
        profileGetS(datablock, PC, owner, sharers, requestor);
}



