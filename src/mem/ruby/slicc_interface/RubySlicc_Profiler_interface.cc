
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

/*
 * slicc_util.C
 *
 * Description: See slicc_util.h
 *
 * $Id$
 *
 */

#include "Global.hh"
#include "System.hh"
#include "Profiler.hh"
#include "AddressProfiler.hh"
#include "Protocol.hh"
#include "RubySlicc_Profiler_interface.hh"
#include "RubySlicc_ComponentMapping.hh"
// #include "TransactionInterfaceManager.hh"

void profile_request(int cache_state, Directory_State directory_state, GenericRequestType request_type)
{
  string requestStr = L1Cache_State_to_string(L1Cache_State(cache_state))+":"+
    Directory_State_to_string(directory_state)+":"+
    GenericRequestType_to_string(request_type);
  g_system_ptr->getProfiler()->profileRequest(requestStr);
}

void profile_request(const string& L1CacheState, const string& L2CacheState, const string& directoryState, const string& requestType)
{
  string requestStr = L1CacheState+":"+L2CacheState+":"+directoryState+":"+requestType;
  g_system_ptr->getProfiler()->profileRequest(requestStr);
}

void profile_outstanding_request(int outstanding)
{
  g_system_ptr->getProfiler()->profileOutstandingRequest(outstanding);
}

void profile_outstanding_persistent_request(int outstanding)
{
  g_system_ptr->getProfiler()->profileOutstandingPersistentRequest(outstanding);
}

void profile_average_latency_estimate(int latency)
{
  g_system_ptr->getProfiler()->profileAverageLatencyEstimate(latency);
}

void profile_sharing(const Address& addr, AccessType type, NodeID requestor, const Set& sharers, const Set& owner)
{
  g_system_ptr->getProfiler()->profileSharing(addr, type, requestor, sharers, owner);
}

void profile_miss(const CacheMsg& msg, NodeID id)
{
  // CMP profile address after L1 misses, not L2
  ASSERT (!Protocol::m_CMP);
  g_system_ptr->getProfiler()->addAddressTraceSample(msg, id);

  g_system_ptr->getProfiler()->profileConflictingRequests(msg.getAddress());

  g_system_ptr->getProfiler()->addSecondaryStatSample(msg.getType(),
                                                      msg.getAccessMode(), msg.getSize(), msg.getPrefetch(), id);
}

void profile_L1Cache_miss(const CacheMsg& msg, NodeID id)
{
  // only called by protocols assuming non-zero cycle hits
  ASSERT (REMOVE_SINGLE_CYCLE_DCACHE_FAST_PATH);

  g_system_ptr->getProfiler()->addPrimaryStatSample(msg, id);
}

void profileMsgDelay(int virtualNetwork, int delayCycles)
{
  g_system_ptr->getProfiler()->profileMsgDelay(virtualNetwork, delayCycles);
}

void profile_L2Cache_miss(GenericRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID nodeID)
{
  g_system_ptr->getProfiler()->addSecondaryStatSample(requestType, type, msgSize, pfBit, nodeID);
}

void profile_token_retry(const Address& addr, AccessType type, int count)
{
  g_system_ptr->getProfiler()->getAddressProfiler()->profileRetry(addr, type, count);
}

void profile_filter_action(int action)
{
  g_system_ptr->getProfiler()->profileFilterAction(action);
}

void profile_persistent_prediction(const Address& addr, AccessType type)
{
  g_system_ptr->getProfiler()->getAddressProfiler()->profilePersistentPrediction(addr, type);
}

void profile_multicast_retry(const Address& addr, int count)
{
  g_system_ptr->getProfiler()->profileMulticastRetry(addr, count);
}

void profileGetX(const Address& datablock, const Address& PC, const Set& owner, const Set& sharers, NodeID requestor)
{
  g_system_ptr->getProfiler()->getAddressProfiler()->profileGetX(datablock, PC, owner, sharers, requestor);
}

void profileGetS(const Address& datablock, const Address& PC, const Set& owner, const Set& sharers, NodeID requestor)
{
  g_system_ptr->getProfiler()->getAddressProfiler()->profileGetS(datablock, PC, owner, sharers, requestor);
}

void profileOverflow(const Address & addr, MachineID mach)
{
  if(mach.type == MACHINETYPE_L1CACHE_ENUM){
    // for L1 overflows
    int proc_num = L1CacheMachIDToProcessorNum(mach);
    int chip_num = proc_num/RubyConfig::numberOfProcsPerChip();
    assert(0);
    // g_system_ptr->getChip(chip_num)->m_L1Cache_xact_mgr_vec[proc_num]->profileOverflow(addr, true);
  }
  else if(mach.type == MACHINETYPE_L2CACHE_ENUM){
    // for L2 overflows
    int chip_num = L2CacheMachIDToChipID(mach);
    for(int p=0; p < RubyConfig::numberOfProcessors(); ++p){
      assert(0);
      // g_system_ptr->getChip(chip_num)->m_L1Cache_xact_mgr_vec[p]->profileOverflow(addr, false);
    }
  }
}


