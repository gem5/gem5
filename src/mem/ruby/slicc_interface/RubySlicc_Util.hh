
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
 * slicc_util.hh
 *
 * Description: These are the functions that exported to slicc from ruby.
 *
 * $Id$
 *
 */

#ifndef SLICC_UTIL_H
#define SLICC_UTIL_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/protocol/GenericRequestType.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/protocol/AccessType.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/Directory_State.hh"
#include "mem/protocol/L1Cache_State.hh"
#include "mem/protocol/MessageSizeType.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/ruby/system/System.hh"

#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"

class Set;
class NetDest;

extern inline int random(int n)
{
  return random() % n;
}

extern inline bool multicast_retry()
{
  if (RubySystem::getRandomization()) {
    return (random() & 0x1);
  } else {
    return true;
  }
}

extern inline int cache_state_to_int(L1Cache_State state)
{
  return state;
}

extern inline Time get_time()
{
  return g_eventQueue_ptr->getTime();
}

extern inline Time zero_time()
{
  return 0;
}

extern inline NodeID intToID(int nodenum)
{
  NodeID id = nodenum;
  return id;
}

extern inline int IDToInt(NodeID id)
{
  int nodenum = id;
  return nodenum;
}

extern inline int addressToInt(Address addr)
{
  return (int) addr.getLineAddress();
}

extern inline bool long_enough_ago(Time event)
{
  return ((get_time() - event) > 200);
}

extern inline int getAddThenMod(int addend1, int addend2, int modulus)
{
  return (addend1 + addend2) % modulus;
}

extern inline Time getTimeModInt(Time time, int modulus)
{
  return time % modulus;
}

extern inline Time getTimePlusInt(Time addend1, int addend2)
{
  return (Time) addend1 + addend2;
}

extern inline Time getTimeMinusTime(Time t1, Time t2)
{
  ASSERT(t1 >= t2);
  return t1 - t2;
}

extern inline Time getPreviousDelayedCycles(Time t1, Time t2)
{
  if (RubySystem::getRandomization()) {  // when randomizing delayed
    return 0;
  } else {
    return getTimeMinusTime(t1, t2);
  }
}

extern inline void WARN_ERROR_TIME(Time time)
{
  WARN_EXPR(time);
}

// Return type for time_to_int is "Time" and not "int" so we get a 64-bit integer
extern inline Time time_to_int(Time time)
{
  return time;
}

// Appends an offset to an address
extern inline Address setOffset(Address addr, int offset)
{
  Address result = addr;
  result.setOffset(offset);
  return result;
}

// Makes an address into a line address
extern inline Address makeLineAddress(Address addr)
{
  Address result = addr;
  result.makeLineAddress();
  return result;
}

extern inline int addressOffset(Address addr)
{
  return addr.getOffset();
}

#endif //SLICC_UTIL_H
