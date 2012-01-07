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
 * These are the functions that exported to slicc from ruby.
 */

#ifndef __MEM_RUBY_SLICC_INTERFACE_RUBYSLICCUTIL_HH__
#define __MEM_RUBY_SLICC_INTERFACE_RUBYSLICCUTIL_HH__

#include <cassert>

#include "mem/protocol/AccessType.hh"
#include "mem/protocol/GenericRequestType.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/MessageSizeType.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/ruby/system/System.hh"

class Set;
class NetDest;

inline int
random(int n)
{
  return random() % n;
}

inline bool
multicast_retry()
{
    if (RubySystem::getRandomization()) {
        return (random() & 0x1);
    } else {
        return true;
    }
}

inline Time
get_time()
{
    return g_eventQueue_ptr->getTime();
}

inline Time
zero_time()
{
    return 0;
}

inline NodeID
intToID(int nodenum)
{
    NodeID id = nodenum;
    return id;
}

inline int
IDToInt(NodeID id)
{
    int nodenum = id;
    return nodenum;
}

inline int
addressToInt(Address addr)
{
    return (int)addr.getLineAddress();
}

inline bool
long_enough_ago(Time event)
{
    return ((get_time() - event) > 200);
}

inline int
getAddThenMod(int addend1, int addend2, int modulus)
{
    return (addend1 + addend2) % modulus;
}

inline Time
getTimeModInt(Time time, int modulus)
{
    return time % modulus;
}

inline Time
getTimePlusInt(Time addend1, int addend2)
{
    return (Time) addend1 + addend2;
}

inline Time
getTimeMinusTime(Time t1, Time t2)
{
    assert(t1 >= t2);
    return t1 - t2;
}

inline Time
getPreviousDelayedCycles(Time t1, Time t2)
{
    if (RubySystem::getRandomization()) {  // when randomizing delayed
        return 0;
    } else {
        return getTimeMinusTime(t1, t2);
    }
}

// Return type for time_to_int is "Time" and not "int" so we get a
// 64-bit integer
inline Time
time_to_int(Time time)
{
    return time;
}

// Appends an offset to an address
inline Address
setOffset(Address addr, int offset)
{
    Address result = addr;
    result.setOffset(offset);
    return result;
}

// Makes an address into a line address
inline Address
makeLineAddress(Address addr)
{
    Address result = addr;
    result.makeLineAddress();
    return result;
}

inline int
addressOffset(Address addr)
{
    return addr.getOffset();
}

inline int
mod(int val, int mod)
{
    return val % mod;
}

#endif // __MEM_RUBY_SLICC_INTERFACE_RUBYSLICCUTIL_HH__
