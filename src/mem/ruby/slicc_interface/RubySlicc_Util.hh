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

#include "debug/RubySlicc.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/packet.hh"

inline int
random(int n)
{
  return random() % n;
}

inline Cycles zero_time() { return Cycles(0); }
inline Cycles TimeToCycles(Time t) { return Cycles(t); }

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

inline int max_tokens()
{
  return 1024;
}

/**
 * This function accepts an address, a data block and a packet. If the address
 * range for the data block contains the address which the packet needs to
 * read, then the data from the data block is written to the packet. True is
 * returned if the data block was read, otherwise false is returned.
 */
inline bool
testAndRead(Address addr, DataBlock& blk, Packet *pkt)
{
    Address pktLineAddr(pkt->getAddr());
    pktLineAddr.makeLineAddress();

    Address lineAddr = addr;
    lineAddr.makeLineAddress();

    if (pktLineAddr == lineAddr) {
        uint8_t *data = pkt->getPtr<uint8_t>(true);
        unsigned int size_in_bytes = pkt->getSize();
        unsigned startByte = pkt->getAddr() - lineAddr.getAddress();

        for (unsigned i = 0; i < size_in_bytes; ++i) {
            data[i] = blk.getByte(i + startByte);
        }
        return true;
    }
    return false;
}

/**
 * This function accepts an address, a data block and a packet. If the address
 * range for the data block contains the address which the packet needs to
 * write, then the data from the packet is written to the data block. True is
 * returned if the data block was written, otherwise false is returned.
 */
inline bool
testAndWrite(Address addr, DataBlock& blk, Packet *pkt)
{
    Address pktLineAddr(pkt->getAddr());
    pktLineAddr.makeLineAddress();

    Address lineAddr = addr;
    lineAddr.makeLineAddress();

    if (pktLineAddr == lineAddr) {
        uint8_t *data = pkt->getPtr<uint8_t>(true);
        unsigned int size_in_bytes = pkt->getSize();
        unsigned startByte = pkt->getAddr() - lineAddr.getAddress();

        for (unsigned i = 0; i < size_in_bytes; ++i) {
            blk.setByte(i + startByte, data[i]);
        }
        return true;
    }
    return false;
}

#endif // __MEM_RUBY_SLICC_INTERFACE_RUBYSLICCUTIL_HH__
