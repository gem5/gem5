/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/**
 * @file
 * Declaration of the Packet Class, a packet is a transaction occuring
 * between a single level of the memory heirarchy (ie L1->L2).
 */

#ifndef __MEM_PACKET_HH__
#define __MEM_PACKET_HH__

#include "mem/request.hh"
#include "arch/isa_traits.hh"
#include "sim/root.hh"

struct Packet;
typedef Packet* PacketPtr;
typedef uint8_t* PacketDataPtr;

/** List of all commands associated with a packet. */
enum Command
{
    Read,
    Write
};

/** The result of a particular pakets request. */
enum PacketResult
{
    Success,
    BadAddress,
    Unknown
};

class SenderState{};
class Coherence{};

/**
 * A Packet is the structure to handle requests between two levels
 * of the memory system.  The Request is a global object that trancends
 * all of the memory heirarchy, but at each levels interface a packet
 * is created to transfer data/requests.  For example, a request would
 * be used to initiate a request to go to memory/IOdevices, as the request
 * passes through the memory system several packets will be created.  One
 * will be created to go between the L1 and L2 caches and another to go to
 * the next level and so forth.
 *
 * Packets are assumed to be returned in the case of a single response.  If
 * the transaction has no response, then the consumer will delete the packet.
 */
struct Packet
{
    /** The address of the request, could be virtual or physical (depending on
        cache configurations). */
    Addr addr;

    /** Flag structure to hold flags for this particular packet */
    uint64_t flags;

    /** A pointer to the overall request. */
    RequestPtr req;

    /** A virtual base opaque structure used to hold
        coherence status messages. */
    Coherence *coherence;  // virtual base opaque,
                           // assert(dynamic_cast<Foo>) etc.

    /** A virtual base opaque structure used to hold the senders state. */
    SenderState *senderState; // virtual base opaque,
                           // assert(dynamic_cast<Foo>) etc.

    /** A pointer to the data being transfered.  It can be differnt sizes
        at each level of the heirarchy so it belongs in the packet,
        not request*/
    PacketDataPtr data;

    /** Indicates the size of the request. */
    int size;

    /** A index of the source of the transaction. */
    short src;

    /** A index to the destination of the transaction. */
    short dest;

    /** The command of the transaction. */
    Command cmd;

    /** The result of the packet transaction. */
    PacketResult result;

    /** Accessor function that returns the source index of the packet. */
    short getSrc() const { return src; }

    /** Accessor function that returns the destination index of
        the packet. */
    short getDest() const { return dest; }
};

#endif //__MEM_PACKET_HH
