/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
class Packet
{
  private:
   /** A pointer to the data being transfered.  It can be differnt sizes
        at each level of the heirarchy so it belongs in the packet,
        not request. This may or may not be populated when a responder recieves
        the packet. If not populated it memory should be allocated.
    */
    PacketDataPtr data;

    /** Is the data pointer set to a value that shouldn't be freed when the
     * packet is destroyed? */
    bool staticData;
    /** The data pointer points to a value that should be freed when the packet
     * is destroyed. */
    bool dynamicData;
    /** the data pointer points to an array (thus delete [] ) needs to be called
     * on it rather than simply delete.*/
    bool arrayData;


    /** The address of the request, could be virtual or physical (depending on
        cache configurations). */
    Addr addr;

     /** Indicates the size of the request. */
    int size;

    /** A index of the source of the transaction. */
    short src;

    /** A index to the destination of the transaction. */
    short dest;

    bool addrValid;
    bool sizeValid;
    bool srcValid;

  public:

    static const short Broadcast = -1;

    /** A pointer to the overall request. */
    RequestPtr req;

    class CoherenceState {
      public:
        virtual ~CoherenceState() {}
    };

    /** A virtual base opaque structure used to hold
        coherence status messages. */
    CoherenceState *coherence;  // virtual base opaque,
                           // assert(dynamic_cast<Foo>) etc.

    class SenderState {
      public:
        virtual ~SenderState() {}
    };

    /** A virtual base opaque structure used to hold the senders state. */
    SenderState *senderState; // virtual base opaque,
    // assert(dynamic_cast<Foo>) etc.

  private:
    /** List of command attributes. */
    enum CommandAttribute
    {
        IsRead		= 1 << 0,
        IsWrite		= 1 << 1,
        IsPrefetch	= 1 << 2,
        IsInvalidate	= 1 << 3,
        IsRequest	= 1 << 4,
        IsResponse 	= 1 << 5,
        NeedsResponse	= 1 << 6,
    };

  public:
    /** List of all commands associated with a packet. */
    enum Command
    {
        ReadReq		= IsRead  | IsRequest | NeedsResponse,
        WriteReq	= IsWrite | IsRequest | NeedsResponse,
        WriteReqNoAck	= IsWrite | IsRequest,
        ReadResp	= IsRead  | IsResponse,
        WriteResp	= IsWrite | IsResponse
    };

    const std::string &cmdString() const;

    /** The command of the transaction. */
    Command cmd;

    bool isRead() 	 { return (cmd & IsRead)  != 0; }
    bool isRequest()	 { return (cmd & IsRequest)  != 0; }
    bool isResponse()	 { return (cmd & IsResponse) != 0; }
    bool needsResponse() { return (cmd & NeedsResponse) != 0; }

    void makeTimingResponse() {
        assert(needsResponse());
        int icmd = (int)cmd;
        icmd &= ~(IsRequest | NeedsResponse);
        icmd |= IsResponse;
        cmd = (Command)icmd;
        dest = src;
        srcValid = false;
    }

    /** The time this request was responded to. Used to calculate latencies. */
    Tick time;

    /** The result of a particular packets request. */
    enum Result
    {
        Success,
        BadAddress,
        Unknown
    };

    /** The result of the packet transaction. */
    Result result;

    /** Accessor function that returns the source index of the packet. */
    short getSrc() const { assert(srcValid); return src; }
    void setSrc(short _src) { src = _src; srcValid = true; }

    /** Accessor function that returns the destination index of
        the packet. */
    short getDest() const { return dest; }
    void setDest(short _dest) { dest = _dest; }

    Addr getAddr() const { assert(addrValid); return addr; }
    void setAddr(Addr _addr) { addr = _addr; addrValid = true; }

    int getSize() const { assert(sizeValid); return size; }
    void setSize(int _size) { size = _size; sizeValid = true; }


    Packet(Request *_req, Command _cmd, short _dest)
        :  data(NULL), staticData(false), dynamicData(false), arrayData(false),
           addr(_req->paddr), size(_req->size), dest(_dest),
           addrValid(_req->validPaddr), sizeValid(_req->validSize),
           srcValid(false),
           req(_req), coherence(NULL), senderState(NULL), cmd(_cmd),
           time(curTick), result(Unknown)
    {
    }

    ~Packet()
    { deleteData(); }


    /** Minimally reset a packet so something like simple cpu can reuse it. */
    void reset();

    void reinitFromRequest() {
        if (req->validPaddr) setAddr(req->paddr);
        if (req->validSize)  setSize(req->size);
    }

    /** Set the data pointer to the following value that should not be freed. */
    template <typename T>
    void dataStatic(T *p);

    /** Set the data pointer to a value that should have delete [] called on it.
     */
    template <typename T>
    void dataDynamicArray(T *p);

    /** set the data pointer to a value that should have delete called on it. */
    template <typename T>
    void dataDynamic(T *p);

    /** return the value of what is pointed to in the packet. */
    template <typename T>
    T get();

    /** get a pointer to the data ptr. */
    template <typename T>
    T* getPtr();

    /** set the value in the data pointer to v. */
    template <typename T>
    void set(T v);

    /** delete the data pointed to in the data pointer. Ok to call to matter how
     * data was allocted. */
    void deleteData();

    /** If there isn't data in the packet, allocate some. */
    void allocate();

    /** Do the packet modify the same addresses. */
    bool intersect(Packet *p);
};

bool fixPacket(Packet *func, Packet *timing);
#endif //__MEM_PACKET_HH
