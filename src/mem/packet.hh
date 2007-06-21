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
 *
 * Authors: Ron Dreslinski
 *          Steve Reinhardt
 *          Ali Saidi
 */

/**
 * @file
 * Declaration of the Packet class.
 */

#ifndef __MEM_PACKET_HH__
#define __MEM_PACKET_HH__

#include <cassert>
#include <list>
#include <bitset>

#include "base/compiler.hh"
#include "base/fast_alloc.hh"
#include "base/misc.hh"
#include "mem/request.hh"
#include "sim/host.hh"
#include "sim/core.hh"


struct Packet;
typedef Packet *PacketPtr;
typedef uint8_t* PacketDataPtr;
typedef std::list<PacketPtr> PacketList;

class MemCmd
{
  public:

    /** List of all commands associated with a packet. */
    enum Command
    {
        InvalidCmd,
        ReadReq,
        ReadResp,
        WriteReq,
        WriteResp,
        Writeback,
        SoftPFReq,
        HardPFReq,
        SoftPFResp,
        HardPFResp,
        WriteInvalidateReq,
        WriteInvalidateResp,
        UpgradeReq,
        UpgradeResp,
        ReadExReq,
        ReadExResp,
        LoadLockedReq,
        LoadLockedResp,
        StoreCondReq,
        StoreCondResp,
        SwapReq,
        SwapResp,
        NUM_MEM_CMDS
    };

  private:
    /** List of command attributes. */
    enum Attribute
    {
        IsRead,         //!< Data flows from responder to requester
        IsWrite,        //!< Data flows from requester to responder
        IsPrefetch,     //!< Not a demand access
        IsInvalidate,
        NeedsExclusive, //!< Requires exclusive copy to complete in-cache
        IsRequest,      //!< Issued by requester
        IsResponse,     //!< Issue by responder
        NeedsResponse,  //!< Requester needs response from target
        IsSWPrefetch,
        IsHWPrefetch,
        IsLocked,       //!< Alpha/MIPS LL or SC access
        HasData,        //!< There is an associated payload
        NUM_COMMAND_ATTRIBUTES
    };

    /** Structure that defines attributes and other data associated
     * with a Command. */
    struct CommandInfo {
        /** Set of attribute flags. */
        const std::bitset<NUM_COMMAND_ATTRIBUTES> attributes;
        /** Corresponding response for requests; InvalidCmd if no
         * response is applicable. */
        const Command response;
        /** String representation (for printing) */
        const std::string str;
    };

    /** Array to map Command enum to associated info. */
    static const CommandInfo commandInfo[];

  private:

    Command cmd;

    bool testCmdAttrib(MemCmd::Attribute attrib) const {
        return commandInfo[cmd].attributes[attrib] != 0;
    }

  public:

    bool isRead() const         { return testCmdAttrib(IsRead); }
    bool isWrite()  const       { return testCmdAttrib(IsWrite); }
    bool isRequest() const      { return testCmdAttrib(IsRequest); }
    bool isResponse() const     { return testCmdAttrib(IsResponse); }
    bool needsExclusive() const  { return testCmdAttrib(NeedsExclusive); }
    bool needsResponse() const  { return testCmdAttrib(NeedsResponse); }
    bool isInvalidate() const   { return testCmdAttrib(IsInvalidate); }
    bool hasData() const        { return testCmdAttrib(HasData); }
    bool isReadWrite() const    { return isRead() && isWrite(); }
    bool isLocked() const       { return testCmdAttrib(IsLocked); }

    const Command responseCommand() const {
        return commandInfo[cmd].response;
    }

    /** Return the string to a cmd given by idx. */
    const std::string &toString() const {
        return commandInfo[cmd].str;
    }

    int toInt() const { return (int)cmd; }

    MemCmd(Command _cmd)
        : cmd(_cmd)
    { }

    MemCmd(int _cmd)
        : cmd((Command)_cmd)
    { }

    MemCmd()
        : cmd(InvalidCmd)
    { }

    bool operator==(MemCmd c2) { return (cmd == c2.cmd); }
    bool operator!=(MemCmd c2) { return (cmd != c2.cmd); }

    friend class Packet;
};

/**
 * A Packet is used to encapsulate a transfer between two objects in
 * the memory system (e.g., the L1 and L2 cache).  (In contrast, a
 * single Request travels all the way from the requester to the
 * ultimate destination and back, possibly being conveyed by several
 * different Packets along the way.)
 */
class Packet : public FastAlloc
{
  public:

    typedef MemCmd::Command Command;

  private:
   /** A pointer to the data being transfered.  It can be differnt
    *    sizes at each level of the heirarchy so it belongs in the
    *    packet, not request. This may or may not be populated when a
    *    responder recieves the packet. If not populated it memory
    *    should be allocated.
    */
    PacketDataPtr data;

    /** Is the data pointer set to a value that shouldn't be freed
     *   when the packet is destroyed? */
    bool staticData;
    /** The data pointer points to a value that should be freed when
     *   the packet is destroyed. */
    bool dynamicData;
    /** the data pointer points to an array (thus delete [] ) needs to
     *   be called on it rather than simply delete.*/
    bool arrayData;

    /** The address of the request.  This address could be virtual or
     *   physical, depending on the system configuration. */
    Addr addr;

     /** The size of the request or transfer. */
    int size;

    /** Device address (e.g., bus ID) of the source of the
     *   transaction. The source is not responsible for setting this
     *   field; it is set implicitly by the interconnect when the
     *   packet is first sent.  */
    short src;

    /** Device address (e.g., bus ID) of the destination of the
     *   transaction. The special value Broadcast indicates that the
     *   packet should be routed based on its address. This field is
     *   initialized in the constructor and is thus always valid
     *   (unlike * addr, size, and src). */
    short dest;

    /** Are the 'addr' and 'size' fields valid? */
    bool addrSizeValid;
    /** Is the 'src' field valid? */
    bool srcValid;

    enum SnoopFlag {
        MemInhibit,
        Shared,
        NUM_SNOOP_FLAGS
    };

    /** Coherence snoopFlags for snooping */
    std::bitset<NUM_SNOOP_FLAGS> snoopFlags;

  public:

    /** Used to calculate latencies for each packet.*/
    Tick time;

    /** The time at which the packet will be fully transmitted */
    Tick finishTime;

    /** The time at which the first chunk of the packet will be transmitted */
    Tick firstWordTime;

    /** The special destination address indicating that the packet
     *   should be routed based on its address. */
    static const short Broadcast = -1;

    /** A pointer to the original request. */
    RequestPtr req;

    /** A virtual base opaque structure used to hold coherence-related
     *    state.  A specific subclass would be derived from this to
     *    carry state specific to a particular coherence protocol.  */
    class CoherenceState : public FastAlloc {
      public:
        virtual ~CoherenceState() {}
    };

    /** This packet's coherence state.  Caches should use
     *   dynamic_cast<> to cast to the state appropriate for the
     *   system's coherence protocol.  */
    CoherenceState *coherence;

    /** A virtual base opaque structure used to hold state associated
     *    with the packet but specific to the sending device (e.g., an
     *    MSHR).  A pointer to this state is returned in the packet's
     *    response so that the sender can quickly look up the state
     *    needed to process it.  A specific subclass would be derived
     *    from this to carry state specific to a particular sending
     *    device.  */
    class SenderState : public FastAlloc {
      public:
        virtual ~SenderState() {}
    };

    /** This packet's sender state.  Devices should use dynamic_cast<>
     *   to cast to the state appropriate to the sender. */
    SenderState *senderState;

  public:

    /** The command field of the packet. */
    MemCmd cmd;

    /** Return the string name of the cmd field (for debugging and
     *   tracing). */
    const std::string &cmdString() const { return cmd.toString(); }

    /** Return the index of this command. */
    inline int cmdToIndex() const { return cmd.toInt(); }

  public:

    bool isRead() const         { return cmd.isRead(); }
    bool isWrite()  const       { return cmd.isWrite(); }
    bool isRequest() const      { return cmd.isRequest(); }
    bool isResponse() const     { return cmd.isResponse(); }
    bool needsExclusive() const  { return cmd.needsExclusive(); }
    bool needsResponse() const  { return cmd.needsResponse(); }
    bool isInvalidate() const   { return cmd.isInvalidate(); }
    bool hasData() const        { return cmd.hasData(); }
    bool isReadWrite() const    { return cmd.isReadWrite(); }
    bool isLocked() const       { return cmd.isLocked(); }

    void assertMemInhibit()     { snoopFlags[MemInhibit] = true; }
    void assertShared()         { snoopFlags[Shared] = true; }
    bool memInhibitAsserted()   { return snoopFlags[MemInhibit]; }
    bool sharedAsserted()       { return snoopFlags[Shared]; }

    bool nic_pkt() { panic("Unimplemented"); M5_DUMMY_RETURN }

    /** Possible results of a packet's request. */
    enum Result
    {
        Success,
        BadAddress,
        Nacked,
        Unknown
    };

    /** The result of this packet's request. */
    Result result;

    /** Accessor function that returns the source index of the packet. */
    short getSrc() const { assert(srcValid); return src; }
    void setSrc(short _src) { src = _src; srcValid = true; }
    /** Reset source field, e.g. to retransmit packet on different bus. */
    void clearSrc() { srcValid = false; }

    /** Accessor function that returns the destination index of
        the packet. */
    short getDest() const { return dest; }
    void setDest(short _dest) { dest = _dest; }

    Addr getAddr() const { assert(addrSizeValid); return addr; }
    int getSize() const { assert(addrSizeValid); return size; }
    Addr getOffset(int blkSize) const { return addr & (Addr)(blkSize - 1); }

    void addrOverride(Addr newAddr) { assert(addrSizeValid); addr = newAddr; }
    void cmdOverride(MemCmd newCmd) { cmd = newCmd; }

    /** Constructor.  Note that a Request object must be constructed
     *   first, but the Requests's physical address and size fields
     *   need not be valid. The command and destination addresses
     *   must be supplied.  */
    Packet(Request *_req, MemCmd _cmd, short _dest)
        :  data(NULL), staticData(false), dynamicData(false), arrayData(false),
           addr(_req->paddr), size(_req->size), dest(_dest),
           addrSizeValid(_req->validPaddr), srcValid(false),
           snoopFlags(0),
           time(curTick),
           req(_req), coherence(NULL), senderState(NULL), cmd(_cmd),
           result(Unknown)
    {
    }

    /** Alternate constructor if you are trying to create a packet with
     *  a request that is for a whole block, not the address from the req.
     *  this allows for overriding the size/addr of the req.*/
    Packet(Request *_req, MemCmd _cmd, short _dest, int _blkSize)
        :  data(NULL), staticData(false), dynamicData(false), arrayData(false),
           addr(_req->paddr & ~(_blkSize - 1)), size(_blkSize), dest(_dest),
           addrSizeValid(_req->validPaddr), srcValid(false),
           snoopFlags(0),
           time(curTick),
           req(_req), coherence(NULL), senderState(NULL), cmd(_cmd),
           result(Unknown)
    {
    }

    /** Alternate constructor for copying a packet.  Copy all fields
     * *except* set data allocation as static... even if the original
     * packet's data was dynamic, we don't want to free it when the
     * new packet is deallocated.  Note that if original packet used
     * dynamic data, user must guarantee that the new packet's
     * lifetime is less than that of the original packet. */
    Packet(Packet *origPkt)
        :  data(NULL), staticData(false), dynamicData(false), arrayData(false),
           addr(origPkt->addr), size(origPkt->size),
           dest(origPkt->dest),
           addrSizeValid(origPkt->addrSizeValid), srcValid(origPkt->srcValid),
           snoopFlags(origPkt->snoopFlags),
           time(curTick),
           req(origPkt->req), coherence(origPkt->coherence),
           senderState(origPkt->senderState), cmd(origPkt->cmd),
           result(origPkt->result)
    {
    }

    /** Destructor. */
    ~Packet()
    { if (staticData || dynamicData) deleteData(); }

    /** Reinitialize packet address and size from the associated
     *   Request object, and reset other fields that may have been
     *   modified by a previous transaction.  Typically called when a
     *   statically allocated Request/Packet pair is reused for
     *   multiple transactions. */
    void reinitFromRequest() {
        assert(req->validPaddr);
        snoopFlags = 0;
        addr = req->paddr;
        size = req->size;
        time = req->time;
        addrSizeValid = true;
        result = Unknown;
        if (dynamicData) {
            deleteData();
            dynamicData = false;
            arrayData = false;
        }
    }

    /**
     * Take a request packet and modify it in place to be suitable for
     * returning as a response to that request.  The source and
     * destination fields are *not* modified, as is appropriate for
     * atomic accesses.
     */
    void makeAtomicResponse()
    {
        assert(needsResponse());
        assert(isRequest());
        assert(result == Unknown);
        cmd = cmd.responseCommand();
        result = Success;
    }

    /**
     * Perform the additional work required for timing responses above
     * and beyond atomic responses; i.e., change the destination to
     * point back to the requester and clear the source field.
     */
    void convertAtomicToTimingResponse()
    {
        dest = src;
        srcValid = false;
    }

    /**
     * Take a request packet and modify it in place to be suitable for
     * returning as a response to a timing request.
     */
    void makeTimingResponse()
    {
        makeAtomicResponse();
        convertAtomicToTimingResponse();
    }

    /**
     * Take a request packet that has been returned as NACKED and
     * modify it so that it can be sent out again. Only packets that
     * need a response can be NACKED, so verify that that is true.
     */
    void
    reinitNacked()
    {
        assert(needsResponse() && result == Nacked);
        dest =  Broadcast;
        result = Unknown;
    }


    /**
     * Set the data pointer to the following value that should not be
     * freed.
     */
    template <typename T>
    void
    dataStatic(T *p)
    {
        if(dynamicData)
            dynamicData = false;
        data = (PacketDataPtr)p;
        staticData = true;
    }

    /**
     * Set the data pointer to a value that should have delete []
     * called on it.
     */
    template <typename T>
    void
    dataDynamicArray(T *p)
    {
        assert(!staticData && !dynamicData);
        data = (PacketDataPtr)p;
        dynamicData = true;
        arrayData = true;
    }

    /**
     * set the data pointer to a value that should have delete called
     * on it.
     */
    template <typename T>
    void
    dataDynamic(T *p)
    {
        assert(!staticData && !dynamicData);
        data = (PacketDataPtr)p;
        dynamicData = true;
        arrayData = false;
    }

    /** get a pointer to the data ptr. */
    template <typename T>
    T*
    getPtr()
    {
        assert(staticData || dynamicData);
        return (T*)data;
    }

    /** return the value of what is pointed to in the packet. */
    template <typename T>
    T get();

    /** set the value in the data pointer to v. */
    template <typename T>
    void set(T v);

    /**
     * Copy data into the packet from the provided pointer.
     */
    void setData(uint8_t *p)
    {
        std::memcpy(getPtr<uint8_t>(), p, getSize());
    }

    /**
     * Copy data into the packet from the provided block pointer,
     * which is aligned to the given block size.
     */
    void setDataFromBlock(uint8_t *blk_data, int blkSize)
    {
        setData(blk_data + getOffset(blkSize));
    }

    /**
     * Copy data from the packet to the provided block pointer, which
     * is aligned to the given block size.
     */
    void writeData(uint8_t *p)
    {
        std::memcpy(p, getPtr<uint8_t>(), getSize());
    }

    /**
     * Copy data from the packet to the memory at the provided pointer.
     */
    void writeDataToBlock(uint8_t *blk_data, int blkSize)
    {
        writeData(blk_data + getOffset(blkSize));
    }

    /**
     * delete the data pointed to in the data pointer. Ok to call to
     * matter how data was allocted.
     */
    void deleteData();

    /** If there isn't data in the packet, allocate some. */
    void allocate();

    /** Do the packet modify the same addresses. */
    bool intersect(PacketPtr p);

    /**
     * Check a functional request against a memory value represented
     * by a base/size pair and an associated data array.  If the
     * functional request is a read, it may be satisfied by the memory
     * value.  If the functional request is a write, it may update the
     * memory value.
     */
    bool checkFunctional(Addr base, int size, uint8_t *data);

    /**
     * Check a functional request against a memory value stored in
     * another packet (i.e. an in-transit request or response).
     */
    bool checkFunctional(PacketPtr otherPkt) {
        return (otherPkt->hasData() &&
                checkFunctional(otherPkt->getAddr(), otherPkt->getSize(),
                                otherPkt->getPtr<uint8_t>()));
    }
};



/** Temporary for backwards compatibility.
 */
inline
bool fixPacket(PacketPtr func, PacketPtr timing) {
    return !func->checkFunctional(timing);
}

/** This function is a wrapper for the fixPacket field that toggles
 * the hasData bit it is used when a response is waiting in the
 * caches, but hasn't been marked as a response yet (so the fixPacket
 * needs to get the correct value for the hasData)
 */
bool fixDelayedResponsePacket(PacketPtr func, PacketPtr timing);

std::ostream & operator<<(std::ostream &o, const Packet &p);

#endif //__MEM_PACKET_HH
