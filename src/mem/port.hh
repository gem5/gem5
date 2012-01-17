/*
 * Copyright (c) 2011 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 */

/**
 * @file
 * Port Object Declaration. Ports are used to interface memory objects to
 * each other.  They will always come in pairs, and we refer to the other
 * port object as the peer.  These are used to make the design more
 * modular so that a specific interface between every type of objcet doesn't
 * have to be created.
 */

#ifndef __MEM_PORT_HH__
#define __MEM_PORT_HH__

#include <list>

#include "base/misc.hh"
#include "base/range.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

/** This typedef is used to clean up getAddrRanges(). It's declared
 * outside the Port object since it's also used by some mem objects.
 * Eventually we should move this typedef to wherever Addr is
 * defined.
 */

typedef std::list<Range<Addr> > AddrRangeList;
typedef std::list<Range<Addr> >::iterator AddrRangeIter;

class MemObject;

/**
 * Ports are used to interface memory objects to
 * each other.  They will always come in pairs, and we refer to the other
 * port object as the peer.  These are used to make the design more
 * modular so that a specific interface between every type of objcet doesn't
 * have to be created.
 *
 * Recv accesor functions are being called from the peer interface.
 * Send accessor functions are being called from the device the port is
 * associated with, and it will call the peer recv. accessor function.
 */
class Port
{
  protected:
    /** Descriptive name (for DPRINTF output) */
    mutable std::string portName;

    /** A pointer to the peer port.  Ports always come in pairs, that way they
        can use a standardized interface to communicate between different
        memory objects. */
    Port *peer;

    /** A pointer to the MemObject that owns this port. This may not be set. */
    MemObject *owner;

  public:
    /**
     * Constructor.
     *
     * @param _name Port name for DPRINTF output.  Should include name
     * of memory system object to which the port belongs.
     * @param _owner Pointer to the MemObject that owns this port.
     * Will not necessarily be set.
     */
    Port(const std::string &_name, MemObject *_owner);

    /** Return port name (for DPRINTF). */
    const std::string &name() const { return portName; }

    virtual ~Port();

    void setName(const std::string &name)
    { portName = name; }

    /** Function to set the pointer for the peer port. */
    virtual void setPeer(Port *port);

    /** Function to get the pointer to the peer port. */
    Port *getPeer() { return peer; }

    /** Function to set the owner of this port. */
    void setOwner(MemObject *_owner);

    /** Function to return the owner of this port. */
    MemObject *getOwner() { return owner; }

    bool isConnected() { return peer != NULL; }

  protected:

    /** These functions are protected because they should only be
     * called by a peer port, never directly by any outside object. */

    /** Called to recive a timing call from the peer port. */
    virtual bool recvTiming(PacketPtr pkt) = 0;

    /** Called to recive a atomic call from the peer port. */
    virtual Tick recvAtomic(PacketPtr pkt) = 0;

    /** Called to recive a functional call from the peer port. */
    virtual void recvFunctional(PacketPtr pkt) = 0;

    /** Called to recieve an address range change from the peer port. */
    virtual void recvRangeChange() = 0;

    /** Called by a peer port if the send was unsuccesful, and had to
        wait.  This shouldn't be valid for response paths (IO Devices).
        so it is set to panic if it isn't already defined.
    */
    virtual void recvRetry() { panic("??"); }

    /** Called by a peer port in order to determine the block size of the
        device connected to this port.  It sometimes doesn't make sense for
        this function to be called, so it just returns 0. Anytthing that is
        concerned with the size should just ignore that.
    */
    virtual unsigned deviceBlockSize() const { return 0; }

  public:

    /**
     * Get a list of the non-overlapping address ranges we are
     * responsible for. The default implementation returns an empty
     * list and thus no address ranges. Any slave port must override
     * this function and return a populated list with at least one
     * item.
     *
     * @return a list of ranges responded to
     */
    virtual AddrRangeList getAddrRanges()
    { AddrRangeList ranges; return ranges; }

    /**
     * Determine if this port is snooping or not. The default
     * implementation returns false and thus tells the neighbour we
     * are not snooping. Any port that is to snoop (e.g. a cache
     * connected to a bus) has to override this function.
     *
     * @return true if the port should be considered a snooper
     */
    virtual bool isSnooping()
    { return false; }

    /** Function called by associated memory device (cache, memory, iodevice)
        in order to send a timing request to the port.  Simply calls the peer
        port receive function.
        @return This function returns if the send was succesful in it's
        recieve. If it was a failure, then the port will wait for a recvRetry
        at which point it can possibly issue a successful sendTiming.  This is used in
        case a cache has a higher priority request come in while waiting for
        the bus to arbitrate.
    */
    bool sendTiming(PacketPtr pkt) { return peer->recvTiming(pkt); }

    /** Function called by the associated device to send an atomic
     *   access, an access in which the data is moved and the state is
     *   updated in one cycle, without interleaving with other memory
     *   accesses.  Returns estimated latency of access.
     */
    Tick sendAtomic(PacketPtr pkt)
        { return peer->recvAtomic(pkt); }

    /** Function called by the associated device to send a functional access,
        an access in which the data is instantly updated everywhere in the
        memory system, without affecting the current state of any block or
        moving the block.
    */
    void sendFunctional(PacketPtr pkt)
        { return peer->recvFunctional(pkt); }

    /**
     * Called by the associated device to send a status range to the
     * peer interface.
     */
    void sendRangeChange() const { peer->recvRangeChange(); }

    /** When a timing access doesn't return a success, some time later the
        Retry will be sent.
    */
    void sendRetry() { return peer->recvRetry(); }

    /** Called by the associated device if it wishes to find out the blocksize
        of the device on attached to the peer port.
    */
    unsigned peerBlockSize() const { return peer->deviceBlockSize(); }

    /** This function is a wrapper around sendFunctional()
        that breaks a larger, arbitrarily aligned access into
        appropriate chunks.  The default implementation can use
        getBlockSize() to determine the block size and go from there.
    */
    virtual void readBlob(Addr addr, uint8_t *p, int size);

    /** This function is a wrapper around sendFunctional()
        that breaks a larger, arbitrarily aligned access into
        appropriate chunks.  The default implementation can use
        getBlockSize() to determine the block size and go from there.
    */
    virtual void writeBlob(Addr addr, uint8_t *p, int size);

    /** Fill size bytes starting at addr with byte value val.  This
        should not need to be virtual, since it can be implemented in
        terms of writeBlob().  However, it shouldn't be
        performance-critical either, so it could be if we wanted to.
    */
    virtual void memsetBlob(Addr addr, uint8_t val, int size);

    /** Inject a PrintReq for the given address to print the state of
     * that address throughout the memory system.  For debugging.
     */
    void printAddr(Addr a);

  private:

    /** Internal helper function for read/writeBlob().
     */
    void blobHelper(Addr addr, uint8_t *p, int size, MemCmd cmd);
};

#endif //__MEM_PORT_HH__
