/*
 * Copyright (c) 2011-2012 ARM Limited
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
 *          Andreas Hansson
 *          William Wang
 */

/**
 * @file
 * Port Object Declaration.
 */

#ifndef __MEM_PORT_HH__
#define __MEM_PORT_HH__

#include <list>

#include "base/range.hh"
#include "mem/packet.hh"

/**
 * This typedef is used to clean up getAddrRanges(). It's declared
 * outside the Port object since it's also used by some mem objects.
 * Eventually we should move this typedef to wherever Addr is
 * defined.
 */

typedef std::list<Range<Addr> > AddrRangeList;
typedef std::list<Range<Addr> >::iterator AddrRangeIter;

class MemObject;

/**
 * Ports are used to interface memory objects to each other. A port is
 * either a master or a slave and the connected peer is always of the
 * opposite role.
 *
 * Each port has a name and an owner, and enables three basic types of
 * accesses to the peer port: functional, atomic and timing.
 */
class Port
{

  private:

    /** Descriptive name (for DPRINTF output) */
    std::string portName;

  protected:

    /**
     * A numeric identifier to distinguish ports in a vector, and set
     * to InvalidPortID in case this port is not part of a vector.
     */
    const PortID id;

    /** A pointer to the peer port.  */
    Port* peer;

    /** A reference to the MemObject that owns this port. */
    MemObject& owner;

    /**
     * Abstract base class for ports
     *
     * @param _name Port name including the owners name
     * @param _owner The MemObject that is the structural owner of this port
     * @param _id A port identifier for vector ports
     */
    Port(const std::string& _name, MemObject& _owner, PortID _id);

    /**
     * Virtual destructor due to inheritance.
     */
    virtual ~Port();

  public:

    /** Return port name (for DPRINTF). */
    const std::string name() const { return portName; }

    /** Get the port id. */
    PortID getId() const { return id; }

  protected:

    /**
     * Called by a peer port if sendTimingReq, sendTimingResp or
     * sendTimingSnoopResp was unsuccesful, and had to wait.
     */
    virtual void recvRetry() = 0;

  public:

    /**
     * Send a retry to a peer port that previously attempted a
     * sendTimingReq, sendTimingResp or sendTimingSnoopResp which was
     * unsuccessful.
     */
    void sendRetry() { return peer->recvRetry(); }

};

/** Forward declaration */
class SlavePort;

/**
 * A MasterPort is a specialisation of a port. In addition to the
 * basic functionality of sending packets to its slave peer, it also
 * has functions specific to a master, e.g. to receive range changes
 * or determine if the port is snooping or not.
 */
class MasterPort : public Port
{

    friend class SlavePort;

  private:

    SlavePort* _slavePort;

  public:

    MasterPort(const std::string& name, MemObject* owner,
               PortID id = InvalidPortID);
    virtual ~MasterPort();

    void bind(SlavePort& slave_port);
    SlavePort& getSlavePort() const;
    bool isConnected() const;

    /**
     * Send an atomic request packet, where the data is moved and the
     * state is updated in zero time, without interleaving with other
     * memory accesses.
     *
     * @param pkt Packet to send.
     *
     * @return Estimated latency of access.
     */
    Tick sendAtomic(PacketPtr pkt);

    /**
     * Send a functional request packet, where the data is instantly
     * updated everywhere in the memory system, without affecting the
     * current state of any block or moving the block.
     *
     * @param pkt Packet to send.
     */
    void sendFunctional(PacketPtr pkt);

    /**
     * Attempt to send a timing request to the slave port by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvRetry at which point it can re-issue a
     * sendTimingReq.
     *
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
    */
    bool sendTimingReq(PacketPtr pkt);

    /**
     * Attempt to send a timing snoop response packet to the slave
     * port by calling its corresponding receive function. If the send
     * does not succeed, as indicated by the return value, then the
     * sender must wait for a recvRetry at which point it can re-issue
     * a sendTimingSnoopResp.
     *
     * @param pkt Packet to send.
     */
    bool sendTimingSnoopResp(PacketPtr pkt);

    /**
     * Determine if this master port is snooping or not. The default
     * implementation returns false and thus tells the neighbour we
     * are not snooping. Any master port that wants to receive snoop
     * requests (e.g. a cache connected to a bus) has to override this
     * function.
     *
     * @return true if the port should be considered a snooper
     */
    virtual bool isSnooping() const { return false; }

    /**
     * Called by a peer port in order to determine the block size of
     * the owner of this port.
     */
    virtual unsigned deviceBlockSize() const { return 0; }

    /** Called by the associated device if it wishes to find out the blocksize
        of the device on attached to the peer port.
    */
    unsigned peerBlockSize() const;

    /** Inject a PrintReq for the given address to print the state of
     * that address throughout the memory system.  For debugging.
     */
    void printAddr(Addr a);

  protected:

    /**
     * Receive an atomic snoop request packet from the slave port.
     */
    virtual Tick recvAtomicSnoop(PacketPtr pkt)
    {
        panic("%s was not expecting an atomic snoop request\n", name());
        return 0;
    }

    /**
     * Receive a functional snoop request packet from the slave port.
     */
    virtual void recvFunctionalSnoop(PacketPtr pkt)
    {
        panic("%s was not expecting a functional snoop request\n", name());
    }

    /**
     * Receive a timing response from the slave port.
     */
    virtual bool recvTimingResp(PacketPtr pkt) = 0;

    /**
     * Receive a timing snoop request from the slave port.
     */
    virtual void recvTimingSnoopReq(PacketPtr pkt)
    {
        panic("%s was not expecting a timing snoop request\n", name());
    }

    /**
     * Called to receive an address range change from the peer slave
     * port. the default implementation ignored the change and does
     * nothing. Override this function in a derived class if the owner
     * needs to be aware of he laesddress ranges, e.g. in an
     * interconnect component like a bus.
     */
    virtual void recvRangeChange() { }
};

/**
 * A SlavePort is a specialisation of a port. In addition to the
 * basic functionality of sending packets to its master peer, it also
 * has functions specific to a slave, e.g. to send range changes
 * and get the address ranges that the port responds to.
 */
class SlavePort : public Port
{

    friend class MasterPort;

  private:

    MasterPort* _masterPort;

  public:

    SlavePort(const std::string& name, MemObject* owner,
              PortID id = InvalidPortID);
    virtual ~SlavePort();

    void bind(MasterPort& master_port);
    MasterPort& getMasterPort() const;
    bool isConnected() const;

    /**
     * Send an atomic snoop request packet, where the data is moved
     * and the state is updated in zero time, without interleaving
     * with other memory accesses.
     *
     * @param pkt Snoop packet to send.
     *
     * @return Estimated latency of access.
     */
    Tick sendAtomicSnoop(PacketPtr pkt);

    /**
     * Send a functional snoop request packet, where the data is
     * instantly updated everywhere in the memory system, without
     * affecting the current state of any block or moving the block.
     *
     * @param pkt Snoop packet to send.
     */
    void sendFunctionalSnoop(PacketPtr pkt);

    /**
     * Attempt to send a timing response to the master port by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvRetry at which point it can re-issue a
     * sendTimingResp.
     *
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
    */
    bool sendTimingResp(PacketPtr pkt);

    /**
     * Attempt to send a timing snoop request packet to the master port
     * by calling its corresponding receive function. Snoop requests
     * always succeed and hence no return value is needed.
     *
     * @param pkt Packet to send.
     */
    void sendTimingSnoopReq(PacketPtr pkt);

    /**
     * Called by a peer port in order to determine the block size of
     * the owner of this port.
     */
    virtual unsigned deviceBlockSize() const { return 0; }

    /** Called by the associated device if it wishes to find out the blocksize
        of the device on attached to the peer port.
    */
    unsigned peerBlockSize() const;

    /**
     * Called by the owner to send a range change
     */
    void sendRangeChange() const { _masterPort->recvRangeChange(); }

    /**
     * Get a list of the non-overlapping address ranges the owner is
     * responsible for. All slave ports must override this function
     * and return a populated list with at least one item.
     *
     * @return a list of ranges responded to
     */
    virtual AddrRangeList getAddrRanges() = 0;

  protected:

    /**
     * Receive an atomic request packet from the master port.
     */
    virtual Tick recvAtomic(PacketPtr pkt) = 0;

    /**
     * Receive a functional request packet from the master port.
     */
    virtual void recvFunctional(PacketPtr pkt) = 0;

    /**
     * Receive a timing request from the master port.
     */
    virtual bool recvTimingReq(PacketPtr pkt) = 0;

    /**
     * Receive a timing snoop response from the master port.
     */
    virtual bool recvTimingSnoopResp(PacketPtr pkt)
    {
        panic("%s was not expecting a timing snoop response\n", name());
    }

};

#endif //__MEM_PORT_HH__
