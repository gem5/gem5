/*
 * Copyright (c) 2011-2012,2015,2017 ARM Limited
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
 */

/**
 * @file
 * Port Object Declaration.
 */

#ifndef __MEM_PORT_HH__
#define __MEM_PORT_HH__

#include "base/addr_range.hh"
#include "mem/packet.hh"
#include "mem/protocol/atomic.hh"
#include "mem/protocol/functional.hh"
#include "mem/protocol/timing.hh"
#include "sim/port.hh"

class SimObject;

/** Forward declaration */
class MasterPort;
class SlavePort;

class ResponsePort;

/**
 * A RequestPort is a specialisation of a Port, which
 * implements the default protocol for the three different level of
 * transport functions. In addition to the basic functionality of
 * sending packets, it also has functions to receive range changes or
 * determine if the port is snooping or not.
 *
 * The three protocols are atomic, timing, and functional, each with its own
 * header file.
 */
class RequestPort: public Port, public AtomicRequestProtocol,
    public TimingRequestProtocol, public FunctionalRequestProtocol
{
    friend class ResponsePort;

  private:
    ResponsePort *_responsePort;

  protected:
    SimObject &owner;

  public:
    RequestPort(const std::string& name, SimObject* _owner,
               PortID id=InvalidPortID);
    virtual ~RequestPort();

    /**
     * Bind this request port to a response port. This also does the
     * mirror action and binds the response port to the request port.
     */
    void bind(Port &peer) override;

    /**
     * Unbind this request port and the associated response port.
     */
    void unbind() override;

    /**
     * Determine if this request port is snooping or not. The default
     * implementation returns false and thus tells the neighbour we
     * are not snooping. Any request port that wants to receive snoop
     * requests (e.g. a cache connected to a bus) has to override this
     * function.
     *
     * @return true if the port should be considered a snooper
     */
    virtual bool isSnooping() const { return false; }

    /**
     * Get the address ranges of the connected responder port.
     */
    AddrRangeList getAddrRanges() const;

    /**
     * Inject a PrintReq for the given address to print the state of
     * that address throughout the memory system.  For debugging.
     */
    void printAddr(Addr a);

  public:
    /* The atomic protocol. */

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
     * Send an atomic request packet like above, but also request a backdoor
     * to the data being accessed.
     *
     * @param pkt Packet to send.
     * @param backdoor Can be set to a back door pointer by the target to let
     *        caller have direct access to the requested data.
     *
     * @return Estimated latency of access.
     */
    Tick sendAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor);

  public:
    /* The functional protocol. */

    /**
     * Send a functional request packet, where the data is instantly
     * updated everywhere in the memory system, without affecting the
     * current state of any block or moving the block.
     *
     * @param pkt Packet to send.
     */
    void sendFunctional(PacketPtr pkt) const;

  public:
    /* The timing protocol. */

    /**
     * Attempt to send a timing request to the responder port by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvReqRetry at which point it can re-issue a
     * sendTimingReq.
     *
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
    */
    bool sendTimingReq(PacketPtr pkt);

    /**
     * Check if the responder can handle a timing request.
     *
     * If the send cannot be handled at the moment, as indicated by
     * the return value, then the sender will receive a recvReqRetry
     * at which point it can re-issue a sendTimingReq.
     *
     * @param pkt Packet to send.
     *
     * @return If the send was successful or not.
     */
    bool tryTiming(PacketPtr pkt) const;

    /**
     * Attempt to send a timing snoop response packet to the response
     * port by calling its corresponding receive function. If the send
     * does not succeed, as indicated by the return value, then the
     * sender must wait for a recvRetrySnoop at which point it can
     * re-issue a sendTimingSnoopResp.
     *
     * @param pkt Packet to send.
     */
    bool sendTimingSnoopResp(PacketPtr pkt);

    /**
     * Send a retry to the response port that previously attempted a
     * sendTimingResp to this request port and failed. Note that this
     * is virtual so that the "fake" snoop response port in the
     * coherent crossbar can override the behaviour.
     */
    virtual void sendRetryResp();

  protected:
    /**
     * Called to receive an address range change from the peer response
     * port. The default implementation ignores the change and does
     * nothing. Override this function in a derived class if the owner
     * needs to be aware of the address ranges, e.g. in an
     * interconnect component like a bus.
     */
    virtual void recvRangeChange() { }

    /**
     * Default implementations.
     */
    Tick
    recvAtomicSnoop(PacketPtr pkt) override
    {
        panic("%s was not expecting an atomic snoop request\n", name());
        return 0;
    }

    void
    recvFunctionalSnoop(PacketPtr pkt) override
    {
        panic("%s was not expecting a functional snoop request\n", name());
    }

    void
    recvTimingSnoopReq(PacketPtr pkt) override
    {
        panic("%s was not expecting a timing snoop request.\n", name());
    }

    void
    recvRetrySnoopResp() override
    {
        panic("%s was not expecting a snoop retry.\n", name());
    }
};

class M5_DEPRECATED MasterPort : public RequestPort
{
  public:
    MasterPort(const std::string& name, SimObject* _owner,
               PortID id=InvalidPortID) : RequestPort(name, _owner, id)
               {}
};

/**
 * A ResponsePort is a specialization of a port. In addition to the
 * basic functionality of sending packets to its requestor peer, it also
 * has functions specific to a responder, e.g. to send range changes
 * and get the address ranges that the port responds to.
 *
 * The three protocols are atomic, timing, and functional, each with its own
 * header file.
 */
class ResponsePort : public Port, public AtomicResponseProtocol,
    public TimingResponseProtocol, public FunctionalResponseProtocol
{
    friend class RequestPort;

  private:
    RequestPort* _requestPort;

    bool defaultBackdoorWarned;

  protected:
    SimObject& owner;

  public:
    ResponsePort(const std::string& name, SimObject* _owner,
              PortID id=InvalidPortID);
    virtual ~ResponsePort();

    /**
     * Find out if the peer request port is snooping or not.
     *
     * @return true if the peer request port is snooping
     */
    bool isSnooping() const { return _requestPort->isSnooping(); }

    /**
     * Called by the owner to send a range change
     */
    void sendRangeChange() const { _requestPort->recvRangeChange(); }

    /**
     * Get a list of the non-overlapping address ranges the owner is
     * responsible for. All response ports must override this function
     * and return a populated list with at least one item.
     *
     * @return a list of ranges responded to
     */
    virtual AddrRangeList getAddrRanges() const = 0;

    /**
     * We let the request port do the work, so these don't do anything.
     */
    void unbind() override {}
    void bind(Port &peer) override {}

  public:
    /* The atomic protocol. */

    /**
     * Send an atomic snoop request packet, where the data is moved
     * and the state is updated in zero time, without interleaving
     * with other memory accesses.
     *
     * @param pkt Snoop packet to send.
     *
     * @return Estimated latency of access.
     */
    Tick
    sendAtomicSnoop(PacketPtr pkt)
    {
        try {
            return AtomicResponseProtocol::sendSnoop(_requestPort, pkt);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

  public:
    /* The functional protocol. */

    /**
     * Send a functional snoop request packet, where the data is
     * instantly updated everywhere in the memory system, without
     * affecting the current state of any block or moving the block.
     *
     * @param pkt Snoop packet to send.
     */
    void
    sendFunctionalSnoop(PacketPtr pkt) const
    {
        try {
            FunctionalResponseProtocol::sendSnoop(_requestPort, pkt);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

  public:
    /* The timing protocol. */

    /**
     * Attempt to send a timing response to the request port by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvRespRetry at which point it can re-issue a
     * sendTimingResp.
     *
     * @param pkt Packet to send.
     *
     * @return If the send was successful or not.
    */
    bool
    sendTimingResp(PacketPtr pkt)
    {
        try {
            return TimingResponseProtocol::sendResp(_requestPort, pkt);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

    /**
     * Attempt to send a timing snoop request packet to the request port
     * by calling its corresponding receive function. Snoop requests
     * always succeed and hence no return value is needed.
     *
     * @param pkt Packet to send.
     */
    void
    sendTimingSnoopReq(PacketPtr pkt)
    {
        try {
            TimingResponseProtocol::sendSnoopReq(_requestPort, pkt);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

    /**
     * Send a retry to the request port that previously attempted a
     * sendTimingReq to this response port and failed.
     */
    void
    sendRetryReq()
    {
        try {
            TimingResponseProtocol::sendRetryReq(_requestPort);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

    /**
     * Send a retry to the request port that previously attempted a
     * sendTimingSnoopResp to this response port and failed.
     */
    void
    sendRetrySnoopResp()
    {
        try {
            TimingResponseProtocol::sendRetrySnoopResp(_requestPort);
        } catch (UnboundPortException) {
            reportUnbound();
        }
    }

  protected:
    /**
     * Called by the request port to unbind. Should never be called
     * directly.
     */
    void responderUnbind();

    /**
     * Called by the request port to bind. Should never be called
     * directly.
     */
    void responderBind(RequestPort& request_port);

    /**
     * Default implementations.
     */
    Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override;

    bool
    tryTiming(PacketPtr pkt) override
    {
        panic("%s was not expecting a %s\n", name(), __func__);
    }

    bool
    recvTimingSnoopResp(PacketPtr pkt) override
    {
        panic("%s was not expecting a timing snoop response\n", name());
    }
};

class M5_DEPRECATED SlavePort : public ResponsePort
{
  public:
    SlavePort(const std::string& name, SimObject* _owner,
              PortID id=InvalidPortID) : ResponsePort(name, _owner, id)
              {}
};

inline Tick
RequestPort::sendAtomic(PacketPtr pkt)
{
    try {
        return AtomicRequestProtocol::send(_responsePort, pkt);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline Tick
RequestPort::sendAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    try {
        return AtomicRequestProtocol::sendBackdoor(_responsePort,
                                                    pkt, backdoor);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline void
RequestPort::sendFunctional(PacketPtr pkt) const
{
    try {
        return FunctionalRequestProtocol::send(_responsePort, pkt);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline bool
RequestPort::sendTimingReq(PacketPtr pkt)
{
    try {
        return TimingRequestProtocol::sendReq(_responsePort, pkt);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline bool
RequestPort::tryTiming(PacketPtr pkt) const
{
    try {
        return TimingRequestProtocol::trySend(_responsePort, pkt);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline bool
RequestPort::sendTimingSnoopResp(PacketPtr pkt)
{
    try {
        return TimingRequestProtocol::sendSnoopResp(_responsePort, pkt);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

inline void
RequestPort::sendRetryResp()
{
    try {
        TimingRequestProtocol::sendRetryResp(_responsePort);
    } catch (UnboundPortException) {
        reportUnbound();
    }
}

#endif //__MEM_PORT_HH__
