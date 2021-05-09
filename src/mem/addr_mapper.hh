/*
 * Copyright (c) 2012 ARM Limited
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

#ifndef __MEM_ADDR_MAPPER_HH__
#define __MEM_ADDR_MAPPER_HH__

#include "mem/port.hh"
#include "params/AddrMapper.hh"
#include "params/RangeAddrMapper.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * An address mapper changes the packet addresses in going from the
 * response port side of the mapper to the request port side. When the
 * response port is queried for the address ranges, it also performs the
 * necessary range updates. Note that snoop requests that travel from
 * the request port (i.e. the memory side) to the response port are
 * currently not modified.
 */

class AddrMapper : public SimObject
{
  public:
    AddrMapper(const AddrMapperParams &params);

    virtual ~AddrMapper() = default;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void init() override;

  protected:
    /**
     * This function does the actual remapping of one address to another.
     * It is pure virtual in this case to to allow any implementation
     * required.
     * @param addr the address to remap
     * @return the new address (can be unchanged)
     */
    virtual Addr remapAddr(Addr addr) const = 0;

    class AddrMapperSenderState : public Packet::SenderState
    {

      public:

        /**
         * Construct a new sender state to remember the original address.
         *
         * @param _origAddr Address before remapping
         */
        AddrMapperSenderState(Addr _origAddr) : origAddr(_origAddr)
        {}

        /** Destructor */
        ~AddrMapperSenderState() {}

        /** The original address the packet was destined for */
        Addr origAddr;
    };

    class MapperRequestPort : public RequestPort
    {
      public:
        MapperRequestPort(const std::string& _name, AddrMapper& _mapper)
            : RequestPort(_name, &_mapper), mapper(_mapper)
        { }

      protected:
        void
        recvFunctionalSnoop(PacketPtr pkt) override
        {
            mapper.recvFunctionalSnoop(pkt);
        }

        Tick
        recvAtomicSnoop(PacketPtr pkt) override
        {
            return mapper.recvAtomicSnoop(pkt);
        }

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return mapper.recvTimingResp(pkt);
        }

        void
        recvTimingSnoopReq(PacketPtr pkt) override
        {
            mapper.recvTimingSnoopReq(pkt);
        }

        void
        recvRangeChange() override
        {
            mapper.recvRangeChange();
        }

        bool
        isSnooping() const override
        {
            return mapper.isSnooping();
        }

        void
        recvReqRetry() override
        {
            mapper.recvReqRetry();
        }

      private:
        AddrMapper& mapper;
    };

    /** Instance of request port, facing the memory side */
    MapperRequestPort memSidePort;

    class MapperResponsePort : public ResponsePort
    {
      public:
        MapperResponsePort(const std::string& _name, AddrMapper& _mapper)
            : ResponsePort(_name, &_mapper), mapper(_mapper)
        {}

      protected:
        void
        recvFunctional(PacketPtr pkt) override
        {
            mapper.recvFunctional(pkt);
        }

        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return mapper.recvAtomic(pkt);
        }

        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return mapper.recvTimingReq(pkt);
        }

        bool
        recvTimingSnoopResp(PacketPtr pkt) override
        {
            return mapper.recvTimingSnoopResp(pkt);
        }

        AddrRangeList
        getAddrRanges() const override
        {
            return mapper.getAddrRanges();
        }

        void
        recvRespRetry() override
        {
            mapper.recvRespRetry();
        }

      private:
        AddrMapper& mapper;
    };

    /** Instance of response port, i.e. on the CPU side */
    MapperResponsePort cpuSidePort;

    void recvFunctional(PacketPtr pkt);

    void recvFunctionalSnoop(PacketPtr pkt);

    Tick recvAtomic(PacketPtr pkt);

    Tick recvAtomicSnoop(PacketPtr pkt);

    bool recvTimingReq(PacketPtr pkt);

    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    bool recvTimingSnoopResp(PacketPtr pkt);

    virtual AddrRangeList getAddrRanges() const = 0;

    bool isSnooping() const;

    void recvReqRetry();

    void recvRespRetry();

    virtual void recvRangeChange();
};

/**
 * Range address mapper that maps a set of original ranges to a set of
 * remapped ranges, where a specific range is of the same size
 * (original and remapped), only with an offset. It's useful for cases
 * where memory is mapped to two different locations
 */
class RangeAddrMapper : public AddrMapper
{
  public:
    RangeAddrMapper(const RangeAddrMapperParams &p);

    ~RangeAddrMapper() = default;

    AddrRangeList getAddrRanges() const override;

    void
    init() override
    {
        AddrMapper::init();
        cpuSidePort.sendRangeChange();
    }

  protected:
    /**
     * This contains a list of ranges the should be remapped. It must
     * be the exact same length as remappedRanges which describes what
     * manipulation should be done to each range.
     */
    std::vector<AddrRange> originalRanges;

    /**
     * This contains a list of ranges that addresses should be
     * remapped to. See the description for originalRanges above
     */
    std::vector<AddrRange> remappedRanges;

    Addr remapAddr(Addr addr) const override;
    void
    recvRangeChange() override
    {
        // TODO Check that our peer is actually expecting to receive accesses
        // in our output range(s).
    }
};

} // namespace gem5

#endif //__MEM_ADDR_MAPPER_HH__
