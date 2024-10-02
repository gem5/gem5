/*
 * Copyright (c) 2012-2013, 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
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

/**
 * @file
 * Declaration of the base generator class for all generators.
 */

#ifndef __CPU_TRAFFIC_GEN_BASE_GEN_HH__
#define __CPU_TRAFFIC_GEN_BASE_GEN_HH__

#include <cstdint>
#include <string>

#include "base/random.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

class BaseTrafficGen;
class SimObject;

/**
 * Base class for all generators, with the shared functionality and
 * virtual functions for entering, executing and leaving the
 * generator.
 */
class BaseGen
{

  protected:

    /** Name to use for status and debug printing */
    const std::string _name;

    /** The RequestorID used for generating requests */
    const RequestorID requestorId;

    mutable Random::RandomPtr rng = Random::genRandom();

    /**
     * Generate a new request and associated packet
     *
     * @param addr Physical address to use
     * @param size Size of the request
     * @param cmd Memory command to send
     * @param flags Optional request flags
     */
    PacketPtr getPacket(Addr addr, unsigned size, const MemCmd& cmd,
                        Request::FlagsType flags = 0);

  public:

    /** Time to spend in this state */
    const Tick duration;

    /**
     * Create a base generator.
     *
     * @param obj simobject owning the generator
     * @param requestor_id RequestorID set on each request
     * @param _duration duration of this state before transitioning
     */
    BaseGen(SimObject &obj, RequestorID requestor_id, Tick _duration);

    virtual ~BaseGen() { }

    /**
     * Get the name, useful for DPRINTFs.
     *
     * @return the given name
     */
    std::string name() const { return _name; }

    /**
     * Enter this generator state.
     */
    virtual void enter() = 0;

    /**
     * Get the next generated packet.
     *
     * @return A packet to be sent at the current tick
     */
    virtual PacketPtr getNextPacket() = 0;

    /**
     * Exit this generator state. By default do nothing.
     */
    virtual void exit() { };

    /**
     * Determine the tick when the next packet is available. MaxTick
     * means that there will not be any further packets in the current
     * activation cycle of the generator.
     *
     * @param elastic should the injection respond to flow control or not
     * @param delay time the previous packet spent waiting
     * @return next tick when a packet is available
     */
    virtual Tick nextPacketTick(bool elastic, Tick delay) const = 0;

};

class StochasticGen : public BaseGen
{
  public:
    StochasticGen(SimObject &obj,
                  RequestorID requestor_id, Tick _duration,
                  Addr start_addr, Addr end_addr,
                  Addr _blocksize, Addr cacheline_size,
                  Tick min_period, Tick max_period,
                  uint8_t read_percent, Addr data_limit);

  protected:
    /** Start of address range */
    const Addr startAddr;

    /** End of address range */
    const Addr endAddr;

    /** Blocksize and address increment */
    const Addr blocksize;

    /** Cache line size in the simulated system */
    const Addr cacheLineSize;

    /** Request generation period */
    const Tick minPeriod;
    const Tick maxPeriod;

    /**
     * Percent of generated transactions that should be reads
     */
    const uint8_t readPercent;

    /** Maximum amount of data to manipulate */
    const Addr dataLimit;
};

} // namespace gem5

#endif
